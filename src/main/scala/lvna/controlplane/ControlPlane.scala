package lvna

import Chisel._
import boom.system.{HasBoomTiles, HasBoomTilesModuleImp}
import chisel3.VecInit
import chisel3.core.{IO, Input, Output}
import freechips.rocketchip.config.{Config, Field, Parameters}
import freechips.rocketchip.diplomacy.{LazyModule, LazyModuleImp}
import freechips.rocketchip.subsystem._
import freechips.rocketchip.system.{ExampleRocketSystem, ExampleRocketSystemModuleImp, UseEmu}
import freechips.rocketchip.tile.XLen
import freechips.rocketchip.util._

import scala.math.round
import freechips.rocketchip.util.GTimer

case object ProcDSidWidth extends Field[Int](3)

trait HasControlPlaneParameters {
  implicit val p: Parameters
  val nTiles = p(NTiles)
  val ldomDSidWidth = log2Up(nTiles)
  val procDSidWidth = p(ProcDSidWidth)
  val dsidWidth = ldomDSidWidth + procDSidWidth
  val nDSID = 1 << dsidWidth
  val cacheCapacityWidth = if (p(NL2CacheCapacity) != 0) log2Ceil(p(NL2CacheCapacity) * 1024 / 64) else 1
}

/**
 * From ControlPlane's side of view.
 */
class ControlPlaneIO(implicit val p: Parameters) extends Bundle with HasControlPlaneParameters {
  private val indexWidth = 32

  val updateData   = UInt(INPUT, 32)
  val traffic      = UInt(OUTPUT, 32)
  val cycle        = UInt(OUTPUT, 32)
  val capacity     = UInt(OUTPUT, cacheCapacityWidth)
  val hartDsid     = UInt(OUTPUT, ldomDSidWidth)
  val hartDsidWen  = Bool(INPUT)
  val memBase      = UInt(OUTPUT, p(XLen))
  val memBaseLoWen = Bool(INPUT)
  val memBaseHiWen = Bool(INPUT)
  val memMask      = UInt(OUTPUT, p(XLen))
  val memMaskLoWen = Bool(INPUT)
  val memMaskHiWen = Bool(INPUT)
  val bucket       = new BucketBundle().asOutput
  val bktFreqWen   = Bool(INPUT)
  val bktSizeWen   = Bool(INPUT)
  val bktIncWen    = Bool(INPUT)
  val waymask      = UInt(OUTPUT, p(NL2CacheWays))
  val waymaskWen   = Bool(INPUT)
  val hartSel      = UInt(OUTPUT, indexWidth)
  val hartSelWen   = Bool(INPUT)
  val dsidSel      = UInt(OUTPUT, dsidWidth)
  val dsidSelWen   = Bool(INPUT)

  val limitIndex        = UInt(OUTPUT, 4)
  val limitIndexWen     = Bool(INPUT)
  val limit             = UInt(OUTPUT, 16)
  val limitWen          = Bool(INPUT)
  val lowThreshold      = UInt(OUTPUT, 8)
  val lowThresholdWen   = Bool(INPUT)
  val highThreshold     = UInt(OUTPUT, 8)
  val highThresholdWen  = Bool(INPUT)

  val maxQuota      = UInt(OUTPUT, 8)
  val maxQuotaWen   = Bool(INPUT)
  val minQuota      = UInt(OUTPUT, 8)
  val minQuotaWen   = Bool(INPUT)
  val quotaStep     = UInt(OUTPUT, 8)
  val quotaStepWen  = Bool(INPUT)

  val readPC              = Bool(INPUT)
  val doneReadPC          = Bool(INPUT)
  val autoPCSnapShotWen   = Bool(INPUT)
  val autoPCSnapShotEn    = Bool(OUTPUT)
  val PC                  = UInt(OUTPUT, p(XLen))

  val assertDebugInt      = Bool(INPUT)
}

/* From ControlPlane's View */
class CPToL2CacheIO(implicit val p: Parameters) extends Bundle with HasControlPlaneParameters {
  val waymask = Output(UInt(p(NL2CacheWays).W))  // waymask returned to L2cache (1 cycle delayed)
  val dsid = Input(UInt(dsidWidth.W))  // DSID from requests L2 cache received
  val capacity = Input(UInt(cacheCapacityWidth.W))  // Count on way numbers
  val capacity_dsid = Output(UInt(dsidWidth.W))  // Capacity query dsid
}

class BucketState(implicit val p: Parameters) extends Bundle with HasControlPlaneParameters with HasTokenBucketParameters {
  val nToken = SInt(tokenBucketSizeWidth.W)
  val traffic = UInt(tokenBucketSizeWidth.W)
  val counter = UInt(32.W)
  val enable = Bool()
}

class BucketIO(implicit val p: Parameters) extends Bundle with HasControlPlaneParameters with HasTokenBucketParameters {
  val dsid = Input(UInt(dsidWidth.W))
  val size = Input(UInt(tokenBucketSizeWidth.W))
  val fire = Input(Bool())
  val enable = Output(Bool())
}

trait HasTokenBucketPlane extends HasControlPlaneParameters with HasTokenBucketParameters {
  private val bucket_debug = false

  val bucketParams = RegInit(Vec(Seq.fill(nDSID){
    Cat(128.U(tokenBucketSizeWidth.W), 8.U(tokenBucketFreqWidth.W), 1.U(tokenBucketSizeWidth.W)).asTypeOf(new BucketBundle)
  }))

  val bucketState = RegInit(Vec(Seq.fill(nDSID){
    Cat(0.U(tokenBucketSizeWidth.W), 0.U(tokenBucketSizeWidth.W), 0.U(32.W), true.B).asTypeOf(new BucketState)
  }))

  val bucketIO = IO(Vec(nTiles, new BucketIO()))

  val timer = GTimer()

  bucketState.zipWithIndex.foreach { case (state, i) =>
    state.counter := Mux(state.counter >= bucketParams(i).freq, 0.U, state.counter + 1.U)
    val req_sizes = bucketIO.map{bio => Mux(bio.dsid === i.U && bio.fire, bio.size, 0.U) }
    val req_all = req_sizes.reduce(_ + _) >> 3
    val updating = state.counter >= bucketParams(i).freq
    val inc_size = Mux(updating, bucketParams(i).inc.asSInt(), 0.S)
    val enable_next = state.nToken + inc_size > req_all.asSInt
    val calc_next = state.nToken + inc_size - req_all.asSInt
    val limit_next = Mux(calc_next < bucketParams(i).size.asSInt, calc_next, bucketParams(i).size.asSInt)

    val has_requester = bucketIO.map{bio => bio.dsid === i.U && bio.fire}.reduce(_ || _)
    when (has_requester) {
      state.traffic := state.traffic + req_all.asUInt
    }

    when (has_requester || updating) {
//      state.nToken := Mux(enable_next, limit_next, 0.U)
      state.nToken := limit_next
      state.enable := enable_next || (bucketParams(i).freq === 0.U)
    }

    if (bucket_debug && i <= 1) {
      printf(s"cycle: %d bucket %d req_all %d tokens %d inc %d enable_next %b counter %d traffic %d\n",
        GTimer(), i.U(dsidWidth.W), req_all, state.nToken, inc_size, enable_next, state.counter, state.traffic)
    }
  }

  bucketIO.foreach { bio =>
    bio.enable := bucketState(bio.dsid).enable
  }
}

class ControlPlane()(implicit p: Parameters) extends LazyModule
with HasControlPlaneParameters
with HasTokenBucketParameters
{
  private val memAddrWidth = p(XLen)
  private val instAddrWidth = p(XLen)
  private val totalBW = if (p(UseEmu)) 55*8 else 20*8

  override lazy val module = new LazyModuleImp(this) with HasTokenBucketPlane {
    val io = IO(new Bundle {
      val hartDsids = Vec(nTiles, UInt(ldomDSidWidth.W)).asOutput
      val memBases  = Vec(nTiles, UInt(memAddrWidth.W)).asOutput
      val memMasks  = Vec(nTiles, UInt(memAddrWidth.W)).asOutput
      val pc        = Vec(nTiles, UInt(instAddrWidth.W)).asInput
      val l2        = new CPToL2CacheIO()
      val cp        = new ControlPlaneIO()
    })

    val hartSel   = RegInit(0.U(ldomDSidWidth.W))
    val hartDsids = RegInit(Vec(Seq.tabulate(nTiles)(_.U(ldomDSidWidth.W))))
    val memBases  = RegInit(Vec(Seq.tabulate(nTiles){ i =>
      if (p(UseEmu)) {
        val memSize: BigInt = p(ExtMem).map { m => m.master.size }.getOrElse(0x80000000)
        (i * memSize / nTiles).U(memAddrWidth.W)
      } else {
        0.U(memAddrWidth.W)
      }
    }))
    val memMasks  = RegInit(Vec(Seq.fill(nTiles)(~0.U(memAddrWidth.W))))
    val waymasks  = RegInit(Vec(Seq.fill(nTiles){ ((1L << p(NL2CacheWays)) - 1).U }))
    val l2dsid_reg = RegNext(io.l2.dsid)  // 1 cycle delay
    io.l2.waymask := waymasks(l2dsid_reg)

    val currDsid = RegEnable(io.cp.updateData, 0.U, io.cp.dsidSelWen)
    io.cp.dsidSel := currDsid
    io.cp.waymask := waymasks(currDsid)
    io.cp.traffic := bucketState(currDsid).traffic
    io.cp.cycle := timer
    io.cp.capacity := io.l2.capacity
    io.l2.capacity_dsid := currDsid

    io.cp.hartDsid := hartDsids(hartSel)
    io.cp.hartSel := hartSel
    io.cp.memBase := memBases(hartSel)
    io.cp.memMask := memMasks(hartSel)
    io.cp.bucket := bucketParams(currDsid)
    io.hartDsids := hartDsids
    io.memBases := memBases
    io.memMasks := memMasks

    when (io.cp.hartSelWen) {
      hartSel := io.cp.updateData
    }

    when (io.cp.hartDsidWen) {
      hartDsids(hartSel) := io.cp.updateData
    }

    val mem_base_lo_tmp = RegInit(0.U(32.W))
    val mem_mask_lo_tmp = RegInit(~0.U(32.W))

    when (io.cp.memBaseLoWen) {
      mem_base_lo_tmp := io.cp.updateData
    }

    when (io.cp.memBaseHiWen) {
      memBases(hartSel) := Cat(io.cp.updateData, mem_base_lo_tmp)
    }

    when (io.cp.memMaskLoWen) {
      mem_mask_lo_tmp := io.cp.updateData
    }

    when (io.cp.memMaskHiWen) {
      memMasks(hartSel) := Cat(io.cp.updateData, mem_mask_lo_tmp)
    }

    when (io.cp.bktFreqWen) {
      bucketParams(currDsid).freq := io.cp.updateData
    }

    when (io.cp.bktSizeWen) {
      bucketParams(currDsid).size := io.cp.updateData
    }

    when (io.cp.bktIncWen) {
      bucketParams(currDsid).inc := io.cp.updateData
    }

    when (io.cp.waymaskWen) {
      waymasks(currDsid) := io.cp.updateData
    }

    // AutoMBA goes here

    val accountingCycle = 10000
    val cycleCounter = RegInit(0.asUInt(64.W))
    val lastTraffic = RegInit(Vec(Seq.fill(nTiles){ 0.U(32.W) }))

    when (GTimer() >= cycleCounter) {
      for (i <- 0 until nTiles) {
        // 输出每段时间内，产生了多少流量，用于精细观察流量控制策略
        // 输出到了每个时间点后的CDF，用于看整体的流量曲线，观察流量控制效果
        lastTraffic(i) := bucketState(i).traffic
      }
      cycleCounter := cycleCounter + accountingCycle.U
    }

    private val policy = "yzh"

    private val highPriorIndex = if (p(UseEmu)) 0 else nTiles
    private val startIndex = if (p(UseEmu)) 1 else 0
    private val limitScaleIndex= if (p(UseEmu)) 1 else 3

    private val quota = RegInit((totalBW/10*5).U(8.W))
    private val curLevel = RegInit(4.asUInt(4.W))

    private val maxQuota = RegEnable(io.cp.updateData, (totalBW/10*9).U(11.W), io.cp.maxQuotaWen)
    private val minQuota = RegEnable(io.cp.updateData, (totalBW/10).U(11.W), io.cp.minQuotaWen)
    private val quotaStep = RegEnable(io.cp.updateData, (totalBW/10).U(11.W), io.cp.quotaStepWen)

    io.cp.maxQuota := maxQuota
    io.cp.minQuota := minQuota
    io.cp.quotaStep := quotaStep

    private val nLevels = 9
    private val lowPriorFreqLimits = RegInit(Vec(Seq.fill(nLevels){ 8.U(16.W) })) // bigger is more strict
    private val limitIndex = RegEnable(io.cp.updateData, 0.U(4.W), io.cp.limitIndexWen)
    private val lowerThreshold = RegEnable(io.cp.updateData, 64.U(8.W), io.cp.lowThresholdWen)
    private val higherThreshold = RegEnable(io.cp.updateData, 112.U(8.W), io.cp.highThresholdWen)

    io.cp.lowThreshold := lowerThreshold
    io.cp.highThreshold := higherThreshold
    io.cp.limitIndex := limitIndex
    io.cp.limit := lowPriorFreqLimits(limitIndex)

    when (io.cp.limitWen) {
      lowPriorFreqLimits(limitIndex) := io.cp.updateData
    }

    if (policy == "solo") {
      ///////////////////////////////////////////
      // c0 Solo:
      for (i <- startIndex until nTiles) {
        bucketParams(i).inc := 0.U
      }
      bucketParams(highPriorIndex).freq := 0.U

    } else if (policy == "yzh") {
      //////////////////////////////////////////////
      // Yu Zihao:
      val startTraffic = RegInit(0.asUInt(32.W))
      val windowCounter = RegInit(0.asUInt(16.W))

      val windowSize = 1000


      // def freqTable() = {
      //   val allocatedBW = (1 until 10).map{totalBW.toDouble/10*_}
      //   val freqs = allocatedBW.map(bw => round(windowSize.toDouble/bw))
      //   println(freqs)
      //   VecInit(freqs.map(_.asUInt(10.W)))
      // }
      // def limitFreqTable() = {
      //   // 这里的3是假设4个核心，剩下3个核心共享余下带宽
      //   val allocatedBW = (1 until 10).map{totalBW - totalBW.toDouble/10*_}
      //   val freqs = allocatedBW.map(bw => round(windowSize.toDouble/(bw/limitScaleIndex)))
      //   println(freqs)
      //   VecInit(freqs.map(_.asUInt(10.W)))
      // }

      when (windowCounter >= windowSize.U) {
        windowCounter := 1.U

        val bandwidthUsage = ((bucketState(highPriorIndex).traffic - startTraffic) << 3).asUInt()
        startTraffic := bucketState(highPriorIndex).traffic

        val nextLevel = Wire(UInt(4.W))
        val nextQuota = Wire(UInt(8.W))

        nextLevel := curLevel
        nextQuota := quota

        when (bandwidthUsage >= ((quota*higherThreshold) >> 7).asUInt ) {
          nextLevel := Mux(curLevel === 8.U, 8.U, curLevel + 1.U)
          nextQuota := Mux(quota >= maxQuota, maxQuota, quota + quotaStep)

        } .elsewhen (bandwidthUsage < ((quota*lowerThreshold) >> 7).asUInt ) {
          nextLevel := Mux(curLevel === 0.U, 0.U, curLevel - 1.U)
          nextQuota := Mux(quota <= minQuota, minQuota, quota - quotaStep)
        }

        bucketParams(highPriorIndex).freq := 0.U
//        bucketParams(highPriorIndex).inc := 1.U
        for (i <- startIndex until nTiles) {
          bucketParams(i).freq := lowPriorFreqLimits(nextLevel)
          bucketParams(i).inc := 1.U
        }
        curLevel := nextLevel
        quota := nextQuota

      }
      when (windowCounter < windowSize.U) {
        windowCounter := windowCounter + 1.U
      }

    } else if (policy == "lzg") {
      ///////////////////////////////////////////////
      // Liu Zhigang:
      val regulationCycles = 16000
      val samplingCycles = 1000
      val startTraffic = RegInit(0.asUInt(32.W))

      val samplingCounter = RegInit(0.asUInt(32.W))
      val regulationCounter = RegInit(0.asUInt(32.W))

      val s_sample :: s_regulate :: Nil = Enum(UInt(), 2)
      val state = Reg(init = s_sample)

      // during s_sample state, high priority app runs alone
      // all others' memory requests are blocked
      when (state === s_sample) {
        samplingCounter := samplingCounter + 1.U
        for (i <- startIndex until nTiles) {
          bucketParams(i).freq := 3200.U
          bucketParams(i).inc := 1.U
        }
        when (samplingCounter >= samplingCycles.U) {
          // estimate high priority app's memory bandwidth demand
          // set low priority app's bucket accordingly
          val bandwidthUsage = bucketState(highPriorIndex).traffic - startTraffic
          startTraffic := bucketState(highPriorIndex).traffic
          // val estimatedBandwidth = startTraffic << 4
          // 经验数据，仿真时，一万个周期，两个核最多往下面推了1000个beat
          // val totalBandwidth = regulationCycles / 10
          // 假设统计出来1000个周期内高优先级用了x beat，则在接下来的16000个周期里
          // 低优先级最多可以传输的beat数为 1600 - 16x
          // 相应地，其freq应该为 16000 / (1600 - 16x)
          // 由于这个无法简单表示，所以，我们手动将其分段表示
          // assume NTiles = 4
          // val newFreq = (regulationCycles.U - estimatedBandwidth) >> 4

          // 不那么激进的参数，这个是按照公式算的freq
          // val newFreq = Mux(bandwidthUsage >= 90.U, 100.U,
          //   Mux(bandwidthUsage >= 80.U, 50.U,
          //     Mux(bandwidthUsage >= 70.U, 33.U,
          //       Mux(bandwidthUsage >= 60.U, 25.U,
          //         Mux(bandwidthUsage >= 50.U, 20.U, 10.U)))))

          // 激进的参数，把上面根据freq算的数据，手动扩大几倍，把低优先级限制得更加死
          val newFreq = Mux(bandwidthUsage >= 90.U, 400.U,
            Mux(bandwidthUsage >= 80.U, 300.U,
              Mux(bandwidthUsage >= 70.U, 200.U,
                Mux(bandwidthUsage >= 60.U, 100.U,
                  Mux(bandwidthUsage >= 50.U, 100.U, 10.U)))))


          for (i <- startIndex until nTiles) {
            bucketParams(i).freq := newFreq
            bucketParams(i).inc := 1.U
          }

          regulationCounter := 0.U
          state := s_regulate
        }
      }

      when (state === s_regulate) {
        regulationCounter := regulationCounter + 1.U
        when (regulationCounter >= regulationCycles.U) {
          // temporarily disable all others' memory requests
          // let high priority app runs solo
          samplingCounter := 0.U
          state := s_sample
        }
      }
    }

    //Boom debug

    val snapshotPC = RegInit(0.asUInt(instAddrWidth.W))
    val hasNewPCSnapshot = RegInit(false.B)
    val autoSnapshot = RegEnable(io.cp.updateData =/= 0.U, true.B, io.cp.autoPCSnapShotWen)

    when ((autoSnapshot || io.cp.readPC) && !hasNewPCSnapshot) {
      snapshotPC := io.pc(hartSel)
      hasNewPCSnapshot := true.B
    }

    when (io.cp.doneReadPC) {
      hasNewPCSnapshot := false.B
    }
    io.cp.autoPCSnapShotEn := autoSnapshot
    io.cp.PC := snapshotPC
  }
}

trait HasControlPlane extends HasRocketTiles {
  this: BaseSubsystem =>
  val controlPlane = LazyModule(new ControlPlane())
}

trait HasBoomControlPlane extends HasBoomTiles {
  this: BaseSubsystem =>
  val controlPlane = LazyModule(new ControlPlane())
}

trait HasControlPlaneModuleImpl extends HasRocketTilesModuleImp {
  val outer: HasControlPlane

  (outer.rocketTiles zip outer.tokenBuckets).zipWithIndex.foreach { case((tile, token), i) =>
    val cpio = outer.controlPlane.module.io
    tile.module.dsid := cpio.hartDsids(i.U)
    tile.module.memBase := cpio.memBases(i.U)
    tile.module.memMask := cpio.memMasks(i.U)
    token.module.bucketIO <> outer.controlPlane.module.bucketIO(i)
  }

  outer.debug.module.io.cp <> outer.controlPlane.module.io.cp
}

trait HasControlPlaneBoomModuleImpl extends HasBoomTilesModuleImp {
  val outer: HasBoomControlPlane

  (outer.boomTiles zip outer.tokenBuckets).zipWithIndex.foreach { case((tile, token), i) =>
    val cpio = outer.controlPlane.module.io
    tile.module.dsid := cpio.hartDsids(i.U)
    tile.module.memBase := cpio.memBases(i.U)
    tile.module.memMask := cpio.memMasks(i.U)
    cpio.pc(i.U) := tile.module.pc
    token.module.bucketIO <> outer.controlPlane.module.bucketIO(i)
  }

  outer.debug.module.io.cp <> outer.controlPlane.module.io.cp
}

trait BindL2WayMask extends HasRocketTiles {
  this: BaseSubsystem with HasControlPlane with CanHaveMasterAXI4MemPort =>
  val _cp = controlPlane
  val _l2 = l2cache
}

trait BindL2WayMaskModuleImp extends HasRocketTilesModuleImp {
  val outer: BindL2WayMask
  if (p(NL2CacheCapacity) != 0) {
    outer._l2.module.cp <> outer._cp.module.io.l2
  }
}
