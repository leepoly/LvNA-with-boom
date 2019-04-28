//**************************************************************************
// L2 Cache
//--------------------------------------------------------------------------
//
// Zhigang Liu
// 2018 Aug 15

package freechips.rocketchip.subsystem

import Chisel._
import chisel3.util.IrrevocableIO
import chisel3.core.{Input, Output}
import freechips.rocketchip.config.{Field, Parameters}
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.util._
import freechips.rocketchip.tilelink._
import lvna.{HasControlPlaneParameters, CPToL2CacheIO}

case class TLL2CacheParams(
  debug: Boolean = true
)

trait L2CacheParams1 {
  val blockSize: Int = 64 * 8
  val blockBytes: Int = blockSize / 8

}

class MetadataEntry(tagBits: Int, dsidWidth: Int) extends Bundle {
  val valid = Bool()
  val dirty = Bool()
  val tag = UInt(width = tagBits.W)
  val rr_state = Bool()
  val dsid = UInt(width = dsidWidth.W)
  override def cloneType = new MetadataEntry(tagBits, dsidWidth).asInstanceOf[this.type]
}

class L2CacheDecodeInfo(nWays: Int, addrWidth: Int) extends Bundle with L2CacheParams1 {
  val hit_way_reg = UInt(log2Ceil(nWays).W)
  val repl_way_reg = UInt(log2Ceil(nWays).W)
  val read_hit_reg = Bool()
  val write_hit_reg = Bool()
  val read_miss = Bool()
  val write_miss = Bool()
  val read_replay_writeback = Bool()
  val write_replay_writeback = Bool()
  val read_replay_no_writeback = Bool()
  val write_replay_no_writeback = Bool()
  val writeback_addr_reg = UInt(addrWidth.W)
  override def cloneType = new L2CacheDecodeInfo(nWays, addrWidth).asInstanceOf[this.type]
}

class L2CacheReq(params: TLBundleParameters, dsidWidth: Int) extends Bundle with L2CacheParams1 {
  val is_replay = Bool()
  val opcode = UInt(3.W)
  val dsid = UInt(width = dsidWidth.W)
  val param = UInt(1.W)
  val size = UInt(width = log2Up(log2Ceil(params.dataBits / 8) + log2Ceil(blockSize / params.dataBits)).W)
  val source = UInt(width = params.sourceBits.W)
  val address = UInt(width = params.addressBits.W)
  val mask = Vec(blockSize / params.dataBits, UInt((params.dataBits / 8).W))
  val data = Vec(blockSize / params.dataBits, UInt(params.dataBits.W))
  val in_len = UInt(4.W)
  val debug_timer = UInt(10.W)
  override def cloneType = new L2CacheReq(params, dsidWidth).asInstanceOf[this.type]
}

class L2CacheStageInfo(params: TLBundleParameters, dsidWidth: Int) extends Bundle with L2CacheParams1 {
  val req = Output(new L2CacheReq(params, dsidWidth))
  val valid = Output(Bool())
  val ready = Input(Bool())
  override def cloneType = new L2CacheStageInfo(params, dsidWidth).asInstanceOf[this.type]
}

final class BitsSnoop[+T <: Data](gen: T) extends Bundle
{
  val bits = gen.asOutput
  override def cloneType: this.type = new BitsSnoop(gen).asInstanceOf[this.type]
}


class L2MSHREntry(params: TLBundleParameters, dsidWidth: Int) extends Bundle with L2CacheParams1{
  val req = new L2CacheReq(params, dsidWidth)
  val valid = Bool()
  override def cloneType = new L2MSHREntry(params, dsidWidth).asInstanceOf[this.type]
}

class L2MSHR(params: TLBundleParameters, dsidWidth: Int) extends Module with L2CacheParams1 {
  val io = new Bundle {
    val add_entry = Input(new L2CacheReq(params, dsidWidth))
    val add_entry_valid = Input(Bool())
    val add_entry_ready = Output(Bool())

    val rf_addr = Output(UInt(width = params.addressBits.W))
    val rf_addr_valid = Output(Bool())
    val rf_ready = Output(Bool()) //indicate MSHR is idle to receive refill_data
    val rf_data = Input(Vec(blockSize / params.dataBits, UInt(params.dataBits.W)))
    val rf_data_valid = Input(Bool())

    val replay_valid = Output(Bool())
    val replay_req = Output(new L2CacheReq(params, dsidWidth))
    val replay_ready = Input(Bool())

    // val rst = Input(Bool()) //clear mshr entry valid bit
  }
  val outerBeatSize = params.dataBits
  val outerDataBeats = blockSize / outerBeatSize
  
  val s_wait_ram_arready :: s_replay_wait_cache :: Nil = Enum(UInt(), 2)
  val state = Reg(init = s_wait_ram_arready)
  val mshr_debug = true
  val nL2MSHRs = 2

  val mshr_file = Module(new Queue(new L2MSHREntry(params, dsidWidth), nL2MSHRs))
  //val mshr_file = Reg(Vec(nL2MSHRs, new L2MSHREntry(params, dsidWidth)))
  // when (rst) {
  //   for (i <- 0 until nL2MSHRs) {
  //     val mshr_entry_i = mshr_file(i)
  //     mshr_entry_i.valid := false.B
  //   }
  // }

  //val free_mshr_entry_orR = mshr_file.io.count < nL2MSHRs.U
  val mshr_file_enq = Wire(DecoupledIO(new L2MSHREntry(params, dsidWidth)).flip)
  io.add_entry_ready := state === s_wait_ram_arready && mshr_file_enq.ready
  mshr_file_enq.bits.req := io.add_entry
  mshr_file_enq.bits.valid := true.B // valid bit actually no use, for debugging
  mshr_file_enq.valid := io.add_entry_valid && state === s_wait_ram_arready

  mshr_file.io.enq.bits := mshr_file_enq.bits
  mshr_file.io.enq.valid := mshr_file_enq.valid
  mshr_file_enq.ready := mshr_file.io.enq.ready
  when (mshr_file.io.enq.valid && mshr_file.io.enq.ready) {
    printf("[mshr-add-entry]: cycle: %d addr %x\n", GTimer(), mshr_file_enq.bits.req.address)
  }

  //val valid_mshr_entry_orR = mshr_file.io.count > 0.U
  val mshr_file_deq = Wire(DecoupledIO(new L2MSHREntry(params, dsidWidth)))
  
  val replay_req = Reg(new L2CacheReq(params, dsidWidth))

  //mshr_file_deq <> mshr_file.io.deq
  mshr_file_deq.bits := mshr_file.io.deq.bits
  mshr_file_deq.valid := mshr_file.io.deq.valid
  mshr_file.io.deq.ready := mshr_file_deq.ready

  io.rf_addr_valid := mshr_file_deq.valid && state === s_wait_ram_arready
  io.rf_addr := mshr_file_deq.bits.req.address
  mshr_file_deq.ready := io.rf_data_valid && state === s_wait_ram_arready //entry exists until refill comes
  io.rf_ready := state === s_wait_ram_arready
  when (state === s_wait_ram_arready) {
    when (io.rf_data_valid) {

      def mergePutData(old_data: UInt, new_data: UInt, wmask: UInt): UInt = {
        val full_wmask = FillInterleaved(8, wmask)
        ((~full_wmask & old_data) | (full_wmask & new_data))
      }
      val merged_data = Wire(Vec(blockSize / params.dataBits, UInt(params.dataBits.W)))
      for (merge_idx <- 0 until outerDataBeats) {
        val idx = merge_idx.U
        when (idx <= mshr_file_deq.bits.req.in_len) {
            //io.rf_data: old data(from hit tag's data)
            //mshr_file_deq.bits.req.data: new data(from cache_s1.wdata)
            merged_data(idx) := mergePutData(io.rf_data(idx), mshr_file_deq.bits.req.data(idx), mshr_file_deq.bits.req.mask(idx))
          }
      }

      when (!(mshr_file_deq.bits.req.opcode === TLMessages.Get)) {
        for (merge_idx <- 0 until outerDataBeats) {
          val idx = merge_idx.U
          printf("[mshr_merge] put_data_buf %x / %x, data_buf %x, addr %x, currbeat%x, lastbeat%x\n",
            mshr_file_deq.bits.req.data(idx), mshr_file_deq.bits.req.mask(idx), io.rf_data(idx), mshr_file_deq.bits.req.address, idx, mshr_file_deq.bits.req.in_len)
        }
      }

      state := s_replay_wait_cache
      replay_req := mshr_file_deq.bits.req
      replay_req.is_replay := true.B
      replay_req.data := Mux(mshr_file_deq.bits.req.opcode === TLMessages.Get, io.rf_data,
                              merged_data)
    }
  }

  io.replay_valid := state === s_replay_wait_cache
  io.replay_req := replay_req
  when (state === s_replay_wait_cache) {
    printf("[mshr] cycle: %d waiting replay addr %x data %x opcode %x replay %x\n", GTimer(),
      replay_req.address, replay_req.data.asUInt, replay_req.opcode, replay_req.is_replay)
    when (io.replay_ready) {
      state := s_wait_ram_arready
    }
  }

  if (mshr_debug) {
    when (GTimer() % 10000.U === 0.U) {
      printf("[mshr-report] cycle: %d m_state %x cnt %x e1_addr%x e2_addr%x \n", GTimer(), state, mshr_file.io.count, 
        mshr_file.io.enq.bits.req.address, mshr_file.io.deq.bits.req.address)
    }
  }

}

class TLCacheConvertorIn(params: TLBundleParameters, dsidWidth: Int, L2param: TLL2CacheParams) extends Module with L2CacheParams1{
  val io = new Bundle {
    // in.a: L1 ----> Convertor ----> L2
    // in.d: L1 <---- Convertor <---- L2
    val tl_in_a = new BitsSnoop(new TLBundleA(params)).flip
    val tl_in_d = new BitsSnoop(new TLBundleD(params))
    val tl_in_a_valid = Input(Bool())
    val tl_in_a_ready = Output(Bool())
    val tl_in_d_valid = Output(Bool())
    val tl_in_d_ready = Input(Bool())
    val cache_s0 = new L2CacheStageInfo(params, dsidWidth)
    val cache_rst = Bool(INPUT)
    val cache_s3 = new L2CacheStageInfo(params, dsidWidth).flip
  }
  val s0_idle :: s0_gather_write_data :: s0_send_bresp :: s0_wait_cache_ready :: Nil = Enum(UInt(), 4)
  val s0_state = Reg(init = s0_idle)
  val convertor_debug = L2param.debug

  //cache only sees 64B per cycle
  val innerBeatSize = io.tl_in_d.bits.params.dataBits
  val innerBeatBytes = innerBeatSize / 8
  val innerDataBeats = blockSize / innerBeatSize
  val innerBeatBits = log2Ceil(innerBeatBytes)
  val innerBeatIndexBits = log2Ceil(innerDataBeats)
  val innerBeatLSB = innerBeatBits
  val innerBeatMSB = innerBeatLSB + innerBeatIndexBits - 1

  val addrWidth = io.tl_in_a.bits.params.addressBits
  val innerIdWidth = io.tl_in_a.bits.params.sourceBits

  val in_opcode = io.tl_in_a.bits.opcode
  val in_dsid = io.tl_in_a.bits.dsid
  val s0_in_addr = io.tl_in_a.bits.address
  val in_id   = io.tl_in_a.bits.source

  val in_len_shift = io.tl_in_a.bits.size >= innerBeatBits.U
  val s0_in_len  = Mux(in_len_shift, ((1.U << io.tl_in_a.bits.size) >> innerBeatBits) - 1.U, 0.U)  // #word, i.e., arlen in AXI
  val in_data = io.tl_in_a.bits.data
  val in_data_mask = io.tl_in_a.bits.mask

  val in_recv_fire = io.tl_in_a_valid && io.tl_in_a_ready
  val in_read_req = in_recv_fire && (in_opcode === TLMessages.Get)
  val in_write_req = in_recv_fire && (in_opcode === TLMessages.PutFullData || in_opcode === TLMessages.PutPartialData)
  //val in_recv_handshake = io.tl_in_a_ready

  val s0_start_beat = s0_in_addr(innerBeatMSB, innerBeatLSB)
  val s0_inner_end_beat_reg = Reg(UInt(4.W))
  val s0_inner_end_beat = Mux(s0_state === s0_idle, s0_start_beat + s0_in_len, s0_inner_end_beat_reg)
  val gather_curr_beat_reg = RegInit(0.asUInt(log2Ceil(innerDataBeats).W))
  val gather_curr_beat = Mux(s0_state === s0_idle, s0_start_beat, gather_curr_beat_reg)
  val gather_last_beat = gather_curr_beat === s0_inner_end_beat

  val s0_info = Reg(new L2CacheReq(params, dsidWidth))
  val s0_valid = Wire(init = Bool(false))

  val s3_idle :: s3_data_resp :: Nil = Enum(UInt(), 2)
  val s3_state = Reg(init = s3_idle)

  io.cache_s0.req := s0_info
  io.cache_s0.valid := s0_valid

  when (s0_state === s0_idle && s3_state === s3_idle) {
    when (in_read_req) {
      s0_info.is_replay := false.B
      s0_info.address := s0_in_addr
      s0_info.source := in_id
      s0_info.opcode := in_opcode
      s0_info.dsid := in_dsid
      s0_info.size := io.tl_in_a.bits.size
      s0_inner_end_beat_reg := s0_start_beat + s0_in_len
      s0_info.in_len := s0_in_len
      s0_info.debug_timer := GTimer() % 512.U

      s0_state := s0_wait_cache_ready
    } .elsewhen (in_write_req) {
      s0_info.is_replay := false.B
      s0_info.address := s0_in_addr
      s0_info.source := in_id
      s0_info.opcode := in_opcode
      s0_info.dsid := in_dsid
      s0_info.size := io.tl_in_a.bits.size
      s0_inner_end_beat_reg := s0_start_beat + s0_in_len
      s0_info.in_len := s0_in_len
      s0_info.debug_timer := GTimer() % 512.U

      s0_state := s0_gather_write_data
    }
  }

  if (convertor_debug) {
    when (in_recv_fire) {
      printf("[in.a] cycle: %d a_addr %x a_opcode%x s0_state%x cache_s0.valid%x cache_s0.ready%x a_ready%x a_valid%x a_data %x, in_len %x\n", 
        GTimer(), io.tl_in_a.bits.address,
        io.tl_in_a.bits.opcode, s0_state, io.cache_s0.valid, io.cache_s0.ready,
        io.tl_in_a_ready, io.tl_in_a_valid, 
        io.tl_in_a.bits.data,
        s0_in_len
      )
    }
  }

  // *** gather_wdata ***
  // s_gather_write_data:
  // gather write data
  when ((in_write_req || (s0_state === s0_gather_write_data && in_recv_fire)) && (s3_state === s3_idle)) {
    when (s0_state === s0_idle) {
      gather_curr_beat_reg := s0_start_beat + 1.U
    } .elsewhen (s0_state === s0_gather_write_data) {
      gather_curr_beat_reg := gather_curr_beat_reg + 1.U
    } .otherwise {
      assert(Bool(false), "state error")
    }

    s0_info.data(gather_curr_beat) := in_data(params.dataBits - 1, 0)
    s0_info.mask(gather_curr_beat) := in_data_mask(params.dataBits / 8 - 1, 0)
    when (gather_last_beat) {
      s0_state := s0_send_bresp
    }
  }

  // s_send_bresp:
  // send bresp, end write transaction
  val in_write_ok = s0_state === s0_send_bresp && s3_state === s3_idle
  val in_send_ok = io.tl_in_d_valid && io.tl_in_d_ready

  when (s0_state === s0_send_bresp && in_send_ok && s3_state === s3_idle) {
    s0_state := s0_wait_cache_ready
  }
  //TODO: input queue could be added here, for non-blocking

  when (s0_state === s0_wait_cache_ready && s3_state === s3_idle) { //won't give req to cache until resp finished
    s0_valid := Bool(true)
    when (io.cache_s0.ready) {
      s0_state := s0_idle
    }
  }

  // *** data resp ***

  val s3_info = Reg(new L2CacheReq(params, dsidWidth))
  //val s3_in_addr = s3_info.address
  val s3_start_beat = io.cache_s3.req.address(innerBeatMSB, innerBeatLSB)
  val s3_inner_end_beat_reg = Reg(UInt(4.W))
  val s3_inner_end_beat = Mux(s3_state === s3_idle, s3_start_beat + io.cache_s3.req.in_len, s3_inner_end_beat_reg)
  io.cache_s3.ready := s3_state === s3_idle
  //val data_resp_buf = Reg(Vec(blockSize / params.dataBits, UInt(params.dataBits.W)))
  val resp_curr_beat = RegInit(0.asUInt(log2Ceil(innerDataBeats).W))
  val resp_last_beat = resp_curr_beat === s3_inner_end_beat
  val data_resp = Wire(UInt(params.dataBits.W))
  data_resp := s3_info.data(resp_curr_beat)

  val in_read_ok = s3_state === s3_data_resp //data_resp is superior to accepting a req

  when (io.cache_s3.valid && s3_state === s3_idle) {
    s3_info := io.cache_s3.req
    //data_resp_buf := io.cache_s3.req.data
    s3_state := s3_data_resp
    resp_curr_beat := s3_start_beat
    s3_inner_end_beat_reg := s3_start_beat + io.cache_s3.req.in_len
  }
  when (s3_state === s3_data_resp && in_send_ok) {
    resp_curr_beat := resp_curr_beat + 1.U
    when (resp_last_beat) {
      s3_state := s3_idle
    }
  }
  io.tl_in_d_valid := in_write_ok || in_read_ok
  io.tl_in_d.bits.opcode  := Mux(in_read_ok, TLMessages.AccessAckData, TLMessages.AccessAck)
  io.tl_in_d.bits.param   := UInt(0)
  io.tl_in_d.bits.size    := Mux(in_read_ok, s3_info.size, s0_info.size)
  io.tl_in_d.bits.source  := Mux(in_read_ok, s3_info.source, s0_info.source)
  io.tl_in_d.bits.sink    := UInt(0)
  io.tl_in_d.bits.denied  := Bool(false)
  io.tl_in_d.bits.data    := data_resp
  io.tl_in_d.bits.corrupt := Bool(false)
  
  if (convertor_debug) {
    when ((s3_state =/= s3_idle)) {
      printf("[in.d] cycle: %d d_opcode%x s3_state%x, cache_s3.valid%x d_ready%x d_valid%x d_source%x resp_curr_beat%x d_data %x\n", 
        GTimer(),
        io.tl_in_d.bits.opcode, s3_state, io.cache_s3.valid, 
        io.tl_in_d_ready, io.tl_in_d_valid, 
        io.tl_in_d.bits.source,
        resp_curr_beat, data_resp
        )
    }
  }

  val raddr_recv_ready = s0_state === s0_idle && !io.cache_rst && s3_state === s3_idle
  val waddr_recv_ready = s0_state === s0_idle && !io.cache_rst && s3_state === s3_idle
  val wdata_recv_ready = s0_state === s0_gather_write_data && s3_state === s3_idle
  io.tl_in_a_ready := raddr_recv_ready || waddr_recv_ready || wdata_recv_ready
  val raddr_fire = raddr_recv_ready && in_recv_fire
  val waddr_fire = waddr_recv_ready && in_recv_fire
  val wdata_fire = wdata_recv_ready && in_recv_fire
}


class TLCacheConvertorOut(params: TLBundleParameters, dsidWidth: Int, L2param: TLL2CacheParams) extends Module with L2CacheParams1{
  val io = new Bundle {
    // out.a: L2 ----> ConvertorOut ----> Mem
    // out.d: L2 <---- ConvertorOut <---- Mem
    val tl_out_a = new BitsSnoop(new TLBundleA(params))
    val tl_out_d = new BitsSnoop(new TLBundleD(params)).flip
    val tl_out_a_valid = Output(Bool())
    val tl_out_a_ready = Input(Bool())
    val tl_out_d_valid = Input(Bool())
    val tl_out_d_ready = Output(Bool())
    //writeback
    val wb_addr = Input(UInt(width = params.addressBits.W))
    val wb_data = Input(Vec(blockSize / params.dataBits, UInt(params.dataBits.W)))
    val wb_valid = Input(Bool())
    val wb_ready = Output(Bool())
    //refill
    val rf_addr = Input(UInt(width = params.addressBits.W))
    val rf_data = Output(Vec(blockSize / params.dataBits, UInt(params.dataBits.W)))
    val rf_data_mask = Input(UInt((params.dataBits/8).W))
    val rf_addr_valid = Input(Bool())
    val rf_data_valid = Output(Bool())
    val rf_ready = Input(Bool())
    val dsid = Input(UInt(dsidWidth.W))
  }
  val outerBeatSize = io.tl_out_d.bits.params.dataBits
  val outerDataBeats = blockSize / outerBeatSize
  val outerBeatBytes = outerBeatSize / 8
  val outerBeatLen = log2Ceil(outerBeatBytes)
  val outerBurstLen = outerBeatLen + log2Ceil(outerDataBeats)

  val out_raddr_fire = io.tl_out_a_valid && io.tl_out_a_ready
  val out_waddr_fire = io.tl_out_a_valid && io.tl_out_a_ready
  val out_wdata_fire = io.tl_out_a_valid && io.tl_out_a_ready
  val out_wreq_fire = io.tl_out_d_valid && io.tl_out_d_ready
  val out_rdata_fire = io.tl_out_d_valid && io.tl_out_d_ready
  val out_rdata = io.tl_out_d.bits.data

  val wb_idle :: wb_wait_ram_awready :: wb_do_ram_write :: wb_wait_ram_bresp :: Nil = Enum(UInt(), 4)
  val rf_idle :: rf_wait_ram_ar :: rf_do_ram_read :: rf_wait_cache_ready :: Nil = Enum(UInt(), 4)
  val wb_state = Reg(init = wb_idle)
  val rf_state = Reg(init = rf_idle)
  val out_idle :: out_wb :: out_rf :: Nil = Enum(UInt(), 3)
  val debug_out_state = Reg(init = out_idle)
  val convertor_out_debug = L2param.debug

  //writeback
  val wb_addr_buf = Reg(UInt(width = params.addressBits.W))
  val wb_data_buf = Reg(Vec(blockSize / params.dataBits, UInt(params.dataBits.W)))

  when (GTimer() % 10000.U === 0.U) {
    printf("[out-report] cycle: %d, outstate %x, wb_state %x, wb_addr %x, wb_data %x rf_state %x, rf_addr%x %x\n",
        GTimer(), debug_out_state,
        wb_state,
        io.wb_addr,
        io.wb_data.asUInt,
        rf_state, io.rf_addr_valid, io.rf_addr
        )
  }

  when (wb_state === wb_idle && rf_state === rf_idle && io.wb_valid) {
    wb_state := wb_do_ram_write
    wb_addr_buf := io.wb_addr
    wb_data_buf := io.wb_data
    debug_out_state := out_wb
    if (convertor_out_debug) {
      printf("[out-wb-recv-data] cycle: %d, wb_state %x, wb_addr %x, wb_data %x \n",
        GTimer(),
        wb_state,
        io.wb_addr,
        io.wb_data.asUInt
        )
    }
  }

  val (wb_cnt, wb_done) = Counter(out_wdata_fire && wb_state === wb_do_ram_write, outerDataBeats)
  when (wb_state === wb_do_ram_write && wb_done) {
    wb_state := wb_wait_ram_bresp
  }
  when (wb_state === wb_wait_ram_bresp && out_wreq_fire) {
    wb_state := wb_idle
    debug_out_state := out_idle
    printf("[out-wb-finish] cycle: %d, outstate%x \n", GTimer(), debug_out_state)
  }
  val out_write_valid = wb_state === wb_do_ram_write && rf_state === rf_idle
  io.wb_ready := wb_state === wb_idle && rf_state === rf_idle

  //refill
  when (rf_state === rf_idle && wb_state === wb_idle && io.rf_addr_valid && !io.wb_valid) { //let writeback prior to refill
    rf_state := rf_wait_ram_ar
    debug_out_state := out_rf
    printf("[out-rf-start] cycle: %d \n", GTimer())
  }
  io.rf_data_valid := rf_state === rf_wait_cache_ready
  val rf_mem_data = Reg(Vec(blockSize / params.dataBits, UInt(params.dataBits.W)))
  when (rf_state === rf_wait_ram_ar && out_raddr_fire) {
    rf_state := rf_do_ram_read
  }
  val (rf_mem_cnt, rf_mem_done) = Counter(rf_state === rf_do_ram_read && out_rdata_fire, outerDataBeats)
  when (rf_state === rf_do_ram_read && out_rdata_fire) {
    rf_mem_data(rf_mem_cnt) := out_rdata
    when (rf_mem_done) {
      rf_state := rf_wait_cache_ready
    }
  }
  io.rf_data := rf_mem_data
  when (rf_state === rf_wait_cache_ready && io.rf_ready) {
    debug_out_state := out_idle
    rf_state := rf_idle
  }

  if (convertor_out_debug) {
    when (rf_state =/= rf_idle && rf_state =/= rf_do_ram_read || (out_rdata_fire)) {
      printf("cycle%d [out rf] rf_state%x, rf_addr_valid%x, rf_data_valid%x, a_valid%x, a_ready%x, d_valid%x, d_ready%x rf_mem_cnt%x, data %x \n",
        GTimer(),
        rf_state,
        io.rf_addr_valid, io.rf_data_valid,
        io.tl_out_a_valid, io.tl_out_a_ready, io.tl_out_d_valid, io.tl_out_d_ready,
        rf_mem_cnt, 
        io.tl_out_d.bits.data
      )
    }
  }

  //outer wire
  val out_read_valid = (rf_state === rf_wait_ram_ar) && (wb_state === wb_idle)
  val out_data = wb_data_buf(wb_cnt)
  val out_addr = Mux(out_read_valid, io.rf_addr, wb_addr_buf)
  val out_opcode = Mux(out_read_valid, TLMessages.Get, TLMessages.PutFullData)
  io.tl_out_a.bits.opcode  := out_opcode
  io.tl_out_a.bits.dsid    := io.dsid
  io.tl_out_a.bits.param   := UInt(0)
  io.tl_out_a.bits.size    := outerBurstLen.U
  io.tl_out_a.bits.source  := 0.asUInt(params.sourceBits.W)
  io.tl_out_a.bits.address := out_addr
  io.tl_out_a.bits.mask    := io.rf_data_mask
  io.tl_out_a.bits.data    := out_data
  io.tl_out_a.bits.corrupt := Bool(false)

  io.tl_out_a_valid := out_write_valid || out_read_valid
  // read data channel signals
  io.tl_out_d_ready := (rf_state === rf_do_ram_read) || (wb_state === wb_wait_ram_bresp)

}


// ============================== DCache ==============================
class TLSimpleL2Cache(bankid: Int, param: TLL2CacheParams)(implicit p: Parameters) extends LazyModule
with HasControlPlaneParameters
{
  val node = TLAdapterNode(
    //clientFn = { c => c.copy(clients = c.clients map { c2 => c2.copy(sourceId = IdRange(0, 1))} )}
  )

  lazy val module = new LazyModuleImp(this) {
    println(s"bankid=$bankid")
    val nWays = p(NL2CacheWays)
    println(s"nWays = $nWays")
    //val nBanks = p(NBanksPerMemChannel)
    val nSets = p(NL2CacheCapacity) * 1024 / 64 / nWays // / nBanks
    println(s"nSets = $nSets")
    val cp = IO(new CPToL2CacheIO().flip())
    (node.in zip node.out) foreach { case ((in, edgeIn), (out, edgeOut)) =>
      require(isPow2(nSets))
      require(isPow2(nWays))

      /* parameters */
      val Y = true.B
      val N = false.B

      val blockSize = 64 * 8
      val blockBytes = blockSize / 8
      val innerBeatSize = in.d.bits.params.dataBits
      val innerBeatBytes = innerBeatSize / 8
      val innerDataBeats = blockSize / innerBeatSize
      val outerBeatSize = out.d.bits.params.dataBits
      val outerBeatBytes = outerBeatSize / 8
      val outerDataBeats = blockSize / outerBeatSize
      //val split = innerBeatSize / outerBeatSize
      //val splitBits = log2Ceil(split)
      val addrWidth = in.a.bits.params.addressBits
      val innerIdWidth = in.a.bits.params.sourceBits
      val outerIdWidth = out.a.bits.params.sourceBits
      val outerBeatLen = log2Ceil(outerBeatBytes)
      val outerBurstLen = outerBeatLen + log2Ceil(outerDataBeats)

      val innerBeatIndexBits = log2Ceil(innerDataBeats)
      val innerBeatBits = log2Ceil(innerBeatBytes)
      val innerBeatLSB = innerBeatBits
      val innerBeatMSB = innerBeatLSB + innerBeatIndexBits - 1

      // to keep L1 miss & L2 hit penalty small, inner axi bus width should be as large as possible
      // on loongson and zedboard, outer axi bus width are usually 32bit
      // so we require that innerBeatSize to be multiples of outerBeatSize

      val indexBits = log2Ceil(nSets)
      val blockOffsetBits = log2Ceil(blockBytes)
      //val bankBits = log2Ceil(nBanks)
      val tagBits = addrWidth - indexBits - blockOffsetBits // - bankBits
      val offsetLSB = 0
      val offsetMSB = blockOffsetBits - 1
      //val bankLSB = offsetMSB + 1
      //val bankMSB = bankLSB + bankBits - 1
      val indexLSB = offsetMSB + 1 //bankMSB + 1 
      val indexMSB = indexLSB + indexBits - 1
      val tagLSB = indexMSB + 1
      val tagMSB = tagLSB + tagBits - 1
      println(s"tag$tagMSB-$tagLSB index$indexMSB-$indexLSB offset$offsetMSB-$offsetLSB")

      val rst_cnt = RegInit(0.asUInt(log2Up(2 * nSets + 1).W))
      val rst = (rst_cnt < UInt(2 * nSets)) && !reset.toBool
      when (rst) { rst_cnt := rst_cnt + 1.U }

      val s_idle :: s_gather_write_data :: s_send_bresp :: s_update_meta :: s_tag_read_req :: s_tag_read_resp :: s_tag_read :: s_merge_put_data :: s_data_read :: s_data_write :: s_wait_ram_awready :: s_do_ram_write :: s_wait_ram_bresp :: s_wait_ram_arready :: s_do_ram_read :: s_data_resp :: Nil = Enum(UInt(), 16)
      // stage 1: tag_read
      val s1_idle :: s1_tag_read_req :: s1_tag_read_resp :: s1_tag_read :: s1_wait :: Nil = Enum(UInt(), 5)
      // stage 2: data_read
      val s2_idle :: s2_data_read :: s2_stay :: s2_wait :: Nil = Enum(UInt(), 4)
      // stage 3: merge wdata, data_write, data resp, replay_read/write
      val s3_idle :: s3_wait_ram_arready :: s3_wait_ram_awready :: s3_merge_put_data :: s3_data_write :: s3_replay_write :: s3_data_resp :: s3_replay_resp :: Nil = Enum(UInt(), 8)
      val s1_state = Reg(init = s1_idle)
      val s2_state = Reg(init = s2_idle)
      val s3_state = Reg(init = s3_idle)

      def op_wen(op: UInt): Bool = op === TLMessages.PutFullData || op === TLMessages.PutPartialData
      def op_ren(op: UInt): Bool = op === TLMessages.Get
      def decode_way(decode: L2CacheDecodeInfo): UInt = 
        Mux(decode.read_hit_reg || decode.write_hit_reg, decode.hit_way_reg, decode.repl_way_reg)
      def decode_idx(addr: UInt): UInt = addr(indexMSB, indexLSB)

      // state transitions:
      // s_idle: idle state
      // capture requests
      val statistic_flag: Boolean = false

      val mshr = Module(new L2MSHR(edgeIn.bundle, dsidWidth))
      val TL2CacheInput = Module(new TLCacheConvertorIn(edgeIn.bundle, dsidWidth, param))
      val s1_in = Reg(new L2CacheReq(edgeIn.bundle, dsidWidth))
      val s1_decode = Wire(new L2CacheDecodeInfo(nWays, addrWidth))
      val cache_s2 = Reg(new L2CacheReq(edgeIn.bundle, dsidWidth))
      val s2_decode = Reg(new L2CacheDecodeInfo(nWays, addrWidth))
      val cache_s3 = Reg(new L2CacheReq(edgeIn.bundle, dsidWidth))
      val s3_decode = Reg(new L2CacheDecodeInfo(nWays, addrWidth))
      val s1_valid = TL2CacheInput.io.cache_s0.valid
      val s2_ready = s2_state === s2_idle
      val s3_ready = s3_state === s3_idle
      val s1_ready = !rst && s1_state === s_idle && 
                      (!mshr.io.replay_valid)
                      (!(s1_valid && !s2_ready && TL2CacheInput.io.cache_s0.req.address === cache_s2.address/* && op_wen(cache_s2.opcode) && op_ren(TL2CacheInput.io.cache_s0.req.opcode)*/)) &&
                      (!(s1_valid && !s3_ready && TL2CacheInput.io.cache_s0.req.address === cache_s3.address/* && op_wen(cache_s3.opcode) && op_ren(TL2CacheInput.io.cache_s0.req.opcode)*/))
      val s1_rawblocking = ((s1_valid && !s2_ready && TL2CacheInput.io.cache_s0.req.address === cache_s2.address/* && op_wen(cache_s2.opcode) && op_ren(TL2CacheInput.io.cache_s0.req.opcode)*/)) ||
                      ((s1_valid && !s3_ready && TL2CacheInput.io.cache_s0.req.address === cache_s3.address/* && op_wen(cache_s3.opcode) && op_ren(TL2CacheInput.io.cache_s0.req.opcode)*/))

      val s3_ren = cache_s3.opcode === TLMessages.Get

      val s1_ren = s1_in.opcode === TLMessages.Get
      val s1_wen = s1_in.opcode === TLMessages.PutFullData || s1_in.opcode === TLMessages.PutPartialData
      
      val s1_idx = s1_in.address(indexMSB, indexLSB)
      //val s1_bank = s1_in.address(bankMSB, bankLSB)

      TL2CacheInput.io.tl_in_a.bits := in.a.bits
      TL2CacheInput.io.tl_in_a_valid := in.a.valid
      in.a.ready := TL2CacheInput.io.tl_in_a_ready

      in.d.bits := TL2CacheInput.io.tl_in_d.bits
      in.d.valid := TL2CacheInput.io.tl_in_d_valid
      TL2CacheInput.io.tl_in_d_ready := in.d.ready
      
      TL2CacheInput.io.cache_rst := rst
      TL2CacheInput.io.cache_s0.ready := s1_ready

      val timer = GTimer()
      val log_prefix = "cycle: %d bankid: %d [L2Cache]s1 %d s2 %d s3 %d s1_addr %x s2_addr %x s3_addr %x "
      def log_raw(prefix: String, fmt: String, tail: String, args: Bits*) = {
        if (param.debug) {
          printf(prefix + fmt + tail, args:_*)
        }
      }
      /** Single log */
      def log(fmt: String, args: Bits*) = log_raw(log_prefix, fmt, "\n", timer +: bankid.U +: s1_state +: s2_state +: s3_state +: s1_in.address +: cache_s2.address +: cache_s3.address +: args:_*)
      /** Log with line continued */
      def log_part(fmt: String, args: Bits*) = log_raw(log_prefix, fmt, "", timer +: bankid.U +: s1_state +: s2_state +: s3_state +: s1_in.address +: cache_s2.address +: cache_s3.address +: args:_*)
      /** Log with nothing added */
      def log_plain(fmt: String, args: Bits*) = log_raw("", fmt, "", args:_*)

      // when (out.a.fire()) {
      //   log("out.a.opcode %x, dsid %x, param %x, size %x, source %x, address %x, mask %x, data %x s1_len%x",
      //     out.a.bits.opcode,
      //     out.a.bits.dsid,
      //     out.a.bits.param,
      //     out.a.bits.size,
      //     out.a.bits.source,
      //     out.a.bits.address,
      //     out.a.bits.mask,
      //     out.a.bits.data,
      //     s1_in.in_len)
      // }

      mshr.io.replay_ready := !rst && s1_state === s1_idle
      when (mshr.io.replay_ready && mshr.io.replay_valid) {
        s1_in := mshr.io.replay_req
        s1_state := s1_tag_read_req
      }
      .elsewhen (s1_valid && s1_ready) {
        s1_in := TL2CacheInput.io.cache_s0.req
        s1_state := s1_tag_read_req
      }

      when ((s1_ready && s1_valid)) {
        val wire_opcode = TL2CacheInput.io.cache_s0.req.opcode
        val wire_ren = wire_opcode === TLMessages.Get
        val wire_wen = wire_opcode === TLMessages.PutFullData || wire_opcode === TLMessages.PutPartialData
        log("[s0] opcode %x, dsid %x, address %x, mask %x, data %x, ren%x/wen%x s1_valid%x s1_ready%x replay%x",
          TL2CacheInput.io.cache_s0.req.opcode,
          TL2CacheInput.io.cache_s0.req.dsid,
          TL2CacheInput.io.cache_s0.req.address,
          TL2CacheInput.io.cache_s0.req.mask.asUInt,
          TL2CacheInput.io.cache_s0.req.data.asUInt,
          wire_ren, wire_wen,
          s1_valid,
          s1_ready,
          TL2CacheInput.io.cache_s0.req.is_replay
          )
      }

      when (mshr.io.replay_ready && mshr.io.replay_valid) {
        val wire_opcode = mshr.io.replay_req.opcode
        val wire_ren = wire_opcode === TLMessages.Get
        val wire_wen = wire_opcode === TLMessages.PutFullData || wire_opcode === TLMessages.PutPartialData
        log("[s0] opcode %x, dsid %x, address %x, mask %x, data %x, ren%x/wen%x s1_valid%x s1_ready%x replay%x",
          mshr.io.replay_req.opcode,
          mshr.io.replay_req.dsid,
          mshr.io.replay_req.address,
          mshr.io.replay_req.mask.asUInt,
          mshr.io.replay_req.data.asUInt,
          wire_ren, wire_wen,
          s1_valid,
          s1_ready,
          mshr.io.replay_req.is_replay
          )
      }

      // s_tag_read: inspecting meta data
      // to keep the sram access path short, sram addr and output are latched
      // now, tag access has three stages:
      // 1. read req  2. read response  3. check hit, miss

      // metadata array
      val meta_array = DescribedSRAM(
        name = "L2_meta_array",
        desc = "L2 cache metadata array",
        size = nSets,
        data = Vec(nWays, new MetadataEntry(tagBits, dsidWidth))
      )

      //replacing policy
      val meta_array_wen = rst || s1_state === s1_tag_read
      val read_tag_req = (s1_state === s1_tag_read_req)
      val meta_rdata = meta_array.read(s1_idx, read_tag_req && !meta_array_wen)

      def wayMap[T <: Data](f: Int => T) = Vec((0 until nWays).map(f))

      val vb_rdata = wayMap((w: Int) => meta_rdata(w).valid).asUInt
      val db_rdata = wayMap((w: Int) => meta_rdata(w).dirty).asUInt
      val tag_rdata = wayMap((w: Int) => meta_rdata(w).tag)
      val curr_state = wayMap((w: Int) => meta_rdata(w).rr_state).asUInt
      val set_dsids = wayMap((w: Int) => meta_rdata(w).dsid)

      val vb_rdata_reg = Reg(Bits(width = nWays.W))
      val db_rdata_reg = Reg(Bits(width = nWays.W))
      val tag_rdata_reg = Reg(Vec(nWays, UInt(width = tagBits.W)))
      val curr_state_reg = Reg(Bits(width = nWays))
      val set_dsids_reg = Reg(Vec(nWays, UInt(width = dsidWidth.W)))

      when (s1_state === s1_tag_read_req) {
        s1_state := s1_tag_read_resp
      }

      when (s1_state === s1_tag_read_resp) {
        s1_state := s1_tag_read
        vb_rdata_reg := vb_rdata
        db_rdata_reg := db_rdata
        tag_rdata_reg := tag_rdata
        curr_state_reg := curr_state
        set_dsids_reg := set_dsids
      }

      // check hit, miss, repl_way
      val s1_tag = s1_in.address(tagMSB, tagLSB)
      val s1_tag_eq_way = wayMap((w: Int) => tag_rdata_reg(w) === s1_tag)
      val s1_tag_match_way = wayMap((w: Int) => s1_tag_eq_way(w) && vb_rdata_reg(w)).asUInt
      val s1_hit = s1_tag_match_way.orR
      s1_decode.read_hit_reg := s1_hit && s1_ren
      s1_decode.write_hit_reg := s1_hit && s1_wen
      s1_decode.read_miss := !s1_hit && s1_ren && !s1_in.is_replay
      s1_decode.write_miss := !s1_hit && s1_wen && !s1_in.is_replay
      (0 until nWays).foreach(i => when (s1_tag_match_way(i)) { s1_decode.hit_way_reg  := Bits(i) })

      cp.dsid := s1_in.dsid
      val curr_mask = 0xffff.U
      s1_decode.repl_way_reg := Mux((curr_state_reg & curr_mask).orR, PriorityEncoder(curr_state_reg & curr_mask),
                Mux(curr_mask.orR, PriorityEncoder(curr_mask), UInt(0)))
      val repl_dsid = set_dsids_reg(s1_decode.repl_way_reg)
      val dsid_occupacy = RegInit(Vec(Seq.fill(1 << dsidWidth){ 0.U(log2Ceil(p(NL2CacheCapacity) * 1024 / blockBytes).W) }))
      val requester_occupacy = dsid_occupacy(s1_in.dsid)
      val victim_occupacy = dsid_occupacy(repl_dsid)
      
      cp.capacity := dsid_occupacy(cp.capacity_dsid)

      // valid and dirty
      val need_writeback = vb_rdata_reg(s1_decode.repl_way_reg) && db_rdata_reg(s1_decode.repl_way_reg)
      val writeback_tag = tag_rdata_reg(s1_decode.repl_way_reg)
      s1_decode.writeback_addr_reg := Cat(writeback_tag, Cat(s1_idx, 0.U(blockOffsetBits.W)))
      //s1_decode.writeback_addr_reg := Cat(writeback_tag, Cat(s1_idx, Cat(s1_bank, 0.U(blockOffsetBits.W))))

      val s1_replay_read = s1_ren && s1_in.is_replay
      val s1_replay_write = s1_wen && s1_in.is_replay
      s1_decode.read_replay_writeback := s1_replay_read && need_writeback
      s1_decode.read_replay_no_writeback := s1_replay_read && !need_writeback
      s1_decode.write_replay_writeback := s1_replay_write && need_writeback
      s1_decode.write_replay_no_writeback := s1_replay_write && !need_writeback

      when (s1_state === s1_tag_read || s1_state === s1_wait) {
        when (s1_state === s1_tag_read) {
          log("hit: %d wb: %d s1_idx: %d curr_state_reg: %x hit_way: %x repl_way: %x repl_addr %x waymask %x", s1_hit, need_writeback, s1_idx, curr_state_reg, s1_decode.hit_way_reg, s1_decode.repl_way_reg, s1_decode.writeback_addr_reg, cp.waymask)
          log("rhit%x whit%x rmiss%x wmiss%x rreplay%x wreplay%x rrp_wb%x wrp_wb%x", s1_decode.read_hit_reg, s1_decode.write_hit_reg, s1_decode.read_miss, s1_decode.write_miss,
              s1_decode.read_replay_no_writeback, s1_decode.write_replay_no_writeback, s1_decode.read_replay_writeback, s1_decode.write_replay_writeback)
          log("s1 tags: " + Seq.fill(tag_rdata_reg.size)("%x").mkString(" "), tag_rdata_reg:_*)
          log("s1 vb: %x db: %x", vb_rdata_reg, db_rdata_reg)
        }
        val raw_blocking3 = (!s3_ready) && (decode_way(s1_decode) === decode_way(s3_decode)) && (decode_idx(s1_in.address) === decode_idx(cache_s3.address))
        val raw_blocking2 = (!s2_ready) && (decode_way(s1_decode) === decode_way(s2_decode)) && (decode_idx(s1_in.address) === decode_idx(cache_s2.address))
        val raw_blocking = raw_blocking2 || raw_blocking3
        when (raw_blocking) {
          log("raw_blocking")
        }
        when ((s1_decode.read_hit_reg || s1_decode.write_hit_reg || s1_decode.read_replay_writeback || s1_decode.write_replay_writeback)) {
          s1_state := s1_wait
          when (s2_ready && !raw_blocking) {
            s1_state := s1_idle
            s2_state := s2_data_read

            s2_decode := s1_decode
            cache_s2 := s1_in
          }
        } .elsewhen ((s1_decode.read_replay_no_writeback || s1_decode.write_replay_no_writeback || s1_decode.read_miss || s1_decode.write_miss)) {
          s1_state := s1_wait
          when (s2_ready && !raw_blocking) {
            s1_state := s1_idle
            s2_state := s2_wait

            s2_decode := s1_decode
            cache_s2 := s1_in
          }
        } 
      }

      val rst_metadata = Wire(Vec(nWays, new MetadataEntry(tagBits, 16)))
      for (i <- 0 until nWays) {
        val metadata = rst_metadata(i)
        metadata.valid := false.B
        metadata.dirty := false.B
        metadata.tag := 0.U
        metadata.dsid := 0.U
        metadata.rr_state := false.B
      }

      // update metadata
      val update_way = Mux(s1_hit, s1_decode.hit_way_reg, s1_decode.repl_way_reg)
      val next_state = Wire(Bits())
      
      when (s1_state === s1_tag_read) {
        when (!s1_hit && !s1_in.is_replay) { //modify tag only when req hit or replay
          next_state := curr_state_reg
        } .otherwise {
          when (!(curr_state_reg & curr_mask).orR) {
            next_state := curr_state_reg | curr_mask
          } .otherwise {
            next_state := curr_state_reg.bitSet(update_way, Bool(false))
          }
        }
        log("[update metadata]dsid: %d set: %d hit: %d rw: %d update_way: %d curr_state: %x next_state: %x",
          s1_in.dsid, s1_idx, s1_hit, s1_ren, update_way, curr_state_reg, next_state)
      }

      val update_metadata = Wire(Vec(nWays, new MetadataEntry(tagBits, 16)))
      for (i <- 0 until nWays) {
        val metadata = update_metadata(i)
        val is_update_way = update_way === i.U
        when (is_update_way && (s1_hit || s1_in.is_replay)) {
          metadata.valid := true.B
          //read_hit -> db_rdata_reg(update_way); write_hit -> 1; replay_read(w/ or w/o wb) -> 0; replay_write -> 1
          metadata.dirty := Mux(s1_decode.read_hit_reg, db_rdata_reg(update_way),
            Mux(s1_replay_read, false.B, true.B))
          metadata.tag := s1_tag
          metadata.dsid := s1_in.dsid
        } .otherwise { //all miss req his its second chance to set metadata(replay), so we don't do it rightnow for atomicity
          metadata.valid := vb_rdata_reg(i)
          metadata.dirty := db_rdata_reg(i)
          metadata.tag := tag_rdata_reg(i)
          metadata.dsid := set_dsids_reg(i)
        }
        metadata.rr_state := next_state(i)
      }

      val meta_array_widx = Mux(rst, rst_cnt, s1_idx)
      val meta_array_wdata = Mux(rst, rst_metadata, update_metadata)

      when (meta_array_wen) {
        meta_array.write(meta_array_widx, meta_array_wdata)
        when (!s1_hit && !rst) {
          assert(update_way === s1_decode.repl_way_reg, "update must = repl way when decrease a dsid's occupacy")
          val victim_valid = vb_rdata_reg(s1_decode.repl_way_reg)
          dsid_occupacy.zipWithIndex foreach { case (dsid_occ, i) =>
              when (i.U === s1_in.dsid && (!victim_valid || i.U =/= repl_dsid)) {
                dsid_occ := requester_occupacy + 1.U
              }.elsewhen(i.U =/= s1_in.dsid && i.U === repl_dsid && victim_valid) {
                dsid_occ := victim_occupacy - 1.U
              }
          }
          // when (victim_valid) {
          //   log("victim dsid %d dec way %d old_value %d", repl_dsid, s1_decode.repl_way_reg, victim_occupacy)
          // }
          // log("dsid %d inc way %d old_value %d", s1_in.dsid, update_way, requester_occupacy)
        }
      }

      // ###############################################################
      // #                  data array read/write                      #
      // ###############################################################
      val s2_idx = cache_s2.address(indexMSB, indexLSB)
      val s3_idx = cache_s3.address(indexMSB, indexLSB)
      val data_read_way = Mux(s2_decode.read_hit_reg || s2_decode.write_hit_reg, s2_decode.hit_way_reg, s2_decode.repl_way_reg)
      // incase it overflows
      val data_read_cnt = RegInit(0.asUInt((log2Ceil(innerDataBeats) + 1).W))
      val data_read_valid = (s2_state === s2_data_read && data_read_cnt =/= innerDataBeats.U) && !(s3_state === s3_data_write || s3_state === s3_replay_write) //r/w at same time is not allowed
      val data_read_idx = s2_idx << log2Ceil(innerDataBeats) | data_read_cnt
      val dout = Wire(UInt(outerBeatSize.W))

      val data_write_way = Mux(s3_decode.write_hit_reg, s3_decode.hit_way_reg, s3_decode.repl_way_reg)
      val data_write_cnt = Wire(UInt())
      val data_write_valid = s3_state === s3_data_write || s3_state === s3_replay_write
      val data_write_idx = s3_idx << log2Ceil(innerDataBeats) | data_write_cnt
      val din = Wire(UInt(outerBeatSize.W))
      
      val s2_data_buf = Reg(Vec(outerDataBeats, UInt(outerBeatSize.W))) // store hit data from cache
      val s3_data_buf = Reg(Vec(outerDataBeats, UInt(outerBeatSize.W))) // pass s2_data_buf to stage 3

      val data_array = DescribedSRAM(
          name = "L2_data_array",
          desc = "L2 data array",
          size = nSets * innerDataBeats,
          data = Vec(nWays, UInt(width = outerBeatSize.W))
        ) 
      when (data_write_valid) {
        log("write data array: idx: %d way: %d data: %x", data_write_idx, data_write_way, din)
        data_array.write(data_write_idx, Vec.fill(nWays)(din), (0 until nWays).map(data_write_way === UInt(_)))
      }
      dout := data_array.read(data_read_idx, data_read_valid && !data_write_valid)(data_read_way)
      when (RegNext(data_read_valid, N)) {
        log("read data array: idx: %d way: %d data: %x cnt: %x",
          RegNext(data_read_idx), RegNext(data_read_way), dout, data_read_cnt)
        s2_data_buf(data_read_cnt - 1.U) := dout
      }
      when (data_read_valid) {
        data_read_cnt := data_read_cnt + 1.U
      }

      // s_data_read
      // read miss or need write back
      when (GTimer() % 10000.U === 0.U) {
        log("s2_read_cnt: %x", data_read_cnt)
      }
      when (s2_state === s2_data_read) {
        when (data_read_cnt === innerDataBeats.U) {
          data_read_cnt := 0.U
          s2_state := s2_wait
        }
      }
      when (s2_state === s2_wait) {
        when (s2_decode.read_hit_reg && s3_ready) {
          s3_state := s3_data_resp
          s2_state := s2_idle

          cache_s3 := cache_s2
          s3_decode := s2_decode
          s3_data_buf := s2_data_buf
          // val req_using_time = (512.U + GTimer() - cache_s2.debug_timer) % 512.U
          // if (statistic_flag) {
          //   printf("now: %d [statistic] addr %x cycle %d read rhit%x rmiss%x replaywb%x \n", GTimer(), cache_s2.address, req_using_time, s2_decode.read_hit_reg, s2_decode.read_miss_no_writeback, s2_decode.read_miss_writeback_reg)
          // }
        } .elsewhen ((s2_decode.write_hit_reg) && s3_ready) {
          s3_state := s3_merge_put_data
          s2_state := s2_idle

          cache_s3 := cache_s2
          s3_decode := s2_decode
          s3_data_buf := s2_data_buf
        } .elsewhen ((s2_decode.read_miss || s2_decode.write_miss) && s3_ready) {
          s3_state := s3_wait_ram_arready
          s2_state := s2_idle

          cache_s3 := cache_s2
          s3_decode := s2_decode
        } .elsewhen ((s2_decode.read_replay_no_writeback || s2_decode.write_replay_no_writeback) && s3_ready) {
          s3_state := s3_replay_write
          s2_state := s2_idle

          cache_s3 := cache_s2
          s3_decode := s2_decode
        } .elsewhen ((s2_decode.read_replay_writeback || s2_decode.write_replay_writeback) & s3_ready) {
          s3_state := s3_wait_ram_awready
          s2_state := s2_idle

          cache_s3 := cache_s2
          s3_decode := s2_decode
          s3_data_buf := s2_data_buf
        }
      }

      // s_merge_put_data: merge data_buf and put_data_buf, and store the final result in data_buf
      // the old data(either read from data array or from refill) resides in data_buf
      // the new data(gathered from inner write) resides in put_data_buf
      def mergePutData(old_data: UInt, new_data: UInt, wmask: UInt): UInt = {
        val full_wmask = FillInterleaved(8, wmask)
        ((~full_wmask & old_data) | (full_wmask & new_data))
      }
      when (s3_state === s3_merge_put_data) {
        for (merge_idx <- 0 until outerDataBeats) {
          val idx = merge_idx.U
          when (idx <= cache_s3.in_len) {
            //s3_data_buf: old data(from hit tag's data)
            //cache_s3.data: new data(from cache_s1.wdata)
            s3_data_buf(idx) := mergePutData(s3_data_buf(idx), cache_s3.data(idx), cache_s3.mask(idx))
            log("[merge] put_data_buf %x / %x, data_buf %x, addr %x, currbeat%x, lastbeat%x",
              cache_s3.data(idx), cache_s3.mask(idx), s3_data_buf(idx), cache_s3.address, idx , cache_s3.in_len)
          }
        }
        s3_state := s3_data_write
      }

      // s_data_write
      val (write_cnt, write_done) = Counter(s3_state === s3_data_write || s3_state === s3_replay_write, innerDataBeats)
      data_write_cnt := write_cnt
      din := Mux(cache_s3.is_replay, cache_s3.data(data_write_cnt), s3_data_buf(data_write_cnt))
      
      when ((s3_state === s3_data_write || s3_state === s3_replay_write) && write_done) {
        //val req_using_time = (512.U + GTimer() - cache_s3.debug_timer) % 512.U
        when (s3_ren) {
          s3_state := Mux(s3_state === s3_replay_write, s3_replay_resp, s3_data_resp)
          // if (statistic_flag) {
          //   printf("now: %d [statistic] addr %x cycle %d read rhit%x rmiss%x rmisswb%x \n", GTimer(), cache_s3.address, req_using_time, s3_decode.read_hit_reg, s3_decode.read_miss_no_writeback, s3_decode.read_miss_writeback_reg)
          // }
        } .otherwise {
          // if (statistic_flag) {
          //   printf("now: %d [statistic] addr %x cycle %d write whit%x wmiss%x wmisswb%x \n", GTimer(), cache_s3.address, req_using_time, s3_decode.write_hit_reg, s3_decode.write_miss_no_writeback, s3_decode.write_miss_writeback_reg)
          // }
          s3_state := s3_idle
        }
      }

      // #########################################################
      // #                  write back path                      #
      // #########################################################
      val TL2CacheOutput = Module(new TLCacheConvertorOut(edgeOut.bundle, dsidWidth, param))
      TL2CacheOutput.io.dsid := s1_in.dsid
      out.a.bits := TL2CacheOutput.io.tl_out_a.bits
      out.a.valid := TL2CacheOutput.io.tl_out_a_valid
      TL2CacheOutput.io.tl_out_a_ready := out.a.ready
      TL2CacheOutput.io.tl_out_d.bits := out.d.bits
      TL2CacheOutput.io.tl_out_d_valid := out.d.valid
      out.d.ready := TL2CacheOutput.io.tl_out_d_ready

      TL2CacheOutput.io.wb_addr := s3_decode.writeback_addr_reg
      TL2CacheOutput.io.wb_data := s3_data_buf
      TL2CacheOutput.io.wb_valid := s3_state === s3_wait_ram_awready
      when (TL2CacheOutput.io.wb_ready && s3_state === s3_wait_ram_awready) {
        when (cache_s3.is_replay) {
          s3_state := s3_replay_write
        } .otherwise {
          s3_state := s3_wait_ram_arready // also wb_valid -> false
        }
      }

      // #####################################################
      // #                  refill path                      #
      // #####################################################
      mshr.io.add_entry := cache_s3
      mshr.io.add_entry_valid := s3_state === s3_wait_ram_arready
      when (mshr.io.add_entry_ready && mshr.io.add_entry_valid) {
        //log("added to mshr file, cache processing finish")
        s3_state := s3_idle
      }

      val rf_mem_addr = Cat(mshr.io.rf_addr(tagMSB, indexLSB), 0.asUInt(blockOffsetBits.W))
      //val rf_mem_addr = Cat(mshr.io.rf_addr(tagMSB, bankLSB), 0.asUInt(blockOffsetBits.W))
      TL2CacheOutput.io.rf_addr := rf_mem_addr
      TL2CacheOutput.io.rf_addr_valid := mshr.io.rf_addr_valid
      TL2CacheOutput.io.rf_data_mask := edgeOut.mask(rf_mem_addr, outerBurstLen.U)
      TL2CacheOutput.io.rf_ready := mshr.io.rf_ready
      mshr.io.rf_data := TL2CacheOutput.io.rf_data
      mshr.io.rf_data_valid := TL2CacheOutput.io.rf_data_valid

      // ########################################################
      // #                  data resp path                      #
      // ########################################################
      TL2CacheInput.io.cache_s3.req := cache_s3
      TL2CacheInput.io.cache_s3.req.data := Mux(s3_state === s3_replay_resp, cache_s3.data, s3_data_buf)
      TL2CacheInput.io.cache_s3.valid := s3_state === s3_data_resp || s3_state === s3_replay_resp
      when (s3_state === s3_data_resp || s3_state === s3_replay_resp) {
        when (TL2CacheInput.io.cache_s3.ready) {
          s3_state := s3_idle
        }
      }

      // outer tilelink interface
      when (in.b.fire() || in.c.fire() || in.e.fire()) {
        assert(Bool(false), "Inner tilelink Unexpected handshake")
      }
      if (edgeOut.manager.anySupportAcquireB && edgeOut.client.anySupportProbe) {
        in.b <> out.b
        out.c <> in.c
        out.e <> in.e
      } else {
        in.b.valid := Bool(false)
        in.c.ready := Bool(true)
        in.e.ready := Bool(true)
        out.b.ready := Bool(true)
        out.c.valid := Bool(false)
        out.e.valid := Bool(false)
      }
    }
  }
}

object TLSimpleL2Cache
{
  def apply(bankid: Int)(implicit p: Parameters): TLNode =
  {
    if (p(NL2CacheCapacity) != 0) {
      val tlsimpleL2cache = LazyModule(new TLSimpleL2Cache(bankid, TLL2CacheParams()))
      tlsimpleL2cache.node
    }
    else {
      val tlsimpleL2cache = LazyModule(new TLBuffer(BufferParams.none))
      tlsimpleL2cache.node
    }
  }
}

object TLSimpleL2CacheRef
{
  def apply(bankid: Int)(implicit p: Parameters): TLSimpleL2Cache = {
    val tlsimpleL2cache = LazyModule(new TLSimpleL2Cache(bankid, TLL2CacheParams()))
    tlsimpleL2cache
  }
}

