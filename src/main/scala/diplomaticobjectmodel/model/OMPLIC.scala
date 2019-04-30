// See LICENSE.SiFive for license details.

package freechips.rocketchip.diplomaticobjectmodel.model

sealed trait OMPrivilegeMode extends OMEnum
case object OMMachineMode extends OMPrivilegeMode
case object OMSupervisorMode extends OMPrivilegeMode
case object OMUserMode extends OMPrivilegeMode

object OMModes {
  def getModes(useVM: Boolean): Seq[OMPrivilegeMode] = {
    useVM match {
      case false => Seq(OMMachineMode)
      case true => Seq(OMMachineMode, OMSupervisorMode)
    }
  }
}

case class OMInterruptTarget(
  hartId: Int,
  modes: Seq[OMPrivilegeMode],
  _types: Seq[String] = Seq("OMInterruptTarget", "OMCompoundType")
) extends OMCompoundType

case class OMPLIC(
  memoryRegions: Seq[OMMemoryRegion],
  interrupts: Seq[OMInterrupt],
  specifications: Seq[OMSpecification],
  latency: Int,
  nPriorities: Int,
  targets: Seq[OMInterruptTarget],
  _types: Seq[String] = Seq("OMPLIC", "OMDevice", "OMComponent", "OMCompoundType")
) extends OMDevice

object OMPLIC {
  def getMode(length: Int): Seq[OMPrivilegeMode] = {
    length match {
      case 1 => Seq(OMMachineMode)
      case 2 => Seq(OMMachineMode,OMSupervisorMode)
      case _ => throw new IllegalArgumentException
    }
  }
}