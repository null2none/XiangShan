package xiangshan.backend.fu.NewCSR

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import xiangshan.{DebugOptionsKey, XSTester}
import xiangshan.backend.decode.isa.CSRs

/** Test-only wrapper around the real CSR block and one regional event copy. */
class HPMWriteHarness(implicit p: Parameters) extends Module {
  val request = IO(Flipped(Decoupled(new Bundle {
    val wen = Bool()
    val addr = UInt(12.W)
    val data = UInt(64.W)
  })))
  val illegal = IO(Output(Bool()))
  val readData = IO(Output(UInt(64.W)))
  val copies = IO(Output(Vec(29, UInt(64.W))))
  val notification = IO(new xiangshan.DistributedCSRIO)

  val csr = Module(new NewCSR)
  Seq[Data](csr.io.fromTop, csr.io.trapInst, csr.io.fromMem, csr.io.fromRob,
    csr.io.fromRename, csr.io.fromVecExcpMod, csr.io.perf, csr.io.oldPrivState,
    csr.platformIRP, csr.nonMaskableIRP, csr.fromAIA).foreach { input =>
    input := 0.U.asTypeOf(input)
  }
  csr.io.fetchMalTval := 0.U
  csr.io.oldSatpMode := 0.U
  csr.io.oldVsatpMode := 0.U
  csr.io.out.ready := true.B
  csr.io.in.valid := request.valid
  request.ready := csr.io.in.ready
  csr.io.in.bits := 0.U.asTypeOf(csr.io.in.bits)
  csr.io.in.bits.wen := request.bits.wen
  csr.io.in.bits.ren := !request.bits.wen
  csr.io.in.bits.addr := request.bits.addr
  csr.io.in.bits.wdata := request.bits.data
  illegal := csr.io.out.bits.EX_II || csr.io.out.bits.EX_VI
  readData := csr.io.out.bits.rData
  notification := csr.io.mhpmeventWrite

  // Match the wrapper's existing direct-write broadcast and indirect override.
  val copy = Module(new PFEvent)
  copy.io.distribute_csr.w.valid := csr.io.distributedWenLegal
  copy.io.distribute_csr.w.bits.addr := RegEnable(request.bits.addr, request.fire)
  copy.io.distribute_csr.w.bits.data := RegEnable(request.bits.data, request.fire)
  when(csr.io.mhpmeventWrite.w.valid) {
    copy.io.distribute_csr := csr.io.mhpmeventWrite
  }
  copies := copy.io.hpmevent
}

class HPMEventRegressionTest extends XSTester {
  it should "synchronize direct and indirect HPM configuration writes without changing MINH" in {
    val params = config.alterPartial {
      case DebugOptionsKey => config(DebugOptionsKey).copy(AlwaysBasicDiff = false, EnableDifftest = false)
    }
    simulate(new HPMWriteHarness()(params)) { dut =>
      dut.request.valid.poke(false.B)
      dut.request.bits.wen.poke(false.B)
      dut.request.bits.addr.poke(0.U)
      dut.request.bits.data.poke(0.U)
      def access(addr: Int, data: BigInt = 0, write: Boolean = true,
                 target: Option[Int] = None, effectiveData: BigInt = 0,
                 rejected: Boolean = false, expectedRead: Option[BigInt] = None): Unit = {
        dut.request.ready.expect(true.B)
        dut.request.bits.addr.poke(addr.U)
        dut.request.bits.data.poke(data.U)
        dut.request.bits.wen.poke(write.B)
        dut.request.valid.poke(true.B)
        dut.clock.step()
        dut.request.valid.poke(false.B)
        dut.illegal.expect(rejected.B)
        dut.notification.w.valid.expect(target.isDefined.B)
        target.foreach { index =>
          dut.notification.w.bits.addr.expect((CSRs.mhpmevent3 + index).U)
          dut.notification.w.bits.data.expect(effectiveData.U)
        }
        expectedRead.foreach(value => dut.readData.expect(value.U))
        dut.clock.step()
        dut.notification.w.valid.expect(false.B)
      }
      access(CSRs.menvcfg, BigInt(1) << 60) // Enable counter delegation.
      access(CSRs.mcounteren, (BigInt(1) << 3) | (BigInt(1) << 31))
      for ((index, minh) <- Seq((0, BigInt(1) << 62), (28, BigInt(0)))) {
        val addr = CSRs.mhpmevent3 + index
        access(addr, minh | 1)
        dut.copies(index).expect((minh | 1).U)
        access(CSRs.siselect, BigInt(0x43 + index))
        // Try both clearing and setting MINH through the indirect view.
        val data = ((BigInt(1) << 62) ^ minh) | 2
        access(CSRs.sireg2, data, target = Some(index), effectiveData = minh | 2)
        dut.copies(index).expect((minh | 2).U)
        access(addr, write = false, expectedRead = Some(minh | 2))
        access(CSRs.sireg2, write = false, expectedRead = Some(2))
      }
      // An undelegated target must neither broadcast nor alter its copy.
      access(CSRs.siselect, 0x44)
      access(CSRs.sireg2, 3, rejected = true)
      dut.copies(1).expect(0.U)
      // cyclecfg is an indirect configuration write, but is not an HPM event.
      access(CSRs.mcounteren, 1)
      access(CSRs.siselect, 0x40)
      access(CSRs.sireg2, 0)
      dut.copies(0).expect(((BigInt(1) << 62) | 2).U)
      dut.copies(28).expect(2.U)
    }
  }
}
