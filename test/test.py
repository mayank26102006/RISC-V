
# SPDX-FileCopyrightText: © 2026 TinyGPU-RV32
# SPDX-License-Identifier: Apache-2.0
#
# Loads the program through the real external loader pins (ui_in[3:4], see
# ext_loader.sv) rather than by hierarchical injection, so the same test runs
# identically against RTL and against the synthesized gate-level netlist.
#
# ---------------------------------------------------------------------------
# GL FAILURE ROOT CAUSE (fixed 2026-07-29)
#
# Diagnosed from the failing gl_test run's own waveform, not from guesswork.
# Every single event in that entire gate-level simulation landed at either a
# clock edge (offset 0) or EXACTLY 1 ns after it -- nothing in between. That
# is because with -DFUNCTIONAL the sky130 combinational cells are zero-delay,
# and only the flops carry -DUNIT_DELAY=#1. So uio_out[7] (ext_load_ready)
# transitions at precisely edge + 1 ns.
#
# The previous version did:
#
#       await RisingEdge(dut.clk)
#       await ReadOnly()
#       await Timer(1, unit="ns")     # <-- lands ON the transition
#       ready = ...uio_out[7]...
#
# The 1 ns settle was added to "give gate propagation room", but 1 ns is
# exactly the flop CLK->Q delay, so the read sampled the output at the very
# timestamp it changes. It lost that race, read a stale 0, and therefore
# waited one extra clock before driving its first data bit. The loader --
# which is correct -- had already begun sampling on schedule and consumed a
# spurious ui_in[4]=0.
#
# Consequence: the whole 480-bit stream arrived shifted right by one bit.
# Word 0 landed as 0x00280049 instead of 0x00500093; opcode 0x49 is not a
# valid RV32I opcode, so the CPU raised TRAP_ILLEGAL_INSTR three cycles after
# release and reported uo_out = 0x23 (halted=1, trap=1, trap_cause=1).
#
# Under RTL the flop updates at edge + 0, so the 1 ns read was safely after
# the transition. That is the entire reason this passed RTL and failed GL.
#
# What changed here:
#   1. Outputs are sampled MID-CYCLE (SETTLE_NS), never at edge + 1 ns.
#   2. Inputs are driven just after an edge, so they are stable for nearly a
#      full period before the edge that samples them.
#   3. ready is polled ONCE, then bits are clocked out one per edge.
#      ext_loader.sv holds ready high for the whole in-range transfer, so
#      re-polling per bit bought nothing and added 480 more chances to lose
#      the same race.
#   4. ready is still *checked* every bit, but as an assertion rather than as
#      flow control -- if it ever does drop mid-transfer the test says so
#      loudly instead of silently corrupting the bitstream.
# ---------------------------------------------------------------------------
#
# Program (15 words, fits the 16-word ASIC-target scratchpad):
#   x1 = 5; x2 = 7; x3 = x1 + x2 (expect 12); compare via BEQ (fallthrough
#   = fail, taken = pass, avoiding an extra unconditional jump); report
#   PASS/FAIL via the debug PASSFAIL register; EBREAK.
 
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, Timer
 
MINI_PROGRAM = [
    0x00500093,  # addi x1, x0, 5
    0x00700113,  # addi x2, x0, 7
    0x002081B3,  # add  x3, x1, x2      -> expect 12
    0x00C00413,  # addi x8, x0, 12
    0x90000737,  # lui  x14, DBG_PASSFAIL(hi)
    0x01470713,  # addi x14, x14, DBG_PASSFAIL(lo)   -> 0x90000014
    0x00818A63,  # beq  x3, x8, pass                 -> +20 -> 0x2c
    # fail (fallthrough):
    0xDEAD07B7,  # lui  x15, FAIL(1)(hi)
    0x00178793,  # addi x15, x15, FAIL(1)(lo)
    0x00F72023,  # sw   x15, 0(x14)
    0x00100073,  # ebreak
    # pass:
    0xCAFEC7B7,  # lui  x15, PASS_SIGNATURE(hi)
    0xABE78793,  # addi x15, x15, PASS_SIGNATURE(lo) -> 0xcafebabe
    0x00F72023,  # sw   x15, 0(x14)
    0x00100073,  # ebreak
]
 
# Pin bit assignments, from tt_um_tinygpu_rv32.sv:
UI_EXT_LOAD_MODE = 3
UI_EXT_LOAD_BIT = 4
UO_CPU_HALTED = 0
UO_CPU_TRAP = 1
UIO_EXT_LOAD_READY = 7
 
CLK_PERIOD_NS = 10_000
 
# Where in the cycle to sample DUT outputs and drive DUT inputs. Must be
# comfortably greater than the gate-level flop CLK->Q delay (1 ns under
# -DUNIT_DELAY=#1) and comfortably less than a full clock period. 10% of the
# period satisfies both by three orders of magnitude in each direction.
SETTLE_NS = CLK_PERIOD_NS // 10
 
TRAP_CAUSE_NAMES = {
    0: "TRAP_NONE",
    1: "TRAP_ILLEGAL_INSTR",
    2: "TRAP_INSTR_MISALIGNED",
    3: "TRAP_LOAD_MISALIGNED",
    4: "TRAP_STORE_MISALIGNED",
    5: "TRAP_BUS_ERROR",
    6: "TRAP_ECALL",
    7: "TRAP_EBREAK",
}
 
 
def read_bits(sig, hi, lo):
    """X-tolerant slice read. Returns None if any bit in the value is X or Z.
 
    Gate-level nets are legitimately X before reset propagates, so a bare
    int() would raise ValueError. Returning None lets callers distinguish
    "not known yet" from a real 0.
    """
    try:
        return (int(sig.value) >> lo) & ((1 << (hi - lo + 1)) - 1)
    except ValueError:
        return None
 
 
async def settle():
    """Advance to a stable mid-cycle observation point after a clock edge."""
    await Timer(SETTLE_NS, unit="ns")
 
 
async def load_program_via_pins(dut, words):
    """Load a program using ONLY the external ui_in/uio pins.
 
    This is the real ext_loader.sv protocol, so it behaves identically under
    RTL and gate-level simulation -- pins are the only interface that
    survives synthesis.
    """
    mode = 1 << UI_EXT_LOAD_MODE
 
    # Assert load mode just after an edge, so it is stable for nearly a full
    # period before the edge that samples it.
    await RisingEdge(dut.clk)
    await settle()
    dut.ui_in.value = mode
 
    # ext_load_ready_o is a REGISTERED output, so it lags mode by at least two
    # cycles by construction (IDLE -> ACTIVE, then ready_d -> ready_q). Poll
    # for it mid-cycle. Bounded so a regression reports a clear failure
    # instead of hanging.
    for _ in range(16):
        await RisingEdge(dut.clk)
        await settle()
        if read_bits(dut.uio_out, UIO_EXT_LOAD_READY, UIO_EXT_LOAD_READY) == 1:
            break
    else:
        raise AssertionError(
            f"ext_load_ready (uio_out[{UIO_EXT_LOAD_READY}]) never asserted "
            "after ext_load_mode was raised -- loader did not enter LOAD_ACTIVE"
        )
 
    # ready is high now and, per ext_loader.sv, stays high for the entire
    # in-range transfer. The loader therefore samples ext_load_bit_i on EVERY
    # rising edge from here on, so drive exactly one bit per edge. Each bit is
    # driven mid-cycle and sampled at the following edge.
    for w, word in enumerate(words):
        for i in range(31, -1, -1):
            dut.ui_in.value = mode | (((word >> i) & 1) << UI_EXT_LOAD_BIT)
            await RisingEdge(dut.clk)  # <-- loader samples the bit on this edge
            await settle()
 
            ready = read_bits(dut.uio_out, UIO_EXT_LOAD_READY, UIO_EXT_LOAD_READY)
            if ready != 1:
                raise AssertionError(
                    f"ext_load_ready dropped mid-transfer at word {w}, "
                    f"bit {31 - i} (read {ready}) -- the bitstream would be "
                    "corrupted from this point on"
                )
 
    # The final bit was sampled on the edge above, which is also the edge that
    # commits the last word to the scratchpad. Safe to release the CPU now.
    dut.ui_in.value = 0
    await settle()
 
 
@cocotb.test()
async def test_project(dut):
    dut._log.info("Start")
 
    cocotb.start_soon(Clock(dut.clk, CLK_PERIOD_NS, unit="ns").start())
 
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
 
    for _ in range(5):
        await RisingEdge(dut.clk)
    await settle()
    dut._log.info("Release reset")
    dut.rst_n.value = 1
    for _ in range(2):
        await RisingEdge(dut.clk)
    await settle()
 
    dut._log.info(f"Loading {len(MINI_PROGRAM)}-word program via external pins only")
    await load_program_via_pins(dut, MINI_PROGRAM)
    dut._log.info("Load complete, CPU released to run")
 
    # Wait for cpu_halted_o (uo_out[0]) with a bounded timeout, so a
    # regression reports a clear failure instead of hanging. Sampled
    # mid-cycle for the same reason as everything else above.
    timeout_cycles = 300
    halted = False
    for i in range(timeout_cycles):
        await RisingEdge(dut.clk)
        await settle()
        uo = read_bits(dut.uo_out, 7, 0)
        if uo is None:
            continue
        if uo & (1 << UO_CPU_HALTED):
            dut._log.info(f"cpu_halted asserted at cycle {i} (uo_out=0x{uo:02x})")
            halted = True
            break
 
    assert halted, (
        f"TIMEOUT: cpu_halted (uo_out[{UO_CPU_HALTED}]) never asserted "
        f"within {timeout_cycles} cycles"
    )
 
    uo = read_bits(dut.uo_out, 7, 0)
    trapped = bool(uo & (1 << UO_CPU_TRAP))
    trap_cause = (uo >> 5) & 0x7
 
    assert not trapped, (
        "CPU halted via an unexpected trap, not the program's own EBREAK "
        f"(uo_out=0x{uo:02x}, trap_cause={trap_cause} "
        f"{TRAP_CAUSE_NAMES.get(trap_cause, '?')}). A trap_cause of 1 here "
        "means the loaded program was corrupted -- check the loader handshake "
        "timing before suspecting the CPU."
    )
 
    # Best-effort extra verification: hierarchically peek the real PASS/FAIL
    # result via debug_regs' sticky bits. This ONLY works at the RTL level --
    # gracefully degrade under gate-level simulation, where module hierarchy
    # does not survive synthesis, and fall back to "halted cleanly without
    # trapping" as the success criterion.
    try:
        pass_seen = int(dut.user_project.u_soc.u_debug_regs.pass_seen_q.value)
        fail_seen = int(dut.user_project.u_soc.u_debug_regs.fail_seen_q.value)
        passfail_reg = int(dut.user_project.u_soc.u_debug_regs.passfail_q.value)
        dut._log.info(
            f"[RTL-only check] pass_seen={pass_seen} fail_seen={fail_seen} "
            f"passfail_reg=0x{passfail_reg:08x}"
        )
        assert pass_seen and not fail_seen, (
            f"Program did not report PASS (pass_seen={pass_seen} "
            f"fail_seen={fail_seen} passfail_reg=0x{passfail_reg:08x})"
        )
        dut._log.info("TinyGPU-RV32 loaded via pins and ran a real program, reporting PASS")
    except AttributeError:
        dut._log.info(
            "Hierarchical PASS/FAIL peek unavailable (expected under gate-level "
            "simulation -- module hierarchy doesn't survive synthesis). Success "
            "criterion for this run: CPU halted cleanly without trapping, "
            "entirely via external pins."
        )
        dut._log.info("TinyGPU-RV32 loaded via pins and halted cleanly (gate-level-compatible check)")
