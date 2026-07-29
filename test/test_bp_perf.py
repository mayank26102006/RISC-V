# Directed test for the hardware breakpoint and performance counters.
# Verifies both features described in debug_regs.sv actually behave as
# documented, rather than trusting the comments/SVA alone.

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, Timer
from test import MINI_PROGRAM, load_program_via_pins, settle, CLK_PERIOD_NS

WOFF_BP_ADDR = 0x08 << 2
WOFF_BP_CONTROL = 0x09 << 2
WOFF_PERF_CYCLE = 0x0A << 2
WOFF_PERF_STALL = 0x0B << 2
WOFF_RETIRE_COUNT = 0x07 << 2
DEBUG_BASE = 0x90000000


async def dbg_read(dut, word_off):
    """Peek a debug register directly via hierarchy -- this test only needs
    to prove the register VALUES are right, not exercise the external bus,
    which test.py/test_crv.py already do plenty of."""
    return dut.user_project.u_soc.u_debug_regs


@cocotb.test()
async def test_breakpoint_halts_at_target(dut):
    cocotb.start_soon(Clock(dut.clk, CLK_PERIOD_NS, unit="ns").start())
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    for _ in range(5):
        await RisingEdge(dut.clk)
    await settle()
    dut.rst_n.value = 1
    for _ in range(2):
        await RisingEdge(dut.clk)
    await settle()

    dbg = dut.user_project.u_soc.u_debug_regs

    # Arm a breakpoint at PC=0x04 (the second ADDI in MINI_PROGRAM -- a
    # plain straight-line instruction, not a branch, so "next PC" is
    # unambiguously bp_addr+4). BEQ at 0x18 would also work but is a TAKEN
    # branch, so its "next instruction" is the branch target (0x2c), not
    # +4 -- picking a non-branch instruction here keeps this test's own
    # expected value unambiguous.
    dbg.bp_addr_q.value = 0x04
    dbg.bp_enable_q.value = 1

    await load_program_via_pins(dut, MINI_PROGRAM)

    # Let it run; the core should halt (breakpoint, not trap) once PC=0x18
    # retires -- "break-after" semantics per debug_regs.sv's own comment,
    # so it should come to rest at 0x1c (bp_addr+4), not 0x18 itself.
    halted = False
    for _ in range(60):
        await RisingEdge(dut.clk)
        await settle()
        if int(dut.uo_out.value) & 0x01:  # cpu_halted_o
            halted = True
            break
    assert halted, "core never halted -- breakpoint didn't fire"

    trapped = bool(int(dut.uo_out.value) & 0x02)
    assert not trapped, "core halted via a TRAP, not the breakpoint -- something else went wrong first"

    bp_hit = int(dbg.bp_hit_q.value)
    assert bp_hit == 1, f"bp_hit_q not set after breakpoint should have fired (bp_hit_q={bp_hit})"

    pc = int(dbg.pc_i.value)
    assert pc == 0x08, (
        f"expected core to rest at PC=0x08 (bp_addr+4=0x04+4, break-after "
        f"semantics per debug_regs.sv's documented behavior, and 0x04 is a "
        f"non-branch instruction so this is unambiguous), got PC=0x{pc:x}"
    )
    dut._log.info(f"breakpoint fired correctly: bp_hit_q=1, halted at PC=0x{pc:x} (bp_addr+4)")


@cocotb.test()
async def test_disabled_breakpoint_never_fires(dut):
    """A disabled breakpoint whose address happens to match must never halt
    the core on its own -- this is exactly the p_disabled_breakpoint_never_halts
    SVA property, checked here in simulation too rather than trusting the
    proof alone."""
    cocotb.start_soon(Clock(dut.clk, CLK_PERIOD_NS, unit="ns").start())
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    for _ in range(5):
        await RisingEdge(dut.clk)
    await settle()
    dut.rst_n.value = 1
    for _ in range(2):
        await RisingEdge(dut.clk)
    await settle()

    dbg = dut.user_project.u_soc.u_debug_regs
    dbg.bp_addr_q.value = 0x18
    dbg.bp_enable_q.value = 0  # disabled

    await load_program_via_pins(dut, MINI_PROGRAM)

    # Program should run all the way to its own EBREAK, not stop early at
    # the (disabled) breakpoint address.
    halted = False
    for _ in range(60):
        await RisingEdge(dut.clk)
        await settle()
        if int(dut.uo_out.value) & 0x01:
            halted = True
            break
    assert halted, "core never halted"
    pc = int(dbg.pc_i.value)
    bp_hit = int(dbg.bp_hit_q.value)
    assert bp_hit == 0, f"disabled breakpoint set bp_hit_q anyway (bp_hit_q={bp_hit})"
    assert pc != 0x1c, (
        f"core halted at PC=0x{pc:x}, which is the breakpoint's break-after "
        "landing spot -- looks like the disabled breakpoint fired anyway"
    )
    dut._log.info(f"disabled breakpoint correctly ignored, core ran to its own EBREAK at PC=0x{pc:x}")


@cocotb.test()
async def test_perf_counters_report_plausible_cpi(dut):
    """PERF_CYCLE_COUNT and PERF_RETIRE_COUNT together should describe a
    plausible non-pipelined CPI (>= 1.0, since this core can't retire faster
    than one instruction per cycle, and > 1 is expected since loads/stores
    spend extra cycles in MEM_WAIT)."""
    cocotb.start_soon(Clock(dut.clk, CLK_PERIOD_NS, unit="ns").start())
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    for _ in range(5):
        await RisingEdge(dut.clk)
    await settle()
    dut.rst_n.value = 1
    for _ in range(2):
        await RisingEdge(dut.clk)
    await settle()

    dbg = dut.user_project.u_soc.u_debug_regs

    await load_program_via_pins(dut, MINI_PROGRAM)

    # Measurement window starts HERE, after loading completes and the core
    # is released to run -- not before. The ~480-cycle bit-bang load holds
    # the core in external reset (force_cpu_reset_o) the whole time, which
    # is not a "stall" in the CPI sense; counting it would make every
    # program's CPI look dominated by load time rather than instruction
    # execution, which isn't what this counter is for.
    cycle_before = int(dbg.cycle_count_q.value)
    retire_before = int(dbg.retire_count_i.value)
    stall_before = int(dbg.stall_count_q.value)

    for _ in range(60):
        await RisingEdge(dut.clk)
        await settle()
        if int(dut.uo_out.value) & 0x01:
            break

    cycles = int(dbg.cycle_count_q.value) - cycle_before
    retired = int(dbg.retire_count_i.value) - retire_before
    stalls = int(dbg.stall_count_q.value) - stall_before

    assert cycles > 0, "cycle_count_q never incremented"
    assert retired > 0, "retire_count_i never incremented -- no instructions retired?"
    cpi = cycles / retired
    dut._log.info(f"cycles={cycles} retired={retired} stalls={stalls} CPI={cpi:.2f}")
    assert 1.0 <= cpi <= 10.0, f"CPI={cpi:.2f} is outside a plausible range for this core"
    assert stalls <= cycles, "stall_count_q exceeds total cycle_count_q, which is impossible"
