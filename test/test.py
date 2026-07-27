# SPDX-FileCopyrightText: © 2026 TinyGPU-RV32
# SPDX-License-Identifier: Apache-2.0
#
# Replaces the original placeholder test (which asserted uo_out == 50
# against a leftover example module never wired to this design). This
# test actually exercises TinyGPU-RV32: it preloads a small, independently
# verified RV32I program into the shared scratchpad, releases reset, waits
# for the CPU to halt, and checks the real PASS/FAIL result the running
# program reported through the debug module -- the same mechanism used by
# tb_tinygpu_soc.sv's full 6-test suite, just entered through cocotb here.
#
# Program (15 words, fits the 16-word ASIC-target scratchpad -- shrunk
# 2026-07-27 from an 18-word program when the real-chip memory default was
# reduced 32->16 words as part of a real-hardening-driven area revamp):
#   x1 = 5; x2 = 7; x3 = x1 + x2 (expect 12); compare via BEQ (fallthrough
#   = fail, taken = pass, avoiding an extra unconditional jump); report
#   PASS/FAIL via the debug PASSFAIL register; EBREAK.
# Independently verified with a from-scratch Python RV32I functional model
# before ever being used here -- see the project's test-generation notes.

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge, Timer

MINI_PROGRAM = [
    0x00500093,  # addi x1, x0, 5
    0x00700113,  # addi x2, x0, 7
    0x002081b3,  # add  x3, x1, x2      -> expect 12
    0x00c00413,  # addi x8, x0, 12
    0x90000737,  # lui  x14, DBG_PASSFAIL(hi)
    0x01470713,  # addi x14, x14, DBG_PASSFAIL(lo)
    0x00818a63,  # beq  x3, x8, pass
    # fail (fallthrough):
    0xdead07b7,  # lui  x15, FAIL(1)(hi)
    0x00178793,  # addi x15, x15, FAIL(1)(lo)
    0x00f72023,  # sw   x15, 0(x14)
    0x00100073,  # ebreak
    # pass:
    0xcafec7b7,  # lui  x15, PASS_SIGNATURE(hi)
    0xabe78793,  # addi x15, x15, PASS_SIGNATURE(lo)
    0x00f72023,  # sw   x15, 0(x14)
    0x00100073,  # ebreak
]

# uo_out bit assignments, from tt_um_tinygpu_rv32.sv:
UO_CPU_HALTED = 0
UO_CPU_TRAP = 1


async def preload_program(dut):
    """Hierarchically force the mini program into the shared scratchpad
    before releasing reset. Mirrors tb_tinygpu_soc.sv's load_program task.
    """
    mem = dut.user_project.u_soc.u_mem.mem_q
    for i in range(16):
        mem[i].value = 0
    for i, word in enumerate(MINI_PROGRAM):
        mem[i].value = word


@cocotb.test()
async def test_project(dut):
    dut._log.info("Start")

    clock = Clock(dut.clk, 10, unit="us")
    cocotb.start_soon(clock.start())

    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0

    # Preload while held in reset, matching tb_tinygpu_soc.sv's sequencing.
    await ClockCycles(dut.clk, 3)
    await preload_program(dut)
    await ClockCycles(dut.clk, 2)

    dut._log.info("Release reset")
    dut.rst_n.value = 1

    # Wait for cpu_halted_o (uo_out[0]) with a timeout, same safety-net
    # pattern used in tb_tinygpu_soc.sv -- report a clear failure instead
    # of letting the simulation hang if something regresses. Polled in
    # bulk (every CHECK_EVERY cycles) rather than every single cycle to
    # cut Python/VPI round-trip overhead.
    timeout_cycles = 300
    check_every = 20
    halted = False
    for i in range(0, timeout_cycles, check_every):
        await ClockCycles(dut.clk, check_every)
        val = dut.uo_out.value
        try:
            ival = int(val)
        except ValueError:
            continue
        dut._log.info(f"cycle ~{i}: uo_out=0x{ival:02x}")
        if ival & (1 << UO_CPU_HALTED):
            dut._log.info(f"cpu_halted asserted by cycle ~{i}")
            halted = True
            break

    assert halted, f"TIMEOUT: cpu_halted (uo_out[{UO_CPU_HALTED}]) never asserted within {timeout_cycles} cycles"

    trapped = bool(int(dut.uo_out.value) & (1 << UO_CPU_TRAP))
    assert not trapped, "CPU halted via an unexpected trap, not the program's own EBREAK"

    # Check the real result the program reported, via debug_regs' sticky
    # pass/fail bits -- the same signals tb_tinygpu_soc.sv checks.
    pass_seen = int(dut.user_project.u_soc.u_debug_regs.pass_seen_q.value)
    fail_seen = int(dut.user_project.u_soc.u_debug_regs.fail_seen_q.value)
    passfail_reg = int(dut.user_project.u_soc.u_debug_regs.passfail_q.value)

    dut._log.info(
        f"pass_seen={pass_seen} fail_seen={fail_seen} "
        f"passfail_reg=0x{passfail_reg:08x}"
    )

    assert pass_seen and not fail_seen, (
        f"Program did not report PASS "
        f"(pass_seen={pass_seen} fail_seen={fail_seen} "
        f"passfail_reg=0x{passfail_reg:08x})"
    )

    dut._log.info("TinyGPU-RV32 ran a real program and reported PASS")
