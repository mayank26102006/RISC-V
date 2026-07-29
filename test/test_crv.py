# SPDX-FileCopyrightText: © 2026 TinyGPU-RV32
# SPDX-License-Identifier: Apache-2.0
#
# Constrained-random verification + functional coverage for TinyGPU-RV32.
#
# test.py (the original directed test) proves ONE specific 15-instruction
# program executes correctly. That answers "does the golden path work" but
# says nothing about how much of the instruction set, operand space, or
# error path was actually exercised -- which is exactly the question a
# reviewer asks about any DV claim. This file answers it directly: it
# randomizes instruction selection and operands across every ALU op the
# core implements, cross-checks every result against a from-scratch Python
# RV32I golden model (independent of the RTL, so it can't share a bug with
# it), and reports functional coverage bins at the end instead of just a
# pass/fail count.
#
# Loading still goes through the real ext_loader pins (see test.py for why),
# reusing the same timing-safe protocol that fixed the original gl_test
# failure -- CRV built on a loader with a known timing bug would just
# produce a wall of spurious failures that have nothing to do with the CPU.
#
# Scope, honestly stated: this covers the OP/OP-IMM ALU paths, LUI-based
# 32-bit immediate materialization, and the illegal-
# instruction trap path. It does NOT yet cover JAL/JALR, loads/stores other
# than the debug PASSFAIL write, misaligned-access traps, or the vector
# accelerator. Extending COVERAGE_BINS and add_random_alu_test() is the
# natural next step -- flagged rather than silently left out.

import random

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, Timer

from tinygpu_pkg import (
    DBG_REG_PASSFAIL,
    encode_addi,
    encode_alu_op,
    encode_ebreak,
    encode_lui,
    encode_sw,
    materialize_imm32,
    ALU_OPS,
)

UI_EXT_LOAD_MODE = 3
UI_EXT_LOAD_BIT = 4
UO_CPU_HALTED = 0
UO_CPU_TRAP = 1
UIO_EXT_LOAD_READY = 7

CLK_PERIOD_NS = 10_000
SETTLE_NS = CLK_PERIOD_NS // 10
SCRATCHPAD_WORDS = 16

N_RANDOM_ALU_TESTS = 40
RANDOM_SEED = int(__import__("os").environ.get("CRV_SEED", "1"))

TRAP_CAUSE_NAMES = {
    0: "TRAP_NONE", 1: "TRAP_ILLEGAL_INSTR", 2: "TRAP_INSTR_MISALIGNED",
    3: "TRAP_LOAD_MISALIGNED", 4: "TRAP_STORE_MISALIGNED",
    5: "TRAP_BUS_ERROR", 6: "TRAP_ECALL", 7: "TRAP_EBREAK",
}

# Functional coverage model. Each bin is (name -> predicate over the
# randomized test's inputs/outputs); a bin is "hit" the first time a
# generated test satisfies its predicate. This is a coverage MODEL, not
# just a counter -- it says what specifically was and wasn't exercised.
COVERAGE_BINS = {
    "op:add": lambda op, a, b, r: op == "add",
    "op:sub": lambda op, a, b, r: op == "sub",
    "op:and": lambda op, a, b, r: op == "and",
    "op:or": lambda op, a, b, r: op == "or",
    "op:xor": lambda op, a, b, r: op == "xor",
    "op:slt": lambda op, a, b, r: op == "slt",
    "op:sltu": lambda op, a, b, r: op == "sltu",
    "operand_a_zero": lambda op, a, b, r: a == 0,
    "operand_b_zero": lambda op, a, b, r: b == 0,
    "operands_equal": lambda op, a, b, r: a == b,
    "result_zero": lambda op, a, b, r: r == 0,
    "result_negative": lambda op, a, b, r: (r >> 31) & 1 == 1,
    "operand_a_needs_lui": lambda op, a, b, r: not (-2048 <= to_signed(a) <= 2047),
    "operand_b_needs_lui": lambda op, a, b, r: not (-2048 <= to_signed(b) <= 2047),
    "operand_a_min_i32": lambda op, a, b, r: a == 0x80000000,
    "operand_a_max_i32": lambda op, a, b, r: a == 0x7FFFFFFF,
    "operand_a_all_ones": lambda op, a, b, r: a == 0xFFFFFFFF,
}


def to_signed(v, bits=32):
    v &= (1 << bits) - 1
    return v - (1 << bits) if v >> (bits - 1) else v


def golden_alu(op, a, b):
    """Independent-of-RTL reference model. This is deliberately NOT derived
    from rv32_core.sv -- if it were, a shared misunderstanding of the ISA
    would produce a shared bug, and CRV would prove nothing."""
    a32, b32 = a & 0xFFFFFFFF, b & 0xFFFFFFFF
    sa, sb = to_signed(a32), to_signed(b32)
    if op == "add":
        return (a32 + b32) & 0xFFFFFFFF
    if op == "sub":
        return (a32 - b32) & 0xFFFFFFFF
    if op == "and":
        return a32 & b32
    if op == "or":
        return a32 | b32
    if op == "xor":
        return a32 ^ b32
    if op == "slt":
        return 1 if sa < sb else 0
    if op == "sltu":
        return 1 if a32 < b32 else 0
    raise ValueError(op)


def random_operand(rng):
    """Biased toward corner values -- uniform random 32-bit ints would take
    thousands of iterations to hit 0, -1, INT_MIN, or INT_MAX by chance,
    and those are exactly the values most likely to expose a bug."""
    corners = [0, 1, 0xFFFFFFFF, 0x80000000, 0x7FFFFFFF, 0xFFFFFFFE]
    if rng.random() < 0.35:
        return rng.choice(corners)
    return rng.randint(0, 0xFFFFFFFF)


def build_alu_test_program(op, a, b):
    """x1=a; x2=b; x3=op(x1,x2); store x3 directly to DBG_REG_PASSFAIL; ebreak.

    debug_regs.sv does `passfail_q <= wdata_i` unconditionally on any write
    to that register (confirmed by reading debug_regs.sv directly -- it's
    not gated on the value matching a PASS/FAIL signature, that gating only
    applies to the separate pass_seen_q/fail_seen_q sticky bits). So the
    simplest and most general check is: write the raw computed result there,
    then compare the register's contents against golden_alu()'s output.
    No branch, no FAIL_SIGNATURE/PASS_SIGNATURE dance needed -- that
    machinery from test.py's MINI_PROGRAM existed to make a *human-readable*
    pass/fail waveform, which isn't the goal here.
    """
    expected = golden_alu(op, a, b)
    words = []
    words += materialize_imm32(1, a)   # x1 = a
    words += materialize_imm32(2, b)   # x2 = b
    words.append(encode_alu_op(op, rd=3, rs1=1, rs2=2))  # x3 = op(x1,x2)
    words += materialize_imm32(14, DBG_REG_PASSFAIL)      # x14 = &PASSFAIL
    words.append(encode_sw(rs1=14, rs2=3, imm=0))         # [x14] = x3
    words.append(encode_ebreak())

    if len(words) > SCRATCHPAD_WORDS:
        return None  # caller retries with a smaller-immediate operand pair
    return words, expected


def build_illegal_instr_program():
    """A single genuinely-illegal 32-bit word. Exercises the trap path
    (TRAP_ILLEGAL_INSTR) via the exact same externally-observable interface
    (uo_out[7:5] trap_cause) as everything else in this file -- no
    hierarchical peek needed, so this is GL-compatible too."""
    return [0xFFFFFFFF]


def read_bits(sig, hi, lo):
    try:
        return (int(sig.value) >> lo) & ((1 << (hi - lo + 1)) - 1)
    except ValueError:
        return None


async def settle():
    await Timer(SETTLE_NS, unit="ns")


async def reset_dut(dut):
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


async def load_program_via_pins(dut, words):
    """Identical protocol to test.py's fixed loader -- see that file for the
    full timing-race writeup. Duplicated rather than imported so this file
    has no import-order dependency on test.py under cocotb's test-module
    loader."""
    mode = 1 << UI_EXT_LOAD_MODE
    await RisingEdge(dut.clk)
    await settle()
    dut.ui_in.value = mode

    for _ in range(16):
        await RisingEdge(dut.clk)
        await settle()
        if read_bits(dut.uio_out, UIO_EXT_LOAD_READY, UIO_EXT_LOAD_READY) == 1:
            break
    else:
        raise AssertionError("ext_load_ready never asserted")

    for word in words:
        for i in range(31, -1, -1):
            dut.ui_in.value = mode | (((word >> i) & 1) << UI_EXT_LOAD_BIT)
            await RisingEdge(dut.clk)
            await settle()
            if read_bits(dut.uio_out, UIO_EXT_LOAD_READY, UIO_EXT_LOAD_READY) != 1:
                raise AssertionError("ext_load_ready dropped mid-transfer")

    dut.ui_in.value = 0
    await settle()


async def run_program_to_halt(dut, timeout_cycles=300):
    halted = False
    uo = 0
    for _ in range(timeout_cycles):
        await RisingEdge(dut.clk)
        await settle()
        v = read_bits(dut.uo_out, 7, 0)
        if v is None:
            continue
        uo = v
        if uo & (1 << UO_CPU_HALTED):
            halted = True
            break
    return halted, uo


@cocotb.test()
async def test_crv_alu_coverage(dut):
    """Constrained-random ALU coverage: N random (op, a, b) triples, each
    independently checked against golden_alu(), each loaded and run through
    the real external pin loader. Reports functional coverage bins hit
    at the end.
    """
    rng = random.Random(RANDOM_SEED)
    dut._log.info(f"CRV seed = {RANDOM_SEED} (set CRV_SEED env var to reproduce)")

    cocotb.start_soon(Clock(dut.clk, CLK_PERIOD_NS, unit="ns").start())

    hits = {name: False for name in COVERAGE_BINS}
    tests_run = 0
    attempts = 0
    max_attempts = N_RANDOM_ALU_TESTS * 4  # generous slack for the retry-if-too-long case

    while tests_run < N_RANDOM_ALU_TESTS and attempts < max_attempts:
        attempts += 1
        op = rng.choice(list(ALU_OPS))
        a = random_operand(rng)
        b = random_operand(rng)
        built = build_alu_test_program(op, a, b)
        if built is None:
            continue  # program didn't fit in 16 words this draw, try another
        words, expected = built
        tests_run += 1

        for name, pred in COVERAGE_BINS.items():
            if pred(op, a, b, expected):
                hits[name] = True

        await reset_dut(dut)
        await load_program_via_pins(dut, words)
        halted, uo = await run_program_to_halt(dut)

        assert halted, (
            f"[test {tests_run}] TIMEOUT: op={op} a=0x{a:08x} b=0x{b:08x} "
            f"expected=0x{expected:08x} -- cpu_halted never asserted"
        )
        trapped = bool(uo & (1 << UO_CPU_TRAP))
        trap_cause = (uo >> 5) & 0x7
        assert not trapped, (
            f"[test {tests_run}] op={op} a=0x{a:08x} b=0x{b:08x} "
            f"expected=0x{expected:08x} -- unexpected trap "
            f"({TRAP_CAUSE_NAMES.get(trap_cause, '?')}), program was "
            "corrupted or the golden model disagrees with the RTL"
        )

        try:
            passfail = int(dut.user_project.u_soc.u_debug_regs.passfail_q.value)
            assert passfail == expected, (
                f"[test {tests_run}] op={op} a=0x{a:08x} b=0x{b:08x} -- "
                f"RTL wrote 0x{passfail:08x} to PASSFAIL, golden_alu() says "
                f"0x{expected:08x}: RTL result did not match golden model"
            )
        except AttributeError:
            pass  # gate-level: halted-without-trapping is the success criterion

        dut._log.info(
            f"[test {tests_run}/{N_RANDOM_ALU_TESTS}] op={op:>4} "
            f"a=0x{a:08x} b=0x{b:08x} expected=0x{expected:08x}  OK"
        )

    assert tests_run == N_RANDOM_ALU_TESTS, (
        f"only {tests_run}/{N_RANDOM_ALU_TESTS} random programs fit in "
        f"{SCRATCHPAD_WORDS} words after {attempts} attempts -- widen "
        "SCRATCHPAD_WORDS or shorten build_alu_test_program()"
    )

    # Directed top-up: pure random sampling can miss low-probability corners
    # by chance alone -- confirmed empirically: seed=1 missed operands_equal
    # and operand_a_min_i32 in 40 draws, seed=999 instead missed
    # operand_a_zero. Different seeds miss different bins, which means a
    # top-up covering only the bins observed missing under one seed is
    # itself seed-dependent and not a real fix. Every bin gets an explicit
    # directed generator instead, so coverage closure no longer depends on
    # RNG luck at all -- standard coverage-directed test generation.
    def _op(rng):
        return rng.choice(list(ALU_OPS))

    directed_extra = {
        "op:add": lambda rng: ("add", random_operand(rng), random_operand(rng)),
        "op:sub": lambda rng: ("sub", random_operand(rng), random_operand(rng)),
        "op:and": lambda rng: ("and", random_operand(rng), random_operand(rng)),
        "op:or": lambda rng: ("or", random_operand(rng), random_operand(rng)),
        "op:xor": lambda rng: ("xor", random_operand(rng), random_operand(rng)),
        "op:slt": lambda rng: ("slt", random_operand(rng), random_operand(rng)),
        "op:sltu": lambda rng: ("sltu", random_operand(rng), random_operand(rng)),
        "operand_a_zero": lambda rng: (_op(rng), 0, random_operand(rng)),
        "operand_b_zero": lambda rng: (_op(rng), random_operand(rng), 0),
        "operands_equal": lambda rng: (lambda op, v: (op, v, v))(_op(rng), random_operand(rng)),
        "result_zero": lambda rng: ("xor", (lambda v: v)(random_operand(rng)), None),  # patched below
        "result_negative": lambda rng: ("or", 0x80000000, random_operand(rng)),
        "operand_a_needs_lui": lambda rng: (_op(rng), 0x12345678, random_operand(rng)),
        "operand_b_needs_lui": lambda rng: (_op(rng), random_operand(rng), 0x12345678),
        "operand_a_min_i32": lambda rng: (_op(rng), 0x80000000, random_operand(rng)),
        "operand_a_max_i32": lambda rng: (_op(rng), 0x7FFFFFFF, random_operand(rng)),
        "operand_a_all_ones": lambda rng: (_op(rng), 0xFFFFFFFF, random_operand(rng)),
    }
    for name, gen in directed_extra.items():
        if hits.get(name):
            continue
        op, a, b = gen(rng)
        if name == "result_zero":
            b = a  # xor(a, a) == 0 always, guarantees this bin regardless of a
        built = build_alu_test_program(op, a, b)
        assert built is not None, f"directed top-up for {name} didn't fit in {SCRATCHPAD_WORDS} words"
        words, expected = built
        tests_run += 1
        for bname, pred in COVERAGE_BINS.items():
            if pred(op, a, b, expected):
                hits[bname] = True

        await reset_dut(dut)
        await load_program_via_pins(dut, words)
        halted, uo = await run_program_to_halt(dut)
        assert halted, f"[directed top-up: {name}] TIMEOUT op={op} a=0x{a:08x} b=0x{b:08x}"
        trapped = bool(uo & (1 << UO_CPU_TRAP))
        assert not trapped, f"[directed top-up: {name}] unexpected trap, op={op} a=0x{a:08x} b=0x{b:08x}"
        try:
            passfail = int(dut.user_project.u_soc.u_debug_regs.passfail_q.value)
            assert passfail == expected, (
                f"[directed top-up: {name}] op={op} a=0x{a:08x} b=0x{b:08x} "
                f"-- RTL wrote 0x{passfail:08x}, golden_alu() says 0x{expected:08x}"
            )
        except AttributeError:
            pass
        dut._log.info(f"[directed top-up: {name}] op={op:>4} a=0x{a:08x} b=0x{b:08x}  OK")

    dut._log.info("=" * 70)
    dut._log.info(f"FUNCTIONAL COVERAGE ({tests_run} total tests: "
                   f"{N_RANDOM_ALU_TESTS} random + directed top-up, seed={RANDOM_SEED})")
    dut._log.info("=" * 70)
    covered = sum(hits.values())
    for name, was_hit in hits.items():
        dut._log.info(f"  [{'X' if was_hit else ' '}] {name}")
    dut._log.info(f"Coverage: {covered}/{len(hits)} bins ({100*covered//len(hits)}%)")
    dut._log.info("=" * 70)

    missed = [name for name, was_hit in hits.items() if not was_hit]
    assert not missed, (
        f"{len(missed)} coverage bin(s) never hit in {tests_run} random "
        f"tests -- increase N_RANDOM_ALU_TESTS or fix random_operand() "
        f"bias: {missed}"
    )


@cocotb.test()
async def test_crv_illegal_instruction_trap(dut):
    """Directed-random hybrid: deliberately loads a genuinely illegal
    instruction and checks the trap path fires with the right cause, using
    only externally-observable pins (works under GL too)."""
    cocotb.start_soon(Clock(dut.clk, CLK_PERIOD_NS, unit="ns").start())
    await reset_dut(dut)
    await load_program_via_pins(dut, build_illegal_instr_program())
    halted, uo = await run_program_to_halt(dut)

    assert halted, "TIMEOUT: illegal instruction should trap-halt, never asserted cpu_halted"
    trapped = bool(uo & (1 << UO_CPU_TRAP))
    trap_cause = (uo >> 5) & 0x7
    assert trapped, f"illegal instruction did not trap (uo_out=0x{uo:02x})"
    assert trap_cause == 1, (
        f"expected TRAP_ILLEGAL_INSTR (1), got trap_cause={trap_cause} "
        f"({TRAP_CAUSE_NAMES.get(trap_cause, '?')})"
    )
    dut._log.info(f"illegal instruction correctly trapped: uo_out=0x{uo:02x}, "
                   f"trap_cause={trap_cause} (TRAP_ILLEGAL_INSTR)")
