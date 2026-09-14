

# TinyGPU-RV32

A single-core RISC-V CPU with a memory-mapped debug module and an int8 vector accelerator, built for [Tiny Tapeout](https://tinytapeout.com) on SkyWater 130nm.

> **Note on ISA naming:** internal comments and `info.yaml` disagree on whether this is RV32I or RV32E — the register file (`regs_q`) physically holds 16 entries and `rs1`/`rs2` decode only uses the bottom 4 bits of the 5-bit register address, so registers x16–x31 alias onto x0–x15 rather than trapping. That's closer to RV32E behavior implemented via truncation than either a spec-compliant RV32E (which requires a hardware trap on x16–x31 access) or full RV32I (which needs 32 physical registers). This should be resolved and the docs made consistent before calling the ISA support "done."

## What this is

TinyGPU-RV32 is a non-pipelined, single-issue RISC-V control core integrated into a small SoC with three peripherals sharing one 32-bit MMIO address space:

- **CPU core** (`rv32_core.sv`) — sequential FSM: `RESET → FETCH → DECODE_EXEC → MEM_WAIT → WRITEBACK`, with dedicated `TRAP` and `DEBUG_HALT` states. Implements the base integer ISA plus **Zicsr** (CSR read/modify/write) and trap redirect/`MRET`, added 2026-07-29.  
- **Scratchpad memory** (`scratchpad.sv`) — single shared memory for both instruction fetch and data access. Collision-free by construction: fetch and load/store requests originate from different FSM states of the same core, so they structurally can't assert in the same cycle.  
- **Debug module** (`debug_regs.sv`) — memory-mapped halt / resume / single-step / PC read-write / register read-write / retire-counter / **hardware breakpoint** / **cycle & stall performance counters**, reachable either via `ui_in` pins or by a running program issuing loads/stores to the debug address window (`0x9000_0000`–`0x9000_00FF`).  
- **Vector accelerator** (`vector_accel.sv`) — int8 SIMD unit: `VADD8`, `VSUB8`, `VMAX8`, `RELU8` (4-lane packed ops) and `DOT4I8` (4-lane signed dot product), driven through a memory-mapped command/operand/result register file (`accel_regs.sv`) at `0x8000_0000`–`0x8000_00FF`.  
- **External loader** (`ext_loader.sv`) — a 2-pin serial protocol (`ui_in[3]` \= mode, `ui_in[4]` \= bit) for loading a program into the scratchpad from outside the chip before release from reset.

## Memory map

| Region | Base | Notes |
| :---- | :---- | :---- |
| Scratchpad 0 | `0x0000_0000` | Shared instruction/data memory |
| Scratchpad 1 | `0x0001_0000` | Second scratchpad window |
| Accelerator | `0x8000_0000` | CMD / STATUS / SRC\_A / SRC\_B / SRC\_C / LEN / DST / RESULT / ERROR |
| Debug | `0x9000_0000` | STATUS / CONTROL / PC / REG\_SELECT / REG\_DATA / PASSFAIL / TRAP\_CAUSE / RETIRE\_COUNT / BP\_ADDR / BP\_CONTROL / PERF\_CYCLE\_COUNT / PERF\_STALL\_COUNT / ACCEL\_STATUS / ACCEL\_RESULT |

## Pinout

| Pin | Direction | Function |
| :---- | :---- | :---- |
| `ui_in[0:2]` | in | Debug halt / resume / single-step request |
| `ui_in[3:4]` | in | External loader mode / serial data bit |
| `uo_out[0:1]` | out | `cpu_halted`, `cpu_trap` |
| `uo_out[2:4]` | out | Accelerator `busy`, `done`, `error` |
| `uo_out[5:7]` | out | `trap_cause[2:0]` |
| `uio_out[0:6]` | out | PC\[6:0\] (continuous external trace) |
| `uio_out[7]` | out | External loader ready |

## Verification

Three layers, each covering what the others don't:

**1\. Formal (SymbiYosys / Z3), 13 properties across 3 blocks**

| Block | Properties | What's proven |
| :---- | :---- | :---- |
| `scratchpad` | 5 | `valid → ready` every cycle; out-of-range access always errors; in-range access never errors; out-of-range writes never commit; a zero-`wstrb` write leaves memory unchanged next cycle |
| `ext_loader` | 3 | Writes to memory only happen in the `LOAD_ACTIVE` state; `force_cpu_reset` exactly mirrors `ext_load_mode_i`; write addresses never exceed `SCRATCHPAD_WORDS` |
| `rv32_core` (Zicsr/trap subsystem) | 5 | `EBREAK` always halts for debug regardless of `mtvec`; `mtvec == 0` never redirects (pre-Zicsr behavior preserved exactly); a real trap redirect correctly saves `mepc`/`mtval`/PC; `MRET` restores `MIE`/`MPIE` correctly; CSR writes never reach the register file for an unrecognized address |

All 13 are checked under **fully unconstrained** `imem_rdata_i`/`dmem_rdata_i` — BMC explores every possible instruction encoding every cycle, not just what the loader or test programs happen to produce. Every property is paired with a `cover` statement proving its antecedent is actually reachable, not vacuously true.

*Toolchain note:* the properties are implemented as `bind`\-style harnesses using immediate assertions rather than named concurrent SVA properties, because the installed `yosys-slang` build parses named properties fine but fails at BMC-cell lowering (`"expression of type property with dynamic size unsupported for synthesis"`). Confirmed as a tool limitation (not an RTL issue) against SymbiYosys's own upstream examples. Each RTL module also carries the "real" named-property version in its own `` `ifdef ASSERT_ON `` block for use with a toolchain that supports it (Tabby CAD, JasperGold, VCS, Questa).

**2\. Constrained-random verification (cocotb)**

`test_crv.py` cross-checks randomized ALU operations against an independent, from-scratch Python golden model (deliberately not derived from the RTL, so it can't share a bug with it) across a **17-bin functional coverage model** (each op, zero operands, equal operands, sign-extension corners, `LUI`\-materialization corners, min/max int32). Pure random sampling is seed-dependent — seed 1 misses different bins than seed 999 — so coverage closure uses **directed top-up generators per bin** rather than just increasing the random iteration count, guaranteeing 100% closure deterministically instead of by RNG luck.

**3\. Directed tests**

- `test.py` — golden-path program execution through the real external loader pins (not hierarchical injection), so it runs identically against RTL and the synthesized gate-level netlist.  
- `test_zicsr.py` — CSR read/write round-trip, plus a full trap-redirect → handler → `MRET` → **resumed execution** round-trip (proving recovery, not just handler entry).  
- `test_bp_perf.py` — hardware breakpoint and cycle/stall performance counters, exercised entirely through real chip pins and MMIO so it also runs under gate-level simulation.

### Two real bugs found and fixed during verification

1. **Gate-level-only testbench timing race.** `test.py` originally sampled the external loader's `ready` signal at `posedge clk + 1ns`. Under RTL sim (zero-delay flops) this was safely after the transition; under gate-level sim (`-DUNIT_DELAY=#1`, zero-delay combinational cells) it landed *exactly on* the flop's `CLK→Q` edge, sampled a stale `0`, and shifted the entire 480-bit load stream right by one bit — corrupting word 0 into an illegal opcode and causing a spurious `TRAP_ILLEGAL_INSTR`. Root-caused from the failing waveform (every event landed at either \+0ns or \+1ns, nothing between), not guesswork. Fixed by sampling mid-cycle instead of at the edge.  
2. **Stale-operand bug in the vector accelerator.** Caught during verification and turned into a permanent formal regression property (`p_result_matches_exec_on_start`) proving the registered result always equals the combinational result sampled on the actual start cycle — not a leftover value from a previous command.

## How to test

1. Hold `rst_n` low for a few clock cycles, then release it. The CPU begins fetching from scratchpad address 0\.  
2. Load a program via the external loader protocol on `ui_in[3:4]`, or via the debug module's register-write path once halted.  
3. `uo_out[0]`/`uo_out[1]` report `cpu_halted`/`cpu_trap`; `uo_out[5:7]` report the trap cause; `uo_out[2:4]` report accelerator busy/done/error.  
4. `uio_out[6:0]` continuously exposes the low 7 bits of the PC for external tracing; `uio_out[7]` reports loader readiness.  
5. Drive `ui_in[0]` high for one cycle to request a debug halt, `ui_in[2]` to single-step, `ui_in[1]` to resume.

## Running the test suite

cd test

make \-B                                  \# RTL simulation

make \-B GATES=yes                        \# gate-level, after hardening \+ copying the netlist

CRV\_SEED=\<n\> make \-B TESTCASE=test\_crv   \# constrained-random with a specific seed

cd formal/scratchpad && sby \-f scratchpad.sby

cd formal/ext\_loader  && sby \-f ext\_loader.sby

cd formal/rv32\_core   && sby \-f rv32\_core\_bind.sby

## Known gaps (stated honestly, not silently left out)

- CRV coverage (`test_crv.py`) currently exercises OP/OP-IMM ALU paths, `LUI`\-based immediate materialization, and the illegal-instruction trap path — it does **not** yet cover `JAL`/`JALR`, loads/stores beyond the debug PASSFAIL write, misaligned-access traps, or the vector accelerator's random operand space. Extending `COVERAGE_BINS` is the natural next step.  
- The `info.md` firmware bring-up section still has a TODO: a concrete "here's what a loaded program should do" walkthrough for first-time bring-up hasn't been written yet.  
- The RV32I/RV32E naming inconsistency noted above should be resolved.

