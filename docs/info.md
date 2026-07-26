<!---

This file is used to generate your project datasheet. Please fill in the information below and delete any unused
sections.

You can also include images in this folder and reference them in the markdown. Each image must be less than
512 kb in size, and the combined size of all images must be less than 1 MB.
-->

How it works

TinyGPU-RV32 is a single-issue RV32I RISC-V CPU integrated into a small SoC alongside a memory-mapped debug module and an int8 vector accelerator.

CPU core: implements the RV32I base instruction set (arithmetic, logic, shifts, comparisons, branches, JAL/JALR, loads/stores, ECALL, EBREAK, FENCE, LUI/AUIPC) as a sequential (non-pipelined) finite state machine: fetch, decode/execute, memory-wait, writeback.
Memory: a single shared scratchpad serves both instruction fetch and data access. Fetch and load/store requests are structurally guaranteed never to collide in the same cycle (they come from different states of the same core FSM), so one physical memory safely serves both.
Debug module: memory-mapped halt / resume / single-step / PC read-write / register read-write / retire-counter, reachable either through the ui_in pins below or through software issuing loads and stores to the debug address window.
Vector accelerator: a small int8 SIMD unit supporting VADD8, VSUB8, VMAX8, RELU8 (4-lane packed operations), and DOT4I8 (a 4-lane signed dot product), driven through a small memory-mapped register file (command / source operands / result).

The design was verified with a self-checking testbench covering ALU and register-file correctness, load/store byte/halfword sign and zero extension, branch and jump control flow, the accelerator (including a regression check for a stale-operand bug found and fixed during verification), illegal-instruction trap detection, and the debug halt/single-step/resume flow -- all driven through the same pins exposed on this chip.

How to test
Hold rst_n low for a few clock cycles, then release it.
The CPU begins fetching from address 0 of the shared scratchpad.
uo_out[0] (cpu_halted) and uo_out[1] (cpu_trap) report core status; uo_out[5:7] report the low 3 bits of the trap cause.
uo_out[2:4] report accelerator busy/done/error.
uio_out[7:0] continuously exposes the low byte of the program counter for external tracing.
Drive ui_in[0] high for one cycle to request a debug halt, ui_in[2] to single-step, ui_in[1] to resume.

(TODO before submitting: load a small firmware image into the shared scratchpad -- via  `SIM_INIT / an INIT_FILE in simulation, or via the debug module's register-write path once wired to a real host -- and describe exactly what it should do here, since Tiny Tapeout reviewers and anyone reading the datasheet will want a concrete "here's what you should see happen" description, not just the pin reference above.)

External hardware

None required for basic bring-up -- all status is visible on uo_out and uio_out. A logic analyzer or the Tiny Tapeout demo board's built-in tools are enough to observe cpu_halted and the PC trace during bring-up.
