`ifndef TINYGPU_PKG_SV
`define TINYGPU_PKG_SV

//------------------------------------------------------------------------
// tinygpu_pkg.sv - TinyGPU-RV32 Shared Types and Constants
//------------------------------------------------------------------------
// Project : TinyGPU-RV32
// Package : tinygpu_pkg
// Description : Central package containing shared architectural constants,
// encodings, enums, trap causes, accelerator commands, and
// debug status codes used across the RTL.
//
// Why this package exists:
// Earlier standalone modules used localparams so they could compile alone.
// As the project moves toward full SoC integration, those duplicate
// localparams should be replaced by imports from this package.
//
// Usage:
// `include "tinygpu_pkg.sv"
//
// NOTE (2026-07-26): this file was originally a SystemVerilog `package`,
// brought into each module via `import tinygpu_pkg::*;`. That was changed
// to a plain `include`-guarded header (no `package`/`endpackage` wrapper)
// because Yosys's read_verilog frontend -- used by LibreLane/OpenLane for
// ASIC hardening -- has a confirmed, longstanding limitation: it does not
// support `import package::*;` to bring package-scoped types into another
// file, in any placement (before the module, in the ANSI header via
// `module foo import pkg::*; (...)`, or inside the module body). This is
// tracked upstream as multiple still-open YosysHQ/yosys issues (#2187,
// #4006, #4279) going back to 2020. Reproduced locally against the exact
// yowasp-yosys 0.55 build used in this project's CI before making this
// change -- both the failure with `import` and the fix with plain
// `` `include `` were verified, not assumed. Every file that previously
// had `import tinygpu_pkg::*;` now has `` `include "tinygpu_pkg.sv" ``
// instead, and this header's own pre-existing include guard
// (TINYGPU_PKG_SV) prevents duplicate-declaration errors when multiple
// source files each include it in the same build.
//
// ASIC note:
// - Contains constants/types only.
// - No hardware is inferred directly from this file.
//------------------------------------------------------------------------

//------------------------------------------------------------------
// Global architectural constants
//------------------------------------------------------------------

parameter int XLEN = 32;
parameter int REG_ADDR_W = 5;
parameter int NUM_REGS = 32;

parameter logic [31:0] RESET_VECTOR_DEFAULT = 32'h0000_0000;

// Pass/fail signatures used by test programs and debug/status logic.
parameter logic [31:0] PASS_SIGNATURE = 32'hCAFE_BABE;
parameter logic [31:0] FAIL_PREFIX = 32'hDEAD_0000;
parameter logic [15:0] FAIL_UPPER16 = 16'hDEAD;

//------------------------------------------------------------------
// RV32I opcode constants
//------------------------------------------------------------------

parameter logic [6:0] OPCODE_LUI = 7'b0110111;
parameter logic [6:0] OPCODE_AUIPC = 7'b0010111;
parameter logic [6:0] OPCODE_JAL = 7'b1101111;
parameter logic [6:0] OPCODE_JALR = 7'b1100111;
parameter logic [6:0] OPCODE_BRANCH = 7'b1100011;
parameter logic [6:0] OPCODE_LOAD = 7'b0000011;
parameter logic [6:0] OPCODE_STORE = 7'b0100011;
parameter logic [6:0] OPCODE_OP_IMM = 7'b0010011;
parameter logic [6:0] OPCODE_OP = 7'b0110011;
parameter logic [6:0] OPCODE_FENCE = 7'b0001111;
parameter logic [6:0] OPCODE_SYSTEM = 7'b1110011;

// SYSTEM instruction exact encodings used by TinyGPU-RV32 v1.
parameter logic [31:0] INSTR_ECALL = 32'h0000_0073;
parameter logic [31:0] INSTR_EBREAK = 32'h0010_0073;

// -------------------------------------------------------------------
// Zicsr (2026-07-29): minimal machine-mode CSR file, added specifically
// so this core can execute the boot sequence riscv-arch-test's harness
// requires (mstatus/mepc/mcause/mtvec setup runs unconditionally before
// ANY test, including plain RV32I ones -- confirmed by reading
// rvtest_setup.h directly, not assumed). Machine-mode only: there is no
// S-mode or U-mode on this core, so MPP is not implemented as a real
// field (this core never leaves M-mode, so "previous privilege" is
// always M-mode by construction).
//
// SCOPE, stated plainly: mstatus/mie/mip/mstatush are stored as plain
// 32-bit read/write registers with only MIE (bit 3) and MPIE (bit 7) of
// mstatus given real hardware meaning (interrupt-enable save/restore
// across trap entry and MRET). All other bits round-trip whatever
// software wrote but do nothing -- there are no S/U modes, no PMP, no
// virtual memory, and no interrupt SOURCES wired into this SoC yet, so
// implementing the rest of those fields would be state with no possible
// effect. mie/mip exist only so boot code that writes/reads them doesn't
// trap; there is no interrupt controller behind them yet.
//
// mtvec: direct mode only. The 2 low mode-select bits are stored (so
// software reads back exactly what it wrote) but ignored by hardware --
// every trap redirects to {mtvec[31:2], 2'b00} regardless of mode bits.
// Vectored mode is not implemented.
// -------------------------------------------------------------------

parameter logic [31:0] INSTR_MRET = 32'h3020_0073;

parameter logic [11:0] CSR_MSTATUS = 12'h300;
parameter logic [11:0] CSR_MISA = 12'h301;
parameter logic [11:0] CSR_MIE = 12'h304;
parameter logic [11:0] CSR_MTVEC = 12'h305;
parameter logic [11:0] CSR_MSTATUSH = 12'h310;
parameter logic [11:0] CSR_MSCRATCH = 12'h340;
parameter logic [11:0] CSR_MEPC = 12'h341;
parameter logic [11:0] CSR_MCAUSE = 12'h342;
parameter logic [11:0] CSR_MTVAL = 12'h343;
parameter logic [11:0] CSR_MIP = 12'h344;
parameter logic [11:0] CSR_MVENDORID = 12'hF11;
parameter logic [11:0] CSR_MARCHID = 12'hF12;
parameter logic [11:0] CSR_MIMPID = 12'hF13;
parameter logic [11:0] CSR_MHARTID = 12'hF14;

// misa: MXL=01 (32-bit), Extensions bit for "E" (bit 4) set since this is
// RV32E not RV32I (see regs_q comment in rv32_core.sv). No other
// extension bits set (no M, no C, no F).
parameter logic [31:0] MISA_VALUE = 32'h4000_0010;

parameter int MSTATUS_MIE_BIT = 3;
parameter int MSTATUS_MPIE_BIT = 7;

//------------------------------------------------------------------
// Immediate type encoding
//------------------------------------------------------------------

typedef enum logic [2:0] {
IMM_I = 3'd0,
IMM_S = 3'd1,
IMM_B = 3'd2,
IMM_U = 3'd3,
IMM_J = 3'd4
} imm_type_e;

//------------------------------------------------------------------
// ALU operation encoding
//------------------------------------------------------------------

typedef enum logic [3:0] {
ALU_ADD = 4'd0,
ALU_SUB = 4'd1,
ALU_AND = 4'd2,
ALU_OR = 4'd3,
ALU_XOR = 4'd4,
ALU_SLL = 4'd5,
ALU_SRL = 4'd6,
ALU_SRA = 4'd7,
ALU_SLT = 4'd8,
ALU_SLTU = 4'd9,
ALU_PASS_B = 4'd10
} alu_op_e;

//------------------------------------------------------------------
// ALU source select encoding
//------------------------------------------------------------------

typedef enum logic [1:0] {
ALU_A_RS1 = 2'd0,
ALU_A_PC = 2'd1,
ALU_A_ZERO = 2'd2
} alu_src_a_sel_e;

typedef enum logic [1:0] {
ALU_B_RS2 = 2'd0,
ALU_B_IMM = 2'd1,
ALU_B_FOUR = 2'd2
} alu_src_b_sel_e;

//------------------------------------------------------------------
// Branch type encoding
//------------------------------------------------------------------

typedef enum logic [2:0] {
BR_NONE = 3'd0,
BR_BEQ = 3'd1,
BR_BNE = 3'd2,
BR_BLT = 3'd3,
BR_BGE = 3'd4,
BR_BLTU = 3'd5,
BR_BGEU = 3'd6
} branch_type_e;

//------------------------------------------------------------------
// Writeback select encoding
//------------------------------------------------------------------

typedef enum logic [2:0] {
WB_NONE = 3'd0,
WB_ALU = 3'd1,
WB_LOAD = 3'd2,
WB_PC4 = 3'd3,
WB_IMM = 3'd4
} wb_sel_e;

//------------------------------------------------------------------
// Memory access size encoding
//------------------------------------------------------------------

typedef enum logic [1:0] {
MEM_BYTE = 2'd0,
MEM_HALF = 2'd1,
MEM_WORD = 2'd2
} mem_size_e;

//------------------------------------------------------------------
// PC select encoding
//------------------------------------------------------------------

typedef enum logic [2:0] {
PC_PLUS4 = 3'd0,
PC_BRANCH = 3'd1,
PC_JAL = 3'd2,
PC_JALR = 3'd3,
PC_DEBUG = 3'd4,
PC_HOLD = 3'd5
} pc_sel_e;

//------------------------------------------------------------------
// CPU FSM state encoding
//------------------------------------------------------------------

typedef enum logic [2:0] {
CPU_RESET = 3'd0,
CPU_FETCH = 3'd1,
CPU_DECODE_EXEC = 3'd2,
CPU_MEM_WAIT = 3'd3,
CPU_WRITEBACK = 3'd4,
CPU_TRAP = 3'd5,
CPU_DEBUG_HALT = 3'd6
} cpu_state_e;

//------------------------------------------------------------------
// Trap cause encoding
//------------------------------------------------------------------

typedef enum logic [7:0] {
TRAP_NONE = 8'h00,
TRAP_ILLEGAL_INSTR = 8'h01,
TRAP_INSTR_MISALIGNED = 8'h02,
TRAP_LOAD_MISALIGNED = 8'h03,
TRAP_STORE_MISALIGNED = 8'h04,
TRAP_BUS_ERROR = 8'h05,
TRAP_ECALL = 8'h06,
TRAP_EBREAK = 8'h07
} trap_cause_e;

//------------------------------------------------------------------
// Accelerator command encoding
//------------------------------------------------------------------

typedef enum logic [7:0] {
ACC_CMD_NOP = 8'h00,
ACC_CMD_VADD8 = 8'h01,
ACC_CMD_VSUB8 = 8'h02,
ACC_CMD_VMAX8 = 8'h03,
ACC_CMD_RELU8 = 8'h04,
ACC_CMD_DOT4I8 = 8'h05,
ACC_CMD_MAT2I8 = 8'h06
} accel_cmd_e;

// Accelerator status bit positions.
parameter int ACC_STATUS_BUSY_BIT = 0;
parameter int ACC_STATUS_DONE_BIT = 1;
parameter int ACC_STATUS_ERROR_BIT = 2;
parameter int ACC_STATUS_ILLEGAL_COMMAND_BIT = 3;
parameter int ACC_STATUS_BUSY_VIOLATION_BIT = 4;

// Accelerator error bit positions.
parameter int ACC_ERROR_ILLEGAL_COMMAND_BIT = 0;
parameter int ACC_ERROR_BUSY_VIOLATION_BIT = 1;
parameter int ACC_ERROR_INTERNAL_ERROR_BIT = 2;

//------------------------------------------------------------------
// Debug command encoding
//------------------------------------------------------------------

typedef enum logic [7:0] {
DBG_NOP = 8'h00,
DBG_HALT = 8'h01,
DBG_RESUME = 8'h02,
DBG_STEP = 8'h03,
DBG_READ_PC = 8'h04,
DBG_WRITE_PC = 8'h05,
DBG_READ_REG = 8'h06,
DBG_WRITE_REG = 8'h07,
DBG_READ_STATUS = 8'h08,
DBG_READ_PASSFAIL = 8'h09,
DBG_READ_TRAP_CAUSE = 8'h0A,
DBG_READ_RETIRE_COUNT = 8'h0B,
DBG_READ_ACCEL_STATUS = 8'h0C,
DBG_READ_ACCEL_RESULT = 8'h0D,
DBG_READ_SCRATCH = 8'h0E,
DBG_WRITE_SCRATCH = 8'h0F
} dbg_cmd_e;

// Debug command response/status encoding.
typedef enum logic [7:0] {
DBG_STATUS_OK = 8'h00,
DBG_STATUS_ERROR_UNKNOWN_COMMAND = 8'h01,
DBG_STATUS_ERROR_CPU_NOT_HALTED = 8'h02,
DBG_STATUS_ERROR_BAD_REG_INDEX = 8'h03,
DBG_STATUS_ERROR_BAD_ADDRESS = 8'h04,
DBG_STATUS_ERROR_MISALIGNED_ADDR = 8'h05,
DBG_STATUS_ERROR_TARGET_BUSY = 8'h06,
DBG_STATUS_ERROR_TRAP_ACTIVE = 8'h07,
DBG_STATUS_ERROR_UNSUPPORTED = 8'h08
} dbg_status_e;

// Debug status register bit positions.
parameter int DBG_STATUS_CPU_HALTED_BIT = 0;
parameter int DBG_STATUS_CPU_TRAP_BIT = 1;
parameter int DBG_STATUS_CPU_DEBUG_HALT_BIT = 2;
parameter int DBG_STATUS_CPU_RUNNING_BIT = 3;
parameter int DBG_STATUS_ACCEL_BUSY_BIT = 4;
parameter int DBG_STATUS_ACCEL_DONE_BIT = 5;
parameter int DBG_STATUS_ACCEL_ERROR_BIT = 6;
parameter int DBG_STATUS_PASS_SEEN_BIT = 7;
parameter int DBG_STATUS_FAIL_SEEN_BIT = 8;
parameter int DBG_STATUS_DEBUG_CMD_BUSY_BIT = 9;
parameter int DBG_STATUS_DEBUG_CMD_ERROR_BIT = 10;

// Debug control bit positions.
parameter int DBG_CONTROL_HALT_REQ_BIT = 0;
parameter int DBG_CONTROL_RESUME_REQ_BIT = 1;
parameter int DBG_CONTROL_STEP_REQ_BIT = 2;
parameter int DBG_CONTROL_CLEAR_TRAP_BIT = 3;
parameter int DBG_CONTROL_CLEAR_PASSFAIL_BIT = 4;
parameter int DBG_CONTROL_CLEAR_DEBUG_ERROR_BIT = 5;
parameter int DBG_CONTROL_RESET_RETIRE_COUNT_BIT = 6;
parameter int DBG_CONTROL_RESET_PERF_COUNTERS_BIT = 7;

// Hardware breakpoint control bit positions (DBG_REG_BP_CONTROL).
parameter int DBG_BP_CONTROL_ENABLE_BIT = 0;
parameter int DBG_BP_CONTROL_CLEAR_HIT_BIT = 1;  // write-only, self-clears
parameter int DBG_BP_CONTROL_HIT_BIT = 8;        // read-only, sticky

//------------------------------------------------------------------
// Memory map constants
//------------------------------------------------------------------

parameter logic [31:0] SCRATCH0_BASE = 32'h0000_0000;
parameter logic [31:0] SCRATCH0_END = 32'h0000_03FF;

parameter logic [31:0] SCRATCH1_BASE = 32'h0001_0000;
parameter logic [31:0] SCRATCH1_END = 32'h0001_03FF;

parameter logic [31:0] ACCEL_BASE = 32'h8000_0000;
parameter logic [31:0] ACCEL_END = 32'h8000_00FF;

parameter logic [31:0] DEBUG_BASE = 32'h9000_0000;
parameter logic [31:0] DEBUG_END = 32'h9000_00FF;

// Accelerator MMIO register addresses.
parameter logic [31:0] ACC_REG_CMD = ACCEL_BASE + 32'h00;
parameter logic [31:0] ACC_REG_STATUS = ACCEL_BASE + 32'h04;
parameter logic [31:0] ACC_REG_SRC_A = ACCEL_BASE + 32'h08;
parameter logic [31:0] ACC_REG_SRC_B = ACCEL_BASE + 32'h0C;
parameter logic [31:0] ACC_REG_SRC_C = ACCEL_BASE + 32'h10;
parameter logic [31:0] ACC_REG_LEN = ACCEL_BASE + 32'h14;
parameter logic [31:0] ACC_REG_DST = ACCEL_BASE + 32'h18;
parameter logic [31:0] ACC_REG_RESULT = ACCEL_BASE + 32'h1C;
parameter logic [31:0] ACC_REG_ERROR = ACCEL_BASE + 32'h20;

// Debug MMIO register addresses.
parameter logic [31:0] DBG_REG_STATUS = DEBUG_BASE + 32'h00;
parameter logic [31:0] DBG_REG_CONTROL = DEBUG_BASE + 32'h04;
parameter logic [31:0] DBG_REG_PC = DEBUG_BASE + 32'h08;
parameter logic [31:0] DBG_REG_REG_SELECT = DEBUG_BASE + 32'h0C;
parameter logic [31:0] DBG_REG_REG_DATA = DEBUG_BASE + 32'h10;
parameter logic [31:0] DBG_REG_PASSFAIL = DEBUG_BASE + 32'h14;
parameter logic [31:0] DBG_REG_TRAP_CAUSE = DEBUG_BASE + 32'h18;
parameter logic [31:0] DBG_REG_RETIRE_COUNT = DEBUG_BASE + 32'h1C;
// Hardware breakpoint + performance counter register addresses (this
// window was previously DBG_REG_SCR_ADDR/WDATA/RDATA/CONTROL, reserved
// stubs for a scratchpad peek/poke port that was never wired up -- repurposed
// 2026-07-29 into a real breakpoint compare register + hit flag, and free-
// running cycle/stall performance counters, all four backed by real
// registers in debug_regs.sv).
parameter logic [31:0] DBG_REG_BP_ADDR = DEBUG_BASE + 32'h20;
parameter logic [31:0] DBG_REG_BP_CONTROL = DEBUG_BASE + 32'h24;
parameter logic [31:0] DBG_REG_PERF_CYCLE_COUNT = DEBUG_BASE + 32'h28;
parameter logic [31:0] DBG_REG_PERF_STALL_COUNT = DEBUG_BASE + 32'h2C;
parameter logic [31:0] DBG_REG_ACCEL_STATUS = DEBUG_BASE + 32'h30;
parameter logic [31:0] DBG_REG_ACCEL_RESULT = DEBUG_BASE + 32'h34;

//------------------------------------------------------------------
// Utility functions
//------------------------------------------------------------------

function automatic logic is_word_aligned(input logic [31:0] addr);
is_word_aligned = (addr[1:0] == 2'b00);
endfunction

function automatic logic is_half_aligned(input logic [31:0] addr);
is_half_aligned = (addr[0] == 1'b0);
endfunction

function automatic logic is_scratch0_addr(input logic [31:0] addr);
is_scratch0_addr = (addr >= SCRATCH0_BASE) && (addr <= SCRATCH0_END);
endfunction

function automatic logic is_scratch1_addr(input logic [31:0] addr);
is_scratch1_addr = (addr >= SCRATCH1_BASE) && (addr <= SCRATCH1_END);
endfunction

function automatic logic is_accel_addr(input logic [31:0] addr);
is_accel_addr = (addr >= ACCEL_BASE) && (addr <= ACCEL_END);
endfunction

function automatic logic is_debug_addr(input logic [31:0] addr);
is_debug_addr = (addr >= DEBUG_BASE) && (addr <= DEBUG_END);
endfunction

function automatic logic is_mmio_addr(input logic [31:0] addr);
is_mmio_addr = is_accel_addr(addr) || is_debug_addr(addr);
endfunction

`endif // TINYGPU_PKG_SV

