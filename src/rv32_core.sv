`default_nettype none

//------------------------------------------------------------------------------
// rv32_core.sv - TinyGPU-RV32 RV32I Control Core
//------------------------------------------------------------------------------
// Project : TinyGPU-RV32
// Module : rv32_core
// Description : Simple in-order RV32I control core for TinyGPU-RV32.
//
// This is the first integrated CPU core version. It is intentionally simple and
// implementation-friendly:
// - one instruction in flight
// - valid/ready instruction fetch interface
// - valid/ready data load/store interface
// - RV32I base integer subset as defined in ISA.md
// - trap/halt behavior for unsupported, misaligned, or bus-error cases
// - debug halt/resume/step hooks
// - debug register read/write hooks
//
// Notes:
// - This core is self-contained for integration bring-up. The separately
// generated modules alu.sv/regfile.sv/imm_gen.sv/etc. can later replace
// internal logic if a more separated structural core is desired.
// - Accelerator access is not special here. It happens through normal loads
// and stores to the SoC data bus/MMIO address map.
//
// ASIC note:
// - Single clock domain.
// - Active-low async reset.
// - No generated clocks.
// - No unsynthesizable code in production paths.
//------------------------------------------------------------------------------

import tinygpu_pkg::*;

module rv32_core #(
parameter logic [31:0] RESET_VECTOR = RESET_VECTOR_DEFAULT,
parameter bit RESET_ALL_REGS = 1'b0
) (
input logic clk,
input logic rst_n,

// -------------------------------------------------------------------
// Instruction bus: valid/ready read-only interface
// -------------------------------------------------------------------
output logic imem_valid_o,
output logic [31:0] imem_addr_o,
input logic imem_ready_i,
input logic [31:0] imem_rdata_i,
input logic imem_error_i,

// -------------------------------------------------------------------
// Data bus: valid/ready load/store interface
// -------------------------------------------------------------------
output logic dmem_valid_o,
output logic dmem_we_o,
output logic [31:0] dmem_addr_o,
output logic [31:0] dmem_wdata_o,
output logic [3:0] dmem_wstrb_o,
input logic dmem_ready_i,
input logic [31:0] dmem_rdata_i,
input logic dmem_error_i,

// -------------------------------------------------------------------
// Debug control inputs
// -------------------------------------------------------------------
input logic dbg_halt_req_i,
input logic dbg_resume_req_i,
input logic dbg_step_req_i,

input logic dbg_pc_write_en_i,
input logic [31:0] dbg_pc_write_data_i,

input logic [4:0] dbg_reg_read_addr_i,
output logic [31:0] dbg_reg_read_data_o,

input logic dbg_reg_write_en_i,
input logic [4:0] dbg_reg_write_addr_i,
input logic [31:0] dbg_reg_write_data_i,

// -------------------------------------------------------------------
// Core status outputs
// -------------------------------------------------------------------
output logic cpu_halted_o,
output logic cpu_debug_halt_o,
output logic cpu_trap_o,
output trap_cause_e trap_cause_o,
output logic [31:0] pc_o,
output logic [31:0] retire_count_o,

// Optional lightweight trace/status
output logic instr_retire_o,
output logic [31:0] retired_pc_o,
output logic [31:0] retired_instr_o
);

// -------------------------------------------------------------------
// Architectural state
// -------------------------------------------------------------------

cpu_state_e state_q, state_d;
trap_cause_e trap_cause_q, trap_cause_d;

logic [31:0] pc_q, pc_d;
logic [31:0] instr_q, instr_d;
logic [31:0] instr_pc_q, instr_pc_d;
logic [31:0] retire_count_q, retire_count_d;

logic step_active_q, step_active_d;

// Register file storage. x0 is forced to zero.
logic [31:0] regs_q [31:0];

// Latched data bus request for MEM_WAIT.
logic [31:0] mem_addr_q, mem_addr_d;
logic [31:0] mem_wdata_q, mem_wdata_d;
logic [3:0] mem_wstrb_q, mem_wstrb_d;
logic mem_we_q, mem_we_d;
logic mem_is_load_q, mem_is_load_d;
logic [1:0] mem_size_q, mem_size_d;
logic mem_unsigned_q, mem_unsigned_d;
logic [4:0] mem_rd_q, mem_rd_d;
logic [31:0] mem_next_pc_q, mem_next_pc_d;

// Trace registers.
logic instr_retire_d;
logic [31:0] retired_pc_q, retired_pc_d;
logic [31:0] retired_instr_q, retired_instr_d;

integer i;

// -------------------------------------------------------------------
// Instruction fields
// -------------------------------------------------------------------

logic [6:0] opcode;
logic [4:0] rd;
logic [2:0] funct3;
logic [4:0] rs1;
logic [4:0] rs2;
logic [6:0] funct7;

assign opcode = instr_q[6:0];
assign rd = instr_q[11:7];
assign funct3 = instr_q[14:12];
assign rs1 = instr_q[19:15];
assign rs2 = instr_q[24:20];
assign funct7 = instr_q[31:25];

// Register read data.
logic [31:0] rs1_rdata;
logic [31:0] rs2_rdata;

assign rs1_rdata = (rs1 == 5'd0) ? 32'd0 : regs_q[rs1];
assign rs2_rdata = (rs2 == 5'd0) ? 32'd0 : regs_q[rs2];

assign dbg_reg_read_data_o = (dbg_reg_read_addr_i == 5'd0)
? 32'd0
: regs_q[dbg_reg_read_addr_i];

// -------------------------------------------------------------------
// Immediate helper functions
// -------------------------------------------------------------------

function automatic logic [31:0] imm_i(input logic [31:0] instr);
imm_i = {{20{instr[31]}}, instr[31:20]};
endfunction

function automatic logic [31:0] imm_s(input logic [31:0] instr);
imm_s = {{20{instr[31]}}, instr[31:25], instr[11:7]};
endfunction

function automatic logic [31:0] imm_b(input logic [31:0] instr);
imm_b = {{19{instr[31]}}, instr[31], instr[7], instr[30:25], instr[11:8], 1'b0};
endfunction

function automatic logic [31:0] imm_u(input logic [31:0] instr);
imm_u = {instr[31:12], 12'b0};
endfunction

function automatic logic [31:0] imm_j(input logic [31:0] instr);
imm_j = {{11{instr[31]}}, instr[31], instr[19:12], instr[20], instr[30:21], 1'b0};
endfunction

// -------------------------------------------------------------------
// ALU/data helper functions
// -------------------------------------------------------------------

function automatic logic [31:0] alu_exec(
input alu_op_e op,
input logic [31:0] a,
input logic [31:0] b
);
unique case (op)
ALU_ADD: alu_exec = a + b;
ALU_SUB: alu_exec = a - b;
ALU_AND: alu_exec = a & b;
ALU_OR: alu_exec = a | b;
ALU_XOR: alu_exec = a ^ b;
ALU_SLL: alu_exec = a << b[4:0];
ALU_SRL: alu_exec = a >> b[4:0];
ALU_SRA: alu_exec = $signed(a) >>> b[4:0];
ALU_SLT: alu_exec = ($signed(a) < $signed(b)) ? 32'd1 : 32'd0;
ALU_SLTU: alu_exec = (a < b) ? 32'd1 : 32'd0;
ALU_PASS_B: alu_exec = b;
default: alu_exec = 32'd0;
endcase
endfunction

function automatic logic [31:0] load_extend(
input logic [31:0] rdata,
input logic [31:0] addr,
input mem_size_e size,
input logic unsigned_load
);
logic [7:0] b;
logic [15:0] h;
begin
unique case (addr[1:0])
2'b00: b = rdata[7:0];
2'b01: b = rdata[15:8];
2'b10: b = rdata[23:16];
2'b11: b = rdata[31:24];
default: b = 8'd0;
endcase

h = (addr[1] == 1'b0) ? rdata[15:0] : rdata[31:16];

unique case (size)
MEM_BYTE: load_extend = unsigned_load ? {24'd0, b} : {{24{b[7]}}, b};
MEM_HALF: load_extend = unsigned_load ? {16'd0, h} : {{16{h[15]}}, h};
MEM_WORD: load_extend = rdata;
default: load_extend = 32'd0;
endcase
end
endfunction

function automatic logic access_misaligned(
input logic [31:0] addr,
input mem_size_e size
);
unique case (size)
MEM_BYTE: access_misaligned = 1'b0;
MEM_HALF: access_misaligned = addr[0];
MEM_WORD: access_misaligned = (addr[1:0] != 2'b00);
default: access_misaligned = 1'b1;
endcase
endfunction

// Store strobe/data generation happens in combinational decode variables.
logic [31:0] store_wdata_comb;
logic [3:0] store_wstrb_comb;

always_comb begin
store_wdata_comb = 32'd0;
store_wstrb_comb = 4'b0000;

unique case (mem_size_e'(mem_size_d))
MEM_BYTE: begin
unique case (mem_addr_d[1:0])
2'b00: begin store_wdata_comb = {24'd0, rs2_rdata[7:0]}; store_wstrb_comb =
'b0001; end
2'b01: begin store_wdata_comb = {16'd0, rs2_rdata[7:0], 8'd0}; store_wstrb_comb =
'b0010; end
2'b10: begin store_wdata_comb = {8'd0, rs2_rdata[7:0], 16'd0}; store_wstrb_comb =
'b0100; end
2'b11: begin store_wdata_comb = {rs2_rdata[7:0], 24'd0}; store_wstrb_comb =
'b1000; end
default: begin store_wdata_comb = 32'd0; store_wstrb_comb = 4'b0000; end
endcase
end

MEM_HALF: begin
if (mem_addr_d[1] == 1'b0) begin
store_wdata_comb = {16'd0, rs2_rdata[15:0]};
store_wstrb_comb = 4'b0011;
end else begin
store_wdata_comb = {rs2_rdata[15:0], 16'd0};
store_wstrb_comb = 4'b1100;
end
end

MEM_WORD: begin
store_wdata_comb = rs2_rdata;
store_wstrb_comb = 4'b1111;
end

default: begin
store_wdata_comb = 32'd0;
store_wstrb_comb = 4'b0000;
end
endcase
end

// -------------------------------------------------------------------
// Decode/execute combinational variables
// -------------------------------------------------------------------

logic commit_valid;
logic commit_reg_write;
logic [4:0] commit_rd;
logic [31:0] commit_wdata;
logic [31:0] commit_next_pc;
logic commit_trap;
trap_cause_e commit_trap_cause;

logic [31:0] alu_a;
logic [31:0] alu_b;
logic [31:0] alu_result;
logic [31:0] eff_addr;
logic [31:0] target_pc;
logic take_branch;

// -------------------------------------------------------------------
// Main next-state / decode logic
// -------------------------------------------------------------------

always_comb begin
// State hold defaults
state_d = state_q;
trap_cause_d = trap_cause_q;
pc_d = pc_q;
instr_d = instr_q;
instr_pc_d = instr_pc_q;
retire_count_d = retire_count_q;
step_active_d = step_active_q;

mem_addr_d = mem_addr_q;
mem_wdata_d = mem_wdata_q;
mem_wstrb_d = mem_wstrb_q;
mem_we_d = mem_we_q;
mem_is_load_d = mem_is_load_q;
mem_size_d = mem_size_q;
mem_unsigned_d = mem_unsigned_q;
mem_rd_d = mem_rd_q;
mem_next_pc_d = mem_next_pc_q;

instr_retire_d = 1'b0;
retired_pc_d = retired_pc_q;
retired_instr_d = retired_instr_q;

commit_valid = 1'b0;
commit_reg_write = 1'b0;
commit_rd = rd;
commit_wdata = 32'd0;
commit_next_pc = pc_q + 32'd4;
commit_trap = 1'b0;
commit_trap_cause = TRAP_NONE;

alu_a = rs1_rdata;
alu_b = rs2_rdata;
alu_result = 32'd0;
eff_addr = 32'd0;
target_pc = 32'd0;
take_branch = 1'b0;

unique case (state_q)
CPU_RESET: begin
state_d = CPU_FETCH;
end

CPU_FETCH: begin
if (dbg_halt_req_i) begin
state_d = CPU_DEBUG_HALT;
end else if (pc_q[1:0] != 2'b00) begin
commit_valid = 1'b1;
commit_trap = 1'b1;
commit_trap_cause = TRAP_INSTR_MISALIGNED;
end else if (imem_ready_i) begin
if (imem_error_i) begin
commit_valid = 1'b1;
commit_trap = 1'b1;
commit_trap_cause = TRAP_BUS_ERROR;
end else begin
instr_d = imem_rdata_i;
instr_pc_d = pc_q;
state_d = CPU_DECODE_EXEC;
end
end
end

CPU_DECODE_EXEC: begin
// Defaults for normal instructions.
commit_next_pc = instr_pc_q + 32'd4;

unique case (opcode)
OPCODE_LUI: begin
commit_valid = 1'b1;
commit_reg_write = 1'b1;
commit_rd = rd;
commit_wdata = imm_u(instr_q);
end

OPCODE_AUIPC: begin
commit_valid = 1'b1;
commit_reg_write = 1'b1;
commit_rd = rd;
commit_wdata = instr_pc_q + imm_u(instr_q);
end

OPCODE_JAL: begin
target_pc = instr_pc_q + imm_j(instr_q);
commit_valid = 1'b1;
if (target_pc[1:0] != 2'b00) begin
commit_trap = 1'b1;
commit_trap_cause = TRAP_INSTR_MISALIGNED;
end else begin
commit_reg_write = 1'b1;
commit_rd = rd;
commit_wdata = instr_pc_q + 32'd4;
commit_next_pc = target_pc;
end
end

OPCODE_JALR: begin
if (funct3 != 3'b000) begin
commit_valid = 1'b1;
commit_trap = 1'b1;
commit_trap_cause = TRAP_ILLEGAL_INSTR;
end else begin
target_pc = (rs1_rdata + imm_i(instr_q)) & 32'hFFFF_FFFE;
commit_valid = 1'b1;
if (target_pc[1:0] != 2'b00) begin
commit_trap = 1'b1;
commit_trap_cause = TRAP_INSTR_MISALIGNED;
end else begin
commit_reg_write = 1'b1;
commit_rd = rd;
commit_wdata = instr_pc_q + 32'd4;
commit_next_pc = target_pc;
end
end
end

OPCODE_BRANCH: begin
unique case (funct3)
3'b000: take_branch = (rs1_rdata == rs2_rdata);
3'b001: take_branch = (rs1_rdata != rs2_rdata);
3'b100: take_branch = ($signed(rs1_rdata) < $signed(rs2_rdata));
3'b101: take_branch = ($signed(rs1_rdata) >= $signed(rs2_rdata));
3'b110: take_branch = (rs1_rdata < rs2_rdata);
3'b111: take_branch = (rs1_rdata >= rs2_rdata);
default: take_branch = 1'b0;
endcase

commit_valid = 1'b1;

if (!((funct3 == 3'b000) || (funct3 == 3'b001) ||
(funct3 == 3'b100) || (funct3 == 3'b101) ||
(funct3 == 3'b110) || (funct3 == 3'b111))) begin
commit_trap = 1'b1;
commit_trap_cause = TRAP_ILLEGAL_INSTR;
end else if (take_branch) begin
target_pc = instr_pc_q + imm_b(instr_q);
if (target_pc[1:0] != 2'b00) begin
commit_trap = 1'b1;
commit_trap_cause = TRAP_INSTR_MISALIGNED;
end else begin
commit_next_pc = target_pc;
end
end
end

OPCODE_LOAD: begin
eff_addr = rs1_rdata + imm_i(instr_q);

mem_addr_d = eff_addr;
mem_we_d = 1'b0;
mem_is_load_d = 1'b1;
mem_unsigned_d = 1'b0;
mem_rd_d = rd;
mem_next_pc_d = instr_pc_q + 32'd4;

unique case (funct3)
3'b000: begin mem_size_d = MEM_BYTE; mem_unsigned_d = 1'b0; end // LB
3'b001: begin mem_size_d = MEM_HALF; mem_unsigned_d = 1'b0; end // LH
3'b010: begin mem_size_d = MEM_WORD; mem_unsigned_d = 1'b0; end // LW
3'b100: begin mem_size_d = MEM_BYTE; mem_unsigned_d = 1'b1; end // LBU
3'b101: begin mem_size_d = MEM_HALF; mem_unsigned_d = 1'b1; end // LHU
default: begin mem_size_d = MEM_WORD; end
endcase

if (!((funct3 == 3'b000) || (funct3 == 3'b001) ||
(funct3 == 3'b010) || (funct3 == 3'b100) ||
(funct3 == 3'b101))) begin
commit_valid = 1'b1;
commit_trap = 1'b1;
commit_trap_cause = TRAP_ILLEGAL_INSTR;
end else if (access_misaligned(eff_addr, mem_size_e'(mem_size_d))) begin
commit_valid = 1'b1;
commit_trap = 1'b1;
commit_trap_cause = TRAP_LOAD_MISALIGNED;
end else begin
state_d = CPU_MEM_WAIT;
end
end

OPCODE_STORE: begin
eff_addr = rs1_rdata + imm_s(instr_q);

mem_addr_d = eff_addr;
mem_we_d = 1'b1;
mem_is_load_d = 1'b0;
mem_unsigned_d = 1'b0;
mem_rd_d = 5'd0;
mem_next_pc_d = instr_pc_q + 32'd4;

unique case (funct3)
3'b000: mem_size_d = MEM_BYTE; // SB
3'b001: mem_size_d = MEM_HALF; // SH
3'b010: mem_size_d = MEM_WORD; // SW
default: mem_size_d = MEM_WORD;
endcase

// store_wdata_comb uses mem_addr_d/mem_size_d/rs2_rdata.
mem_wdata_d = store_wdata_comb;
mem_wstrb_d = store_wstrb_comb;

if (!((funct3 == 3'b000) || (funct3 == 3'b001) || (funct3 == 3'b010))) begin
commit_valid = 1'b1;
commit_trap = 1'b1;
commit_trap_cause = TRAP_ILLEGAL_INSTR;
end else if (access_misaligned(eff_addr, mem_size_e'(mem_size_d))) begin
commit_valid = 1'b1;
commit_trap = 1'b1;
commit_trap_cause = TRAP_STORE_MISALIGNED;
end else begin
state_d = CPU_MEM_WAIT;
end
end

OPCODE_OP_IMM: begin
commit_valid = 1'b1;
commit_reg_write = 1'b1;
commit_rd = rd;
alu_a = rs1_rdata;
alu_b = imm_i(instr_q);

unique case (funct3)
3'b000: alu_result = alu_exec(ALU_ADD, alu_a, alu_b); // ADDI
3'b010: alu_result = alu_exec(ALU_SLT, alu_a, alu_b); // SLTI
3'b011: alu_result = alu_exec(ALU_SLTU, alu_a, alu_b); // SLTIU
3'b100: alu_result = alu_exec(ALU_XOR, alu_a, alu_b); // XORI
3'b110: alu_result = alu_exec(ALU_OR, alu_a, alu_b); // ORI
3'b111: alu_result = alu_exec(ALU_AND, alu_a, alu_b); // ANDI
3'b001: begin
if (funct7 == 7'b0000000) alu_result = alu_exec(ALU_SLL, alu_a, alu_b);
else begin commit_trap = 1'b1; commit_trap_cause = TRAP_ILLEGAL_INSTR; commit_reg_write = 1'b0; end
end
3'b101: begin
if (funct7 == 7'b0000000) alu_result = alu_exec(ALU_SRL, alu_a, alu_b);
else if (funct7 == 7'b0100000) alu_result = alu_exec(ALU_SRA, alu_a, alu_b);
else begin commit_trap = 1'b1; commit_trap_cause = TRAP_ILLEGAL_INSTR; commit_reg_write = 1'b0; end
end
default: begin
commit_trap = 1'b1;
commit_trap_cause = TRAP_ILLEGAL_INSTR;
commit_reg_write = 1'b0;
end
endcase
commit_wdata = alu_result;
end

OPCODE_OP: begin
commit_valid = 1'b1;
commit_reg_write = 1'b1;
commit_rd = rd;
alu_a = rs1_rdata;
alu_b = rs2_rdata;

unique case (funct3)
3'b000: begin
if (funct7 == 7'b0000000) alu_result = alu_exec(ALU_ADD, alu_a, alu_b);
else if (funct7 == 7'b0100000) alu_result = alu_exec(ALU_SUB, alu_a, alu_b);
else begin commit_trap = 1'b1; commit_trap_cause = TRAP_ILLEGAL_INSTR; commit_reg_write = 1'b0; end
end
3'b001: begin
if (funct7 == 7'b0000000) alu_result = alu_exec(ALU_SLL, alu_a, alu_b);
else begin commit_trap = 1'b1; commit_trap_cause = TRAP_ILLEGAL_INSTR; commit_reg_write = 1'b0; end
end
3'b010: begin
if (funct7 == 7'b0000000) alu_result = alu_exec(ALU_SLT, alu_a, alu_b);
else begin commit_trap = 1'b1; commit_trap_cause = TRAP_ILLEGAL_INSTR; commit_reg_write = 1'b0; end
end
3'b011: begin
if (funct7 == 7'b0000000) alu_result = alu_exec(ALU_SLTU, alu_a, alu_b);
else begin commit_trap = 1'b1; commit_trap_cause = TRAP_ILLEGAL_INSTR; commit_reg_write = 1'b0; end
end
3'b100: begin
if (funct7 == 7'b0000000) alu_result = alu_exec(ALU_XOR, alu_a, alu_b);
else begin commit_trap = 1'b1; commit_trap_cause = TRAP_ILLEGAL_INSTR; commit_reg_write = 1'b0; end
end
3'b101: begin
if (funct7 == 7'b0000000) alu_result = alu_exec(ALU_SRL, alu_a, alu_b);
else if (funct7 == 7'b0100000) alu_result = alu_exec(ALU_SRA, alu_a, alu_b);
else begin commit_trap = 1'b1; commit_trap_cause = TRAP_ILLEGAL_INSTR; commit_reg_write = 1'b0; end
end
3'b110: begin
if (funct7 == 7'b0000000) alu_result = alu_exec(ALU_OR, alu_a, alu_b);
else begin commit_trap = 1'b1; commit_trap_cause = TRAP_ILLEGAL_INSTR; commit_reg_write = 1'b0; end
end
3'b111: begin
if (funct7 == 7'b0000000) alu_result = alu_exec(ALU_AND, alu_a, alu_b);
else begin commit_trap = 1'b1; commit_trap_cause = TRAP_ILLEGAL_INSTR; commit_reg_write = 1'b0; end
end
default: begin
commit_trap = 1'b1;
commit_trap_cause = TRAP_ILLEGAL_INSTR;
commit_reg_write = 1'b0;
end
endcase
commit_wdata = alu_result;
end

OPCODE_FENCE: begin
// FENCE is treated as NOP in TinyGPU-RV32 v1.
commit_valid = 1'b1;
end

OPCODE_SYSTEM: begin
commit_valid = 1'b1;
if (instr_q == INSTR_ECALL) begin
commit_trap = 1'b1;
commit_trap_cause = TRAP_ECALL;
end else if (instr_q == INSTR_EBREAK) begin
commit_trap = 1'b1;
commit_trap_cause = TRAP_EBREAK;
end else begin
commit_trap = 1'b1;
commit_trap_cause = TRAP_ILLEGAL_INSTR;
end
end

default: begin
commit_valid = 1'b1;
commit_trap = 1'b1;
commit_trap_cause = TRAP_ILLEGAL_INSTR;
end
endcase
end

CPU_MEM_WAIT: begin
if (dmem_ready_i) begin
commit_valid = 1'b1;
commit_next_pc = mem_next_pc_q;

if (dmem_error_i) begin
commit_trap = 1'b1;
commit_trap_cause = TRAP_BUS_ERROR;
end else if (mem_is_load_q) begin
commit_reg_write = 1'b1;
commit_rd = mem_rd_q;
commit_wdata = load_extend(dmem_rdata_i, mem_addr_q, mem_size_e'(mem_size_q), mem_unsigned_q);
end
end
end

CPU_TRAP: begin
if (dbg_pc_write_en_i && dbg_pc_write_data_i[1:0] == 2'b00) begin
pc_d = dbg_pc_write_data_i;
end
if (dbg_resume_req_i) begin
trap_cause_d = TRAP_NONE;
state_d = CPU_FETCH;
end else if (dbg_step_req_i) begin
trap_cause_d = TRAP_NONE;
step_active_d = 1'b1;
state_d = CPU_FETCH;
end
end

CPU_DEBUG_HALT: begin
if (dbg_pc_write_en_i && dbg_pc_write_data_i[1:0] == 2'b00) begin
pc_d = dbg_pc_write_data_i;
end
if (dbg_resume_req_i) begin
step_active_d = 1'b0;
state_d = CPU_FETCH;
end else if (dbg_step_req_i) begin
step_active_d = 1'b1;
state_d = CPU_FETCH;
end
end

default: begin
state_d = CPU_RESET;
end
endcase

// Central commit handling affects next architectural state.
if (commit_valid) begin
if (commit_trap) begin
trap_cause_d = commit_trap_cause;
state_d = (commit_trap_cause == TRAP_EBREAK) ? CPU_DEBUG_HALT : CPU_TRAP;
end else begin
pc_d = commit_next_pc;
retire_count_d = retire_count_q + 32'd1;
instr_retire_d = 1'b1;
retired_pc_d = instr_pc_q;
retired_instr_d = instr_q;

if (step_active_q || dbg_halt_req_i) begin
step_active_d = 1'b0;
state_d = CPU_DEBUG_HALT;
end else begin
state_d = CPU_FETCH;
end
end
end
end

// -------------------------------------------------------------------
// Sequential architectural state update
// -------------------------------------------------------------------

always_ff @(posedge clk or negedge rst_n) begin
if (!rst_n) begin
state_q <= CPU_RESET;
trap_cause_q <= TRAP_NONE;
pc_q <= RESET_VECTOR;
instr_q <= 32'd0;
instr_pc_q <= RESET_VECTOR;
retire_count_q <= 32'd0;
step_active_q <= 1'b0;

mem_addr_q <= 32'd0;
mem_wdata_q <= 32'd0;
mem_wstrb_q <= 4'd0;
mem_we_q <= 1'b0;
mem_is_load_q <= 1'b0;
mem_size_q <= MEM_WORD;
mem_unsigned_q <= 1'b0;
mem_rd_q <= 5'd0;
mem_next_pc_q <= RESET_VECTOR;

instr_retire_o <= 1'b0;
retired_pc_q <= 32'd0;
retired_instr_q <= 32'd0;

regs_q[0] <= 32'd0;
if (RESET_ALL_REGS) begin
for (i = 1; i < 32; i = i + 1) begin
regs_q[i] <= 32'd0;
end
end
end else begin
state_q <= state_d;
trap_cause_q <= trap_cause_d;
pc_q <= pc_d;
instr_q <= instr_d;
instr_pc_q <= instr_pc_d;
retire_count_q <= retire_count_d;
step_active_q <= step_active_d;

mem_addr_q <= mem_addr_d;
mem_wdata_q <= mem_wdata_d;
mem_wstrb_q <= mem_wstrb_d;
mem_we_q <= mem_we_d;
mem_is_load_q <= mem_is_load_d;
mem_size_q <= mem_size_d;
mem_unsigned_q <= mem_unsigned_d;
mem_rd_q <= mem_rd_d;
mem_next_pc_q <= mem_next_pc_d;

instr_retire_o <= instr_retire_d;
retired_pc_q <= retired_pc_d;
retired_instr_q <= retired_instr_d;

regs_q[0] <= 32'd0;

// Debug register write is allowed during halted states only.
if ((state_q == CPU_DEBUG_HALT || state_q == CPU_TRAP) &&
dbg_reg_write_en_i && dbg_reg_write_addr_i != 5'd0) begin
regs_q[dbg_reg_write_addr_i] <= dbg_reg_write_data_i;
end

// Architectural writeback on commit. This has priority over debug.
if (commit_valid && !commit_trap && commit_reg_write && commit_rd != 5'd0) begin
regs_q[commit_rd] <= commit_wdata;
end
end
end

// -------------------------------------------------------------------
// External bus outputs
// -------------------------------------------------------------------

always_comb begin
imem_valid_o = (state_q == CPU_FETCH) && (pc_q[1:0] == 2'b00) && !dbg_halt_req_i;
imem_addr_o = pc_q;

dmem_valid_o = (state_q == CPU_MEM_WAIT);
dmem_we_o = mem_we_q;
dmem_addr_o = {mem_addr_q[31:2], 2'b00};
dmem_wdata_o = mem_wdata_q;
dmem_wstrb_o = mem_we_q ? mem_wstrb_q : 4'b0000;
end

// -------------------------------------------------------------------
// Status outputs
// -------------------------------------------------------------------

assign cpu_debug_halt_o = (state_q == CPU_DEBUG_HALT);
assign cpu_trap_o = (state_q == CPU_TRAP);
assign cpu_halted_o = (state_q == CPU_DEBUG_HALT) || (state_q == CPU_TRAP);
assign trap_cause_o = trap_cause_q;
assign pc_o = pc_q;
assign retire_count_o = retire_count_q;
assign retired_pc_o = retired_pc_q;
assign retired_instr_o = retired_instr_q;

`ifdef ASSERT_ON
// -------------------------------------------------------------------
// Assertions
// -------------------------------------------------------------------

// x0 must always remain zero.
property p_x0_zero;
@(posedge clk) disable iff (!rst_n)
regs_q[0] == 32'd0;
endproperty
assert property (p_x0_zero);

// Instruction fetch address is aligned whenever valid.
property p_imem_aligned_when_valid;
@(posedge clk) disable iff (!rst_n)
imem_valid_o |-> imem_addr_o[1:0] == 2'b00;
endproperty
assert property (p_imem_aligned_when_valid);

// Data bus address is word aligned whenever valid.
property p_dmem_aligned_when_valid;
@(posedge clk) disable iff (!rst_n)
dmem_valid_o |-> dmem_addr_o[1:0] == 2'b00;
endproperty
assert property (p_dmem_aligned_when_valid);

// No memory write strobes during a data-bus read.
property p_no_wstrb_on_read;
@(posedge clk) disable iff (!rst_n)
dmem_valid_o && !dmem_we_o |-> dmem_wstrb_o == 4'b0000;
endproperty
assert property (p_no_wstrb_on_read);

// Retire pulse should not occur while halted or trapped.
property p_no_retire_while_halted;
@(posedge clk) disable iff (!rst_n)
cpu_halted_o |-> !instr_retire_o;
endproperty
assert property (p_no_retire_while_halted);

// Valid instruction retirement increments retire count by one.
property p_retire_count_increments;
@(posedge clk) disable iff (!rst_n)
instr_retire_o |-> retire_count_o == $past(retire_count_o) + 32'd1;
endproperty
assert property (p_retire_count_increments);

// Debug writes must never modify x0.
property p_debug_x0_write_ignored;
@(posedge clk) disable iff (!rst_n)
dbg_reg_write_en_i && dbg_reg_write_addr_i == 5'd0 |=> regs_q[0] == 32'd0;
endproperty
assert property (p_debug_x0_write_ignored);
`endif

endmodule

`default_nettype wire

