`default_nettype none

//------------------------------------------------------------------------------
// rv32_core.sv - TinyGPU-RV32 RV32I Control Core
// Description : Simple in-order RV32I control core for TinyGPU-RV32.
//------------------------------------------------------------------------------

`include "tinygpu_pkg.sv"

module rv32_core #(
parameter logic [31:0] RESET_VECTOR = RESET_VECTOR_DEFAULT,
parameter bit RESET_ALL_REGS = 1'b0
) (
input logic clk,
input logic rst_n,

output logic imem_valid_o,
output logic [31:0] imem_addr_o,
input logic imem_ready_i,
input logic [31:0] imem_rdata_i,
input logic imem_error_i,

output logic dmem_valid_o,
output logic dmem_we_o,
output logic [31:0] dmem_addr_o,
output logic [31:0] dmem_wdata_o,
output logic [3:0] dmem_wstrb_o,
input logic dmem_ready_i,
input logic [31:0] dmem_rdata_i,
input logic dmem_error_i,

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

output logic cpu_halted_o,
output logic cpu_debug_halt_o,
output logic cpu_trap_o,
output trap_cause_e trap_cause_o,
output logic [31:0] pc_o,
output logic [31:0] retire_count_o,

output logic instr_retire_o,
output logic [31:0] retired_pc_o,
output logic [31:0] retired_instr_o
);

cpu_state_e state_q, state_d;
trap_cause_e trap_cause_q, trap_cause_d;

logic [31:0] pc_q, pc_d;
logic [31:0] instr_q, instr_d;
logic [31:0] instr_pc_q, instr_pc_d;
logic [31:0] retire_count_q, retire_count_d;

logic step_active_q, step_active_d;

logic mstatus_mie_q, mstatus_mie_d;
logic mstatus_mpie_q, mstatus_mpie_d;
logic [31:0] mie_q, mie_d;
logic [31:0] mip_q, mip_d;
logic [31:0] mtvec_q, mtvec_d;
logic [31:0] mstatush_q, mstatush_d;
logic [31:0] mscratch_q, mscratch_d;
logic [31:0] mepc_q, mepc_d;
logic [31:0] mcause_q, mcause_d;
logic [31:0] mtval_q, mtval_d;

logic [31:0] regs_q [15:0];

logic [31:0] mem_addr_q, mem_addr_d;
logic [31:0] mem_wdata_q, mem_wdata_d;
logic [3:0] mem_wstrb_q, mem_wstrb_d;
logic mem_we_q, mem_we_d;
logic mem_is_load_q, mem_is_load_d;
mem_size_e mem_size_q, mem_size_d;
logic mem_unsigned_q, mem_unsigned_d;
logic [4:0] mem_rd_q, mem_rd_d;
logic [31:0] mem_next_pc_q, mem_next_pc_d;

logic instr_retire_d;
logic [31:0] retired_pc_q, retired_pc_d;
logic [31:0] retired_instr_q, retired_instr_d;

integer i;

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

logic [31:0] rs1_rdata;
logic [31:0] rs2_rdata;

assign rs1_rdata = (rs1 == 5'd0) ? 32'd0 : regs_q[rs1[3:0]];
assign rs2_rdata = (rs2 == 5'd0) ? 32'd0 : regs_q[rs2[3:0]];

assign dbg_reg_read_data_o = (dbg_reg_read_addr_i == 5'd0)
? 32'd0
: regs_q[dbg_reg_read_addr_i[3:0]];

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

function automatic logic [31:0] csr_read(input logic [11:0] addr);
logic [31:0] rdata;
begin
unique case (addr)
CSR_MSTATUS: begin
rdata = 32'd0;
rdata[MSTATUS_MIE_BIT] = mstatus_mie_q;
rdata[MSTATUS_MPIE_BIT] = mstatus_mpie_q;
rdata[12:11] = 2'b11;
end
CSR_MISA: rdata = MISA_VALUE;
CSR_MIE: rdata = mie_q;
CSR_MTVEC: rdata = mtvec_q;
CSR_MSTATUSH: rdata = mstatush_q;
CSR_MSCRATCH: rdata = mscratch_q;
CSR_MEPC: rdata = mepc_q;
CSR_MCAUSE: rdata = mcause_q;
CSR_MTVAL: rdata = mtval_q;
CSR_MIP: rdata = mip_q;
CSR_MVENDORID, CSR_MARCHID, CSR_MIMPID, CSR_MHARTID: rdata = 32'd0;
default: rdata = 32'd0;
endcase
csr_read = rdata;
end
endfunction

function automatic logic csr_addr_valid(input logic [11:0] addr);
unique case (addr)
CSR_MSTATUS, CSR_MISA, CSR_MIE, CSR_MTVEC, CSR_MSTATUSH, CSR_MSCRATCH,
CSR_MEPC, CSR_MCAUSE, CSR_MTVAL, CSR_MIP,
CSR_MVENDORID, CSR_MARCHID, CSR_MIMPID, CSR_MHARTID: csr_addr_valid = 1'b1;
default: csr_addr_valid = 1'b0;
endcase
endfunction

function automatic logic [3:0] trap_to_mcause(
input trap_cause_e cause,
input logic in_fetch_state,
input logic mem_write
);
unique case (cause)
TRAP_INSTR_MISALIGNED: trap_to_mcause = 4'd0;
TRAP_ILLEGAL_INSTR: trap_to_mcause = 4'd2;
TRAP_EBREAK: trap_to_mcause = 4'd3;
TRAP_LOAD_MISALIGNED: trap_to_mcause = 4'd4;
TRAP_STORE_MISALIGNED: trap_to_mcause = 4'd6;
TRAP_ECALL: trap_to_mcause = 4'd11;
TRAP_BUS_ERROR: begin
if (in_fetch_state) trap_to_mcause = 4'd1;
else if (mem_write) trap_to_mcause = 4'd7;
else trap_to_mcause = 4'd5;
end
default: trap_to_mcause = 4'd2;
endcase
endfunction

logic commit_valid;
logic commit_reg_write;
logic [4:0] commit_rd;
logic [31:0] commit_wdata;
logic [31:0] commit_next_pc;
logic commit_trap;
trap_cause_e commit_trap_cause;
logic [31:0] commit_trap_pc;
logic [31:0] commit_trap_val;
logic commit_csr_write;
logic [11:0] commit_csr_addr;
logic [31:0] commit_csr_wdata;
logic commit_mret;

logic [31:0] alu_a;
logic [31:0] alu_b;
logic [31:0] alu_result;
logic [31:0] eff_addr;
logic [31:0] target_pc;
logic take_branch;

always_comb begin
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

mstatus_mie_d = mstatus_mie_q;
mstatus_mpie_d = mstatus_mpie_q;
mie_d = mie_q;
mip_d = mip_q;
mtvec_d = mtvec_q;
mstatush_d = mstatush_q;
mscratch_d = mscratch_q;
mepc_d = mepc_q;
mcause_d = mcause_q;
mtval_d = mtval_q;

commit_valid = 1'b0;
commit_reg_write = 1'b0;
commit_rd = rd;
commit_wdata = 32'd0;
commit_next_pc = pc_q + 32'd4;
commit_trap = 1'b0;
commit_trap_cause = TRAP_NONE;
commit_trap_pc = instr_pc_q;
commit_trap_val = 32'd0;
commit_csr_write = 1'b0;
commit_csr_addr = 12'd0;
commit_csr_wdata = 32'd0;
commit_mret = 1'b0;

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
commit_trap_pc = pc_q;
commit_trap_val = pc_q;
end else if (imem_ready_i) begin
if (imem_error_i) begin
commit_valid = 1'b1;
commit_trap = 1'b1;
commit_trap_cause = TRAP_BUS_ERROR;
commit_trap_pc = pc_q;
commit_trap_val = pc_q;
end else begin
instr_d = imem_rdata_i;
instr_pc_d = pc_q;
state_d = CPU_DECODE_EXEC;
end
end
end

CPU_DECODE_EXEC: begin
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
3'b000: begin mem_size_d = MEM_BYTE; mem_unsigned_d = 1'b0; end
3'b001: begin mem_size_d = MEM_HALF; mem_unsigned_d = 1'b0; end
3'b010: begin mem_size_d = MEM_WORD; mem_unsigned_d = 1'b0; end
3'b100: begin mem_size_d = MEM_BYTE; mem_unsigned_d = 1'b1; end
3'b101: begin mem_size_d = MEM_HALF; mem_unsigned_d = 1'b1; end
default: begin mem_size_d = MEM_WORD; end
endcase

if (!((funct3 == 3'b000) || (funct3 == 3'b001) ||
(funct3 == 3'b010) || (funct3 == 3'b100) ||
(funct3 == 3'b101))) begin
commit_valid = 1'b1;
commit_trap = 1'b1;
commit_trap_cause = TRAP_ILLEGAL_INSTR;
end else if (access_misaligned(eff_addr, mem_size_d)) begin
commit_valid = 1'b1;
commit_trap = 1'b1;
commit_trap_cause = TRAP_LOAD_MISALIGNED;
commit_trap_val = eff_addr;
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
3'b000: mem_size_d = MEM_BYTE;
3'b001: mem_size_d = MEM_HALF;
3'b010: mem_size_d = MEM_WORD;
default: mem_size_d = MEM_WORD;
endcase

unique case (mem_size_d)
MEM_BYTE: begin
unique case (mem_addr_d[1:0])
2'b00: begin mem_wdata_d = {24'd0, rs2_rdata[7:0]}; mem_wstrb_d = 4'b0001; end
2'b01: begin mem_wdata_d = {16'd0, rs2_rdata[7:0], 8'd0}; mem_wstrb_d = 4'b0010; end
2'b10: begin mem_wdata_d = {8'd0, rs2_rdata[7:0], 16'd0}; mem_wstrb_d = 4'b0100; end
2'b11: begin mem_wdata_d = {rs2_rdata[7:0], 24'd0}; mem_wstrb_d = 4'b1000; end
default: begin mem_wdata_d = 32'd0; mem_wstrb_d = 4'b0000; end
endcase
end
MEM_HALF: begin
if (mem_addr_d[1] == 1'b0) begin
mem_wdata_d = {16'd0, rs2_rdata[15:0]};
mem_wstrb_d = 4'b0011;
end else begin
mem_wdata_d = {rs2_rdata[15:0], 16'd0};
mem_wstrb_d = 4'b1100;
end
end
MEM_WORD: begin
mem_wdata_d = rs2_rdata;
mem_wstrb_d = 4'b1111;
end
default: begin
mem_wdata_d = 32'd0;
mem_wstrb_d = 4'b0000;
end
endcase

if (!((funct3 == 3'b000) || (funct3 == 3'b001) || (funct3 == 3'b010))) begin
commit_valid = 1'b1;
commit_trap = 1'b1;
commit_trap_cause = TRAP_ILLEGAL_INSTR;
end else if (access_misaligned(eff_addr, mem_size_d)) begin
commit_valid = 1'b1;
commit_trap = 1'b1;
commit_trap_cause = TRAP_STORE_MISALIGNED;
commit_trap_val = eff_addr;
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
3'b000: alu_result = alu_exec(ALU_ADD, alu_a, alu_b);
3'b010: alu_result = alu_exec(ALU_SLT, alu_a, alu_b);
3'b011: alu_result = alu_exec(ALU_SLTU, alu_a, alu_b);
3'b100: alu_result = alu_exec(ALU_XOR, alu_a, alu_b);
3'b110: alu_result = alu_exec(ALU_OR, alu_a, alu_b);
3'b111: alu_result = alu_exec(ALU_AND, alu_a, alu_b);
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
commit_valid = 1'b1;
end

OPCODE_SYSTEM: begin
commit_valid = 1'b1;
if (funct3 == 3'b000) begin
if (instr_q == INSTR_ECALL) begin
commit_trap = 1'b1;
commit_trap_cause = TRAP_ECALL;
end else if (instr_q == INSTR_EBREAK) begin
commit_trap = 1'b1;
commit_trap_cause = TRAP_EBREAK;
end else if (instr_q == INSTR_MRET) begin
commit_mret = 1'b1;
end else begin
commit_trap = 1'b1;
commit_trap_cause = TRAP_ILLEGAL_INSTR;
end
end else if (csr_addr_valid(instr_q[31:20])) begin
commit_reg_write = 1'b1;
commit_wdata = csr_read(instr_q[31:20]);
commit_csr_write = 1'b1;
commit_csr_addr = instr_q[31:20];
unique case (funct3)
3'b001: commit_csr_wdata = rs1_rdata;
3'b010: commit_csr_wdata = commit_wdata | rs1_rdata;
3'b011: commit_csr_wdata = commit_wdata & ~rs1_rdata;
3'b101: commit_csr_wdata = {27'd0, rs1};
3'b110: commit_csr_wdata = commit_wdata | {27'd0, rs1};
3'b111: commit_csr_wdata = commit_wdata & ~{27'd0, rs1};
default: commit_csr_wdata = commit_wdata;
endcase
if ((funct3 == 3'b010 || funct3 == 3'b011 ||
funct3 == 3'b110 || funct3 == 3'b111) && rs1 == 5'd0) begin
commit_csr_write = 1'b0;
end
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
commit_trap_val = mem_addr_q;
end else if (mem_is_load_q) begin
commit_reg_write = 1'b1;
commit_rd = mem_rd_q;
commit_wdata = load_extend(dmem_rdata_i, mem_addr_q, mem_size_q, mem_unsigned_q);
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

if (commit_valid) begin
if (commit_trap) begin
trap_cause_d = commit_trap_cause;
mepc_d = commit_trap_pc;
mcause_d = {28'd0, trap_to_mcause(commit_trap_cause, (state_q == CPU_FETCH), mem_we_q)};
mtval_d = commit_trap_val;
if (commit_trap_cause == TRAP_EBREAK) begin
state_d = CPU_DEBUG_HALT;
end else if (mtvec_q != 32'd0) begin
mstatus_mpie_d = mstatus_mie_q;
mstatus_mie_d = 1'b0;
pc_d = {mtvec_q[31:2], 2'b00};
state_d = CPU_FETCH;
end else begin
state_d = CPU_TRAP;
end
end else begin
pc_d = commit_next_pc;
retire_count_d = retire_count_q + 32'd1;
instr_retire_d = 1'b1;
retired_pc_d = instr_pc_q;
retired_instr_d = instr_q;

if (commit_csr_write) begin
unique case (commit_csr_addr)
CSR_MSTATUS: begin
mstatus_mie_d = commit_csr_wdata[MSTATUS_MIE_BIT];
mstatus_mpie_d = commit_csr_wdata[MSTATUS_MPIE_BIT];
end
CSR_MIE: mie_d = commit_csr_wdata;
CSR_MTVEC: mtvec_d = commit_csr_wdata;
CSR_MSTATUSH: mstatush_d = commit_csr_wdata;
CSR_MSCRATCH: mscratch_d = commit_csr_wdata;
CSR_MEPC: mepc_d = commit_csr_wdata;
CSR_MCAUSE: mcause_d = commit_csr_wdata;
CSR_MTVAL: mtval_d = commit_csr_wdata;
CSR_MIP: mip_d = commit_csr_wdata;
default: ;
endcase
end

if (commit_mret) begin
pc_d = mepc_q;
mstatus_mie_d = mstatus_mpie_q;
mstatus_mpie_d = 1'b1;
end

if (step_active_q || dbg_halt_req_i) begin
step_active_d = 1'b0;
state_d = CPU_DEBUG_HALT;
end else begin
state_d = CPU_FETCH;
end
end
end
end

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
for (i = 1; i < 16; i = i + 1) begin
regs_q[i] <= 32'd0;
end
end

mstatus_mie_q <= 1'b0;
mstatus_mpie_q <= 1'b0;
mie_q <= 32'd0;
mip_q <= 32'd0;
mtvec_q <= 32'd0;
mstatush_q <= 32'd0;
mscratch_q <= 32'd0;
mepc_q <= 32'd0;
mcause_q <= 32'd0;
mtval_q <= 32'd0;
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

mstatus_mie_q <= mstatus_mie_d;
mstatus_mpie_q <= mstatus_mpie_d;
mie_q <= mie_d;
mip_q <= mip_d;
mtvec_q <= mtvec_d;
mstatush_q <= mstatush_d;
mscratch_q <= mscratch_d;
mepc_q <= mepc_d;
mcause_q <= mcause_d;
mtval_q <= mtval_d;

regs_q[0] <= 32'd0;

if ((state_q == CPU_DEBUG_HALT || state_q == CPU_TRAP) &&
dbg_reg_write_en_i && dbg_reg_write_addr_i != 5'd0) begin
regs_q[dbg_reg_write_addr_i[3:0]] <= dbg_reg_write_data_i;
end

if (commit_valid && !commit_trap && commit_reg_write && commit_rd != 5'd0) begin
regs_q[commit_rd[3:0]] <= commit_wdata;
end
end
end

always_comb begin
imem_valid_o = (state_q == CPU_FETCH) && (pc_q[1:0] == 2'b00) && !dbg_halt_req_i;
imem_addr_o = pc_q;

dmem_valid_o = (state_q == CPU_MEM_WAIT);
dmem_we_o = mem_we_q;
dmem_addr_o = {mem_addr_q[31:2], 2'b00};
dmem_wdata_o = mem_wdata_q;
dmem_wstrb_o = mem_we_q ? mem_wstrb_q : 4'b0000;
end

assign cpu_debug_halt_o = (state_q == CPU_DEBUG_HALT);
assign cpu_trap_o = (state_q == CPU_TRAP);
assign cpu_halted_o = (state_q == CPU_DEBUG_HALT) || (state_q == CPU_TRAP);
assign trap_cause_o = trap_cause_q;
assign pc_o = pc_q;
assign retire_count_o = retire_count_q;
assign retired_pc_o = retired_pc_q;
assign retired_instr_o = retired_instr_q;

`ifdef ASSERT_ON

property p_x0_zero;
@(posedge clk) disable iff (!rst_n)
regs_q[0] == 32'd0;
endproperty
assert property (p_x0_zero);

property p_imem_aligned_when_valid;
@(posedge clk) disable iff (!rst_n)
imem_valid_o |-> imem_addr_o[1:0] == 2'b00;
endproperty
assert property (p_imem_aligned_when_valid);

property p_dmem_aligned_when_valid;
@(posedge clk) disable iff (!rst_n)
dmem_valid_o |-> dmem_addr_o[1:0] == 2'b00;
endproperty
assert property (p_dmem_aligned_when_valid);

property p_no_wstrb_on_read;
@(posedge clk) disable iff (!rst_n)
dmem_valid_o && !dmem_we_o |-> dmem_wstrb_o == 4'b0000;
endproperty
assert property (p_no_wstrb_on_read);

property p_no_retire_while_halted;
@(posedge clk) disable iff (!rst_n)
cpu_halted_o |-> !instr_retire_o;
endproperty
assert property (p_no_retire_while_halted);

property p_retire_count_increments;
@(posedge clk) disable iff (!rst_n)
instr_retire_o |-> retire_count_o == $past(retire_count_o) + 32'd1;
endproperty
assert property (p_retire_count_increments);

property p_debug_x0_write_ignored;
@(posedge clk) disable iff (!rst_n)
dbg_reg_write_en_i && dbg_reg_write_addr_i == 5'd0 |=> regs_q[0] == 32'd0;
endproperty
assert property (p_debug_x0_write_ignored);
`endif

endmodule

`default_nettype wire
