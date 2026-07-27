`default_nettype none

//------------------------------------------------------------------------------
// accel_regs.sv - TinyGPU-RV32 Accelerator Register Block (MMIO)
//------------------------------------------------------------------------------
// Project : TinyGPU-RV32
// Module : accel_regs
// Description : Memory-mapped control/status register block for the vector/
// dot-product accelerator.
//
// Address map (offsets from ACCEL_BASE):
// 0x00 ACC_CMD RW Command register (write triggers start)
// 0x04 ACC_STATUS RO/W1C Status bits
// 0x08 ACC_SRC_A RW Source operand A
// 0x0C ACC_SRC_B RW Source operand B
// 0x10 ACC_SRC_C RW Optional config/operand C
// 0x14 ACC_LEN RW Optional length/config
// 0x18 ACC_DST RW Optional destination/config
// 0x1C ACC_RESULT RO Result register
// 0x20 ACC_ERROR RO/W1C Error bits
//
// Bus behavior:
// - Word-aligned accesses only (enforced by bus_interconnect)
// - Zero-wait-state ready_o
// - Write-one-to-clear (W1C) semantics for STATUS/ERROR sticky bits
//
// Protocol to accelerator core:
// - Write to ACC_CMD with nonzero command while idle generates accel_start_o
// - Operands/config are latched at start
// - STATUS busy/done/error reflect accelerator signals
//
// ASIC note:
// - Small register bank, synthesizable
// - No latches, no combinational feedback
//------------------------------------------------------------------------------

`include "tinygpu_pkg.sv"

module accel_regs (
input logic clk,
input logic rst_n,

// -------------------------------------------------------------------
// Bus slave interface
// -------------------------------------------------------------------
input logic valid_i,
input logic we_i,
input logic [31:0] addr_i,
input logic [31:0] wdata_i,
input logic [3:0] wstrb_i,

output logic ready_o,
output logic [31:0] rdata_o,
output logic error_o,

// -------------------------------------------------------------------
// Interface to accelerator core
// -------------------------------------------------------------------
output logic accel_start_o,
output accel_cmd_e accel_cmd_o,
output logic [31:0] accel_src_a_o,
output logic [31:0] accel_src_b_o,
output logic [31:0] accel_src_c_o,
output logic [31:0] accel_len_o,
output logic [31:0] accel_dst_o,

input logic accel_busy_i,
input logic accel_done_i,
input logic accel_error_i,
input logic [31:0] accel_result_i
);

// -------------------------------------------------------------------
// Local address decode (word offsets)
// -------------------------------------------------------------------

logic [7:0] word_off;
assign word_off = addr_i[9:2]; // 256B window => 64 words

// -------------------------------------------------------------------
// Registers
// -------------------------------------------------------------------

accel_cmd_e cmd_q;
logic [31:0] src_a_q;
logic [31:0] src_b_q;
// src_c_q/len_q/dst_q removed 2026-07-27 (96 bits) -- confirmed never
// read by vector_accel.sv (only present in its port list, for future
// MAT2I8/MAC-style multi-operand operations that don't exist yet).
// accel_src_c_o/accel_len_o/accel_dst_o are tied to a constant below
// instead of being backed by real flip-flops.

// Sticky status/error bits
logic status_done_q;
logic status_illegal_cmd_q;
logic status_busy_violation_q;

logic error_illegal_cmd_q;
logic error_busy_violation_q;
logic error_internal_q;

// -------------------------------------------------------------------
// Ready / error (zero-wait-state)
// -------------------------------------------------------------------

always_comb begin
ready_o = valid_i;
error_o = 1'b0;
end

// -------------------------------------------------------------------
// Read mux
// -------------------------------------------------------------------

always_comb begin
rdata_o = 32'd0;

unique case (word_off)
8'h00: rdata_o = cmd_q;
8'h01: rdata_o = {
27'd0,
status_busy_violation_q,
status_illegal_cmd_q,
accel_error_i | error_illegal_cmd_q | error_busy_violation_q | error_internal_q,
accel_done_i | status_done_q,
accel_busy_i
};
8'h02: rdata_o = src_a_q;
8'h03: rdata_o = src_b_q;
8'h04: rdata_o = 32'd0; // src_c_q removed 2026-07-27, never read by vector_accel.sv
8'h05: rdata_o = 32'd0; // len_q removed 2026-07-27, never read by vector_accel.sv
8'h06: rdata_o = 32'd0; // dst_q removed 2026-07-27, never read by vector_accel.sv
8'h07: rdata_o = accel_result_i;
8'h08: rdata_o = {
29'd0,
error_internal_q,
error_busy_violation_q,
error_illegal_cmd_q
};
default: rdata_o = 32'd0;
endcase
end

// -------------------------------------------------------------------
// Write handling and start pulse generation
// -------------------------------------------------------------------

always_ff @(posedge clk or negedge rst_n) begin
if (!rst_n) begin
cmd_q <= ACC_CMD_NOP;
src_a_q <= 32'd0;
src_b_q <= 32'd0;

status_done_q <= 1'b0;
status_illegal_cmd_q <= 1'b0;
status_busy_violation_q <= 1'b0;

error_illegal_cmd_q <= 1'b0;
error_busy_violation_q <= 1'b0;
error_internal_q <= 1'b0;

accel_start_o <= 1'b0;
accel_cmd_o <= ACC_CMD_NOP;
end else begin
accel_start_o <= 1'b0; // default pulse low

// Latch live accelerator error into sticky error
if (accel_error_i) begin
error_internal_q <= 1'b1;
end

// Latch completion
if (accel_done_i) begin
status_done_q <= 1'b1;
end

if (valid_i && we_i) begin
unique case (word_off)

// ---------------------------------------------------
// ACC_CMD
// ---------------------------------------------------
8'h00: begin
if (wdata_i != 32'd0) begin
if (accel_busy_i) begin
status_busy_violation_q <= 1'b1;
error_busy_violation_q <= 1'b1;
end else begin
// Accept command
cmd_q <= accel_cmd_e'(wdata_i[7:0]);
accel_cmd_o <= accel_cmd_e'(wdata_i[7:0]);
accel_start_o <= 1'b1;
status_done_q <= 1'b0;

// Illegal command check
if (!(wdata_i[7:0] == ACC_CMD_VADD8 ||
      wdata_i[7:0] == ACC_CMD_VSUB8 ||
      wdata_i[7:0] == ACC_CMD_VMAX8 ||
      wdata_i[7:0] == ACC_CMD_RELU8 ||
      wdata_i[7:0] == ACC_CMD_DOT4I8 ||
      wdata_i[7:0] == ACC_CMD_MAT2I8)) begin
status_illegal_cmd_q <= 1'b1;
error_illegal_cmd_q <= 1'b1;
end
end
end
end

// ---------------------------------------------------
// ACC_STATUS (W1C)
// ---------------------------------------------------
8'h01: begin
if (wdata_i[ACC_STATUS_DONE_BIT]) status_done_q <= 1'b0;
if (wdata_i[ACC_STATUS_ILLEGAL_COMMAND_BIT]) status_illegal_cmd_q <= 1'b0;
if (wdata_i[ACC_STATUS_BUSY_VIOLATION_BIT]) status_busy_violation_q <= 1'b0;
end

// ---------------------------------------------------
// ACC_SRC_A/B -- SRC_C/LEN/DST removed 2026-07-27, writes to
// those addresses are now no-ops (never read by vector_accel.sv)
// ---------------------------------------------------
8'h02: src_a_q <= wdata_i;
8'h03: src_b_q <= wdata_i;

// ---------------------------------------------------
// ACC_ERROR (W1C)
// ---------------------------------------------------
8'h08: begin
if (wdata_i[ACC_ERROR_ILLEGAL_COMMAND_BIT]) error_illegal_cmd_q <= 1'b0;
if (wdata_i[ACC_ERROR_BUSY_VIOLATION_BIT]) error_busy_violation_q <= 1'b0;
if (wdata_i[ACC_ERROR_INTERNAL_ERROR_BIT]) error_internal_q <= 1'b0;
end

default: begin
// no-op
end
endcase
end
end
end

// -------------------------------------------------------------------
// Outputs
// -------------------------------------------------------------------

assign accel_src_a_o = src_a_q;
assign accel_src_b_o = src_b_q;
assign accel_src_c_o = 32'd0; // src_c_q removed 2026-07-27
assign accel_len_o   = 32'd0; // len_q removed 2026-07-27
assign accel_dst_o   = 32'd0; // dst_q removed 2026-07-27

`ifdef ASSERT_ON
// -------------------------------------------------------------------
// Assertions
// -------------------------------------------------------------------

// Start pulse should only be one cycle
property p_start_pulse_single_cycle;
@(posedge clk) disable iff (!rst_n)
accel_start_o |-> ##1 !accel_start_o;
endproperty
assert property (p_start_pulse_single_cycle);

// Cannot start new command while busy without flagging violation
property p_busy_violation_flag;
@(posedge clk) disable iff (!rst_n)
(valid_i && we_i && word_off == 8'h00 && accel_busy_i && wdata_i != 32'd0)
|-> status_busy_violation_q;
endproperty
assert property (p_busy_violation_flag);

// Illegal command sets error
property p_illegal_command_sets_error;
@(posedge clk) disable iff (!rst_n)
(accel_start_o && !(accel_cmd_o == ACC_CMD_VADD8 ||
                     accel_cmd_o == ACC_CMD_VSUB8 ||
                     accel_cmd_o == ACC_CMD_VMAX8 ||
                     accel_cmd_o == ACC_CMD_RELU8 ||
                     accel_cmd_o == ACC_CMD_DOT4I8 ||
                     accel_cmd_o == ACC_CMD_MAT2I8)) |-> error_illegal_cmd_q;
endproperty
assert property (p_illegal_command_sets_error);
`endif

endmodule

`default_nettype wire

