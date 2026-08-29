`default_nettype none

//------------------------------------------------------------------------------
// bus_interconnect.sv - TinyGPU-RV32 Single-Master MMIO Interconnect
// Description : Routes one CPU data-bus master to scratchpad, accelerator
// registers, and debug/status registers using address decode.
//------------------------------------------------------------------------------

`include "tinygpu_pkg.sv"

module bus_interconnect #(
parameter bit ENABLE_SCRATCHPAD1 = 1'b0
) (

input logic cpu_valid_i,
input logic cpu_we_i,
input logic [31:0] cpu_addr_i,
input logic [31:0] cpu_wdata_i,
input logic [3:0] cpu_wstrb_i,

output logic cpu_ready_o,
output logic [31:0] cpu_rdata_o,
output logic cpu_err_o,


output logic scratch0_valid_o,
output logic scratch0_we_o,
output logic [31:0] scratch0_addr_o,
output logic [31:0] scratch0_wdata_o,
output logic [3:0] scratch0_wstrb_o,
input logic scratch0_ready_i,
input logic [31:0] scratch0_rdata_i,
input logic scratch0_error_i,


output logic scratch1_valid_o,
output logic scratch1_we_o,
output logic [31:0] scratch1_addr_o,
output logic [31:0] scratch1_wdata_o,
output logic [3:0] scratch1_wstrb_o,
input logic scratch1_ready_i,
input logic [31:0] scratch1_rdata_i,
input logic scratch1_error_i,

output logic accel_valid_o,
output logic accel_we_o,
output logic [31:0] accel_addr_o,
output logic [31:0] accel_wdata_o,
output logic [3:0] accel_wstrb_o,
input logic accel_ready_i,
input logic [31:0] accel_rdata_i,
input logic accel_error_i,


output logic debug_valid_o,
output logic debug_we_o,
output logic [31:0] debug_addr_o,
output logic [31:0] debug_wdata_o,
output logic [3:0] debug_wstrb_o,
input logic debug_ready_i,
input logic [31:0] debug_rdata_i,
input logic debug_error_i
);


logic scratch0_sel;
logic scratch1_sel_raw;
logic scratch1_sel;
logic accel_sel;
logic debug_sel;
logic no_slave_sel;

logic mmio_sel;
logic mmio_bad_align;
logic mmio_bad_wstrb;
logic mmio_access_error;


logic [31:0] scratch0_local_addr;
logic [31:0] scratch1_local_addr;
logic [31:0] accel_local_addr;
logic [31:0] debug_local_addr;

always_comb begin
scratch0_sel = is_scratch0_addr(cpu_addr_i);
scratch1_sel_raw = is_scratch1_addr(cpu_addr_i);
scratch1_sel = ENABLE_SCRATCHPAD1 && scratch1_sel_raw;
accel_sel = is_accel_addr(cpu_addr_i);
debug_sel = is_debug_addr(cpu_addr_i);

no_slave_sel = !(scratch0_sel || scratch1_sel || accel_sel || debug_sel);

mmio_sel = accel_sel || debug_sel;

mmio_bad_align = mmio_sel && (cpu_addr_i[1:0] != 2'b00);

mmio_bad_wstrb = mmio_sel && cpu_we_i && (cpu_wstrb_i != 4'b1111);

mmio_access_error = mmio_bad_align || mmio_bad_wstrb;

scratch0_local_addr = cpu_addr_i - SCRATCH0_BASE;
scratch1_local_addr = cpu_addr_i - SCRATCH1_BASE;
accel_local_addr = cpu_addr_i - ACCEL_BASE;
debug_local_addr = cpu_addr_i - DEBUG_BASE;
end

always_comb begin
scratch0_valid_o = 1'b0;
scratch0_we_o = 1'b0;
scratch0_addr_o = scratch0_local_addr;
scratch0_wdata_o = cpu_wdata_i;
scratch0_wstrb_o = cpu_wstrb_i;

scratch1_valid_o = 1'b0;
scratch1_we_o = 1'b0;
scratch1_addr_o = scratch1_local_addr;
scratch1_wdata_o = cpu_wdata_i;
scratch1_wstrb_o = cpu_wstrb_i;

accel_valid_o = 1'b0;
accel_we_o = 1'b0;
accel_addr_o = accel_local_addr;
accel_wdata_o = cpu_wdata_i;
accel_wstrb_o = cpu_wstrb_i;

debug_valid_o = 1'b0;
debug_we_o = 1'b0;
debug_addr_o = debug_local_addr;
debug_wdata_o = cpu_wdata_i;
debug_wstrb_o = cpu_wstrb_i;

if (cpu_valid_i) begin
if (scratch0_sel) begin
scratch0_valid_o = 1'b1;
scratch0_we_o = cpu_we_i;
end else if (scratch1_sel) begin
scratch1_valid_o = 1'b1;
scratch1_we_o = cpu_we_i;
end else if (accel_sel && !mmio_access_error) begin
accel_valid_o = 1'b1;
accel_we_o = cpu_we_i;
end else if (debug_sel && !mmio_access_error) begin
debug_valid_o = 1'b1;
debug_we_o = cpu_we_i;
end
end
end


always_comb begin
cpu_ready_o = 1'b0;
cpu_rdata_o = 32'd0;
cpu_err_o = 1'b0;

if (cpu_valid_i) begin
if (scratch0_sel) begin
cpu_ready_o = scratch0_ready_i;
cpu_rdata_o = scratch0_rdata_i;
cpu_err_o = scratch0_error_i;
end else if (scratch1_sel) begin
cpu_ready_o = scratch1_ready_i;
cpu_rdata_o = scratch1_rdata_i;
cpu_err_o = scratch1_error_i;
end else if (scratch1_sel_raw && !ENABLE_SCRATCHPAD1) begin
// Address belongs to optional scratchpad1, but the region is disabled.
cpu_ready_o = 1'b1;
cpu_rdata_o = 32'd0;
cpu_err_o = 1'b1;
end else if (accel_sel) begin
if (mmio_access_error) begin
cpu_ready_o = 1'b1;
cpu_rdata_o = 32'd0;
cpu_err_o = 1'b1;
end else begin
cpu_ready_o = accel_ready_i;
cpu_rdata_o = accel_rdata_i;
cpu_err_o = accel_error_i;
end
end else if (debug_sel) begin
if (mmio_access_error) begin
cpu_ready_o = 1'b1;
cpu_rdata_o = 32'd0;
cpu_err_o = 1'b1;
end else begin
cpu_ready_o = debug_ready_i;
cpu_rdata_o = debug_rdata_i;
cpu_err_o = debug_error_i;
end
end else begin
// Invalid/unmapped address.
cpu_ready_o = 1'b1;
cpu_rdata_o = 32'd0;
cpu_err_o = 1'b1;
end
end
end

`ifdef ASSERT_ON



property p_slave_select_onehot0;
@(*) $onehot0({scratch0_sel, scratch1_sel, accel_sel, debug_sel});
endproperty
assert property (p_slave_select_onehot0);


property p_slave_valid_onehot0;
@(*) $onehot0({scratch0_valid_o, scratch1_valid_o, accel_valid_o, debug_valid_o});
endproperty
assert property (p_slave_valid_onehot0);


property p_invalid_address_errors;
@(*) (cpu_valid_i && no_slave_sel && !(scratch1_sel_raw && !ENABLE_SCRATCHPAD1)) |->
(cpu_ready_o && cpu_err_o && cpu_rdata_o == 32'd0);
endproperty
assert property (p_invalid_address_errors);


property p_disabled_scratch1_errors;
@(*) (cpu_valid_i && scratch1_sel_raw && !ENABLE_SCRATCHPAD1) |->
(cpu_ready_o && cpu_err_o && cpu_rdata_o == 32'd0);
endproperty
assert property (p_disabled_scratch1_errors);


property p_mmio_misaligned_errors;
@(*) (cpu_valid_i && mmio_sel && mmio_bad_align) |->
(cpu_ready_o && cpu_err_o);
endproperty
assert property (p_mmio_misaligned_errors);


property p_mmio_bad_wstrb_errors;
@(*) (cpu_valid_i && mmio_sel && mmio_bad_wstrb) |->
(cpu_ready_o && cpu_err_o);
endproperty
assert property (p_mmio_bad_wstrb_errors);


property p_bad_mmio_not_forwarded;
@(*) (cpu_valid_i && mmio_access_error) |->
(!accel_valid_o && !debug_valid_o);
endproperty
assert property (p_bad_mmio_not_forwarded);


property p_scratch0_forwarding;
@(*) (cpu_valid_i && scratch0_sel) |->
(scratch0_valid_o && scratch0_we_o == cpu_we_i &&
scratch0_wdata_o == cpu_wdata_i && scratch0_wstrb_o == cpu_wstrb_i);
endproperty
assert property (p_scratch0_forwarding);

property p_accel_forwarding;
@(*) (cpu_valid_i && accel_sel && !mmio_access_error) |->
(accel_valid_o && accel_we_o == cpu_we_i &&
accel_wdata_o == cpu_wdata_i && accel_wstrb_o == cpu_wstrb_i);
endproperty
assert property (p_accel_forwarding);


property p_debug_forwarding;
@(*) (cpu_valid_i && debug_sel && !mmio_access_error) |->
(debug_valid_o && debug_we_o == cpu_we_i &&
debug_wdata_o == cpu_wdata_i && debug_wstrb_o == cpu_wstrb_i);
endproperty
assert property (p_debug_forwarding);
`endif

endmodule

`default_nettype wire

