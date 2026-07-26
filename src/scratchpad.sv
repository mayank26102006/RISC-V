`default_nettype none

//------------------------------------------------------------------------------
// scratchpad.sv - TinyGPU-RV32 Scratchpad Memory
//------------------------------------------------------------------------------
// Project : TinyGPU-RV32
// Module : scratchpad
// Description : Small byte-addressable 32-bit word scratchpad memory with
// byte write strobes.
//
// Purpose:
// - Stores tiny programs/data for simulation and SoC bring-up
// - Supports RV32I load/store accesses through a simple valid/ready slave port
// - Supports byte/halfword/word stores using wstrb[3:0]
//
// Interface behavior:
// - Address is byte address.
// - Internally addressed as 32-bit words.
// - ready_o is combinational and asserted for every valid_i request.
// - Read data is combinational from the addressed word.
// - Writes commit on rising clock edge when valid_i && ready_o && we_i.
//
// Addressing:
// - WORDS = number of 32-bit words.
// - Total size in bytes = WORDS * 4.
// - For WORDS=256, size is 1 KiB and valid byte offsets are 0x000..0x3FF.
//
// Byte lanes:
// - wstrb[0] writes wdata_i[7:0]
// - wstrb[1] writes wdata_i[15:8]
// - wstrb[2] writes wdata_i[23:16]
// - wstrb[3] writes wdata_i[31:24]
//
// ASIC note:
// - RESET_MEM defaults to 0 because resetting memory arrays costs area.
// - For deterministic simulation, set RESET_MEM=1 or use a testbench loader.
// - Optional INIT_FILE is guarded with SIM_INIT so synthesis flows can ignore it.
//------------------------------------------------------------------------------

module scratchpad #(
parameter int unsigned WORDS = 256,
parameter bit RESET_MEM = 1'b0,
parameter string INIT_FILE = ""
) (
input logic clk,
input logic rst_n,

// Slave request
input logic valid_i,
input logic we_i,
input logic [31:0] addr_i,
input logic [31:0] wdata_i,
input logic [3:0] wstrb_i,

// Slave response
output logic ready_o,
output logic [31:0] rdata_o,
output logic error_o
);

// -------------------------------------------------------------------
// Derived constants
// -------------------------------------------------------------------
// WORD_ADDR_W is kept at least 1 so zero-width vectors are never created.

localparam int unsigned WORD_ADDR_W = (WORDS <= 1) ? 1 : $clog2(WORDS);
localparam int unsigned BYTE_SIZE = WORDS * 4;

// -------------------------------------------------------------------
// Memory array
// -------------------------------------------------------------------

logic [31:0] mem_q [0:WORDS-1];

logic [WORD_ADDR_W-1:0] word_index;
logic addr_in_range;

integer i;

// Word index from byte address. For 1 KiB/256 words, this is addr_i[9:2].
assign word_index = addr_i[WORD_ADDR_W+1:2];

// Address is in range when the byte address is below WORDS*4.
// The bus/interconnect normally performs region decode before this module,
// but this local check protects direct unit tests and catches bad routing.
always_comb begin
addr_in_range = (addr_i < BYTE_SIZE[31:0]);
end

// -------------------------------------------------------------------
// Ready/error/read response
// -------------------------------------------------------------------
// This is a simple zero-wait-state slave from the bus perspective.
// If later timing requires it, this can be converted to a one-cycle
// registered read response.

always_comb begin
ready_o = valid_i;
error_o = valid_i && !addr_in_range;

if (valid_i && addr_in_range) begin
rdata_o = mem_q[word_index];
end else begin
rdata_o = 32'd0;
end
end

// -------------------------------------------------------------------
// Optional simulation preload
// -------------------------------------------------------------------
// Use +define+SIM_INIT and set INIT_FILE to preload memory in simulation.
// Do not rely on this for ASIC production state unless the synthesis flow
// explicitly supports memory initialization.

`ifdef SIM_INIT
initial begin
if (INIT_FILE != "") begin
$readmemh(INIT_FILE, mem_q);
end
end
`endif

// -------------------------------------------------------------------
// Write path / optional memory reset
// -------------------------------------------------------------------

always_ff @(posedge clk or negedge rst_n) begin
if (!rst_n) begin
if (RESET_MEM) begin
for (i = 0; i < WORDS; i = i + 1) begin
mem_q[i] <= 32'd0;
end
end
end else begin
if (valid_i && ready_o && we_i && addr_in_range) begin
if (wstrb_i[0]) begin
mem_q[word_index][7:0] <= wdata_i[7:0];
end

if (wstrb_i[1]) begin
mem_q[word_index][15:8] <= wdata_i[15:8];
end

if (wstrb_i[2]) begin
mem_q[word_index][23:16] <= wdata_i[23:16];
end

if (wstrb_i[3]) begin
mem_q[word_index][31:24] <= wdata_i[31:24];
end
end
end
end

`ifdef ASSERT_ON
// -------------------------------------------------------------------
// Assertions
// -------------------------------------------------------------------

// WORDS must be nonzero.
initial begin
assert (WORDS > 0)
else $error("scratchpad WORDS parameter must be greater than zero");
end

// A valid request always receives ready in this zero-wait-state version.
property p_valid_gets_ready;
@(posedge clk) disable iff (!rst_n)
valid_i |-> ready_o;
endproperty
assert property (p_valid_gets_ready);

// Out-of-range requests must produce error.
property p_out_of_range_errors;
@(posedge clk) disable iff (!rst_n)
valid_i && !addr_in_range |-> error_o;
endproperty
assert property (p_out_of_range_errors);

// In-range requests should not produce error.
property p_in_range_no_error;
@(posedge clk) disable iff (!rst_n)
valid_i && addr_in_range |-> !error_o;
endproperty
assert property (p_in_range_no_error);

// Writes outside the valid range must not update memory. This is hard to
// assert for all locations cheaply, so assert the control condition instead.
property p_no_out_of_range_write_commit;
@(posedge clk) disable iff (!rst_n)
valid_i && we_i && !addr_in_range |-> error_o;
endproperty
assert property (p_no_out_of_range_write_commit);

// If write strobe is zero, a write request should not modify selected word.
// This property samples only the addressed word and is valid for in-range
// requests.
property p_zero_wstrb_no_change;
@(posedge clk) disable iff (!rst_n)
valid_i && ready_o && we_i && addr_in_range && (wstrb_i == 4'b0000) |=>
mem_q[$past(word_index)] == $past(mem_q[word_index]);
endproperty
assert property (p_zero_wstrb_no_change);
`endif

endmodule

`default_nettype wire

