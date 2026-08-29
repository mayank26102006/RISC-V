`default_nettype none

//------------------------------------------------------------------------------
// scratchpad.sv - TinyGPU-RV32 Scratchpad Memory
// Description : Small byte-addressable 32-bit word scratchpad memory with
// byte write strobes.
//------------------------------------------------------------------------------

module scratchpad #(
parameter int unsigned WORDS = 256,
parameter bit RESET_MEM = 1'b0
) (
input logic clk,
input logic rst_n,

input logic valid_i,
input logic we_i,
input logic [31:0] addr_i,
input logic [31:0] wdata_i,
input logic [3:0] wstrb_i,

output logic ready_o,
output logic [31:0] rdata_o,
output logic error_o
);

localparam int unsigned WORD_ADDR_W = (WORDS <= 1) ? 1 : $clog2(WORDS);
localparam int unsigned BYTE_SIZE = WORDS * 4;

logic [31:0] mem_q [0:WORDS-1];

logic [WORD_ADDR_W-1:0] word_index;
logic addr_in_range;

integer i;

assign word_index = addr_i[WORD_ADDR_W+1:2];

always_comb begin
addr_in_range = (addr_i < BYTE_SIZE[31:0]);
end

always_comb begin
ready_o = valid_i;
error_o = valid_i && !addr_in_range;

if (valid_i && addr_in_range) begin
rdata_o = mem_q[word_index];
end else begin
rdata_o = 32'd0;
end
end

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

initial begin
assert (WORDS > 0)
else $error("scratchpad WORDS parameter must be greater than zero");
end

property p_valid_gets_ready;
@(posedge clk) disable iff (!rst_n)
valid_i |-> ready_o;
endproperty
assert property (p_valid_gets_ready);

property p_out_of_range_errors;
@(posedge clk) disable iff (!rst_n)
valid_i && !addr_in_range |-> error_o;
endproperty
assert property (p_out_of_range_errors);

property p_in_range_no_error;
@(posedge clk) disable iff (!rst_n)
valid_i && addr_in_range |-> !error_o;
endproperty
assert property (p_in_range_no_error);

property p_no_out_of_range_write_commit;
@(posedge clk) disable iff (!rst_n)
valid_i && we_i && !addr_in_range |-> error_o;
endproperty
assert property (p_no_out_of_range_write_commit);

property p_zero_wstrb_no_change;
@(posedge clk) disable iff (!rst_n)
valid_i && ready_o && we_i && addr_in_range && (wstrb_i == 4'b0000) |=>
mem_q[$past(word_index)] == $past(mem_q[word_index]);
endproperty
assert property (p_zero_wstrb_no_change);
`endif

endmodule

`default_nettype wire
