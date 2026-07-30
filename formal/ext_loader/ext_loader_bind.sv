`default_nettype none

// -----------------------------------------------------------------------
// ext_loader_bind.sv - formal-only binding, NOT part of the design
// -----------------------------------------------------------------------
// See scratchpad_bind.sv for the full explanation of why this exists: this
// yosys-slang build (oss-cad-suite 0.66+179) parses ext_loader.sv's own
// named `property ... endproperty` + `assert property (name)` SVA fine, but
// fails BMC-cell lowering on it with "expression of type property with
// dynamic size unsupported for synthesis". Confirmed as a tool limitation,
// not an RTL issue, against SymbiYosys's own upstream examples. Every
// property below is the exact same condition as ext_loader.sv's own
// `ifdef ASSERT_ON block (p_write_only_when_active, p_force_reset_mirrors_pin,
// p_addr_in_bounds), just written as an immediate assertion instead of a
// named concurrent property, and reaching the DUT's private state_q /
// word_addr_q by hierarchical reference since this is an external bind, not
// a change to ext_loader.sv itself.
// -----------------------------------------------------------------------

module ext_loader_bind #(
    parameter int unsigned SCRATCHPAD_WORDS = 4
) (
    input logic clk,
    input logic rst_n,
    input logic ext_load_mode_i,
    input logic ext_load_bit_i
);

  logic        ext_load_ready_o;
  logic        force_cpu_reset_o;
  logic        mem_valid_o;
  logic [31:0] mem_addr_o;
  logic [31:0] mem_wdata_o;

  // The real, unmodified DUT.
  ext_loader #(
      .SCRATCHPAD_WORDS(SCRATCHPAD_WORDS)
  ) dut (
      .clk              (clk),
      .rst_n            (rst_n),
      .ext_load_mode_i  (ext_load_mode_i),
      .ext_load_bit_i   (ext_load_bit_i),
      .ext_load_ready_o (ext_load_ready_o),
      .force_cpu_reset_o(force_cpu_reset_o),
      .mem_valid_o      (mem_valid_o),
      .mem_addr_o       (mem_addr_o),
      .mem_wdata_o      (mem_wdata_o)
  );

  localparam int unsigned ADDR_BITS = $clog2(SCRATCHPAD_WORDS);
  // Mirrors ext_loader.sv's own LOAD_ACTIVE encoding (state_q is a 1-bit
  // enum: LOAD_IDLE=0, LOAD_ACTIVE=1).
  localparam logic LOAD_ACTIVE = 1'b1;

  // Formal-only: same reasoning as scratchpad_bind.sv -- start every trace
  // from a genuine reset, matching how every real testbench drives rst_n.
  initial assume (!rst_n);

  always @(posedge clk) begin
    if (rst_n) begin
      // p_write_only_when_active
      assert (!mem_valid_o || (dut.state_q == LOAD_ACTIVE));

      // p_force_reset_mirrors_pin
      assert (force_cpu_reset_o == ext_load_mode_i);

      // p_addr_in_bounds
      assert (!mem_valid_o
              || (dut.word_addr_q < SCRATCHPAD_WORDS[ADDR_BITS:0]));

      // Cover statements: prove the antecedents are actually reachable.
      cover (mem_valid_o);
      cover (ext_load_mode_i);
      cover (dut.state_q == LOAD_ACTIVE);
    end
  end

endmodule

`default_nettype wire
