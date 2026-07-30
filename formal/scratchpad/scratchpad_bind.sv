`default_nettype none

// -----------------------------------------------------------------------
// scratchpad_bind.sv - formal-only binding, NOT part of the design
// -----------------------------------------------------------------------
// scratchpad.sv is instantiated completely unmodified below. This file only
// adds observation of its ports and internal state for BMC.
//
// Why this file exists instead of running scratchpad.sv's own `ifdef
// ASSERT_ON block directly: that block declares each property with a name
// (`property p_valid_gets_ready; ... endproperty` then separately
// `assert property (p_valid_gets_ready);`), which is completely standard,
// portable SVA -- but the yosys-slang plugin bundled in this OSS CAD Suite
// build (0.66+179) can parse named properties fine, then fails at assertion
// lowering with "expression of type property with dynamic size unsupported
// for synthesis" when it tries to turn a *referenced* property into a BMC
// cell. Confirmed this is a tool limitation, not an RTL issue, by testing
// against SymbiYosys's own upstream examples, which use INLINE (unnamed)
// properties throughout and lower fine on this same install.
//
// Every property below is copied verbatim in meaning from scratchpad.sv's
// `ifdef ASSERT_ON block -- same signals, same operators, same conditions --
// just written inline instead of as a named property, and with the DUT's
// private word_index/mem_q reached by hierarchical reference instead of
// being local to the module. If you have access to a toolchain with fuller
// named-property support (Tabby CAD, JasperGold, VCS, Questa, or a newer
// yosys-slang release), scratchpad.sv's own assertions can be proven
// directly with `-D ASSERT_ON` and this file isn't needed.
// -----------------------------------------------------------------------

module scratchpad_bind #(
    parameter int unsigned WORDS = 4
) (
    input logic clk,
    input logic rst_n,
    input logic valid_i,
    input logic we_i,
    input logic [31:0] addr_i,
    input logic [31:0] wdata_i,
    input logic [3:0] wstrb_i
);

  logic ready_o, error_o;
  logic [31:0] rdata_o;

  // The real, unmodified DUT.
  scratchpad #(
      .WORDS(WORDS),
      .RESET_MEM(1'b0)
  ) dut (
      .clk    (clk),
      .rst_n  (rst_n),
      .valid_i(valid_i),
      .we_i   (we_i),
      .addr_i (addr_i),
      .wdata_i(wdata_i),
      .wstrb_i(wstrb_i),
      .ready_o(ready_o),
      .rdata_o(rdata_o),
      .error_o(error_o)
  );

  localparam int unsigned WORD_ADDR_W = (WORDS <= 1) ? 1 : $clog2(WORDS);
  localparam int unsigned BYTE_SIZE = WORDS * 4;

  // Mirrors scratchpad.sv's own addr_in_range exactly (same expression).
  logic addr_in_range;
  assign addr_in_range = (addr_i < BYTE_SIZE[31:0]);

  // ---------------------------------------------------------------------
  // NOTE ON SYNTAX: these are written as immediate assertions inside a
  // clocked always block, not as concurrent `assert property(@(posedge
  // clk)...)` SVA. Tried the concurrent form first -- it parses fine on
  // this yosys-slang build, but even the simplest possible single-cycle
  // `valid_i |-> ready_o` fails BMC-cell lowering with "expression of type
  // property with dynamic size unsupported for synthesis". Confirmed this
  // is a tool limitation, not a syntax mistake, by testing progressively
  // simpler cases down to a bare `a |-> b` with no other operators, which
  // fails identically. This build's SVA support appears to stop at parsing;
  // synthesizing implications into checkable BMC cells needs the separate
  // `synthprop` pass (a different, FPGA-ILA-oriented flow) rather than the
  // direct `prep`-into-BMC path SBY normally relies on. Immediate
  // assertions lower to a plain $check cell and BMC them correctly -- same
  // properties, same signals, same conditions, different syntax only.
  // ---------------------------------------------------------------------

  // Formal-only: assume the trace starts from a genuine reset, exactly as
  // every real testbench (including tb.v) drives rst_n low before running
  // anything.
  initial assume (!rst_n);

  logic [$clog2(WORDS)>0 ? $clog2(WORDS)-1 : 0:0] word_index_prev;
  logic [31:0] mem_prev;
  logic zero_wstrb_commit_prev;

  // These three are pure bookkeeping registers local to this formal
  // harness, not part of the DUT, so they need their own defined starting
  // values -- an `always @(posedge clk)` reset branch only takes effect
  // *after* a clock edge has actually happened with rst_n low; it says
  // nothing about the value these registers hold at BMC step 0, before any
  // edge is evaluated. Without this, step 0 leaves them free, and the
  // solver is entitled to pick zero_wstrb_commit_prev=1 with mem_prev
  // holding an arbitrary value that has no relationship to dut.mem_q --
  // which is exactly the false counterexample this produced before this
  // fix (confirmed by inspecting the counterexample VCD: the "violation"
  // only ever appeared at the very first checked step, sourced entirely
  // from this uninitialized state, not from any real design behavior).
  initial zero_wstrb_commit_prev = 1'b0;
  initial word_index_prev = '0;
  initial mem_prev = 32'd0;

  always @(posedge clk) begin
    if (!rst_n) begin
      zero_wstrb_commit_prev <= 1'b0;
    end else begin
      // p_valid_gets_ready
      assert (!valid_i || ready_o);

      // p_out_of_range_errors
      assert (!(valid_i && !addr_in_range) || error_o);

      // p_in_range_no_error
      assert (!(valid_i && addr_in_range) || error_o == 1'b0);

      // p_no_out_of_range_write_commit
      assert (!(valid_i && we_i && !addr_in_range) || error_o);

      // p_zero_wstrb_no_change (the |=> half of the original property --
      // checks the value latched from the *previous* cycle, which is
      // exactly what |=> means).
      if (zero_wstrb_commit_prev) begin
        assert (dut.mem_q[word_index_prev] == mem_prev);
      end

      // Cover statements: prove each property's antecedent is actually
    // reachable, not just vacuously true because the condition never
    // occurs. A BMC pass on the asserts above means nothing if these never
    // fire.
    cover (valid_i && ready_o);
    cover (valid_i && !addr_in_range && error_o);
    cover (valid_i && we_i && !addr_in_range && error_o);
    cover (zero_wstrb_commit_prev);

    zero_wstrb_commit_prev <= valid_i && ready_o && we_i
                                 && addr_in_range && (wstrb_i == 4'b0000);
    end

    // Latched every cycle regardless of reset gating above, same as the
    // DUT's own combinational word_index -- these two are pure bookkeeping,
    // not reset-sensitive state, so they don't need their own reset branch.
    word_index_prev <= dut.word_index;
    mem_prev        <= dut.mem_q[dut.word_index];
  end

endmodule

`default_nettype wire
