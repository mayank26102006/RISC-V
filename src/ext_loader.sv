`default_nettype none


`default_nettype none

//------------------------------------------------------------------------------
// ext_loader.sv - TinyGPU-RV32 External Program Loader
// Description : A genuine external path to load a program into the shared
//               scratchpad from outside the fabricated chip, using only 2
//               input pins and 1 status output pin -- no hierarchical
//               testbench access required, so it works identically in
//               RTL simulation, gate-level simulation, and on real
//               silicon.
//------------------------------------------------------------------------------

module ext_loader #(
    parameter int unsigned SCRATCHPAD_WORDS = 16
) (
    input  logic clk,
    input  logic rst_n,

    input  logic ext_load_mode_i,
    input  logic ext_load_bit_i,
    output logic ext_load_ready_o,

    output logic force_cpu_reset_o,

    output logic         mem_valid_o,
    output logic [31:0]  mem_addr_o,
    output logic [31:0]  mem_wdata_o
);

  localparam int unsigned ADDR_BITS = $clog2(SCRATCHPAD_WORDS);

  typedef enum logic {
    LOAD_IDLE   = 1'b0,
    LOAD_ACTIVE = 1'b1
  } load_state_e;

  load_state_e state_q, state_d;

  logic [31:0]         shift_q, shift_d;
  logic [5:0]          bit_count_q, bit_count_d;
  logic [ADDR_BITS:0]  word_addr_q, word_addr_d;

  logic ext_load_ready_q, ext_load_ready_d;
  assign ext_load_ready_o = ext_load_ready_q;

  logic word_complete;
  logic addr_in_range;

  assign word_complete = (bit_count_q == 6'd31);
  assign addr_in_range = (word_addr_q < SCRATCHPAD_WORDS[ADDR_BITS:0]);

  always_comb begin
    state_d          = state_q;
    shift_d          = shift_q;
    bit_count_d      = bit_count_q;
    word_addr_d      = word_addr_q;
    ext_load_ready_d = 1'b0;

    mem_valid_o = 1'b0;
    mem_addr_o  = {{(32-ADDR_BITS-2){1'b0}}, word_addr_q[ADDR_BITS-1:0], 2'b00};
    mem_wdata_o = shift_q;

    unique case (state_q)
      LOAD_IDLE: begin
        if (ext_load_mode_i) begin
          state_d     = LOAD_ACTIVE;
          shift_d     = 32'd0;
          bit_count_d = 6'd0;
          word_addr_d = '0;
        end
      end

      LOAD_ACTIVE: begin
        if (!ext_load_mode_i) begin
          state_d = LOAD_IDLE;
        end else if (!addr_in_range) begin
          ext_load_ready_d = 1'b0;
        end else begin
          ext_load_ready_d = 1'b1;

          if (ext_load_ready_q) begin
            shift_d     = {shift_q[30:0], ext_load_bit_i};
            bit_count_d = bit_count_q + 6'd1;

            if (word_complete) begin
              mem_valid_o = 1'b1;
              mem_wdata_o = {shift_q[30:0], ext_load_bit_i};
              bit_count_d = 6'd0;
              shift_d     = 32'd0;
              word_addr_d = word_addr_q + 1'b1;
            end
          end
        end
      end

      default: begin
        state_d = LOAD_IDLE;
      end
    endcase
  end

  assign force_cpu_reset_o = ext_load_mode_i;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state_q          <= LOAD_IDLE;
      shift_q          <= 32'd0;
      bit_count_q      <= 6'd0;
      word_addr_q      <= '0;
      ext_load_ready_q <= 1'b0;
    end else begin
      state_q          <= state_d;
      shift_q          <= shift_d;
      bit_count_q      <= bit_count_d;
      word_addr_q      <= word_addr_d;
      ext_load_ready_q <= ext_load_ready_d;
    end
  end

`ifdef ASSERT_ON

  property p_write_only_when_active;
    @(posedge clk) disable iff (!rst_n)
    mem_valid_o |-> (state_q == LOAD_ACTIVE);
  endproperty
  assert property (p_write_only_when_active);

  property p_force_reset_mirrors_pin;
    @(posedge clk) disable iff (!rst_n)
    force_cpu_reset_o == ext_load_mode_i;
  endproperty
  assert property (p_force_reset_mirrors_pin);

  property p_addr_in_bounds;
    @(posedge clk) disable iff (!rst_n)
    mem_valid_o |-> (word_addr_q < SCRATCHPAD_WORDS[ADDR_BITS:0]);
  endproperty
  assert property (p_addr_in_bounds);
`endif

endmodule

`default_nettype wire
