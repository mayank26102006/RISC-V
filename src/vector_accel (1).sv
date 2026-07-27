`default_nettype none

//------------------------------------------------------------------------------
// vector_accel.sv - TinyGPU-RV32 Vector / DOT4I8 Accelerator Core
//------------------------------------------------------------------------------
// Project     : TinyGPU-RV32
// Module      : vector_accel
// Description : Implements packed int8 vector operations and DOT4I8.
//
// Supported commands (see tinygpu_pkg.sv):
//   ACC_CMD_VADD8  : 4-lane int8 add
//   ACC_CMD_VSUB8  : 4-lane int8 subtract
//   ACC_CMD_VMAX8  : 4-lane signed int8 max
//   ACC_CMD_RELU8  : 4-lane signed int8 ReLU
//   ACC_CMD_DOT4I8 : signed 4-lane int8 dot product
//
// Protocol:
//   - accel_start_i asserted for one cycle to start operation
//   - accel_cmd_i/accel_src_a_i/accel_src_b_i are valid and stable on the
//     same cycle accel_start_i is asserted (accel_regs.sv registers them
//     together with the start pulse)
//   - accel_busy_o asserted while operation in progress
//   - accel_done_o pulses when result is ready
//   - accel_error_o asserted on illegal/unsupported conditions
//
// Latency model (v1):
//   - All operations complete in a single cycle after start.
//   - The result is computed directly from the live accel_cmd_i /
//     accel_src_a_i / accel_src_b_i inputs on the start cycle itself.
//
//   FIX NOTE (2026-07-24): the previous version of this file computed
//   exec_result from cmd_q/src_a_q/src_b_q -- registers that are only
//   loaded with this cycle's inputs via a nonblocking assignment in the
//   *same* always_ff block that produces result_q/accel_result_o. Because
//   nonblocking assignments only take effect after the clock edge, that
//   made every operation's result reflect the *previous* command's
//   operands (or all-zero on the very first command ever issued). The
//   fix below computes exec_result straight from accel_cmd_i/
//   accel_src_a_i/accel_src_b_i so the result matches the command that is
//   actually starting this cycle.
//
//   cmd_q/src_a_q/src_b_q are still latched on start and kept around
//   (currently unused by the v1 datapath) so a future multi-cycle
//   extension -- e.g. MAT2I8/MAC roadmap items -- has stable latched
//   operands to work from across an ACC_EXEC state without re-plumbing
//   this module's control-to-accel_regs interface.
//
//   AREA NOTE (2026-07-27): cmd_q/src_a_q/src_b_q described above were
//   actually removed (72 bits) as part of a real-hardening-driven area
//   reduction pass -- confirmed hardening runs showed this design didn't
//   fit even an 8x4 Tiny Tapeout tile grid. If a future multi-cycle
//   extension needs latched operands again, re-add them then rather
//   than paying for them now.
//
//   FIX NOTE 2 (2026-07-24): a second, separate bug was found by actually
//   running the accelerator through simulation (not caught by code review
//   alone). The byte-lane packing for VADD8/VSUB8/VMAX8/RELU8 used casts
//   written as `logic'(a3 + b3)`. In SystemVerilog, an *unsized* cast like
//   `logic'(...)` casts down to a single bit, not 8 bits -- so only the
//   LSB of each lane sum was being packed, with the other 7 bits of each
//   lane silently dropped. This produced a 4-bit garbage value zero-
//   extended into a 32-bit result instead of the real 4-lane int8 answer.
//   Fixed by using a properly sized cast, `8'(...)`, for every byte-lane
//   value below. A regression testbench (tb_tinygpu_soc.sv) exercises
//   VADD8 and DOT4I8 with known operands/expected results specifically to
//   catch this class of bug again if it's ever reintroduced.
//
// ASIC note:
//   - Small, synthesizable datapath.
//   - No latches, no combinational feedback.
//------------------------------------------------------------------------------

`include "tinygpu_pkg.sv"

module vector_accel (
    input  logic         clk,
    input  logic         rst_n,

    // Control from accel_regs
    input  logic         accel_start_i,
    input  accel_cmd_e   accel_cmd_i,
    input  logic [31:0]  accel_src_a_i,
    input  logic [31:0]  accel_src_b_i,
    input  logic [31:0]  accel_src_c_i,
    input  logic [31:0]  accel_len_i,
    input  logic [31:0]  accel_dst_i,

    // Status/result back to accel_regs
    output logic         accel_busy_o,
    output logic         accel_done_o,
    output logic         accel_error_o,
    output logic [31:0]  accel_result_o
);

  // -------------------------------------------------------------------
  // Internal state
  // -------------------------------------------------------------------

  typedef enum logic [1:0] {
    ACC_IDLE = 2'd0,
    ACC_EXEC = 2'd1
  } accel_state_e;

  accel_state_e state_q, state_d;

  // Result register
  logic [31:0] result_q, result_d;

  // -------------------------------------------------------------------
  // Lane extraction helpers
  // -------------------------------------------------------------------
  // Extracted from the *live* inputs so the result matches the command
  // that is starting this cycle, not a stale latched command.

  logic signed [7:0] a0, a1, a2, a3;
  logic signed [7:0] b0, b1, b2, b3;

  assign a0 = accel_src_a_i[7:0];
  assign a1 = accel_src_a_i[15:8];
  assign a2 = accel_src_a_i[23:16];
  assign a3 = accel_src_a_i[31:24];

  assign b0 = accel_src_b_i[7:0];
  assign b1 = accel_src_b_i[15:8];
  assign b2 = accel_src_b_i[23:16];
  assign b3 = accel_src_b_i[31:24];

  // -------------------------------------------------------------------
  // Combinational execution
  // -------------------------------------------------------------------

  logic [31:0] exec_result;

  always_comb begin
    exec_result = 32'd0;

    unique case (accel_cmd_i)
      ACC_CMD_VADD8: begin
        exec_result = {
          8'(a3 + b3),
          8'(a2 + b2),
          8'(a1 + b1),
          8'(a0 + b0)
        };
      end

      ACC_CMD_VSUB8: begin
        exec_result = {
          8'(a3 - b3),
          8'(a2 - b2),
          8'(a1 - b1),
          8'(a0 - b0)
        };
      end

      ACC_CMD_VMAX8: begin
        exec_result = {
          (a3 > b3) ? 8'(a3) : 8'(b3),
          (a2 > b2) ? 8'(a2) : 8'(b2),
          (a1 > b1) ? 8'(a1) : 8'(b1),
          (a0 > b0) ? 8'(a0) : 8'(b0)
        };
      end

      ACC_CMD_RELU8: begin
        exec_result = {
          (a3 < 0) ? 8'd0 : 8'(a3),
          (a2 < 0) ? 8'd0 : 8'(a2),
          (a1 < 0) ? 8'd0 : 8'(a1),
          (a0 < 0) ? 8'd0 : 8'(a0)
        };
      end

      ACC_CMD_DOT4I8: begin
        exec_result =
            $signed(a0) * $signed(b0) +
            $signed(a1) * $signed(b1) +
            $signed(a2) * $signed(b2) +
            $signed(a3) * $signed(b3);
      end

      default: begin
        exec_result = 32'd0;
      end
    endcase
  end

  // -------------------------------------------------------------------
  // FSM next-state logic
  // -------------------------------------------------------------------

  always_comb begin
    state_d       = state_q;
    result_d      = result_q;
    accel_done_o  = 1'b0;
    accel_error_o = 1'b0;

    unique case (state_q)
      ACC_IDLE: begin
        if (accel_start_i) begin
          // Unsupported commands raise error but still complete.
          if (!(accel_cmd_i == ACC_CMD_VADD8 ||
                accel_cmd_i == ACC_CMD_VSUB8 ||
                accel_cmd_i == ACC_CMD_VMAX8 ||
                accel_cmd_i == ACC_CMD_RELU8 ||
                accel_cmd_i == ACC_CMD_DOT4I8)) begin
            accel_error_o = 1'b1;
            result_d      = 32'd0;
            accel_done_o  = 1'b1;
            state_d       = ACC_IDLE;
          end else begin
            // Execute in one cycle, using the live inputs above.
            result_d     = exec_result;
            accel_done_o = 1'b1;
            state_d      = ACC_IDLE;
          end
        end
      end

      default: begin
        state_d = ACC_IDLE;
      end
    endcase
  end

  // -------------------------------------------------------------------
  // Sequential state update
  // -------------------------------------------------------------------

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state_q        <= ACC_IDLE;
      result_q       <= 32'd0;
      accel_busy_o   <= 1'b0;
      accel_result_o <= 32'd0;
    end else begin
      state_q <= state_d;

      result_q       <= result_d;
      accel_result_o <= result_d;

      accel_busy_o <= (state_d != ACC_IDLE);
    end
  end

`ifdef ASSERT_ON
  // -------------------------------------------------------------------
  // Assertions
  // -------------------------------------------------------------------

  // Busy should only be high in non-idle state.
  property p_busy_matches_state;
    @(posedge clk) disable iff (!rst_n)
    accel_busy_o == (state_q != ACC_IDLE);
  endproperty
  assert property (p_busy_matches_state);

  // Done should only pulse when start is seen.
  property p_done_requires_start;
    @(posedge clk) disable iff (!rst_n)
    accel_done_o |-> accel_start_i;
  endproperty
  assert property (p_done_requires_start);

  // DOT4 sanity: if inputs are zero, result must be zero.
  property p_dot4_zero;
    @(posedge clk) disable iff (!rst_n)
    (accel_start_i && accel_cmd_i == ACC_CMD_DOT4I8 &&
     accel_src_a_i == 32'd0 && accel_src_b_i == 32'd0)
    |-> (exec_result == 32'd0);
  endproperty
  assert property (p_dot4_zero);

  // VADD8 sanity: adding zero must return the original operand unchanged.
  property p_vadd8_identity;
    @(posedge clk) disable iff (!rst_n)
    (accel_start_i && accel_cmd_i == ACC_CMD_VADD8 && accel_src_b_i == 32'd0)
    |-> (exec_result == accel_src_a_i);
  endproperty
  assert property (p_vadd8_identity);

  // The registered result on the cycle after start must equal the
  // combinational exec_result sampled on the start cycle. This is the
  // regression check for the stale-operand bug described above.
  property p_result_matches_exec_on_start;
    @(posedge clk) disable iff (!rst_n)
    (accel_start_i && (accel_cmd_i == ACC_CMD_VADD8 ||
                        accel_cmd_i == ACC_CMD_VSUB8 ||
                        accel_cmd_i == ACC_CMD_VMAX8 ||
                        accel_cmd_i == ACC_CMD_RELU8 ||
                        accel_cmd_i == ACC_CMD_DOT4I8))
    |=> (accel_result_o == $past(exec_result));
  endproperty
  assert property (p_result_matches_exec_on_start);
`endif

endmodule

`default_nettype wire
