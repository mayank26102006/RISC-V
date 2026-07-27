`default_nettype none

//------------------------------------------------------------------------------
// ext_loader.sv - TinyGPU-RV32 External Program Loader
//------------------------------------------------------------------------------
// Project     : TinyGPU-RV32
// Module      : ext_loader
// Description : A genuine external path to load a program into the shared
//               scratchpad from outside the fabricated chip, using only 2
//               input pins and 1 status output pin -- no hierarchical
//               testbench access required, so it works identically in
//               RTL simulation, gate-level simulation, and on real
//               silicon.
//
// Why this exists (2026-07-27):
//   Without this, the chip has no way to get a program into memory once
//   fabricated -- memory powers up in an undefined state, and the only
//   previously-existing write path (the debug module's SCR_ADDR/SCR_WDATA)
//   is reachable only through the CPU's own bus, which is useless before
//   the CPU has anything to execute. This module is the actual fix: a
//   real, external, pin-level load path.
//
// Protocol:
//   - Host asserts ext_load_mode_i and holds it high for the entire load.
//     While high, this module also forces the CPU into reset (see
//     force_cpu_reset_o), so there is never any contention for the
//     memory bus between the loader and the running core -- this is a
//     priority mux at the memory port, not true multi-master
//     arbitration.
//   - Host then shifts in 32 bits, MSB-first, one bit per clock cycle
//     on ext_load_bit_i, sampled whenever ext_load_ready_o is high
//     (a real handshake -- the host does not need to precisely count
//     cycles).
//   - Every complete 32-bit word triggers one write to the shared
//     scratchpad at an internal, auto-incrementing address counter
//     (starting at 0), then the counter advances to the next word.
//   - If the address counter would exceed the scratchpad's actual
//     word count, ext_load_ready_o deasserts and further bits are
//     ignored (a safety guard against writing out of bounds).
//   - Dropping ext_load_mode_i mid-word discards the partial word (no
//     partial/garbage writes ever reach memory) and returns to IDLE.
//   - Dropping ext_load_mode_i between words ends loading cleanly; the
//     CPU is released from reset on the same edge and begins fetching
//     from address 0 -- the program just loaded.
//
// HANDSHAKE NOTE (2026-07-27): ext_load_ready_o is a REGISTERED output,
// not combinational. This was found the hard way, via actual simulation:
// an earlier combinational version asserted ready on the very same edge
// state_q transitioned into "active", giving a real host zero warning
// cycles to have its first bit ready -- causing every bit thereafter to
// be silently off by one. A follow-up attempt inserted an extra
// transitional state to "settle" before asserting ready, but that just
// moved the identical race to the transitional state's own entry edge
// instead of removing it. Registering ready removes the entire class of
// bug by construction: ready_q always reflects "was the loader genuinely
// ready as of last cycle", guaranteeing the host sees at least one full
// cycle where mode is asserted and ready is still low before ready can
// ever go high, every time.
//
// ASIC note:
//   - Single clock domain, active-low async reset.
//   - No latches, no combinational feedback.
//------------------------------------------------------------------------------

module ext_loader #(
    parameter int unsigned SCRATCHPAD_WORDS = 16
) (
    input  logic clk,
    input  logic rst_n,

    // External pins
    input  logic ext_load_mode_i,
    input  logic ext_load_bit_i,
    output logic ext_load_ready_o,

    // Forces the CPU to stay in reset for the entire duration
    // ext_load_mode_i is asserted.
    output logic force_cpu_reset_o,

    // Write-only bus master port into the shared scratchpad (priority
    // mux'd at the memory port in tinygpu_soc.sv; see module header).
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
  // One extra bit beyond ADDR_BITS so this counter can actually reach
  // SCRATCHPAD_WORDS (out of range) instead of silently wrapping to 0
  // at exactly the point it needs to be detected as out of range.
  logic [ADDR_BITS:0]  word_addr_q, word_addr_d;

  // Registered ready -- see HANDSHAKE NOTE above. Always reflects
  // whether the loader was genuinely ready to accept a bit as of the
  // *previous* cycle, so it inherently lags state_q by one full cycle
  // no matter how state_q itself transitions.
  logic ext_load_ready_q, ext_load_ready_d;
  assign ext_load_ready_o = ext_load_ready_q;

  logic word_complete;
  logic addr_in_range;

  assign word_complete = (bit_count_q == 6'd31);
  assign addr_in_range = (word_addr_q < SCRATCHPAD_WORDS[ADDR_BITS:0]);

  // -------------------------------------------------------------------
  // Next-state / next-value logic
  // -------------------------------------------------------------------

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
        // ext_load_ready_d stays 0 -- the registered output guarantees
        // at least one full cycle of ready=0 after mode first asserts,
        // regardless of how quickly state_q itself transitions.
      end

      LOAD_ACTIVE: begin
        if (!ext_load_mode_i) begin
          // Host dropped load mode -- end cleanly, discard any partial
          // word in progress (no partial writes ever reach memory).
          state_d = LOAD_IDLE;
        end else if (!addr_in_range) begin
          // Safety guard: out of scratchpad bounds, stop accepting bits.
          ext_load_ready_d = 1'b0;
        end else begin
          // Only actually sample/shift a bit if the host was genuinely
          // told we were ready last cycle (ext_load_ready_q) -- this is
          // what keeps the internal bit-sampling logic perfectly
          // consistent with what the host sees on the ready pin.
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

  // -------------------------------------------------------------------
  // Sequential state update
  // -------------------------------------------------------------------

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
  // -------------------------------------------------------------------
  // Assertions
  // -------------------------------------------------------------------

  // A write is only ever issued while genuinely in LOAD_ACTIVE.
  property p_write_only_when_active;
    @(posedge clk) disable iff (!rst_n)
    mem_valid_o |-> (state_q == LOAD_ACTIVE);
  endproperty
  assert property (p_write_only_when_active);

  // force_cpu_reset_o always mirrors the external pin directly (the CPU
  // must never be released mid-word, only exactly when the host drops
  // ext_load_mode_i).
  property p_force_reset_mirrors_pin;
    @(posedge clk) disable iff (!rst_n)
    force_cpu_reset_o == ext_load_mode_i;
  endproperty
  assert property (p_force_reset_mirrors_pin);

  // Address never exceeds the scratchpad bound.
  property p_addr_in_bounds;
    @(posedge clk) disable iff (!rst_n)
    mem_valid_o |-> (word_addr_q < SCRATCHPAD_WORDS[ADDR_BITS:0]);
  endproperty
  assert property (p_addr_in_bounds);
`endif

endmodule

`default_nettype wire
