`default_nettype none

//------------------------------------------------------------------------------
// debug_regs.sv - TinyGPU-RV32 Debug/Status MMIO Register Block
//------------------------------------------------------------------------------
// Project     : TinyGPU-RV32
// Module      : debug_regs
// Description : Memory-mapped debug/status register block. Turns bus
//               transactions in the DEBUG_BASE..DEBUG_END window into the
//               halt/resume/step/PC-write/register-read-write control
//               signals already implemented inside rv32_core.sv, and
//               exposes CPU/accelerator status plus PASS/FAIL test-
//               signature detection for software-driven verification.
//
//               This module did not previously exist -- tinygpu_soc.sv
//               stubbed the whole debug MMIO window out
//               (ready=valid, rdata=0, error=0) with no register behind
//               it. This file implements that missing block.
//
// Address map (word offsets from DEBUG_BASE; full-word accesses only --
// bus_interconnect.sv already rejects misaligned/partial-strobe accesses
// to this window before they reach this module):
//
//   0x00  DBG_REG_STATUS        RO      packed status bits (see below)
//   0x04  DBG_REG_CONTROL       WO      one-shot command bits (see below)
//   0x08  DBG_REG_PC            RW      core PC (write only takes effect
//                                       while the core is halted)
//   0x0C  DBG_REG_REG_SELECT    RW      register index [4:0] used by
//                                       DBG_REG_REG_DATA
//   0x10  DBG_REG_REG_DATA      RW      read/write regfile[REG_SELECT]
//                                       (write only takes effect while
//                                       the core is halted)
//   0x14  DBG_REG_PASSFAIL      RW      test program writes its pass/
//                                       fail signature here; readback
//                                       returns the last written word
//   0x18  DBG_REG_TRAP_CAUSE    RO      zero-extended trap_cause_e
//   0x1C  DBG_REG_RETIRE_COUNT  RO      retire count, software-
//                                       resettable via DBG_CONTROL bit 6
//   0x20  DBG_REG_BP_ADDR        RW      hardware breakpoint compare
//                                       address (word-aligned PC value).
//                                       NOTE: this is a "break-after", not
//                                       "break-before", breakpoint -- the
//                                       core halts the cycle after the
//                                       instruction AT this address
//                                       retires, coming to rest with PC
//                                       showing the NEXT instruction
//                                       (bp_addr+4), not bp_addr itself.
//                                       This follows directly from
//                                       rv32_core.sv's own commit logic
//                                       (dbg_halt_req_i is only consulted
//                                       at a commit boundary, and that
//                                       commit's retire/PC-advance is
//                                       unconditional regardless of the
//                                       halt decision) and was not changed
//                                       here since doing so would require
//                                       modifying rv32_core.sv's FSM
//                                       itself. See debug_regs.sv's
//                                       bp_match comment for the full
//                                       trace.
//   0x24  DBG_REG_BP_CONTROL     RW      [0] enable  [1] W: clear hit
//                                       (self-clears)  [8] R: hit sticky
//   0x28  DBG_REG_PERF_CYCLE_COUNT RO    free-running cycle counter,
//                                       software-resettable via
//                                       DBG_CONTROL bit 7
//   0x2C  DBG_REG_PERF_STALL_COUNT RO    cycles where the core was
//                                       running but did not retire an
//                                       instruction that cycle (i.e. any
//                                       cycle spent in FETCH/MEM_WAIT
//                                       instead of completing DECODE_EXEC);
//                                       CPI = PERF_CYCLE_COUNT /
//                                       DBG_REG_RETIRE_COUNT, and this
//                                       register breaks that number down
//                                       into "why" for a given run.
//                                       Software-resettable via the same
//                                       DBG_CONTROL bit 7 as the cycle
//                                       counter.
//
//                                       (0x20-0x2C were previously
//                                       DBG_REG_SCR_ADDR/WDATA/RDATA/
//                                       CONTROL, reserved stubs for a
//                                       scratchpad peek/poke port that was
//                                       never wired up. Repurposed
//                                       2026-07-29 -- see git history if
//                                       the old stub behavior is needed.)
//   0x30  DBG_REG_ACCEL_STATUS  RO      {busy, done, error} mirror
//   0x34  DBG_REG_ACCEL_RESULT  RO      last accelerator result mirror
//
// DBG_REG_STATUS bit layout (authoritative positions are in
// tinygpu_pkg.sv; duplicated here as comments only):
//   [0]  cpu_halted        [1] cpu_trap          [2] cpu_debug_halt
//   [3]  cpu_running       [4] accel_busy        [5] accel_done
//   [6]  accel_error       [7] pass_seen         [8] fail_seen
//   [9]  debug_cmd_busy (tied 0 -- this block is zero-wait-state)
//   [10] debug_cmd_error
//
// DBG_REG_CONTROL bit layout (write 1 to act; each bit self-clears):
//   [0] halt request              -> one-shot pulse into rv32_core
//   [1] resume request            -> one-shot pulse into rv32_core
//                                     (this is also what clears trap
//                                     state, since rv32_core only clears
//                                     trap_cause via resume/step today)
//   [2] single-step request       -> one-shot pulse into rv32_core
//   [3] clear-trap                -> accepted, no independent core-side
//                                     effect yet (see note above)
//   [4] clear PASS/FAIL sticky bits
//   [5] clear debug_cmd_error sticky bit
//   [6] reset retire counter      -> implemented as a local baseline
//                                     subtraction, since rv32_core's own
//                                     retire counter only resets on
//                                     rst_n and has no software-reset
//                                     input
//   [7] reset perf counters       -> clears PERF_CYCLE_COUNT and
//                                     PERF_STALL_COUNT to 0 directly
//                                     (these are owned locally by this
//                                     module, so no baseline trick needed)
//
// Bus protocol:
//   - Zero-wait-state slave: ready_o is combinational.
//   - Unmapped word offsets return rdata_o=0 and error_o=1.
//
// ASIC note:
//   - Single clock domain, active-low async reset.
//   - No latches, no combinational feedback.
//------------------------------------------------------------------------------

`include "tinygpu_pkg.sv"

module debug_regs (
    input  logic        clk,
    input  logic        rst_n,

    // -------------------------------------------------------------------
    // Bus slave interface (from bus_interconnect debug port)
    // -------------------------------------------------------------------
    input  logic        valid_i,
    input  logic        we_i,
    input  logic [31:0] addr_i,
    input  logic [31:0] wdata_i,
    input  logic [3:0]  wstrb_i,

    output logic        ready_o,
    output logic [31:0] rdata_o,
    output logic        error_o,

    // -------------------------------------------------------------------
    // CPU status inputs (from rv32_core, via tinygpu_soc)
    // -------------------------------------------------------------------
    input  logic        cpu_halted_i,
    input  logic        cpu_trap_i,
    input  logic        cpu_debug_halt_i,
    input  trap_cause_e trap_cause_i,
    input  logic [31:0] pc_i,
    input  logic [31:0] retire_count_i,

    // -------------------------------------------------------------------
    // Register file debug read port (from rv32_core, combinational)
    // -------------------------------------------------------------------
    input  logic [31:0] dbg_reg_read_data_i,

    // -------------------------------------------------------------------
    // Accelerator status inputs (from tinygpu_soc internal wires)
    // -------------------------------------------------------------------
    input  logic        accel_busy_i,
    input  logic        accel_done_i,
    input  logic        accel_error_i,
    input  logic [31:0] accel_result_i,

    // -------------------------------------------------------------------
    // Debug control outputs to rv32_core
    // -------------------------------------------------------------------
    output logic        dbg_halt_req_o,
    output logic        dbg_resume_req_o,
    output logic        dbg_step_req_o,

    output logic        dbg_pc_write_en_o,
    output logic [31:0] dbg_pc_write_data_o,

    output logic [4:0]  dbg_reg_sel_o,
    output logic        dbg_reg_write_en_o,
    output logic [31:0] dbg_reg_write_data_o
);

  // -------------------------------------------------------------------
  // Local word-offset decode
  // -------------------------------------------------------------------
  // DEBUG window is 256 bytes => 64 words; 6 bits is enough.

  logic [5:0] word_off;
  assign word_off = addr_i[7:2];

  localparam logic [5:0] WOFF_STATUS       = 6'h00;
  localparam logic [5:0] WOFF_CONTROL      = 6'h01;
  localparam logic [5:0] WOFF_PC           = 6'h02;
  localparam logic [5:0] WOFF_REG_SELECT   = 6'h03;
  localparam logic [5:0] WOFF_REG_DATA     = 6'h04;
  localparam logic [5:0] WOFF_PASSFAIL     = 6'h05;
  localparam logic [5:0] WOFF_TRAP_CAUSE   = 6'h06;
  localparam logic [5:0] WOFF_RETIRE_COUNT = 6'h07;
  localparam logic [5:0] WOFF_BP_ADDR      = 6'h08;
  localparam logic [5:0] WOFF_BP_CONTROL   = 6'h09;
  localparam logic [5:0] WOFF_PERF_CYCLE   = 6'h0A;
  localparam logic [5:0] WOFF_PERF_STALL   = 6'h0B;
  localparam logic [5:0] WOFF_ACCEL_STATUS = 6'h0C;
  localparam logic [5:0] WOFF_ACCEL_RESULT = 6'h0D;

  logic word_off_valid;
  always_comb begin
    unique case (word_off)
      WOFF_STATUS, WOFF_CONTROL, WOFF_PC, WOFF_REG_SELECT, WOFF_REG_DATA,
      WOFF_PASSFAIL, WOFF_TRAP_CAUSE, WOFF_RETIRE_COUNT,
      WOFF_BP_ADDR, WOFF_BP_CONTROL, WOFF_PERF_CYCLE, WOFF_PERF_STALL,
      WOFF_ACCEL_STATUS, WOFF_ACCEL_RESULT: word_off_valid = 1'b1;
      default: word_off_valid = 1'b0;
    endcase
  end

  // -------------------------------------------------------------------
  // Registers
  // -------------------------------------------------------------------

  logic [4:0]  reg_sel_q;
  logic [31:0] passfail_q;
  logic        pass_seen_q;
  logic        fail_seen_q;
  logic        debug_cmd_error_q;
  logic [31:0] retire_base_q;

  // scr_addr_q/scr_wdata_q removed 2026-07-27 (was already documented as
  // non-functional stubs, never wired to real scratchpad access). That
  // register space is repurposed below into a real hardware breakpoint
  // and free-running performance counters.

  // Hardware breakpoint.
  logic [31:0] bp_addr_q;
  logic        bp_enable_q;
  logic        bp_hit_q;
  logic        bp_match;
  logic        bp_match_prev_q;
  logic        bp_halt_level_q;

  // Free-running performance counters. Owned locally by this module (not
  // read from rv32_core), so they can be reset directly rather than via
  // the baseline-subtraction trick DBG_REG_RETIRE_COUNT needs.
  logic [31:0] cycle_count_q;
  logic [31:0] stall_count_q;
  logic [31:0] retire_count_prev_q;
  logic        instr_retired_this_cycle;

  // One-shot pulse registers.
  logic        halt_req_q;
  logic        resume_req_q;
  logic        step_req_q;
  logic        pc_write_en_q;
  logic [31:0] pc_write_data_q;
  logic        reg_write_en_q;
  logic [31:0] reg_write_data_q;

  // -------------------------------------------------------------------
  // Hardware breakpoint match (combinational) and retire-edge detect
  // -------------------------------------------------------------------
  // bp_match is a level signal: true for every cycle the core sits at the
  // breakpoint PC while running and enabled. It can stay true for more
  // than one cycle for a single instruction -- specifically, any
  // load/store that spends multiple cycles in rv32_core's MEM_WAIT state
  // shows the same pc_i throughout.
  //
  // IMPORTANT, discovered by tracing rv32_core.sv's own commit logic
  // rather than assumed: dbg_halt_req_i is only consulted at the exact
  // cycle the CURRENTLY MATCHING instruction commits, and that
  // instruction's retire/PC-advance is unconditional in that same commit
  // branch regardless of the halt decision. This means a breakpoint built
  // on this mechanism halts the core the cycle AFTER the target
  // instruction retires, with the core coming to rest at bp_addr+4 (the
  // next instruction), not AT bp_addr with that instruction not yet
  // executed. This is a real, deliberate, documented semantic -- a
  // "break-after" hardware breakpoint, not "break-before" -- not a bug,
  // but genuinely different from the textbook definition of a breakpoint
  // and worth knowing before relying on it. Achieving break-before
  // semantics would require rv32_core.sv's FSM to gate commit itself on
  // the match, which this change deliberately does not do (higher
  // regression risk, out of scope for a debug_regs.sv-only change).
  //
  // Consequence for the halt request: because dbg_halt_req_i is sampled
  // only at that one eventual commit cycle -- which, for a multi-cycle
  // instruction, can be several cycles after bp_match FIRST became true --
  // a one-shot edge-triggered pulse (fired only on bp_match's rising
  // edge) would self-clear long before that commit cycle arrives, and the
  // breakpoint would silently never fire for any load/store sitting in
  // MEM_WAIT. bp_halt_level_q below is a plain one-cycle-delayed
  // level-follower of bp_match instead of an edge pulse, specifically so
  // it stays asserted for the entire span of a multi-cycle instruction,
  // guaranteeing it is still asserted at whatever cycle the eventual
  // commit happens.
  assign bp_match = bp_enable_q && !cpu_halted_i && (pc_i == bp_addr_q);

  // A cycle counts as "retired" for stall-counting purposes if
  // retire_count_i changed since last cycle. This needs no new port from
  // rv32_core -- retire_count_i is already wired in for DBG_REG_RETIRE_COUNT.
  assign instr_retired_this_cycle = (retire_count_i != retire_count_prev_q);

  // -------------------------------------------------------------------
  // Zero-wait-state ready / error
  // -------------------------------------------------------------------

  always_comb begin
    ready_o = valid_i;
    error_o = valid_i && !word_off_valid;
  end

  // -------------------------------------------------------------------
  // Read mux
  // -------------------------------------------------------------------

  logic [31:0] status_word;
  logic [31:0] retire_count_display;

  always_comb begin
    status_word = {
      21'd0,               // [31:11] reserved
      debug_cmd_error_q,   // [10]
      1'b0,                // [9]  debug_cmd_busy: always idle here
      fail_seen_q,         // [8]
      pass_seen_q,         // [7]
      accel_error_i,       // [6]
      accel_done_i,        // [5]
      accel_busy_i,        // [4]
      !cpu_halted_i,       // [3]  cpu_running
      cpu_debug_halt_i,    // [2]
      cpu_trap_i,          // [1]
      cpu_halted_i         // [0]
    };
    retire_count_display = retire_count_i - retire_base_q;
  end

  always_comb begin
    rdata_o = 32'd0;

    if (valid_i) begin
      unique case (word_off)
        WOFF_STATUS:       rdata_o = status_word;
        WOFF_CONTROL:      rdata_o = 32'd0;
        WOFF_PC:           rdata_o = pc_i;
        WOFF_REG_SELECT:   rdata_o = {27'd0, reg_sel_q};
        WOFF_REG_DATA:     rdata_o = dbg_reg_read_data_i;
        WOFF_PASSFAIL:     rdata_o = passfail_q;
        WOFF_TRAP_CAUSE:   rdata_o = {24'd0, trap_cause_i};
        WOFF_RETIRE_COUNT: rdata_o = retire_count_display;
        WOFF_BP_ADDR:      rdata_o = bp_addr_q;
        WOFF_BP_CONTROL:   rdata_o = {23'd0, bp_hit_q, 7'd0, bp_enable_q};
        WOFF_PERF_CYCLE:   rdata_o = cycle_count_q;
        WOFF_PERF_STALL:   rdata_o = stall_count_q;
        WOFF_ACCEL_STATUS: rdata_o = {29'd0, accel_error_i, accel_done_i, accel_busy_i};
        WOFF_ACCEL_RESULT: rdata_o = accel_result_i;
        default:           rdata_o = 32'd0;
      endcase
    end
  end

  // -------------------------------------------------------------------
  // Write handling / pulse generation / sticky bits
  // -------------------------------------------------------------------

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      reg_sel_q         <= 5'd0;
      passfail_q        <= 32'd0;
      pass_seen_q       <= 1'b0;
      fail_seen_q       <= 1'b0;
      debug_cmd_error_q <= 1'b0;
      retire_base_q     <= 32'd0;

      bp_addr_q         <= 32'd0;
      bp_enable_q       <= 1'b0;
      bp_hit_q          <= 1'b0;
      bp_match_prev_q   <= 1'b0;
      bp_halt_level_q   <= 1'b0;

      cycle_count_q       <= 32'd0;
      stall_count_q       <= 32'd0;
      retire_count_prev_q <= 32'd0;

      halt_req_q       <= 1'b0;
      resume_req_q     <= 1'b0;
      step_req_q       <= 1'b0;
      pc_write_en_q    <= 1'b0;
      pc_write_data_q  <= 32'd0;
      reg_write_en_q   <= 1'b0;
      reg_write_data_q <= 32'd0;
    end else begin
      // Default: every pulse output is one cycle wide. Note halt_req_q is
      // now purely the SOFTWARE-requested halt pulse -- the breakpoint's
      // halt contribution is the separate bp_halt_level_q register below,
      // combined into dbg_halt_req_o by the continuous assignment further
      // down. See bp_match's comment above for why these two needed to be
      // decoupled rather than sharing one flop.
      halt_req_q     <= 1'b0;
      resume_req_q   <= 1'b0;
      step_req_q     <= 1'b0;
      pc_write_en_q  <= 1'b0;
      reg_write_en_q <= 1'b0;

      // Breakpoint bookkeeping: always tracked, regardless of any bus
      // transaction this cycle.
      bp_match_prev_q <= bp_match;
      bp_halt_level_q <= bp_match;
      bp_hit_q        <= bp_hit_q | (bp_match && !bp_match_prev_q);

      // Performance counters: always tracked, regardless of any bus
      // transaction this cycle. cycle_count_q is free-running;
      // stall_count_q only increments while the core is running (not
      // intentionally halted for debug) and didn't retire an instruction
      // this cycle -- i.e. every FETCH/MEM_WAIT cycle of a multi-cycle
      // instruction counts as a stall, which is the fair characterization
      // for a non-pipelined core: CPI = PERF_CYCLE_COUNT / RETIRE_COUNT,
      // and PERF_STALL_COUNT explains where those extra cycles went.
      retire_count_prev_q <= retire_count_i;
      cycle_count_q       <= cycle_count_q + 32'd1;
      if (!cpu_halted_i && !instr_retired_this_cycle) begin
        stall_count_q <= stall_count_q + 32'd1;
      end

      if (valid_i && we_i) begin
        unique case (word_off)

          // -----------------------------------------------------
          // DBG_REG_CONTROL: one-shot command bits
          // -----------------------------------------------------
          WOFF_CONTROL: begin
            if (wdata_i[DBG_CONTROL_HALT_REQ_BIT])   halt_req_q   <= 1'b1;
            if (wdata_i[DBG_CONTROL_RESUME_REQ_BIT]) resume_req_q <= 1'b1;
            if (wdata_i[DBG_CONTROL_STEP_REQ_BIT])   step_req_q   <= 1'b1;
            // CLEAR_TRAP_BIT: accepted, no independent core-side effect --
            // rv32_core only clears trap_cause via resume/step today.
            if (wdata_i[DBG_CONTROL_CLEAR_PASSFAIL_BIT]) begin
              pass_seen_q <= 1'b0;
              fail_seen_q <= 1'b0;
            end
            if (wdata_i[DBG_CONTROL_CLEAR_DEBUG_ERROR_BIT]) begin
              debug_cmd_error_q <= 1'b0;
            end
            if (wdata_i[DBG_CONTROL_RESET_RETIRE_COUNT_BIT]) begin
              retire_base_q <= retire_count_i;
            end
            if (wdata_i[DBG_CONTROL_RESET_PERF_COUNTERS_BIT]) begin
              cycle_count_q <= 32'd0;
              stall_count_q <= 32'd0;
            end
          end

          // -----------------------------------------------------
          // DBG_REG_PC: write only takes effect while halted.
          // rv32_core silently ignores a misaligned PC write, so this
          // module additionally flags debug_cmd_error for visibility.
          // -----------------------------------------------------
          WOFF_PC: begin
            if (cpu_halted_i) begin
              pc_write_en_q   <= 1'b1;
              pc_write_data_q <= wdata_i;
              if (wdata_i[1:0] != 2'b00) begin
                debug_cmd_error_q <= 1'b1;
              end
            end else begin
              debug_cmd_error_q <= 1'b1;
            end
          end

          WOFF_REG_SELECT: begin
            reg_sel_q <= wdata_i[4:0];
          end

          // -----------------------------------------------------
          // DBG_REG_REG_DATA: write only takes effect while halted.
          // -----------------------------------------------------
          WOFF_REG_DATA: begin
            if (cpu_halted_i) begin
              reg_write_en_q   <= 1'b1;
              reg_write_data_q <= wdata_i;
            end else begin
              debug_cmd_error_q <= 1'b1;
            end
          end

          // -----------------------------------------------------
          // DBG_REG_PASSFAIL: test-program signature write.
          // -----------------------------------------------------
          WOFF_PASSFAIL: begin
            passfail_q <= wdata_i;
            if (wdata_i == PASS_SIGNATURE) begin
              pass_seen_q <= 1'b1;
            end else if (wdata_i[31:16] == FAIL_UPPER16) begin
              fail_seen_q <= 1'b1;
            end
          end

          // -----------------------------------------------------
          // DBG_REG_BP_ADDR: hardware breakpoint compare address.
          // No alignment enforcement here deliberately -- pc_i will
          // simply never equal an address the core can't fetch from, so
          // a misaligned bp_addr_q is inert rather than an error. That
          // mirrors how a real value never matching is safer than
          // silently rounding it to something the user didn't ask for.
          // -----------------------------------------------------
          WOFF_BP_ADDR: begin
            bp_addr_q <= wdata_i;
          end

          // -----------------------------------------------------
          // DBG_REG_BP_CONTROL: enable bit is level-sensitive (holds its
          // written value); clear-hit bit is one-shot, same "write 1 to
          // act" convention as DBG_REG_CONTROL.
          // -----------------------------------------------------
          WOFF_BP_CONTROL: begin
            bp_enable_q <= wdata_i[DBG_BP_CONTROL_ENABLE_BIT];
            if (wdata_i[DBG_BP_CONTROL_CLEAR_HIT_BIT]) begin
              bp_hit_q <= 1'b0;
            end
          end

          // -----------------------------------------------------
          // DBG_REG_PERF_CYCLE_COUNT / PERF_STALL_COUNT are read-only;
          // a write here is a no-op like any other RO register (falls
          // through to `default`, which already just does nothing).
          // -----------------------------------------------------

          default: begin
            // Unmapped or read-only register write: no state change.
            // (error_o already flags unmapped offsets combinationally.)
          end
        endcase
      end
    end
  end

  // -------------------------------------------------------------------
  // Outputs
  // -------------------------------------------------------------------

  assign dbg_halt_req_o   = halt_req_q | bp_halt_level_q;
  assign dbg_resume_req_o = resume_req_q;
  assign dbg_step_req_o   = step_req_q;

  assign dbg_pc_write_en_o   = pc_write_en_q;
  assign dbg_pc_write_data_o = pc_write_data_q;

  assign dbg_reg_sel_o        = reg_sel_q;
  assign dbg_reg_write_en_o   = reg_write_en_q;
  assign dbg_reg_write_data_o = reg_write_data_q;

`ifdef ASSERT_ON
  // -------------------------------------------------------------------
  // Assertions
  // -------------------------------------------------------------------

  // A valid request always receives ready in this zero-wait-state block.
  property p_valid_gets_ready;
    @(posedge clk) disable iff (!rst_n)
    valid_i |-> ready_o;
  endproperty
  assert property (p_valid_gets_ready);

  // Unmapped word offsets must error.
  property p_unmapped_errors;
    @(posedge clk) disable iff (!rst_n)
    valid_i && !word_off_valid |-> error_o;
  endproperty
  assert property (p_unmapped_errors);

  // Mapped word offsets must not error.
  property p_mapped_no_error;
    @(posedge clk) disable iff (!rst_n)
    valid_i && word_off_valid |-> !error_o;
  endproperty
  assert property (p_mapped_no_error);

  // Halt/resume/step SOFTWARE requests are single-cycle pulses. Scoped to
  // exclude cycles where the breakpoint's level-follower is also holding
  // dbg_halt_req_o high -- that sustained assertion across a multi-cycle
  // instruction's MEM_WAIT is correct by design (see bp_match's comment),
  // not a violation of "software requests pulse for one cycle."
  property p_halt_pulse_single_cycle;
    @(posedge clk) disable iff (!rst_n)
    (dbg_halt_req_o && !bp_halt_level_q) |-> ##1 !(dbg_halt_req_o && !bp_halt_level_q);
  endproperty
  assert property (p_halt_pulse_single_cycle);

  property p_resume_pulse_single_cycle;
    @(posedge clk) disable iff (!rst_n)
    dbg_resume_req_o |-> ##1 !dbg_resume_req_o;
  endproperty
  assert property (p_resume_pulse_single_cycle);

  property p_step_pulse_single_cycle;
    @(posedge clk) disable iff (!rst_n)
    dbg_step_req_o |-> ##1 !dbg_step_req_o;
  endproperty
  assert property (p_step_pulse_single_cycle);

  // Writing the exact PASS signature must set pass_seen on the next cycle.
  property p_pass_signature_detected;
    @(posedge clk) disable iff (!rst_n)
    (valid_i && we_i && word_off == WOFF_PASSFAIL && wdata_i == PASS_SIGNATURE)
    |=> pass_seen_q;
  endproperty
  assert property (p_pass_signature_detected);

  // Writing a DEAD_xxxx pattern must set fail_seen on the next cycle.
  property p_fail_signature_detected;
    @(posedge clk) disable iff (!rst_n)
    (valid_i && we_i && word_off == WOFF_PASSFAIL &&
     wdata_i[31:16] == FAIL_UPPER16 && wdata_i != PASS_SIGNATURE)
    |=> fail_seen_q;
  endproperty
  assert property (p_fail_signature_detected);

  // A PC write while not halted must never produce a pulse.
  property p_pc_write_requires_halt;
    @(posedge clk) disable iff (!rst_n)
    !cpu_halted_i |=> !dbg_pc_write_en_o;
  endproperty
  assert property (p_pc_write_requires_halt);

  // Breakpoint match while enabled and running must eventually halt --
  // bounded to 4 cycles, this core's worst-case single-instruction latency
  // (FETCH + DECODE_EXEC + MEM_WAIT + one cycle for cpu_halted_o to reflect
  // the resulting CPU_DEBUG_HALT state). NOT same-cycle/next-cycle: the
  // breakpoint's target instruction is allowed to retire first (see the
  // "break-after, not break-before" note on bp_match above), and for a
  // load/store that retire can itself take multiple cycles.
  property p_breakpoint_halts;
    @(posedge clk) disable iff (!rst_n)
    (bp_match && !bp_match_prev_q) |-> ##[1:4] cpu_halted_i;
  endproperty
  assert property (p_breakpoint_halts);

  // The hit flag must be set on exactly the cycle after a fresh match,
  // and must stay set until explicitly cleared (sticky, not level).
  property p_breakpoint_hit_sticky;
    @(posedge clk) disable iff (!rst_n)
    (bp_match && !bp_match_prev_q) |=> bp_hit_q;
  endproperty
  assert property (p_breakpoint_hit_sticky);

  // A disabled breakpoint must never assert a halt request on its own
  // (distinguishes "the comparator happens to be true" from "the
  // breakpoint is actually armed").
  property p_disabled_breakpoint_never_halts;
    @(posedge clk) disable iff (!rst_n)
    !bp_enable_q |-> !(bp_match && !bp_match_prev_q);
  endproperty
  assert property (p_disabled_breakpoint_never_halts);

  // The cycle counter must never stop incrementing (free-running, by
  // construction, unless software resets it -- this property is really
  // guarding against a future edit accidentally gating it on something).
  property p_cycle_counter_free_running;
    @(posedge clk) disable iff (!rst_n)
    !(valid_i && we_i && word_off == WOFF_CONTROL &&
      wdata_i[DBG_CONTROL_RESET_PERF_COUNTERS_BIT])
    |=> (cycle_count_q == $past(cycle_count_q) + 32'd1);
  endproperty
  assert property (p_cycle_counter_free_running);

  // Stall count must never increment on a cycle where an instruction
  // retired (the two are meant to be complementary, not overlapping).
  property p_stall_excludes_retire;
    @(posedge clk) disable iff (!rst_n)
    instr_retired_this_cycle |=> (stall_count_q == $past(stall_count_q));
  endproperty
  assert property (p_stall_excludes_retire);
`endif

endmodule

`default_nettype wire
