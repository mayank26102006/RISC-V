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
//   0x20  DBG_REG_SCR_ADDR      RW  \
//   0x24  DBG_REG_SCR_WDATA     RW   \  Reserved for a future direct
//   0x28  DBG_REG_SCR_RDATA     RO   /  scratchpad peek/poke debug port.
//   0x2C  DBG_REG_SCR_CONTROL   RW  /   NOT WIRED TO MEMORY YET -- needs
//                                       a 3rd port added to
//                                       scratchpad.sv. Any CONTROL start
//                                       attempt here is safely accepted
//                                       but sets DBG_STATUS_DEBUG_CMD_
//                                       ERROR_BIT so software can detect
//                                       it, instead of silently returning
//                                       wrong data.
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
  localparam logic [5:0] WOFF_SCR_ADDR     = 6'h08;
  localparam logic [5:0] WOFF_SCR_WDATA    = 6'h09;
  localparam logic [5:0] WOFF_SCR_RDATA    = 6'h0A;
  localparam logic [5:0] WOFF_SCR_CONTROL  = 6'h0B;
  localparam logic [5:0] WOFF_ACCEL_STATUS = 6'h0C;
  localparam logic [5:0] WOFF_ACCEL_RESULT = 6'h0D;

  logic word_off_valid;
  always_comb begin
    unique case (word_off)
      WOFF_STATUS, WOFF_CONTROL, WOFF_PC, WOFF_REG_SELECT, WOFF_REG_DATA,
      WOFF_PASSFAIL, WOFF_TRAP_CAUSE, WOFF_RETIRE_COUNT,
      WOFF_SCR_ADDR, WOFF_SCR_WDATA, WOFF_SCR_RDATA, WOFF_SCR_CONTROL,
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

  // Reserved/stub scratch-debug registers (not wired to memory yet).
  logic [31:0] scr_addr_q;
  logic [31:0] scr_wdata_q;

  // One-shot pulse registers.
  logic        halt_req_q;
  logic        resume_req_q;
  logic        step_req_q;
  logic        pc_write_en_q;
  logic [31:0] pc_write_data_q;
  logic        reg_write_en_q;
  logic [31:0] reg_write_data_q;

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
        WOFF_SCR_ADDR:     rdata_o = scr_addr_q;
        WOFF_SCR_WDATA:    rdata_o = scr_wdata_q;
        WOFF_SCR_RDATA:    rdata_o = 32'd0; // stub: no real memory behind this yet
        WOFF_SCR_CONTROL:  rdata_o = 32'd0; // stub: always reports idle
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

      scr_addr_q  <= 32'd0;
      scr_wdata_q <= 32'd0;

      halt_req_q       <= 1'b0;
      resume_req_q     <= 1'b0;
      step_req_q       <= 1'b0;
      pc_write_en_q    <= 1'b0;
      pc_write_data_q  <= 32'd0;
      reg_write_en_q   <= 1'b0;
      reg_write_data_q <= 32'd0;
    end else begin
      // Default: every pulse output is one cycle wide.
      halt_req_q     <= 1'b0;
      resume_req_q   <= 1'b0;
      step_req_q     <= 1'b0;
      pc_write_en_q  <= 1'b0;
      reg_write_en_q <= 1'b0;

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

          WOFF_SCR_ADDR:  scr_addr_q  <= wdata_i;
          WOFF_SCR_WDATA: scr_wdata_q <= wdata_i;

          // -----------------------------------------------------
          // DBG_REG_SCR_CONTROL: stub. Flags debug_cmd_error instead of
          // silently pretending to access memory. Needs a 3rd port on
          // scratchpad.sv before this can do anything real.
          // -----------------------------------------------------
          WOFF_SCR_CONTROL: begin
            debug_cmd_error_q <= 1'b1;
          end

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

  assign dbg_halt_req_o   = halt_req_q;
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

  // Halt/resume/step requests are single-cycle pulses.
  property p_halt_pulse_single_cycle;
    @(posedge clk) disable iff (!rst_n)
    dbg_halt_req_o |-> ##1 !dbg_halt_req_o;
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
`endif

endmodule

`default_nettype wire
