`default_nettype none

//------------------------------------------------------------------------------
// debug_regs.sv - TinyGPU-RV32 Debug/Status MMIO Register Block
// Description : Memory-mapped debug/status register block.
//------------------------------------------------------------------------------

`include "tinygpu_pkg.sv"

module debug_regs (
    input  logic        clk,
    input  logic        rst_n,

    input  logic        valid_i,
    input  logic        we_i,
    input  logic [31:0] addr_i,
    input  logic [31:0] wdata_i,
    input  logic [3:0]  wstrb_i,

    output logic        ready_o,
    output logic [31:0] rdata_o,
    output logic        error_o,

    input  logic        cpu_halted_i,
    input  logic        cpu_trap_i,
    input  logic        cpu_debug_halt_i,
    input  trap_cause_e trap_cause_i,
    input  logic [31:0] pc_i,
    input  logic [31:0] retire_count_i,

    input  logic [31:0] dbg_reg_read_data_i,

    input  logic        accel_busy_i,
    input  logic        accel_done_i,
    input  logic        accel_error_i,
    input  logic [31:0] accel_result_i,

    output logic        dbg_halt_req_o,
    output logic        dbg_resume_req_o,
    output logic        dbg_step_req_o,

    output logic        dbg_pc_write_en_o,
    output logic [31:0] dbg_pc_write_data_o,

    output logic [4:0]  dbg_reg_sel_o,
    output logic        dbg_reg_write_en_o,
    output logic [31:0] dbg_reg_write_data_o
);

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

  logic [4:0]  reg_sel_q;
  logic [31:0] passfail_q;
  logic        pass_seen_q;
  logic        fail_seen_q;
  logic        debug_cmd_error_q;
  logic [31:0] retire_base_q;

  logic [31:0] bp_addr_q;
  logic        bp_enable_q;
  logic        bp_hit_q;
  logic        bp_match;
  logic        bp_match_prev_q;
  logic        bp_halt_level_q;

  logic [31:0] cycle_count_q;
  logic [31:0] stall_count_q;
  logic [31:0] retire_count_prev_q;
  logic        instr_retired_this_cycle;

  logic        halt_req_q;
  logic        resume_req_q;
  logic        step_req_q;
  logic        pc_write_en_q;
  logic [31:0] pc_write_data_q;
  logic        reg_write_en_q;
  logic [31:0] reg_write_data_q;

  assign bp_match = bp_enable_q && !cpu_halted_i && (pc_i == bp_addr_q);

  assign instr_retired_this_cycle = (retire_count_i != retire_count_prev_q);

  always_comb begin
    ready_o = valid_i;
    error_o = valid_i && !word_off_valid;
  end

  logic [31:0] status_word;
  logic [31:0] retire_count_display;

  always_comb begin
    status_word = {
      21'd0,
      debug_cmd_error_q,
      1'b0,
      fail_seen_q,
      pass_seen_q,
      accel_error_i,
      accel_done_i,
      accel_busy_i,
      !cpu_halted_i,
      cpu_debug_halt_i,
      cpu_trap_i,
      cpu_halted_i
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
      halt_req_q     <= 1'b0;
      resume_req_q   <= 1'b0;
      step_req_q     <= 1'b0;
      pc_write_en_q  <= 1'b0;
      reg_write_en_q <= 1'b0;

      bp_match_prev_q <= bp_match;
      bp_halt_level_q <= bp_match;
      bp_hit_q        <= bp_hit_q | (bp_match && !bp_match_prev_q);

      retire_count_prev_q <= retire_count_i;
      cycle_count_q       <= cycle_count_q + 32'd1;
      if (!cpu_halted_i && !instr_retired_this_cycle) begin
        stall_count_q <= stall_count_q + 32'd1;
      end

      if (valid_i && we_i) begin
        unique case (word_off)

          WOFF_CONTROL: begin
            if (wdata_i[DBG_CONTROL_HALT_REQ_BIT])   halt_req_q   <= 1'b1;
            if (wdata_i[DBG_CONTROL_RESUME_REQ_BIT]) resume_req_q <= 1'b1;
            if (wdata_i[DBG_CONTROL_STEP_REQ_BIT])   step_req_q   <= 1'b1;
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

          WOFF_REG_DATA: begin
            if (cpu_halted_i) begin
              reg_write_en_q   <= 1'b1;
              reg_write_data_q <= wdata_i;
            end else begin
              debug_cmd_error_q <= 1'b1;
            end
          end

          WOFF_PASSFAIL: begin
            passfail_q <= wdata_i;
            if (wdata_i == PASS_SIGNATURE) begin
              pass_seen_q <= 1'b1;
            end else if (wdata_i[31:16] == FAIL_UPPER16) begin
              fail_seen_q <= 1'b1;
            end
          end

          WOFF_BP_ADDR: begin
            bp_addr_q <= wdata_i;
          end

          WOFF_BP_CONTROL: begin
            bp_enable_q <= wdata_i[DBG_BP_CONTROL_ENABLE_BIT];
            if (wdata_i[DBG_BP_CONTROL_CLEAR_HIT_BIT]) begin
              bp_hit_q <= 1'b0;
            end
          end

          default: begin
          end
        endcase
      end
    end
  end

  assign dbg_halt_req_o   = halt_req_q | bp_halt_level_q;
  assign dbg_resume_req_o = resume_req_q;
  assign dbg_step_req_o   = step_req_q;

  assign dbg_pc_write_en_o   = pc_write_en_q;
  assign dbg_pc_write_data_o = pc_write_data_q;

  assign dbg_reg_sel_o        = reg_sel_q;
  assign dbg_reg_write_en_o   = reg_write_en_q;
  assign dbg_reg_write_data_o = reg_write_data_q;

`ifdef ASSERT_ON

  property p_valid_gets_ready;
    @(posedge clk) disable iff (!rst_n)
    valid_i |-> ready_o;
  endproperty
  assert property (p_valid_gets_ready);

  property p_unmapped_errors;
    @(posedge clk) disable iff (!rst_n)
    valid_i && !word_off_valid |-> error_o;
  endproperty
  assert property (p_unmapped_errors);

  property p_mapped_no_error;
    @(posedge clk) disable iff (!rst_n)
    valid_i && word_off_valid |-> !error_o;
  endproperty
  assert property (p_mapped_no_error);

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

  property p_pass_signature_detected;
    @(posedge clk) disable iff (!rst_n)
    (valid_i && we_i && word_off == WOFF_PASSFAIL && wdata_i == PASS_SIGNATURE)
    |=> pass_seen_q;
  endproperty
  assert property (p_pass_signature_detected);

  property p_fail_signature_detected;
    @(posedge clk) disable iff (!rst_n)
    (valid_i && we_i && word_off == WOFF_PASSFAIL &&
     wdata_i[31:16] == FAIL_UPPER16 && wdata_i != PASS_SIGNATURE)
    |=> fail_seen_q;
  endproperty
  assert property (p_fail_signature_detected);

  property p_pc_write_requires_halt;
    @(posedge clk) disable iff (!rst_n)
    !cpu_halted_i |=> !dbg_pc_write_en_o;
  endproperty
  assert property (p_pc_write_requires_halt);

  property p_breakpoint_halts;
    @(posedge clk) disable iff (!rst_n)
    (bp_match && !bp_match_prev_q) |-> ##[1:4] cpu_halted_i;
  endproperty
  assert property (p_breakpoint_halts);

  property p_breakpoint_hit_sticky;
    @(posedge clk) disable iff (!rst_n)
    (bp_match && !bp_match_prev_q) |=> bp_hit_q;
  endproperty
  assert property (p_breakpoint_hit_sticky);

  property p_disabled_breakpoint_never_halts;
    @(posedge clk) disable iff (!rst_n)
    !bp_enable_q |-> !(bp_match && !bp_match_prev_q);
  endproperty
  assert property (p_disabled_breakpoint_never_halts);

  property p_cycle_counter_free_running;
    @(posedge clk) disable iff (!rst_n)
    !(valid_i && we_i && word_off == WOFF_CONTROL &&
      wdata_i[DBG_CONTROL_RESET_PERF_COUNTERS_BIT])
    |=> (cycle_count_q == $past(cycle_count_q) + 32'd1);
  endproperty
  assert property (p_cycle_counter_free_running);

  property p_stall_excludes_retire;
    @(posedge clk) disable iff (!rst_n)
    instr_retired_this_cycle |=> (stall_count_q == $past(stall_count_q));
  endproperty
  assert property (p_stall_excludes_retire);

`endif

endmodule

`default_nettype wire
