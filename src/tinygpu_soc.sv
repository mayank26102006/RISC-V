`default_nettype none

//------------------------------------------------------------------------------
// tinygpu_soc.sv - TinyGPU-RV32 Top-Level SoC Integration
//------------------------------------------------------------------------------
// Project     : TinyGPU-RV32
// Module      : tinygpu_soc
// Description : Wires together rv32_core, scratchpad memory, bus
//               interconnect, debug registers, accelerator registers, and
//               vector accelerator.
//
// This is the SoC top. It is intentionally simple:
//   - single RV32I core
//   - ONE shared physical scratchpad serves both instruction fetch and
//     data access (see "Memory model" below -- this was changed from an
//     earlier two-memory Harvard layout specifically to roughly halve
//     flip-flop count for ASIC area, ahead of a Tiny Tapeout submission)
//   - optional scratchpad1 disabled by default
//   - memory-mapped accelerator + debug via bus_interconnect
//
// Memory model (unified, area-optimized 2026-07-25):
//   - rv32_core.sv's imem_valid_o is asserted only in state CPU_FETCH,
//     and its dmem_valid_o (arriving here as scratch0_valid) only in
//     state CPU_MEM_WAIT -- two different values of the same single FSM
//     state register, so the two requests are structurally guaranteed
//     to never be asserted in the same cycle (see ASSERT_ON property
//     p_imem_dmem_mutually_exclusive below, which fails loudly if that
//     is ever broken by a future core change).
//   - Because of that, a single physical scratchpad instance (u_mem) can
//     safely serve both the fetch and load/store paths through a simple
//     priority mux, instead of two separate physical memories. This
//     halves total memory flip-flop count for the same word capacity --
//     the dominant cost for ASIC tile area on Tiny Tapeout, where plain
//     flip-flop storage runs roughly 320 bits/tile.
//   - IMPORTANT CONSEQUENCE: since instructions and data now share one
//     address space, a running program's stores/loads must target
//     addresses that don't overlap the program's own instruction words
//     (word 0 through however long the program is). This wasn't a
//     concern under the old split-memory model and needs care in any
//     firmware/test program written against this SoC.
//   - Zero-wait-state either way.
//
// Debug model:
//   - debug_regs.sv (new) turns MMIO transactions in the DEBUG_BASE window
//     into the halt/resume/step/PC-write/register-read-write signals that
//     rv32_core.sv already implements. The external dbg_halt_req_i /
//     dbg_resume_req_i / dbg_step_req_i top-level pins are OR'd together
//     with debug_regs' own one-shot pulses, so the core can be controlled
//     either from raw pins (e.g. a simple testbench or the Tiny Tapeout
//     ui_in bits) or from software/an external debug probe issuing MMIO
//     writes in the DEBUG_BASE window -- both paths work simultaneously.
//   - Previously this window was a hardwired stub
//     (ready=valid, rdata=0, error=0) with nothing behind it.
//
// ASIC note:
//   - Single clock domain
//   - No generated clocks
//   - Synthesizable top
//------------------------------------------------------------------------------

`include "tinygpu_pkg.sv"

module tinygpu_soc #(
    parameter logic [31:0]      RESET_VECTOR       = RESET_VECTOR_DEFAULT,
    parameter bit                ENABLE_SCRATCHPAD1 = 1'b0,
    parameter int unsigned        SCRATCHPAD_WORDS   = 256
) (
    input  logic clk,
    input  logic rst_n,

    // Optional external debug control (can be tied off)
    input  logic dbg_halt_req_i,
    input  logic dbg_resume_req_i,
    input  logic dbg_step_req_i,

    // Status outputs
    output logic         cpu_halted_o,
    output logic         cpu_trap_o,
    output trap_cause_e  trap_cause_o,
    output logic [31:0]  dbg_pc_o,
    output logic [31:0]  dbg_retire_count_o,

    // Accelerator status outputs
    output logic accel_busy_o,
    output logic accel_done_o,
    output logic accel_error_o
);

  // -------------------------------------------------------------------
  // Instruction memory (scratchpad0) wires
  // -------------------------------------------------------------------

  logic        imem_valid;
  logic [31:0] imem_addr;
  logic        imem_ready;
  logic [31:0] imem_rdata;
  logic        imem_error;

  // -------------------------------------------------------------------
  // Data memory / bus wires
  // -------------------------------------------------------------------

  logic        dmem_valid;
  logic        dmem_we;
  logic [31:0] dmem_addr;
  logic [31:0] dmem_wdata;
  logic [3:0]  dmem_wstrb;
  logic        dmem_ready;
  logic [31:0] dmem_rdata;
  logic        dmem_error;

  // -------------------------------------------------------------------
  // Scratchpad0 wires
  // -------------------------------------------------------------------

  logic        scratch0_valid;
  logic        scratch0_we;
  logic [31:0] scratch0_addr;
  logic [31:0] scratch0_wdata;
  logic [3:0]  scratch0_wstrb;
  logic        scratch0_ready;
  logic [31:0] scratch0_rdata;
  logic        scratch0_error;

  // -------------------------------------------------------------------
  // Accelerator MMIO wires
  // -------------------------------------------------------------------

  logic        accel_valid;
  logic        accel_we;
  logic [31:0] accel_addr;
  logic [31:0] accel_wdata;
  logic [3:0]  accel_wstrb;
  logic        accel_ready;
  logic [31:0] accel_rdata;
  logic        accel_bus_error;

  logic        accel_start;
  accel_cmd_e  accel_cmd;
  logic [31:0] accel_src_a;
  logic [31:0] accel_src_b;
  logic [31:0] accel_src_c;
  logic [31:0] accel_len;
  logic [31:0] accel_dst;
  logic [31:0] accel_result;

  // -------------------------------------------------------------------
  // Debug MMIO wires
  // -------------------------------------------------------------------

  logic        debug_valid;
  logic        debug_we;
  logic [31:0] debug_addr;
  logic [31:0] debug_wdata;
  logic [3:0]  debug_wstrb;
  logic        debug_ready;
  logic [31:0] debug_rdata;
  logic        debug_error;

  // -------------------------------------------------------------------
  // Core status wires (read by both top-level ports and debug_regs)
  // -------------------------------------------------------------------

  logic        core_halted;
  logic        core_trap;
  logic        core_debug_halt;
  trap_cause_e core_trap_cause;
  logic [31:0] core_pc;
  logic [31:0] core_retire_count;

  // -------------------------------------------------------------------
  // Debug control wires between debug_regs and rv32_core
  // -------------------------------------------------------------------

  logic        dbgregs_halt_req;
  logic        dbgregs_resume_req;
  logic        dbgregs_step_req;

  logic        core_dbg_halt_req;
  logic        core_dbg_resume_req;
  logic        core_dbg_step_req;

  logic        dbg_pc_write_en;
  logic [31:0] dbg_pc_write_data;

  logic [4:0]  dbg_reg_sel;
  logic        dbg_reg_write_en;
  logic [31:0] dbg_reg_write_data;
  logic [31:0] dbg_reg_read_data;

  // External pins and MMIO-driven requests both reach the core.
  assign core_dbg_halt_req   = dbg_halt_req_i   | dbgregs_halt_req;
  assign core_dbg_resume_req = dbg_resume_req_i | dbgregs_resume_req;
  assign core_dbg_step_req   = dbg_step_req_i   | dbgregs_step_req;

  // -------------------------------------------------------------------
  // RV32 core
  // -------------------------------------------------------------------

  rv32_core #(
      .RESET_VECTOR(RESET_VECTOR)
  ) u_core (
      .clk   (clk),
      .rst_n (rst_n),

      .imem_valid_o (imem_valid),
      .imem_addr_o  (imem_addr),
      .imem_ready_i (imem_ready),
      .imem_rdata_i (imem_rdata),
      .imem_error_i (imem_error),

      .dmem_valid_o (dmem_valid),
      .dmem_we_o    (dmem_we),
      .dmem_addr_o  (dmem_addr),
      .dmem_wdata_o (dmem_wdata),
      .dmem_wstrb_o (dmem_wstrb),
      .dmem_ready_i (dmem_ready),
      .dmem_rdata_i (dmem_rdata),
      .dmem_error_i (dmem_error),

      .dbg_halt_req_i   (core_dbg_halt_req),
      .dbg_resume_req_i (core_dbg_resume_req),
      .dbg_step_req_i   (core_dbg_step_req),

      .dbg_pc_write_en_i   (dbg_pc_write_en),
      .dbg_pc_write_data_i (dbg_pc_write_data),

      .dbg_reg_read_addr_i (dbg_reg_sel),
      .dbg_reg_read_data_o (dbg_reg_read_data),
      .dbg_reg_write_en_i  (dbg_reg_write_en),
      .dbg_reg_write_addr_i(dbg_reg_sel),
      .dbg_reg_write_data_i(dbg_reg_write_data),

      .cpu_halted_o      (core_halted),
      .cpu_debug_halt_o  (core_debug_halt),
      .cpu_trap_o        (core_trap),
      .trap_cause_o      (core_trap_cause),
      .pc_o              (core_pc),
      .retire_count_o    (core_retire_count),

      .instr_retire_o   (),
      .retired_pc_o     (),
      .retired_instr_o  ()
  );

  // Top-level status outputs mirror the core's internal status wires.
  assign cpu_halted_o       = core_halted;
  assign cpu_trap_o         = core_trap;
  assign trap_cause_o       = core_trap_cause;
  assign dbg_pc_o           = core_pc;
  assign dbg_retire_count_o = core_retire_count;

  // -------------------------------------------------------------------
  // Instruction memory: scratchpad0
  // -------------------------------------------------------------------

  // -------------------------------------------------------------------
  // Shared instruction+data memory (single physical scratchpad)
  // -------------------------------------------------------------------
  // rv32_core.sv only asserts imem_valid_o while state_q==CPU_FETCH and
  // dmem_valid_o (routed here as scratch0_valid) only while
  // state_q==CPU_MEM_WAIT -- two different values of the same single
  // state register, so these two requests are structurally guaranteed
  // to never be asserted in the same cycle. That means a single shared
  // physical memory can safely serve both the fetch and the load/store
  // path with a simple priority mux instead of needing two separate
  // physical memories (which was the original Harvard-style layout and
  // doubled the flip-flop count for ASIC area purposes). imem is given
  // priority in the mux for defensiveness only -- the two are never
  // actually simultaneous, so the priority never actually resolves a
  // real conflict; see the ASSERT_ON check below, which fails loudly if
  // that assumption is ever violated by a future core change.

  logic        mem_valid;
  logic        mem_we;
  logic [31:0] mem_addr;
  logic [31:0] mem_wdata;
  logic [3:0]  mem_wstrb;
  logic        mem_ready;
  logic [31:0] mem_rdata;
  logic        mem_error;

  assign mem_valid = imem_valid | scratch0_valid;
  assign mem_we     = scratch0_valid ? scratch0_we    : 1'b0;
  assign mem_addr   = imem_valid     ? imem_addr      : scratch0_addr;
  assign mem_wdata  = scratch0_wdata;
  assign mem_wstrb  = scratch0_valid ? scratch0_wstrb : 4'b0000;

  assign imem_ready    = mem_ready & imem_valid;
  assign imem_rdata    = mem_rdata;
  assign imem_error    = mem_error & imem_valid;

  assign scratch0_ready = mem_ready & scratch0_valid & !imem_valid;
  assign scratch0_rdata = mem_rdata;
  assign scratch0_error = mem_error & scratch0_valid & !imem_valid;

`ifdef ASSERT_ON
  // If this ever fires, the mutual-exclusivity assumption above has been
  // broken by a core change and the shared-memory merge is no longer
  // safe as written -- go back to two separate physical memories.
  property p_imem_dmem_mutually_exclusive;
    @(posedge clk) disable iff (!rst_n)
    !(imem_valid && scratch0_valid);
  endproperty
  assert property (p_imem_dmem_mutually_exclusive);
`endif

  scratchpad #(
      .WORDS     (SCRATCHPAD_WORDS),
      .RESET_MEM (1'b0)
  ) u_mem (
      .clk     (clk),
      .rst_n   (rst_n),
      .valid_i (mem_valid),
      .we_i    (mem_we),
      .addr_i  (mem_addr),
      .wdata_i (mem_wdata),
      .wstrb_i (mem_wstrb),
      .ready_o (mem_ready),
      .rdata_o (mem_rdata),
      .error_o (mem_error)
  );

  // -------------------------------------------------------------------
  // Data bus interconnect
  // -------------------------------------------------------------------

  bus_interconnect #(
      .ENABLE_SCRATCHPAD1(ENABLE_SCRATCHPAD1)
  ) u_bus (
      .cpu_valid_i (dmem_valid),
      .cpu_we_i    (dmem_we),
      .cpu_addr_i  (dmem_addr),
      .cpu_wdata_i (dmem_wdata),
      .cpu_wstrb_i (dmem_wstrb),
      .cpu_ready_o (dmem_ready),
      .cpu_rdata_o (dmem_rdata),
      .cpu_err_o   (dmem_error),

      .scratch0_valid_o (scratch0_valid),
      .scratch0_we_o    (scratch0_we),
      .scratch0_addr_o  (scratch0_addr),
      .scratch0_wdata_o (scratch0_wdata),
      .scratch0_wstrb_o (scratch0_wstrb),
      .scratch0_ready_i (scratch0_ready),
      .scratch0_rdata_i (scratch0_rdata),
      .scratch0_error_i (scratch0_error),

      .scratch1_valid_o (),
      .scratch1_we_o    (),
      .scratch1_addr_o  (),
      .scratch1_wdata_o (),
      .scratch1_wstrb_o (),
      .scratch1_ready_i (1'b0),
      .scratch1_rdata_i (32'd0),
      .scratch1_error_i (1'b0),

      .accel_valid_o (accel_valid),
      .accel_we_o    (accel_we),
      .accel_addr_o  (accel_addr),
      .accel_wdata_o (accel_wdata),
      .accel_wstrb_o (accel_wstrb),
      .accel_ready_i (accel_ready),
      .accel_rdata_i (accel_rdata),
      .accel_error_i (accel_bus_error),

      .debug_valid_o (debug_valid),
      .debug_we_o    (debug_we),
      .debug_addr_o  (debug_addr),
      .debug_wdata_o (debug_wdata),
      .debug_wstrb_o (debug_wstrb),
      .debug_ready_i (debug_ready),
      .debug_rdata_i (debug_rdata),
      .debug_error_i (debug_error)
  );

  // -------------------------------------------------------------------
  // Accelerator registers
  // -------------------------------------------------------------------

  accel_regs u_accel_regs (
      .clk   (clk),
      .rst_n (rst_n),

      .valid_i (accel_valid),
      .we_i    (accel_we),
      .addr_i  (accel_addr),
      .wdata_i (accel_wdata),
      .wstrb_i (accel_wstrb),
      .ready_o (accel_ready),
      .rdata_o (accel_rdata),
      .error_o (accel_bus_error),

      .accel_start_o (accel_start),
      .accel_cmd_o   (accel_cmd),
      .accel_src_a_o (accel_src_a),
      .accel_src_b_o (accel_src_b),
      .accel_src_c_o (accel_src_c),
      .accel_len_o   (accel_len),
      .accel_dst_o   (accel_dst),

      .accel_busy_i   (accel_busy_o),
      .accel_done_i   (accel_done_o),
      .accel_error_i  (accel_error_o),
      .accel_result_i (accel_result)
  );

  // -------------------------------------------------------------------
  // Vector accelerator core
  // -------------------------------------------------------------------

  vector_accel u_vector_accel (
      .clk   (clk),
      .rst_n (rst_n),

      .accel_start_i (accel_start),
      .accel_cmd_i   (accel_cmd),
      .accel_src_a_i (accel_src_a),
      .accel_src_b_i (accel_src_b),
      .accel_src_c_i (accel_src_c),
      .accel_len_i   (accel_len),
      .accel_dst_i   (accel_dst),

      .accel_busy_o   (accel_busy_o),
      .accel_done_o   (accel_done_o),
      .accel_error_o  (accel_error_o),
      .accel_result_o (accel_result)
  );

  // -------------------------------------------------------------------
  // Debug registers
  // -------------------------------------------------------------------

  debug_regs u_debug_regs (
      .clk   (clk),
      .rst_n (rst_n),

      .valid_i (debug_valid),
      .we_i    (debug_we),
      .addr_i  (debug_addr),
      .wdata_i (debug_wdata),
      .wstrb_i (debug_wstrb),
      .ready_o (debug_ready),
      .rdata_o (debug_rdata),
      .error_o (debug_error),

      .cpu_halted_i     (core_halted),
      .cpu_trap_i       (core_trap),
      .cpu_debug_halt_i (core_debug_halt),
      .trap_cause_i     (core_trap_cause),
      .pc_i             (core_pc),
      .retire_count_i   (core_retire_count),

      .dbg_reg_read_data_i (dbg_reg_read_data),

      .accel_busy_i   (accel_busy_o),
      .accel_done_i   (accel_done_o),
      .accel_error_i  (accel_error_o),
      .accel_result_i (accel_result),

      .dbg_halt_req_o   (dbgregs_halt_req),
      .dbg_resume_req_o (dbgregs_resume_req),
      .dbg_step_req_o   (dbgregs_step_req),

      .dbg_pc_write_en_o   (dbg_pc_write_en),
      .dbg_pc_write_data_o (dbg_pc_write_data),

      .dbg_reg_sel_o        (dbg_reg_sel),
      .dbg_reg_write_en_o   (dbg_reg_write_en),
      .dbg_reg_write_data_o (dbg_reg_write_data)
  );

endmodule

`default_nettype wire
