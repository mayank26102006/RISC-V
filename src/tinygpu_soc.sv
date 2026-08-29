`default_nettype none

//------------------------------------------------------------------------------
// tinygpu_soc.sv - TinyGPU-RV32 Top-Level SoC Integration
// Description : Wires together rv32_core, scratchpad memory, bus
//               interconnect, debug registers, accelerator registers, and
//               vector accelerator.
//------------------------------------------------------------------------------

`include "tinygpu_pkg.sv"

module tinygpu_soc #(
    parameter logic [31:0]      RESET_VECTOR       = RESET_VECTOR_DEFAULT,
    parameter bit                ENABLE_SCRATCHPAD1 = 1'b0,
    parameter int unsigned        SCRATCHPAD_WORDS   = 16
) (
    input  logic clk,
    input  logic rst_n,
    input  logic dbg_halt_req_i,
    input  logic dbg_resume_req_i,
    input  logic dbg_step_req_i,
    input  logic ext_load_mode_i,
    input  logic ext_load_bit_i,
    
    output logic ext_load_ready_o,
    output logic         cpu_halted_o,
    output logic         cpu_trap_o,
    output trap_cause_e  trap_cause_o,
    output logic [31:0]  dbg_pc_o,
    output logic [31:0]  dbg_retire_count_o,
    output logic accel_busy_o,
    output logic accel_done_o,
    output logic accel_error_o
);

  
  logic        imem_valid;
  logic [31:0] imem_addr;
  logic        imem_ready;
  logic [31:0] imem_rdata;
  logic        imem_error;
  logic        dmem_valid;
  logic        dmem_we;
  logic [31:0] dmem_addr;
  logic [31:0] dmem_wdata;
  logic [3:0]  dmem_wstrb;
  logic        dmem_ready;
  logic [31:0] dmem_rdata;
  logic        dmem_error;
  logic        scratch0_valid;
  logic        scratch0_we;
  logic [31:0] scratch0_addr;
  logic [31:0] scratch0_wdata;
  logic [3:0]  scratch0_wstrb;
  logic        scratch0_ready;
  logic [31:0] scratch0_rdata;
  logic        scratch0_error;


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


    
  logic        debug_valid;
  logic        debug_we;
  logic [31:0] debug_addr;
  logic [31:0] debug_wdata;
  logic [3:0]  debug_wstrb;
  logic        debug_ready;
  logic [31:0] debug_rdata;
  logic        debug_error;



  logic        core_halted;
  logic        core_trap;
  logic        core_debug_halt;
  trap_cause_e core_trap_cause;
  logic [31:0] core_pc;
  logic [31:0] core_retire_count;



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

  assign core_dbg_halt_req   = dbg_halt_req_i   | dbgregs_halt_req;
  assign core_dbg_resume_req = dbg_resume_req_i | dbgregs_resume_req;
  assign core_dbg_step_req   = dbg_step_req_i   | dbgregs_step_req;


  logic        loader_mem_valid;
  logic [31:0] loader_mem_addr;
  logic [31:0] loader_mem_wdata;
  logic        force_cpu_reset;
  logic        core_rst_n;

  assign core_rst_n = rst_n & !force_cpu_reset;

  ext_loader #(
      .SCRATCHPAD_WORDS(SCRATCHPAD_WORDS)
  ) u_ext_loader (
      .clk   (clk),
      .rst_n (rst_n),

      .ext_load_mode_i  (ext_load_mode_i),
      .ext_load_bit_i   (ext_load_bit_i),
      .ext_load_ready_o (ext_load_ready_o),

      .force_cpu_reset_o (force_cpu_reset),

      .mem_valid_o (loader_mem_valid),
      .mem_addr_o  (loader_mem_addr),
      .mem_wdata_o (loader_mem_wdata)
  );

  
  rv32_core #(
      .RESET_VECTOR(RESET_VECTOR)
  ) u_core (
      .clk   (clk),
      .rst_n (core_rst_n),

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

 
  assign cpu_halted_o       = core_halted;
  assign cpu_trap_o         = core_trap;
  assign trap_cause_o       = core_trap_cause;
  assign dbg_pc_o           = core_pc;
  assign dbg_retire_count_o = core_retire_count;


  logic        mem_valid;
  logic        mem_we;
  logic [31:0] mem_addr;
  logic [31:0] mem_wdata;
  logic [3:0]  mem_wstrb;
  logic        mem_ready;
  logic [31:0] mem_rdata;
  logic        mem_error;

 
  assign mem_valid = loader_mem_valid | imem_valid | scratch0_valid;
  assign mem_we     = loader_mem_valid ? 1'b1 : (scratch0_valid ? scratch0_we    : 1'b0);
  assign mem_addr   = loader_mem_valid ? loader_mem_addr : (imem_valid ? imem_addr : scratch0_addr);
  assign mem_wdata  = loader_mem_valid ? loader_mem_wdata : scratch0_wdata;
  assign mem_wstrb  = loader_mem_valid ? 4'b1111 : (scratch0_valid ? scratch0_wstrb : 4'b0000);

  assign imem_ready    = mem_ready & imem_valid;
  assign imem_rdata    = mem_rdata;
  assign imem_error    = mem_error & imem_valid;

  assign scratch0_ready = mem_ready & scratch0_valid & !imem_valid;
  assign scratch0_rdata = mem_rdata;
  assign scratch0_error = mem_error & scratch0_valid & !imem_valid;

`ifdef ASSERT_ON
  property p_imem_dmem_mutually_exclusive;
    @(posedge clk) disable iff (!rst_n)
    !(imem_valid && scratch0_valid);
  endproperty
  assert property (p_imem_dmem_mutually_exclusive);

  property p_loader_mutually_exclusive;
    @(posedge clk) disable iff (!rst_n)
    loader_mem_valid |-> !(imem_valid || scratch0_valid);
  endproperty
  assert property (p_loader_mutually_exclusive);
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
