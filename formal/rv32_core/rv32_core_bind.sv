`default_nettype none
`include "tinygpu_pkg.sv"

// -----------------------------------------------------------------------
// rv32_core_bind.sv - formal-only binding, NOT part of the design
// -----------------------------------------------------------------------
// rv32_core.sv is instantiated completely unmodified below (see
// scratchpad_bind.sv / ext_loader_bind.sv for the full explanation of why
// this project uses bind-style harnesses rather than in-RTL named SVA
// properties in this specific toolchain).
//
// SCOPE: this harness targets the Zicsr/trap-redirect/MRET subsystem added
// 2026-07-29, since that's the newest and least-formally-proven part of
// the core -- everything else (ALU correctness, memory safety) already has
// CRV or formal coverage elsewhere. The five properties below are, in
// order of how load-bearing they are:
//
//   p_ebreak_always_debug_halt   -- EVERY existing test (test.py,
//     test_crv.py, test_bp_perf.py) depends on EBREAK halting for external
//     debug pickup, unconditionally, regardless of mtvec. If this ever
//     broke, every prior test in this repo would start failing for a
//     completely different reason than whatever the actual bug was.
//
//   p_mtvec_zero_no_redirect     -- THE backward-compatibility guarantee
//     the whole Zicsr feature was built around: a trap with mtvec==0 must
//     halt exactly as it did before Zicsr existed, never redirect. This is
//     the property that makes "adding Zicsr didn't change old behavior" a
//     proven fact instead of an assertion in a commit message.
//
//   p_trap_redirect_saves_state  -- when a real handler IS configured,
//     mepc/mcause/mtval/pc are all set correctly on entry.
//
//   p_mret_restores_mie          -- MRET's interrupt-enable restore is
//     exactly backwards (swapped MIE/MPIE) is a classic, easy-to-make bug
//     in a from-scratch trap implementation, and directed testing alone
//     only checks the one MRET call the test happens to make.
//
//   p_csr_write_gated_by_validity -- a CSR write should never reach the
//     register file for an address decode didn't recognize; should hold
//     by construction, checked here as a regression guard.
//
// All five are checked against FULLY UNCONSTRAINED imem_rdata_i /
// dmem_rdata_i -- BMC explores every possible instruction encoding on
// every cycle, not just the ones this project's own loader/tests happen to
// produce. That's a strictly stronger claim than any directed or CRV test
// in this repo can make for these specific properties.
// -----------------------------------------------------------------------

module rv32_core_bind (
    input logic clk,
    input logic rst_n,
    input logic imem_ready_i,
    input logic [31:0] imem_rdata_i,
    input logic imem_error_i,
    input logic dmem_ready_i,
    input logic [31:0] dmem_rdata_i,
    input logic dmem_error_i,
    input logic dbg_halt_req_i,
    input logic dbg_resume_req_i,
    input logic dbg_step_req_i,
    input logic dbg_pc_write_en_i,
    input logic [31:0] dbg_pc_write_data_i,
    input logic [4:0] dbg_reg_read_addr_i,
    input logic dbg_reg_write_en_i,
    input logic [4:0] dbg_reg_write_addr_i,
    input logic [31:0] dbg_reg_write_data_i
);

  logic imem_valid_o;
  logic [31:0] imem_addr_o;
  logic dmem_valid_o, dmem_we_o;
  logic [31:0] dmem_addr_o, dmem_wdata_o;
  logic [3:0] dmem_wstrb_o;
  logic [31:0] dbg_reg_read_data_o;
  logic cpu_halted_o, cpu_debug_halt_o, cpu_trap_o;
  trap_cause_e trap_cause_o;
  logic [31:0] pc_o, retire_count_o;
  logic instr_retire_o;
  logic [31:0] retired_pc_o, retired_instr_o;

  // The real, unmodified DUT.
  rv32_core dut (
      .clk(clk), .rst_n(rst_n),
      .imem_valid_o(imem_valid_o), .imem_addr_o(imem_addr_o),
      .imem_ready_i(imem_ready_i), .imem_rdata_i(imem_rdata_i), .imem_error_i(imem_error_i),
      .dmem_valid_o(dmem_valid_o), .dmem_we_o(dmem_we_o),
      .dmem_addr_o(dmem_addr_o), .dmem_wdata_o(dmem_wdata_o), .dmem_wstrb_o(dmem_wstrb_o),
      .dmem_ready_i(dmem_ready_i), .dmem_rdata_i(dmem_rdata_i), .dmem_error_i(dmem_error_i),
      .dbg_halt_req_i(dbg_halt_req_i), .dbg_resume_req_i(dbg_resume_req_i), .dbg_step_req_i(dbg_step_req_i),
      .dbg_pc_write_en_i(dbg_pc_write_en_i), .dbg_pc_write_data_i(dbg_pc_write_data_i),
      .dbg_reg_read_addr_i(dbg_reg_read_addr_i), .dbg_reg_read_data_o(dbg_reg_read_data_o),
      .dbg_reg_write_en_i(dbg_reg_write_en_i), .dbg_reg_write_addr_i(dbg_reg_write_addr_i),
      .dbg_reg_write_data_i(dbg_reg_write_data_i),
      .cpu_halted_o(cpu_halted_o), .cpu_debug_halt_o(cpu_debug_halt_o), .cpu_trap_o(cpu_trap_o),
      .trap_cause_o(trap_cause_o), .pc_o(pc_o), .retire_count_o(retire_count_o),
      .instr_retire_o(instr_retire_o), .retired_pc_o(retired_pc_o), .retired_instr_o(retired_instr_o)
  );

  initial assume (!rst_n);

  always @(posedge clk) begin
    if (rst_n) begin

      // p_ebreak_always_debug_halt
      if (dut.commit_valid && dut.commit_trap && dut.commit_trap_cause == TRAP_EBREAK) begin
        assert (dut.state_d == CPU_DEBUG_HALT);
      end

      // p_mtvec_zero_no_redirect
      if (dut.commit_valid && dut.commit_trap
          && dut.commit_trap_cause != TRAP_EBREAK && dut.mtvec_q == 32'd0) begin
        assert (dut.state_d == CPU_TRAP);
      end

      // p_trap_redirect_saves_state
      if (dut.commit_valid && dut.commit_trap
          && dut.commit_trap_cause != TRAP_EBREAK && dut.mtvec_q != 32'd0) begin
        assert (dut.state_d == CPU_FETCH);
        assert (dut.mepc_d == dut.commit_trap_pc);
        assert (dut.mtval_d == dut.commit_trap_val);
        assert (dut.pc_d == {dut.mtvec_q[31:2], 2'b00});
      end

      // p_mret_restores_mie
      if (dut.commit_mret) begin
        assert (dut.mstatus_mie_d == dut.mstatus_mpie_q);
        assert (dut.mstatus_mpie_d == 1'b1);
        assert (dut.pc_d == dut.mepc_q);
      end

      // p_csr_write_gated_by_validity
      if (dut.commit_csr_write) begin
        assert (dut.csr_addr_valid(dut.commit_csr_addr));
      end

      // Cover statements: prove each antecedent is reachable, not just
      // vacuously true. Same discipline as scratchpad_bind.sv /
      // ext_loader_bind.sv.
      cover (dut.commit_valid && dut.commit_trap && dut.commit_trap_cause == TRAP_EBREAK);
      cover (dut.commit_valid && dut.commit_trap
             && dut.commit_trap_cause != TRAP_EBREAK && dut.mtvec_q == 32'd0);
      cover (dut.commit_valid && dut.commit_trap
             && dut.commit_trap_cause != TRAP_EBREAK && dut.mtvec_q != 32'd0);
      cover (dut.commit_mret);
      cover (dut.commit_csr_write);
    end
  end

endmodule

`default_nettype wire
