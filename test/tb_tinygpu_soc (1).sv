`timescale 1ns / 1ps
`default_nettype none

//------------------------------------------------------------------------------
// tb_tinygpu_soc.sv - TinyGPU-RV32 Self-Checking Testbench
//------------------------------------------------------------------------------
// Purpose:
//   Runs a set of hand-assembled RV32I test programs against tinygpu_soc and
//   prints a clear, readable PASS/FAIL report for each one -- no waveform
//   viewer needed. Everything you need to check is printed to the console.
//
// How programs are checked:
//   Each test program (except TEST 5) is self-checking: it computes a
//   result, compares it against an expected value using branches, and then
//   writes either the PASS signature (0xCAFEBABE) or a FAIL code
//   (0xDEAD_xxxx) to the debug PASSFAIL register (DBG_REG_PASSFAIL,
//   0x9000_0014) before executing EBREAK to halt. This testbench then peeks
//   the resulting sticky pass_seen_q/fail_seen_q bits inside debug_regs.sv
//   and prints the verdict -- exactly mirroring what real debug software
//   reading DBG_REG_STATUS would see.
//
//   TEST 5 (illegal instruction) has no self-check store -- a trapped CPU
//   can't keep running to report one -- so the testbench checks
//   cpu_trap_o/trap_cause_o directly instead.
//
//   TEST 6 exercises the debug halt/step/resume path through the real
//   top-level pins (dbg_halt_req_i/dbg_resume_req_i/dbg_step_req_i), which
//   is exactly how an external debug probe or the Tiny Tapeout ui_in pins
//   would drive it.
//
// How to run in Vivado:
//   1. Add all 8 design files to the project (see header of tinygpu_soc.sv
//      for the list) plus this file, as SIMULATION-ONLY sources.
//   2. Set tb_tinygpu_soc as the simulation top module.
//   3. Run Behavioral Simulation. Everything prints to the Tcl console.
//   4. A "+define+ASSERT_ON" simulation define is optional but recommended
//      -- every design file has embedded SVA properties that will fire as
//      additional errors in the console if any internal invariant breaks,
//      which is extra free checking on top of this testbench's own checks.
//
// What "PASS" here means and doesn't mean:
//   This is a directed smoke-test suite (ALU/regfile, load/store,
//   branch/jump, accelerator, illegal-instruction trap, debug halt/step/
//   resume). It gives you fast, printed confidence that the core datapath,
//   memory system, accelerator, and debug block are wired up correctly and
//   producing correct results. It is NOT exhaustive ISA coverage (no
//   shifts-by-register-amount edge cases, no SLT/SLTU, no every branch
//   condition, no misaligned-access trap tests, etc.) -- treat this as the
//   first rung of your verification ladder, not the whole ladder.
//------------------------------------------------------------------------------

`include "tinygpu_pkg.sv"

module tb_tinygpu_soc;

  // -------------------------------------------------------------------
  // Clock / reset
  // -------------------------------------------------------------------

  logic clk;
  logic rst_n;

  initial clk = 1'b0;
  always #5 clk = ~clk;   // 100 MHz, 10ns period

  // -------------------------------------------------------------------
  // DUT connections
  // -------------------------------------------------------------------

  logic        dbg_halt_req;
  logic        dbg_resume_req;
  logic        dbg_step_req;

  logic        cpu_halted;
  logic        cpu_trap;
  trap_cause_e trap_cause;
  logic [31:0] dbg_pc;
  logic [31:0] dbg_retire_count;

  logic        accel_busy;
  logic        accel_done;
  logic        accel_error;

  tinygpu_soc #(
      .RESET_VECTOR       (32'h0000_0000),
      .ENABLE_SCRATCHPAD1 (1'b0),
      .SCRATCHPAD_WORDS   (256)
  ) dut (
      .clk   (clk),
      .rst_n (rst_n),

      .dbg_halt_req_i   (dbg_halt_req),
      .dbg_resume_req_i (dbg_resume_req),
      .dbg_step_req_i   (dbg_step_req),

      .cpu_halted_o       (cpu_halted),
      .cpu_trap_o         (cpu_trap),
      .trap_cause_o       (trap_cause),
      .dbg_pc_o           (dbg_pc),
      .dbg_retire_count_o (dbg_retire_count),

      .accel_busy_o  (accel_busy),
      .accel_done_o  (accel_done),
      .accel_error_o (accel_error)
  );

  // -------------------------------------------------------------------
  // Embedded test programs (generated + functionally pre-verified by an
  // independent Python RV32I model before being hand-assembled here --
  // see the project notes for how these were generated).
  // -------------------------------------------------------------------

  localparam int TEST1_LEN = 55;
    localparam logic [31:0] TEST1_PROG [0:255] = '{
      32'h00500093, 32'h00700113, 32'h002081b3, 32'h40208233,
      32'h0020f2b3, 32'h0020e333, 32'h0020c3b3, 32'h00c00a13,
      32'h03419663, 32'h00000a37, 32'hffea0a13, 32'h03421c63,
      32'h00500a13, 32'h05429463, 32'h00700a13, 32'h05431c63,
      32'h00200a13, 32'h07439463, 32'h07c0006f, 32'h90000f37,
      32'h014f0f13, 32'hdead0fb7, 32'h001f8f93, 32'h01ff2023,
      32'h00100073, 32'h90000f37, 32'h014f0f13, 32'hdead0fb7,
      32'h002f8f93, 32'h01ff2023, 32'h00100073, 32'h90000f37,
      32'h014f0f13, 32'hdead0fb7, 32'h003f8f93, 32'h01ff2023,
      32'h00100073, 32'h90000f37, 32'h014f0f13, 32'hdead0fb7,
      32'h004f8f93, 32'h01ff2023, 32'h00100073, 32'h90000f37,
      32'h014f0f13, 32'hdead0fb7, 32'h005f8f93, 32'h01ff2023,
      32'h00100073, 32'h90000f37, 32'h014f0f13, 32'hcafecfb7,
      32'habef8f93, 32'h01ff2023, 32'h00100073, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000
  };

  localparam int TEST2_LEN = 58;
    localparam logic [31:0] TEST2_PROG [0:255] = '{
      32'h11223137, 32'h34410113, 32'h30202023, 32'hfaa00193,
      32'h30300023, 32'h30002203, 32'h112232b7, 32'h3aa28293,
      32'h04521863, 32'h30004303, 32'h000003b7, 32'h0aa38393,
      32'h04731c63, 32'h30000403, 32'h000004b7, 32'hfaa48493,
      32'h06941063, 32'hbeef0537, 32'h00050513, 32'h30a02223,
      32'h0000d5b7, 32'hafe58593, 32'h30b01223, 32'h30402603,
      32'hbeefd6b7, 32'hafe68693, 32'h04d61863, 32'h0640006f,
      32'h90000f37, 32'h014f0f13, 32'hdead0fb7, 32'h001f8f93,
      32'h01ff2023, 32'h00100073, 32'h90000f37, 32'h014f0f13,
      32'hdead0fb7, 32'h002f8f93, 32'h01ff2023, 32'h00100073,
      32'h90000f37, 32'h014f0f13, 32'hdead0fb7, 32'h003f8f93,
      32'h01ff2023, 32'h00100073, 32'h90000f37, 32'h014f0f13,
      32'hdead0fb7, 32'h004f8f93, 32'h01ff2023, 32'h00100073,
      32'h90000f37, 32'h014f0f13, 32'hcafecfb7, 32'habef8f93,
      32'h01ff2023, 32'h00100073, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000
  };

  localparam int TEST3_LEN = 60;
    localparam logic [31:0] TEST3_PROG [0:255] = '{
      32'h00900093, 32'h00900113, 32'h00208463, 32'h0540006f,
      32'h00100193, 32'h00100213, 32'h00200293, 32'h00521463,
      32'h0580006f, 32'h00100313, 32'h008003ef, 32'h0640006f,
      32'h00100413, 32'h000004b7, 32'h04448493, 32'h00048567,
      32'h0680006f, 32'h00100593, 32'h00618633, 32'h00860633,
      32'h00b60633, 32'h00400693, 32'h06d61463, 32'h07c0006f,
      32'h90000f37, 32'h014f0f13, 32'hdead0fb7, 32'h001f8f93,
      32'h01ff2023, 32'h00100073, 32'h90000f37, 32'h014f0f13,
      32'hdead0fb7, 32'h002f8f93, 32'h01ff2023, 32'h00100073,
      32'h90000f37, 32'h014f0f13, 32'hdead0fb7, 32'h003f8f93,
      32'h01ff2023, 32'h00100073, 32'h90000f37, 32'h014f0f13,
      32'hdead0fb7, 32'h004f8f93, 32'h01ff2023, 32'h00100073,
      32'h90000f37, 32'h014f0f13, 32'hdead0fb7, 32'h005f8f93,
      32'h01ff2023, 32'h00100073, 32'h90000f37, 32'h014f0f13,
      32'hcafecfb7, 32'habef8f93, 32'h01ff2023, 32'h00100073,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000
  };

  localparam int TEST4_LEN = 58;
    localparam logic [31:0] TEST4_PROG [0:255] = '{
      32'h800000b7, 32'h00808093, 32'h04030137, 32'h20110113,
      32'h0020a023, 32'h800000b7, 32'h00c08093, 32'h281e1137,
      32'h40a10113, 32'h0020a023, 32'h800000b7, 32'h00008093,
      32'h00100113, 32'h0020a023, 32'h800000b7, 32'h01c08093,
      32'h0000a183, 32'h2c211237, 32'h60b20213, 32'h04419a63,
      32'h800000b7, 32'h00808093, 32'h04030137, 32'h20110113,
      32'h0020a023, 32'h800000b7, 32'h00c08093, 32'h281e1137,
      32'h40a10113, 32'h0020a023, 32'h800000b7, 32'h00008093,
      32'h00500113, 32'h0020a023, 32'h800000b7, 32'h01c08093,
      32'h0000a283, 32'h12c00313, 32'h02629063, 32'h0340006f,
      32'h90000f37, 32'h014f0f13, 32'hdead0fb7, 32'h001f8f93,
      32'h01ff2023, 32'h00100073, 32'h90000f37, 32'h014f0f13,
      32'hdead0fb7, 32'h002f8f93, 32'h01ff2023, 32'h00100073,
      32'h90000f37, 32'h014f0f13, 32'hcafecfb7, 32'habef8f93,
      32'h01ff2023, 32'h00100073, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000
  };

  // TEST 5: one valid instruction, then a reserved/undefined opcode word
  // (7'b0000000 is not one of the 11 defined RV32I base opcodes).
  localparam int TEST5_LEN = 2;
    localparam logic [31:0] TEST5_PROG [0:255] = '{
      32'h00000013, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000,
      32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000
  };

  // -------------------------------------------------------------------
  // Scoreboard
  // -------------------------------------------------------------------

  int total_tests = 0;
  int total_pass  = 0;
  string result_names [10];
  bit    result_pass  [10];

  // -------------------------------------------------------------------
  // Helper tasks
  // -------------------------------------------------------------------

  task automatic apply_reset();
    dbg_halt_req   = 1'b0;
    dbg_resume_req = 1'b0;
    dbg_step_req   = 1'b0;
    rst_n = 1'b0;
    repeat (5) @(posedge clk);
    rst_n = 1'b1;
    @(posedge clk);
  endtask

  // Loads a program into instruction memory. Zeroes the rest of the array
  // first so no stale content from a previous test can ever be fetched.
  task automatic load_program(input logic [31:0] prog [0:255], input int len);
    int i;
    for (i = 0; i < 256; i = i + 1) begin
      dut.u_mem.mem_q[i] = 32'h0000_0000;
    end
    for (i = 0; i < len; i = i + 1) begin
      dut.u_mem.mem_q[i] = prog[i];
    end
  endtask

  // Waits for cpu_halted_o (via EBREAK or trap) with a cycle timeout so a
  // hung core is reported instead of the simulation just hanging forever.
  task automatic wait_for_halt(input int timeout_cycles, output bit timed_out);
    int n;
    timed_out = 1'b0;
    n = 0;
    while (!cpu_halted && n < timeout_cycles) begin
      @(posedge clk);
      n = n + 1;
    end
    if (!cpu_halted) timed_out = 1'b1;
  endtask

  task automatic print_status(input string tag);
    $display("    [%s] pc=0x%08x retire_count=%0d cpu_halted=%0d cpu_trap=%0d trap_cause=%s accel(busy=%0d done=%0d error=%0d)",
              tag, dbg_pc, dbg_retire_count, cpu_halted, cpu_trap,
              trap_cause.name(), accel_busy, accel_done, accel_error);
  endtask

  task automatic record(input string name, input bit pass);
    result_names[total_tests] = name;
    result_pass[total_tests]  = pass;
    total_tests = total_tests + 1;
    if (pass) total_pass = total_pass + 1;
  endtask

  // Runs one self-checking program (PASSFAIL-via-MMIO style) and prints
  // a full verdict block.
  task automatic run_selfcheck_test(input string name, input logic [31:0] prog [0:255], input int len);
    bit timed_out;
    bit pass_seen, fail_seen;
    logic [31:0] passfail_val;

    $display("");
    $display("==================== %s ====================", name);
    apply_reset();
    load_program(prog, len);
    rst_n = 1'b1;

    wait_for_halt(2000, timed_out);

    pass_seen    = dut.u_debug_regs.pass_seen_q;
    fail_seen    = dut.u_debug_regs.fail_seen_q;
    passfail_val = dut.u_debug_regs.passfail_q;

    print_status(name);
    $display("    passfail_reg = 0x%08x  pass_seen=%0d fail_seen=%0d", passfail_val, pass_seen, fail_seen);

    if (timed_out) begin
      $display("    RESULT: FAIL (TIMEOUT -- cpu_halted_o never asserted within 2000 cycles)");
      record(name, 1'b0);
    end else if (cpu_trap && !cpu_halted) begin
      // shouldn't happen (cpu_halted_o covers trap too) but guard anyway
      $display("    RESULT: FAIL (unexpected trap, cause=%s)", trap_cause.name());
      record(name, 1'b0);
    end else if (pass_seen && !fail_seen) begin
      $display("    RESULT: PASS");
      record(name, 1'b1);
    end else begin
      $display("    RESULT: FAIL (pass_seen=%0d fail_seen=%0d passfail_reg=0x%08x)",
                pass_seen, fail_seen, passfail_val);
      record(name, 1'b0);
    end
  endtask

  // -------------------------------------------------------------------
  // TEST 5: illegal instruction trap
  // -------------------------------------------------------------------

  task automatic run_illegal_instr_test();
    bit timed_out;
    $display("");
    $display("==================== TEST5_illegal_instruction ====================");
    apply_reset();
    load_program(TEST5_PROG, TEST5_LEN);
    rst_n = 1'b1;

    wait_for_halt(2000, timed_out);
    print_status("TEST5");

    if (timed_out) begin
      $display("    RESULT: FAIL (TIMEOUT -- expected a trap, core never halted)");
      record("TEST5_illegal_instruction", 1'b0);
    end else if (cpu_trap && trap_cause == TRAP_ILLEGAL_INSTR) begin
      $display("    RESULT: PASS (illegal instruction correctly trapped)");
      record("TEST5_illegal_instruction", 1'b1);
    end else begin
      $display("    RESULT: FAIL (expected cpu_trap=1, trap_cause=TRAP_ILLEGAL_INSTR; got cpu_trap=%0d trap_cause=%s)",
                cpu_trap, trap_cause.name());
      record("TEST5_illegal_instruction", 1'b0);
    end
  endtask

  // -------------------------------------------------------------------
  // TEST 6: debug halt / single-step / resume via top-level pins
  // -------------------------------------------------------------------

  task automatic run_debug_control_test();
    bit timed_out;
    logic [31:0] retire_before_step, retire_after_step;
    bit ok;

    ok = 1'b1;

    $display("");
    $display("==================== TEST6_debug_halt_step_resume ====================");
    apply_reset();
    load_program(TEST1_PROG, TEST1_LEN);
    rst_n = 1'b1;

    // Let a few instructions retire, then request halt via the raw pin.
    repeat (8) @(posedge clk);
    dbg_halt_req = 1'b1;
    @(posedge clk);
    dbg_halt_req = 1'b0;

    // Give the core a couple of cycles to reach CPU_DEBUG_HALT.
    repeat (3) @(posedge clk);
    print_status("after halt_req");

    if (!cpu_halted || cpu_trap) begin
      $display("    CHECK FAILED: expected cpu_halted=1, cpu_trap=0 after halt request");
      ok = 1'b0;
    end else begin
      $display("    CHECK OK: core halted on request (not trapped)");
    end

    retire_before_step = dbg_retire_count;

    // Single-step exactly one instruction.
    dbg_step_req = 1'b1;
    @(posedge clk);
    dbg_step_req = 1'b0;
    repeat (3) @(posedge clk);

    retire_after_step = dbg_retire_count;
    print_status("after single step");

    if (retire_after_step != retire_before_step + 32'd1) begin
      $display("    CHECK FAILED: expected retire_count to increase by exactly 1 (before=%0d after=%0d)",
                retire_before_step, retire_after_step);
      ok = 1'b0;
    end else if (!cpu_halted) begin
      $display("    CHECK FAILED: expected core to return to halted state after one step");
      ok = 1'b0;
    end else begin
      $display("    CHECK OK: exactly one instruction retired on single-step, core re-halted");
    end

    // Resume and let the program run to its own EBREAK/PASSFAIL report.
    dbg_resume_req = 1'b1;
    @(posedge clk);
    dbg_resume_req = 1'b0;

    wait_for_halt(2000, timed_out);
    print_status("after resume, at final halt");

    if (timed_out) begin
      $display("    CHECK FAILED: TIMEOUT waiting for program to finish after resume");
      ok = 1'b0;
    end else if (!dut.u_debug_regs.pass_seen_q) begin
      $display("    CHECK FAILED: program did not reach its own PASS report after resume");
      ok = 1'b0;
    end else begin
      $display("    CHECK OK: program completed and self-reported PASS after resume");
    end

    if (ok) $display("    RESULT: PASS");
    else    $display("    RESULT: FAIL");
    record("TEST6_debug_halt_step_resume", ok);
  endtask

  // -------------------------------------------------------------------
  // Main sequence
  // -------------------------------------------------------------------

  initial begin
    $display("");
    $display("############################################################");
    $display("# TinyGPU-RV32 self-checking testbench starting");
    $display("############################################################");

    run_selfcheck_test("TEST1_alu_regfile",     TEST1_PROG, TEST1_LEN);
    run_selfcheck_test("TEST2_load_store",      TEST2_PROG, TEST2_LEN);
    run_selfcheck_test("TEST3_branch_jump",     TEST3_PROG, TEST3_LEN);
    run_selfcheck_test("TEST4_accelerator",     TEST4_PROG, TEST4_LEN);
    run_illegal_instr_test();
    run_debug_control_test();

    $display("");
    $display("############################################################");
    $display("# SUMMARY");
    $display("############################################################");
    for (int i = 0; i < total_tests; i = i + 1) begin
      $display("  %-32s %s", result_names[i], result_pass[i] ? "PASS" : "FAIL");
    end
    $display("------------------------------------------------------------");
    $display("  TOTAL: %0d / %0d tests passed", total_pass, total_tests);
    if (total_pass == total_tests) begin
      $display("  OVERALL: ALL TESTS PASSED");
    end else begin
      $display("  OVERALL: %0d TEST(S) FAILED -- see per-test detail above", total_tests - total_pass);
    end
    $display("############################################################");
    $display("");

    $finish;
  end

  // Global safety-net watchdog in case something hangs outside the
  // per-test timeouts above (e.g. a task never returns).
  initial begin
    #200000; // 200us of simulated time
    $display("GLOBAL WATCHDOG: simulation did not finish in time -- forcing $finish");
    $finish;
  end

endmodule

`default_nettype wire
