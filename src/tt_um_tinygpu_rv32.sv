`default_nettype none

//------------------------------------------------------------------------------
// tt_um_tinygpu_rv32.sv
// Tiny Tapeout wrapper for TinyGPU-RV32 SoC
//------------------------------------------------------------------------------

module tt_um_mayank26102006_tinygpu_rv32 (
input wire [7:0] ui_in,
output wire [7:0] uo_out,
input wire [7:0] uio_in,
output wire [7:0] uio_out,
output wire [7:0] uio_oe,
input wire ena,
input wire clk,
input wire rst_n
);

logic cpu_halted;
logic cpu_trap;
logic [7:0] trap_cause;
logic [31:0] pc;
logic [31:0] retire_count;
logic accel_busy;
logic accel_done;
logic accel_error;

tinygpu_soc u_soc (
.clk (clk),
.rst_n (rst_n),

.dbg_halt_req_i (ui_in[0]),
.dbg_resume_req_i (ui_in[1]),
.dbg_step_req_i (ui_in[2]),

.cpu_halted_o (cpu_halted),
.cpu_trap_o (cpu_trap),
.trap_cause_o (trap_cause),
.dbg_pc_o (pc),
.dbg_retire_count_o(retire_count),

.accel_busy_o (accel_busy),
.accel_done_o (accel_done),
.accel_error_o (accel_error)
);

// Tiny Tapeout visible outputs
assign uo_out[0] = cpu_halted;
assign uo_out[1] = cpu_trap;
assign uo_out[2] = accel_busy;
assign uo_out[3] = accel_done;
assign uo_out[4] = accel_error;
assign uo_out[7:5] = trap_cause[2:0];

// Expose low byte of PC on bidirectional pins as outputs
assign uio_out = pc[7:0];
assign uio_oe = 8'hFF;

wire _unused = &{ena, uio_in};
endmodule

`default_nettype wire

