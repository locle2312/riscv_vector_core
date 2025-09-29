`include "riscv_pkg.sv"

module alu #(
    parameter int unsigned XLEN = 32,
    parameter int unsigned NumInPort = 2,
    parameter int unsigned NumOutPort = 1
) (
    input  logic                            clk_i,
    input  logic                            rst_ni,
    input  alu_op_t                         alu_op_i,
    input  logic [NumInPort-1:0][XLEN-1:0]  operand_i,
    output logic [NumOutPort-1:0][XLEN-1:0] operand_wdata_o
);

logic [XLEN-1:0] sub_result;
logic [XLEN-1:0] add_result;

assign sub_result = operand_i[0] - operand_i[1];
assign sub_result = operand_i[0] + operand_i[1];

always_comb begin
    unique case (alu_op_i)
        : 
        default: 
    endcase
end
    
endmodule