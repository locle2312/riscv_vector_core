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

assign sub_result = operand_i[0] - operand_i[1];

always_comb begin
    unique case (alu_op_i)
        ALU_NONE: begin
            operand_wdata_o = operand_i[1];
        end
        ALU_ADD: begin
            operand_wdata_o = operand_i[0] + operand_i[1];
        end
        ALU_SUB: begin
            operand_wdata_o = sub_result;
        end
        ALU_SHIFT_LEFT: begin
            operand_wdata_o = operand_i[0] << operand_i[1];
        end
        ALU_SHIFT_RIGHT: begin
            operand_wdata_o = operand_i[0] >> operand_i[1];
        end
        ALU_SHIFT_RIGHT_ARTH: begin
            operand_wdata_o = {{operand_i[1]{operand_i[0][31]}}, operand_i[0][31:operand_i[1]]};
        end
        ALU_LESS_THAN: begin
            operand_wdata_o = operand_i[0][31] != operand_i[1][31] ? operand_i[1][31] == 1'b1 : sub_result[31] == 1'b1;
        end
        ALU_LESS_THAN_S: begin
            operand_wdata_o = operand_i[0][31] != operand_i[1][31] ? operand_i[0][31] == 1'b1 : sub_result[31] == 1'b1;
        end
        ALU_AND: begin
            operand_wdata_o = operand_i[0] & operand_i[1];
        end
        ALU_OR: begin
            operand_wdata_o = operand_i[0] | operand_i[1];
        end
        ALU_XOR: begin
            operand_wdata_o = operand_i[0] ^ operand_i[1];
        end
    endcase
end
    
endmodule