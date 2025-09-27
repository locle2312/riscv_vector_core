module alu #(
    parameter int unsigned XLEN = 32,
    parameter int unsigned NumInPort = 2,
    parameter int unsigned NumOutPort = 1
) (
    input  logic clk_i,
    input  logic rst_ni,
    input  logic [NumInPort-1:0][XLEN-1:0] operand_data_i,
    output logic [NumOutPort-1:0][XLEN-1:0] operand_data_i,
    
);
    
endmodule