`include "riscv_pkg.sv"

module instr_decode #(
    parameter int unsigned XLEN = 32
) (
    input  logic            clk_i,
    input  logic            rst_ni,
    // Interface to fetch stage
    input  logic            fetch_instr_valid_i,
    input  logic [XLEN-1:0] fetch_pc_r_i,
    input  logic [XLEN-1:0] fetch_instr_i,
    output logic            fetch_instr_ready_o,
    input  logic            flush_decode_i,
    // Regfile read interface
    output logic [4:0]      regfile_op_0_raddr_o,
    output logic [4:0]      regfile_op_1_raddr_o,
    output logic [4:0]      regfile_waddr_o,
    input  logic [XLEN-1:0] regfile_op_0_rdata_i,
    input  logic [XLEN-1:0] regfile_op_1_rdata_i,
    // Decode field
    output logic [XLEN-1:0] regfile_op_0_data_o,
    output logic [XLEN-1:0] regfile_op_1_data_o,
    output logic [XLEN-1:0] uncond_jump_pc
    output alu_op_t         alu_op_o,
    output logic            is_alu_o,
    output logic            is_load_o,
    output logic            is_store_o,
    output logic            is_branch_o,
    output logic            is_mul_o,
    output logic            is_div_o,
);

alu_op_t alu_op;
logic    is_alu;
logic    is_load;
logic    is_store;
logic    is_branch;
logic    is_mul;
logic    is_div;

logic [4:0] r_op_reg_0;
logic [4:0] r_op_reg_1;
logic [4:0] w_reg;

// Output reg
always_ff @(posedge clk_i) begin
    alu_op_o <= alu_op;
    is_alu_o <= is_alu;
    is_load_o <= is_load;
    is_store_o <= is_store;
    is_branch_o <= is_branch;
    is_mul_o <= is_mul;
    is_div_o <= is_div;
    regfile_op_0_data_o <= regfile_op_0_rdata_i;
    regfile_op_1_data_o <= regfile_op_1_rdata_i;
    regfile_op_0_raddr_o <= r_op_reg_0;
    regfile_op_1_raddr_o <= r_op_reg_1;
    regfile_waddr_o <= w_reg;
end

assign alu_op = 
assign is_alu = fetch_instr_i == ADDI || fetch_instr_i == SLTI || fetch_instr_i == SLTIU 
                || fetch_instr_i == ANDI || fetch_instr_i == ORI || fetch_instr_i == XORI
                || fetch_instr_i == SLLI || fetch_instr_i == SRLI || fetch_instr_i == SRAI
                || fetch_instr_i == ADD || fetch_instr_i == SLT || fetch_instr_i == SLTU
                || fetch_instr_i == AND || fetch_instr_i == OR || fetch_instr_i == XOR
                || fetch_instr_i == SLL || fetch_instr_i == SRL || fetch_instr_i == SUB || fetch_instr_i == SRA
                || fetch_instr_i == AUIPC || fetch_instr_i == LUI;
assign is_load = 
assign is_store = 
assign is_branch = fetch_instr_i == AUIPC || fetch_instr_i == LUI
                || fetch_instr_i == JAL || fetch_instr_i == JALR
                || fetch_instr_i == BEQ || fetch_instr_i == BNE 
                || fetch_instr_i == BLT || fetch_instr_i == BLTU
                || fetch_instr_i == BGE || fetch_instr_i == BGEU;

assign r_op_reg_0 = fetch_instr_i[19:15];
assign r_op_reg_1 = fetch_instr_i[24:20];
assign w_reg = fetch_instr_i[11:7];

endmodule