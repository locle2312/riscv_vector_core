`include "riscv_pkg.sv"
`include "riscv_define.svh"

module instr_decode #(
    parameter int unsigned XLEN = 32,
    parameter int unsigned XLENB = XLEN/8,
    parameter int unsigned RegWidth = 5 
) (
    input  logic                     clk_i,
    input  logic                     rst_ni,
    input  logic                     decode_stall_i,
    output logic [XLEN-1:0]          decode_pc_o,
    output logic                     exec_req_o,
    input  logic                     exec_ready_i,
    // Interface to fetch stage
    input  logic                     fetch_instr_valid_i,
    input  logic [XLEN-1:0]          fetch_pc_r_i,
    input  logic [XLEN-1:0]          fetch_instr_i,
    output logic                     fetch_instr_ready_o,
    input  logic                     flush_decode_i,
    // Regfile read interface
    output logic [1:0][RegWidth-1:0] regfile_raddr_o,
    input  logic [1:0][XLEN-1:0]     regfile_rdata_i,
    output logic [RegWidth-1:0]      regfile_waddr_o,
    output logic [1:0]               reg_rsel_o,
    // Decode field to execute units
    output logic [1:0][XLEN-1:0]     decode_op_o,
    output logic [XLENB-1:0]         load_byte_mask_o,
    output logic                     load_sign_extend_o,
    output logic [XLEN-1:0]          store_data_o,
    output logic [XLENB-1:0]         store_byte_mask_o,
    output logic [11:0]              csr_addr_o,
    input  logic [XLEN-1:0]          csr_rdata_i,
    output alu_op_t                  alu_op_o,
    output logic                     is_alu_o,
    output logic                     is_load_o,
    output logic                     is_store_o,
    output logic                     is_branch_o,
    output logic                     is_mul_o,
    output logic                     is_div_o,
    output logic                     is_csr_o,
    output logic                     illegal_instr_o
);

alu_op_t alu_op;
mul_op_t mul_op;
div_op_t div_op;
br_op_t br_op;

logic    is_alu;
logic    is_load;
logic    is_store;
logic    is_branch;
logic    is_mul;
logic    is_div;
logic    is_csr;

logic [XLEN-1:0] decode_pc, decode_pc_r;
logic decode_stall;

logic valid_instr;
logic exec_req;

logic [RegWidth-1:0] reg_waddr;
logic [1:0][XLEN-1:0] decode_op;
logic load_sign_extend;
logic [XLENB-1:0] load_byte_mask;
logic [XLEN-1:0] store_data;
logic [XLENB-1:0] store_byte_mask;

assign decode_pc_o = decode_pc_r;
always_ff @(posedge clk_i) decode_pc_r <= decode_pc;
assign decode_pc = decode_stall ? decode_pc_r : fetch_pc_r_i;
assign decode_stall = decode_stall_i | ~exec_ready_i;
assign fetch_instr_ready_o = ~decode_stall;

// Output reg
always_ff @(posedge clk_i) begin
    illegal_instr_o <= ~valid_instr;
    exec_req_o <= exec_req;
    alu_op_o <= alu_op;
    is_alu_o <= is_alu;
    is_load_o <= is_load;
    is_store_o <= is_store;
    is_branch_o <= is_branch;
    is_mul_o <= is_mul;
    is_div_o <= is_div;
    is_csr_o <= is_csr;
    decode_op_o <= decode_op;
    regfile_waddr_o <= reg_waddr;
    load_sign_extend_o <= load_sign_extend;
    load_byte_mask_o <= load_byte_mask;
    store_data_o <= store_data;
    store_byte_mask_o <= store_byte_mask;
end

assign is_alu = fetch_instr_i == ADDI || fetch_instr_i == SLTI || fetch_instr_i == SLTIU 
                || fetch_instr_i == ANDI || fetch_instr_i == ORI || fetch_instr_i == XORI
                || fetch_instr_i == SLLI || fetch_instr_i == SRLI || fetch_instr_i == SRAI
                || fetch_instr_i == ADD || fetch_instr_i == SLT || fetch_instr_i == SLTU
                || fetch_instr_i == AND || fetch_instr_i == OR || fetch_instr_i == XOR
                || fetch_instr_i == SLL || fetch_instr_i == SRL || fetch_instr_i == SUB || fetch_instr_i == SRA
                || fetch_instr_i == LUI || fetch_instr_i == AUIPC || is_branch || is_csr;
assign is_load = fetch_instr_i == LW || fetch_instr_i == LH || fetch_instr_i == LHU || fetch_instr_i == LB || fetch_instr_i == LBU;
assign is_store = fetch_instr_i == SW || fetch_instr_i == SH || fetch_instr_i == SB;
assign is_branch = fetch_instr_i == JAL || fetch_instr_i == JALR
                || fetch_instr_i == BEQ || fetch_instr_i == BNE 
                || fetch_instr_i == BLT || fetch_instr_i == BLTU
                || fetch_instr_i == BGE || fetch_instr_i == BGEU;
assign is_mul = fetch_instr_i == MUL || fetch_instr_i == MULH || fetch_instr_i == MULHSU || fetch_instr_i == MULHU;
assign is_div = fetch_instr_i == DIV || fetch_instr_i == DIVU || fetch_instr_i == REM || fetch_instr_i == REMU;
assign is_csr = fetch_instr_i == CSRRW || fetch_instr_i == CSRRS || fetch_instr_i == CSRRC 
                || fetch_instr_i == CSRRWI || fetch_instr_i == CSRRSI || fetch_instr_i == CSRRCI;
assign valid_instr = is_alu | is_load | is_store | is_branch | is_mul | is_div | is_csr;
assign exec_req = fetch_instr_valid_i & valid_instr & ~decode_stall;

assign regfile_raddr_o[0] = fetch_instr_i[19:15];
assign regfile_raddr_o[1] = fetch_instr_i[24:20];
assign reg_waddr = fetch_instr_i[11:7];
assign csr_addr_o = fetch_instr_i[31:20];

always_comb begin : decode_proc
    alu_op = ALU_NONE;
    mul_op = M_MUL;
    div_op = D_DIV;
    br_op = BR_UNCOND_JUMP;
    reg_rsel_o = '0;
    decode_op = regfile_rdata_i;
    load_sign_extend = 1'b0;
    load_byte_mask = '0;
    store_data = '0;
    store_byte_mask = '0;
    unique case (fetch_instr_i)
        ADDI: begin
            alu_op = ALU_ADD;
            reg_rsel_o[0] = 1'b1;
            decode_op[1] = {{20{fetch_instr_i[31]}}, fetch_instr_i[31:20]};
        end
        SLTI: begin
            alu_op = ALU_LESS_THAN_S;
            reg_rsel_o[0] = 1'b1;
            decode_op[1] = {{20{fetch_instr_i[31]}}, fetch_instr_i[31:20]};
        end
        SLTIU: begin
            alu_op = ALU_LESS_THAN;
            reg_rsel_o[0] = 1'b1;
            decode_op[1] = {{20{fetch_instr_i[31]}}, fetch_instr_i[31:20]};
        end
        ANDI: begin
            alu_op = ALU_AND;
            reg_rsel_o[0] = 1'b1;
            decode_op[1] = {{20{fetch_instr_i[31]}}, fetch_instr_i[31:20]};
        end
        ORI: begin
            alu_op = ALU_OR;
            reg_rsel_o[0] = 1'b1;
            decode_op[1] = {{20{fetch_instr_i[31]}}, fetch_instr_i[31:20]};
        end
        XORI: begin
            alu_op = ALU_XOR;
            reg_rsel_o[0] = 1'b1;
            decode_op[1] = {{20{fetch_instr_i[31]}}, fetch_instr_i[31:20]};
        end
        SLLI: begin
            alu_op = ALU_SHIFT_LEFT;
            reg_rsel_o[0] = 1'b1;
            decode_op[1] = {{27{fetch_instr_i[31]}}, fetch_instr_i[24:20]};
        end
        SRLI: begin
            alu_op = ALU_SHIFT_RIGHT;
            reg_rsel_o[0] = 1'b1;
            decode_op[1] = {{27{fetch_instr_i[31]}}, fetch_instr_i[24:20]};
        end
        SRAI: begin
            alu_op = ALU_SHIFT_RIGHT_ARTH;
            reg_rsel_o[0] = 1'b1;
            decode_op[1] = {{27{fetch_instr_i[31]}}, fetch_instr_i[24:20]};
        end
        LUI: begin
            alu_op = ALU_LUI;
            decode_op[1] = {fetch_instr_i[31:12], 12'd0};
        end
        AUIPC: begin
            alu_op = ALU_ADD;
            decode_op[0] = fetch_pc_r_i;
            decode_op[1] = {fetch_instr_i[31:12], 12'd0};
        end
        ADD: begin
            alu_op = ALU_ADD;
            reg_rsel_o = '1;
        end
        SLT: begin
            alu_op = ALU_LESS_THAN_S;
            reg_rsel_o = '1;
        end
        SLTU: begin
            alu_op = ALU_LESS_THAN;
            reg_rsel_o = '1;
        end
        AND: begin
            alu_op = ALU_AND;
            reg_rsel_o = '1;
        end
        OR: begin
            alu_op = ALU_OR;
            reg_rsel_o = '1;
        end
        XOR: begin
            alu_op = ALU_XOR;
            reg_rsel_o = '1;
        end
        SLL: begin
            alu_op = ALU_SHIFT_LEFT;
            reg_rsel_o = '1;
        end
        SRL: begin
            alu_op = ALU_SHIFT_RIGHT;
            reg_rsel_o = '1;
        end
        SRA: begin
            alu_op = ALU_SHIFT_RIGHT_ARTH;
            reg_rsel_o = '1;
        end
        SUB: begin
            alu_op = ALU_SUB;
            reg_rsel_o = '1;
        end
        JAL: begin
            alu_op = ALU_ADD;
            br_op = BR_UNCOND_JUMP;
            decode_op[0] = fetch_pc_r_i;
            decode_op[1] = {{11{fetch_instr_i[31]}}, fetch_instr_i[31], fetch_instr_i[19:12], fetch_instr_i[20], fetch_instr_i[30:21], 1'b0};
        end
        JALR: begin
            alu_op = ALU_ADD;
            br_op = BR_UNCOND_JUMP;
            reg_rsel_o[0] = 1'b1;
            decode_op[1] = {{20{fetch_instr_i[31]}}, fetch_instr_i[31:20]};
        end
        BEQ: begin
            alu_op = ALU_SUB;
            br_op = BR_BEQ;
            reg_rsel_o = '1;
        end
        BNE: begin
            alu_op = ALU_SUB;
            br_op = BR_BNE;
            reg_rsel_o = '1;
        end
        BLT: begin
            alu_op = ALU_LESS_THAN_S;
            br_op = BR_BLT;
            reg_rsel_o = '1;
        end
        BLTU: begin
            alu_op = ALU_LESS_THAN;
            br_op = BR_BLT;
            reg_rsel_o = '1;
        end
        BGE: begin
            alu_op = ALU_LESS_THAN_S;
            br_op = BR_BGE;
            reg_rsel_o = '1;
        end
        BGEU: begin
            alu_op = ALU_LESS_THAN;
            br_op = BR_BGE;
            reg_rsel_o = '1;
        end
        LW: begin
            alu_op = ALU_ADD;
            reg_rsel_o[0] = 1'b1;
            decode_op[1] = {{20{fetch_instr_i[31]}}, fetch_instr_i[31:20]};
            load_byte_mask = '1;
        end
        LH: begin
            alu_op = ALU_ADD;
            reg_rsel_o[0] = 1'b1;
            decode_op[1] = {{20{fetch_instr_i[31]}}, fetch_instr_i[31:20]};
            load_sign_extend = 1'b1;
            load_byte_mask = 4'b0011;    
        end
        LHU: begin
            alu_op = ALU_ADD;
            reg_rsel_o[0] = 1'b1;
            decode_op[1] = {{20{fetch_instr_i[31]}}, fetch_instr_i[31:20]};
            load_sign_extend = 1'b0;
            load_byte_mask = 4'b0011;
        end
        LB: begin
            alu_op = ALU_ADD;
            reg_rsel_o[0] = 1'b1;
            decode_op[1] = {{20{fetch_instr_i[31]}}, fetch_instr_i[31:20]};
            load_sign_extend = 1'b1;
            load_byte_mask = 4'b0001;
        end
        LBU: begin
            alu_op = ALU_ADD;
            reg_rsel_o[0] = 1'b1;
            decode_op[1] = {{20{fetch_instr_i[31]}}, fetch_instr_i[31:20]};
            load_sign_extend = 1'b0;
            load_byte_mask = 4'b0001;
        end
        SW: begin
            alu_op = ALU_ADD;
            reg_rsel_o = '1;
            decode_op[1] = {{20{fetch_instr_i[31]}}, fetch_instr_i[31:25], fetch_instr_i[11:7]};
            store_data = regfile_rdata_i[1];
            store_byte_mask = '1;
        end
        SH: begin
            alu_op = ALU_ADD;
            reg_rsel_o = '1;
            decode_op[1] = {{20{fetch_instr_i[31]}}, fetch_instr_i[31:25], fetch_instr_i[11:7]};
            store_data = regfile_rdata_i[1];
            store_byte_mask = 4'b0011;
        end
        SB: begin
            alu_op = ALU_ADD;
            reg_rsel_o = '1;
            decode_op[1] = {{20{fetch_instr_i[31]}}, fetch_instr_i[31:25], fetch_instr_i[11:7]};
            store_data = regfile_rdata_i[1];
            store_byte_mask = 4'b0001;
        end
        MUL: begin
            mul_op = M_MUL;
            reg_rsel_o = '1;
        end
        MULH: begin
            mul_op = M_MULH;
            reg_rsel_o = '1;
        end
        MULHU: begin
            mul_op = M_MULHU;
            reg_rsel_o = '1;
        end
        MULHSU: begin
            mul_op = M_MULHSU;
            reg_rsel_o = '1;
        end
        DIV: begin
            mul_op = D_DIV;
            reg_rsel_o = '1;
        end
        DIVU: begin
            mul_op = D_DIVU;
            reg_rsel_o = '1;
        end
        REM: begin
            mul_op = D_REM;
            reg_rsel_o = '1;
        end
        REMU: begin
            mul_op = D_REMU;
            reg_rsel_o = '1;
        end
        CSRRW: begin
            alu_op = ALU_NONE;
            reg_rsel_o[0] = 1'b1;
            decode_op[1] = csr_rdata_i;
        end
        CSRRS: begin
            alu_op = ALU_OR;
            reg_rsel_o[0] = 1'b1;
            decode_op[1] = csr_rdata_i;
        end
        CSRRC: begin
            alu_op = ALU_AND;
            reg_rsel_o[0] = 1'b1;
            decode_op[0] = ~regfile_rdata_i[0];
            decode_op[1] = csr_rdata_i;
        end
        CSRRWI: begin
            alu_op = ALU_NONE;
            decode_op[0] = {27'd0, fetch_instr_i[19:15]};
            decode_op[1] = csr_rdata_i;
        end
        CSRRSI: begin
            alu_op = ALU_OR;
            decode_op[0] = {27'd0, fetch_instr_i[19:15]};
            decode_op[1] = csr_rdata_i;
        end
        CSRRCI: begin
            alu_op = ALU_AND;
            decode_op[0] = ~{27'd0, fetch_instr_i[19:15]};
            decode_op[1] = csr_rdata_i;
        end
    endcase
end

endmodule