typedef enum logic [3:0] { 
    ALU_NONE,
    ALU_ADD
    ALU_SUB
    ALU_SHIFT_LEFT,
    ALU_SHIFT_RIGHT,
    ALU_SHIFT_RIGHT_ARTH,
    ALU_LESS_THAN,
    ALU_LESS_THAN_S,
    ALU_AND,
    ALU_OR,
    ALU_XOR
} alu_op_t;

typedef enum logic [2:0] {
    BR_UNCOND_JUMP,
    BR_BEQ,
    BR_BNE,
    BR_BLT,
    BR_BGE
} br_op_t;

typedef enum logic [1:0] {
    M_MUL,
    M_MULH,
    M_MULHU,
    M_MULHSU
} mul_op_t;

typedef enum logic [1:0] {
    D_DIV,
    D_DIVU,
    D_REM,
    D_REMU
} div_op_t;

typedef enum logic [1:0] {
    CSR_CSRW,
    CSR_CSRS,
    CSR_CSRC
} csr_op_t