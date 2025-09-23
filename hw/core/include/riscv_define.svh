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