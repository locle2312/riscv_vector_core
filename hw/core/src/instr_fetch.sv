module instr_fetch #(
    parameter int unsigned XLEN = 32,
    parameter bit RV_C = 0 // Compressed extension 
) (
    input  logic            clk_i,
    input  logic            rst_ni,
    input  logic            fetch_stall_i,
    input  logic [XLEN-1:0] boot_addr_i,
    // PM interface
    output logic            pm_rd_o,
    output logic [XLEN-1:0] pm_addr_o,
    input  logic            pm_ready_i,
    input  logic            pm_instr_valid_i,
    input  logic [XLEN-1:0] pm_instr_i,
    // Decoder interface
    output logic            instr_valid_o,
    output logic [XLEN-1:0] instr_pc_o,
    output logic [XLEN-1:0] next_pc_o,
    output logic [XLEN-1:0] instr_o,
    input  logic            instr_ready_i,
    // Branch input 
    input  logic            branch_pc_valid_i,
    input  logic [XLEN-1:0] branch_pc_i
    
);

localparam int unsigned InstrSizeByte = XLEN/8;

// PC
logic [XLEN-1:0] pc, pc_r;
logic [XLEN-1:0] next_pc, next_pc_r;
logic fetch_stall;

assign pm_rd_o = ~fetch_stall;
assign pm_addr_o = pc;
assign fetch_stall = fetch_stall_i | ~instr_ready_i;
assign next_pc = pc + InstrSizeByte;

always_ff @(posedge clk_i or negedge rst_ni) begin: pc_proc
    if (~rst_ni) pc <= boot_addr_i;
    else begin
        if (~fetch_stall & pm_ready_i) pc <= next_pc;
        if (branch_pc_valid_i) pc <= branch_pc_i;
    end
end

// Assume 1 cycle read mem delay
always_ff @(posedge clk_i) begin
    next_pc_r <= next_pc;
    pc_r <= pc;
    next_pc_o <= next_pc_r;
    instr_pc_o <= pc_r;
    instr_valid_o <= pm_instr_valid_i;
    instr_o <= pm_instr_i;
end
    
endmodule