module regfile #(
    parameter int unsigned XLEN = 32,
    parameter int unsigned RegNum = 32,
    parameter int unsigned RegWidth = $clog2(RegNum),
    parameter int unsigned NumReadPort = 2,
    parameter int unsigned NumWritePort = 1
) (
    input  logic                                  clk_i,
    input  logic                                  rst_ni,
    input  logic [NumReadPort-1:0][RegWidth-1:0]  reg_raddr_i,
    output logic [NumReadPort-1:0][XLEN-1:0]      reg_rdata_o,
    input  logic [NumWritePort-1:0]               reg_wen_i,
    input  logic [NumWritePort-1:0][RegWidth-1:0] reg_waddr_i,
    input  logic [NumWritePort-1:0][XLEN-1:0]     reg_wdata_i
);

logic [RegNum-1:0][XLEN-1:0] regfile;

always_comb begin
    for (int unsigned i= 0; i < NumReadPort; i++) begin
        reg_rdata_o[i] = regfile[reg_raddr_i[i]];
    end
end

always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) regfile <= '0;
    else begin
        for (int unsigned i= 1; i < NumWritePort; i++) begin
            if (reg_wen_i[i]) regfile[reg_waddr_i[i]] <= reg_wdata_i[i];
        end
    end
end
    
endmodule