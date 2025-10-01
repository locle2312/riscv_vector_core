module lsu #(
    parameter int unsigned XLEN = 32,
    parameter int unsigned XLENB = XLEN/8
) (
    input  logic             clk_i,
    input  logic             rst_ni,
    input  logic             lsu_stall_i,
    // Load/store request
    input  logic             lsu_req_valid_i,
    output logic             lsu_rsp_ready_o,
    input  logic             is_load_i,
    input  logic             is_load_signed_extend_i,
    input  logic [XLENB-1:0] lsu_byte_mask_i,
    input  logic [XLEN-1:0]  lsu_addr_i,
    input  logic [XLEN-1:0]  store_data_i,
    // Load data response
    output logic             load_data_valid_o,
    output logic [XLEN-1:0]  load_data_o,
    // Memory interface
    output logic             mem_req_o,
    input  logic             mem_ready_i,
    input  logic             mem_ack_i,
    output logic             mem_wen_o,
    output logic             mem_wstrb_o,
    output logic [XLEN-1:0]  mem_addr_o,
    output logic [XLEN-1:0]  mem_wdata_o,
    input  logic [XLEN-1:0]  mem_rdata_i,
);

logic lsu_stall;
logic lsu_req_pending, lsu_req_pending_r;
logic last_req_is_load;

always_ff @(posedge clk_i or negedge rst_ni) begin
    if (~rst_ni) begin
        lsu_req_pending_r <= 1'b0;
        last_req_is_load <= 1'b0;
    end else begin
        lsu_req_pending_r <= lsu_req_pending;
        if (lsu_req_valid_i) last_req_is_load <= is_load_i;
    end
end

always_comb begin
    lsu_req_pending = lsu_req_pending_r;
    if (mem_ack_i) lsu_req_pending = 1'b0;
    else if (mem_req_o) lsu_req_pending = 1'b1;
end

assign lsu_stall = lsu_stall_i | lsu_req_pending;
assign lsu_rsp_ready_o = ~lsu_stall;
assign load_data_valid_o = mem_ack_i & last_req_is_load;
assign mem_req_o = lsu_req_valid_i | ~lsu_stall;
    
endmodule