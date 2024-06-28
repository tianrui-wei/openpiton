// disgusting, slow and stupid implementation
// rewrite me for the love of god
module noc_axi4_write_pipe 
import noc_axi4_pkg::*; 
(
	input clk,    // Clock
	input rst_n,  // Asynchronous reset active low

	input flit_op_t flit_op_data,
	input logic flit_op_vld,
	output logic flit_op_rdy,

	output flit_op_t flit_resp_data,
	output logic flit_resp_vld,
	input logic flit_resp_rdy,

    // AXI write interface
    output logic [`AXI4_ID_WIDTH     -1:0]     m_axi_awid,
    output logic [`AXI4_ADDR_WIDTH   -1:0]     m_axi_awaddr,
    output logic [`AXI4_LEN_WIDTH    -1:0]     m_axi_awlen,
    output logic [`AXI4_SIZE_WIDTH   -1:0]     m_axi_awsize,
    output logic [`AXI4_BURST_WIDTH  -1:0]     m_axi_awburst,
    output logic                               m_axi_awlock,
    output logic [`AXI4_CACHE_WIDTH  -1:0]     m_axi_awcache,
    output logic [`AXI4_PROT_WIDTH   -1:0]     m_axi_awprot,
    output logic [`AXI4_QOS_WIDTH    -1:0]     m_axi_awqos,
    output logic [`AXI4_REGION_WIDTH -1:0]     m_axi_awregion,
    output logic [`AXI4_USER_WIDTH   -1:0]     m_axi_awuser,
    output logic                               m_axi_awvalid,
    input  logic                               m_axi_awready,

    output logic  [`AXI4_ID_WIDTH     -1:0]    m_axi_wid,
    output logic  [`AXI4_DATA_WIDTH   -1:0]    m_axi_wdata,
    output logic  [`AXI4_STRB_WIDTH   -1:0]    m_axi_wstrb,
    output logic                               m_axi_wlast,
    output logic  [`AXI4_USER_WIDTH   -1:0]    m_axi_wuser,
    output logic                               m_axi_wvalid,
    input  logic                               m_axi_wready,

    input  logic  [`AXI4_ID_WIDTH     -1:0]    m_axi_bid,
    input  logic  [`AXI4_RESP_WIDTH   -1:0]    m_axi_bresp,
    input  logic  [`AXI4_USER_WIDTH   -1:0]    m_axi_buser,
    input  logic                               m_axi_bvalid,
    output logic                               m_axi_bready	
);
	assert_only_write: assert property (@(posedge clk) disable iff (~rst_n) flit_op_vld |-> flit_op_data.is_write);
	assert_aw_proper_size: assert property (@(posedge clk) disable iff (~rst_n) m_axi_awvalid |-> m_axi_awsize != 3'b111);
	assert_b_valid: assert property (@(posedge clk) disable iff (~rst_n) m_axi_bvalid |-> m_axi_bresp == '0 && m_axi_bid == '0);

// slow fsm for blocking operation only
typedef enum logic [3:0] {S_IDLE, S_AW, S_W, S_B, S_RESP} state_t;

flit_op_t flit_op_r, flit_op_n;

state_t state_r, state_n;

	wire [5:0] address_offset = flit_op_r.addr[5:0];
	wire [511:0] casted_1d_data = flit_op_r.data_flits;
	logic [63:0] write_strb;

	always_comb begin
		write_strb = `AXI4_STRB_WIDTH'hffffffffffffffff;
//		unique case (flit_op_r.size)
//			`MSG_DATA_SIZE_1B: begin
//				write_strb = `AXI4_STRB_WIDTH'h1;
//			end
//			`MSG_DATA_SIZE_2B: begin
//				write_strb = `AXI4_STRB_WIDTH'h3;
//			end
//			`MSG_DATA_SIZE_4B: begin
//				write_strb = `AXI4_STRB_WIDTH'hf;
//			end
//			`MSG_DATA_SIZE_8B: begin
//				write_strb = `AXI4_STRB_WIDTH'hff;
//			end
//			`MSG_DATA_SIZE_16B: begin
//				write_strb = `AXI4_STRB_WIDTH'hffff;
//			end
//			`MSG_DATA_SIZE_32B: begin
//				write_strb = `AXI4_STRB_WIDTH'hffffffff;
//			end
//			`MSG_DATA_SIZE_64B: begin
//				write_strb = `AXI4_STRB_WIDTH'hffffffffffffffff;
//			end
//			default: begin
//				// fail here, should never appear
//				write_strb = 'X;
//			end
//		endcase
	end

	assign m_axi_awid = '0;
	assign m_axi_awaddr = {flit_op_r.addr[`PHY_ADDR_WIDTH-1:5], 5'b0};
    assign m_axi_awlen    = `AXI4_LEN_WIDTH'b0; // Use only length-1 bursts
    assign m_axi_awburst  = `AXI4_BURST_WIDTH'b01; // fixed address in bursts (doesn't matter cause we use length-1 bursts)
    assign m_axi_awlock   = 1'b0; // Do not use locks
    assign m_axi_awcache  = `AXI4_CACHE_WIDTH'b11; // Non-cacheable bufferable requests
    assign m_axi_awprot   = `AXI4_PROT_WIDTH'b0; // Data access, non-secure access, unpriveleged access
    assign m_axi_awqos    = `AXI4_QOS_WIDTH'b0; // Do not use qos
    assign m_axi_awregion = `AXI4_REGION_WIDTH'b0; // Do not use regions
    assign m_axi_awuser   = `AXI4_USER_WIDTH'b0; // Do not use user field
    assign m_axi_awvalid = state_r == S_AW;

	// valid, ready
//    assign m_axi_awsize   = flit_op_r.size - 1; // Always transfer 64 bytes
    assign m_axi_awsize   =  3'b110; // Always transfer 64 bytes

    assign m_axi_wid = '0;
    assign m_axi_wdata = casted_1d_data;
    assign m_axi_wstrb = write_strb;
    assign m_axi_wlast = 1'b1;
    assign m_axi_wvalid = state_r == S_W;
    assign m_axi_wuser    = `AXI4_USER_WIDTH'b0; // Do not use user field

    assign m_axi_bready = state_r == S_B;

assign flit_op_rdy    = state_r == S_IDLE;
assign flit_resp_data = flit_op_r;
assign flit_resp_vld  = state_r == S_RESP;

always_ff @(posedge clk) begin : proc_state_r
	if(~rst_n) begin
		state_r <= S_IDLE;
		flit_op_r <= 'X;
	end else begin
		state_r <= state_n;
		flit_op_r <= flit_op_n;
	end
end

// we don't handle unaligned memory request for now

always_comb begin : proc_state_n
	state_n   = state_r;
	flit_op_n = flit_op_r;
	unique case (state_r)
		S_IDLE : begin
			if (flit_op_vld & flit_op_rdy) begin
				state_n   = S_AW;
				flit_op_n = flit_op_data;
			end
		end
		S_AW : begin
			if (m_axi_awvalid & m_axi_awready) begin
				state_n = S_W;
			end
		end
		S_W : begin
			if (m_axi_wvalid & m_axi_wready) begin
				state_n = S_B;
			end
		end
		S_B : begin
			if (m_axi_bvalid & m_axi_bready) begin
				state_n = S_RESP;
			end
		end
		S_RESP: begin
			if (flit_resp_vld & flit_resp_rdy) begin
				state_n = S_IDLE;
			end
		end
	endcase
end

endmodule : noc_axi4_write_pipe
