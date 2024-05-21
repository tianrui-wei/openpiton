module noc_axi4_read_pipe 
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

    output wire  [`AXI4_ID_WIDTH     -1:0]    m_axi_arid,
    output wire  [`AXI4_ADDR_WIDTH   -1:0]    m_axi_araddr,
    output wire  [`AXI4_LEN_WIDTH    -1:0]    m_axi_arlen,
    output wire  [`AXI4_SIZE_WIDTH   -1:0]    m_axi_arsize,
    output wire  [`AXI4_BURST_WIDTH  -1:0]    m_axi_arburst,
    output wire                               m_axi_arlock,
    output wire  [`AXI4_CACHE_WIDTH  -1:0]    m_axi_arcache,
    output wire  [`AXI4_PROT_WIDTH   -1:0]    m_axi_arprot,
    output wire  [`AXI4_QOS_WIDTH    -1:0]    m_axi_arqos,
    output wire  [`AXI4_REGION_WIDTH -1:0]    m_axi_arregion,
    output wire  [`AXI4_USER_WIDTH   -1:0]    m_axi_aruser,
    output wire                               m_axi_arvalid,
    input  wire                               m_axi_arready,

    input  wire  [`AXI4_ID_WIDTH     -1:0]    m_axi_rid,
    input  wire  [`AXI4_DATA_WIDTH   -1:0]    m_axi_rdata,
    input  wire  [`AXI4_RESP_WIDTH   -1:0]    m_axi_rresp,
    input  wire                               m_axi_rlast,
    input  wire  [`AXI4_USER_WIDTH   -1:0]    m_axi_ruser,
    input  wire                               m_axi_rvalid,
    output wire                               m_axi_rready
);

	typedef enum logic [3:0] {S_IDLE, S_AR, S_R, S_SEND} state_t;
	read_address_aligned: assert property (@(posedge clk) disable iff (~rst_n) flit_op_vld |-> flit_op_data.addr[5:0] == '0);
	state_t state_r, state_n;
    flit_op_t op_r, op_n;
	wire [5:0] address_offset = op_r.addr[5:0];

   	assign m_axi_arlen    = `AXI4_LEN_WIDTH'b0; // Use only length-1 bursts
    assign m_axi_arburst  = `AXI4_BURST_WIDTH'b01; // fixed address in bursts (doesn't matter cause we use length-1 bursts)
    assign m_axi_arlock   = 1'b0; // Do not use locks
    assign m_axi_arcache  = `AXI4_CACHE_WIDTH'b11; // Non-cacheable bufferable requests
    assign m_axi_arprot   = `AXI4_PROT_WIDTH'b0; // Data access, non-secure access, unpriveleged access
    assign m_axi_arqos    = `AXI4_QOS_WIDTH'b0; // Do not use qos
    assign m_axi_arregion = `AXI4_REGION_WIDTH'b0; // Do not use regions
    assign m_axi_aruser   = `AXI4_USER_WIDTH'b0; // Do not use user field

	assert_ar_proper_size: assert property (@(posedge clk) disable iff (~rst_n) m_axi_arvalid |-> m_axi_arsize != 3'b111);
    assign m_axi_arsize   = op_r.size - 1;
    assign m_axi_araddr = op_r.addr;

    assign m_axi_arvalid = state_r == S_AR;
    assign m_axi_rready = state_r == S_R;

    assign flit_resp_vld = state_r == S_SEND;
    assign flit_op_rdy = state_r == S_IDLE;

	assign flit_resp_data = op_r;


    always_ff @(posedge clk) begin : proc_state_r
    	if(~rst_n) begin
    		state_r <= S_IDLE;
    		op_r <= 'X;
    	end else begin
    		state_r <= state_n;
    		op_r <= op_n;
    	end
    end


    always_comb begin : proc_state_n
    	state_n = state_r;
    	op_n = op_r;
    	unique case (state_r)
    		S_IDLE: begin
    			if (flit_op_vld & flit_op_rdy) begin
    				state_n = S_AR;
    				op_n = flit_op_data;
    			end
    		end
    		S_AR: begin
    			if (m_axi_arvalid & m_axi_arready) begin
    				state_n = S_R;
    			end
    		end
    		S_R: begin
    			if (m_axi_rvalid & m_axi_rready) begin
    				state_n = S_SEND;
    				op_n.data_flits = (m_axi_rdata >> (8 * address_offset));
    			end
    		end
    		S_SEND: begin
    			if (flit_resp_vld & flit_resp_rdy) begin
    				state_n = S_IDLE;
    				op_n = 'X;
    			end
    		end
    	endcase
    end


endmodule : noc_axi4_read_pipe
