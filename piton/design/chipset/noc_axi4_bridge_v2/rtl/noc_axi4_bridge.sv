`include "mc_define.h"
`include "define.tmp.h"
`include "noc_axi4_bridge_define.vh"

module noc_axi4_bridge 
import noc_axi4_pkg::*; 
(
    // Clock + Reset
    input  wire                                   clk,
    input  wire                                   rst_n,
    input  wire                                   uart_boot_en,
    input  wire                                   phy_init_done, 

    // Noc interface
    input  wire                                   src_bridge_vr_noc2_val,
    input  wire [`NOC_DATA_WIDTH-1:0]             src_bridge_vr_noc2_dat,
    output wire                                   src_bridge_vr_noc2_rdy,
    output wire                                   bridge_dst_vr_noc3_val,
    output wire [`NOC_DATA_WIDTH-1:0]             bridge_dst_vr_noc3_dat,
    input  wire                                   bridge_dst_vr_noc3_rdy,

    // AXI interface
    output wire [`AXI4_ID_WIDTH     -1:0]    m_axi_awid,
    output wire [`AXI4_ADDR_WIDTH   -1:0]    m_axi_awaddr,
    output wire [`AXI4_LEN_WIDTH    -1:0]    m_axi_awlen,
    output wire [`AXI4_SIZE_WIDTH   -1:0]    m_axi_awsize,
    output wire [`AXI4_BURST_WIDTH  -1:0]    m_axi_awburst,
    output wire                              m_axi_awlock,
    output wire [`AXI4_CACHE_WIDTH  -1:0]    m_axi_awcache,
    output wire [`AXI4_PROT_WIDTH   -1:0]    m_axi_awprot,
    output wire [`AXI4_QOS_WIDTH    -1:0]    m_axi_awqos,
    output wire [`AXI4_REGION_WIDTH -1:0]    m_axi_awregion,
    output wire [`AXI4_USER_WIDTH   -1:0]    m_axi_awuser,
    output wire                              m_axi_awvalid,
    input  wire                              m_axi_awready,

    output wire  [`AXI4_ID_WIDTH     -1:0]    m_axi_wid,
    output wire  [`AXI4_DATA_WIDTH   -1:0]    m_axi_wdata,
    output wire  [`AXI4_STRB_WIDTH   -1:0]    m_axi_wstrb,
    output wire                               m_axi_wlast,
    output wire  [`AXI4_USER_WIDTH   -1:0]    m_axi_wuser,
    output wire                               m_axi_wvalid,
    input  wire                               m_axi_wready,

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
    output wire                               m_axi_rready,

    input  wire  [`AXI4_ID_WIDTH     -1:0]    m_axi_bid,
    input  wire  [`AXI4_RESP_WIDTH   -1:0]    m_axi_bresp,
    input  wire  [`AXI4_USER_WIDTH   -1:0]    m_axi_buser,
    input  wire                               m_axi_bvalid,
    output wire                               m_axi_bready
);

    flit_op_t flit_op_input;


    wire flit_in_val;
    wire [`NOC_DATA_WIDTH-1:0] flit_in_data;
    wire flit_in_rdy;
    flit_op_t flit_op_data;
    logic flit_op_vld;
    logic flit_op_rdy;

    flit_op_t flit_resp_data;
    logic flit_resp_vld;
    logic flit_resp_rdy;

    logic flit_write_vld;
    logic flit_write_rdy;
    logic flit_read_vld;
    logic flit_read_rdy;

	logic decoder_src_bridge_vr_noc2_rdy;
	logic decoder_src_bridge_vr_noc2_val;

	logic flit_encoder_rdy, flit_encoder_vld;
	flit_op_t flit_encoder_data;

    logic flit_write_resp_vld, flit_write_resp_rdy;
    logic flit_read_resp_vld, flit_read_resp_rdy;
    flit_op_t flit_write_resp_data, flit_read_resp_data;

    assign flit_op_rdy = flit_op_data.is_write ? flit_write_rdy : flit_read_rdy;
    assign flit_write_vld = flit_op_data.is_write & flit_op_vld;
    assign flit_read_vld = !flit_op_data.is_write & flit_op_vld;

    typedef enum logic [1:0] {S_IDLE, S_READ, S_WRITE} e_arb;
    
    e_arb arb_r, arb_n;

    always_ff @(posedge clk) begin : proc_arb_latch
        if(~rst_n) begin
            arb_r <= S_IDLE;
        end else begin
            arb_r <= arb_n;
        end
    end

    always_comb begin : proc_arb_n
        arb_n = arb_r;
        unique case (arb_r)
            S_IDLE: begin
                if (flit_write_resp_vld) begin
                    arb_n = S_WRITE; 
                end else if (flit_read_resp_vld) begin
                    arb_n = S_READ;
                end
            end
            S_WRITE: begin
                if (flit_write_resp_vld & flit_write_resp_rdy) begin
                    arb_n = S_IDLE; 
                end
            end
            S_READ: begin 
                if (flit_read_resp_vld & flit_read_resp_rdy) begin
                    arb_n = S_IDLE;
                end
            end 
        endcase
    end


    assign flit_encoder_data = flit_write_resp_vld ? flit_write_resp_data : flit_read_resp_data;
    assign flit_encoder_vld = flit_write_resp_vld ? arb_r == S_WRITE : arb_r == S_READ;

    assign flit_write_resp_rdy = arb_r == S_WRITE & flit_encoder_rdy;
    assign flit_read_resp_rdy = arb_r == S_READ & flit_encoder_rdy;
	assign src_bridge_vr_noc2_rdy = phy_init_done & decoder_src_bridge_vr_noc2_rdy;
	assign decoder_src_bridge_vr_noc2_val = phy_init_done & src_bridge_vr_noc2_val;

noc_axi4_decoder i_noc_axi4_decoder (
    .clk         (clk                   ),
    .rst_n       (rst_n                 ),
    .flit_in_val (decoder_src_bridge_vr_noc2_val),
    .flit_in_data(src_bridge_vr_noc2_dat),
    .flit_in_rdy (decoder_src_bridge_vr_noc2_rdy),
    .flit_op_data(flit_op_data          ),
    .flit_op_vld (flit_op_vld           ),
    .flit_op_rdy (flit_op_rdy           )
);


noc_axi4_write_pipe i_noc_axi4_write_pipe (
    .clk           (clk           ),
    .rst_n         (rst_n         ),
    .flit_op_data  (flit_op_data  ),
    .flit_op_vld   (flit_write_vld   ),
    .flit_op_rdy   (flit_write_rdy   ),
    .flit_resp_data(flit_write_resp_data),
    .flit_resp_vld (flit_write_resp_vld ),
    .flit_resp_rdy (flit_write_resp_rdy ),
    .*
);

noc_axi4_read_pipe i_noc_axi4_read_pipe (
    .clk           (clk           ),
    .rst_n         (rst_n         ),
    .flit_op_data  (flit_op_data  ),
    .flit_op_vld   (flit_read_vld   ),
    .flit_op_rdy   (flit_read_rdy   ),
    .flit_resp_data(flit_read_resp_data),
    .flit_resp_vld (flit_read_resp_vld ),
    .flit_resp_rdy (flit_read_resp_rdy ),
    .*
);


noc_axi4_encoder i_noc_axi4_encoder (
    .clk         (clk                   ),
    .rst_n       (rst_n                 ),
    .flit_op_data(flit_encoder_data          ),
    .flit_op_vld (flit_encoder_vld           ),
    .flit_op_rdy (flit_encoder_rdy           ),
    .flit_out    (bridge_dst_vr_noc3_dat),
    .flit_out_val(bridge_dst_vr_noc3_val),
    .flit_out_rdy(bridge_dst_vr_noc3_rdy)
);

endmodule : noc_axi4_bridge
