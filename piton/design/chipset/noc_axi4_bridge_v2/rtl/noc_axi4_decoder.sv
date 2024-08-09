module noc_axi4_decoder
	import noc_axi4_pkg::*;
(
	input                                   clk         , // Clock
	input                                   rst_n       , // Asynchronous reset active low
	// Noc interface
	input  wire                             flit_in_val ,
	input  wire       [`NOC_DATA_WIDTH-1:0] flit_in_data,
	output wire                             flit_in_rdy ,
	//    output wire                                   bridge_dst_vr_noc3_val,
	//    output wire [`NOC_DATA_WIDTH-1:0]             bridge_dst_vr_noc3_dat,
	//    input  wire                                   bridge_dst_vr_noc3_rdy,
	//    AXI_BUS.Master                                axi
	output      flit_op_t                   flit_op_data,
	output      logic                       flit_op_vld ,
	input       logic                       flit_op_rdy
);

	// an operation must be only be aligned to 64 bytes
	only_full_sized_op: assert property (@(posedge clk) disable iff (~rst_n)
	flit_op_vld |-> 
		(flit_op_data.size == `MSG_DATA_SIZE_64B) | (flit_op_data.size == `MSG_DATA_SIZE_0B));

	// an operation must be aligned to its size
	no_misaligned_req: assert property (@(posedge clk) disable iff (~rst_n) flit_op_vld && (flit_op_data.size == `MSG_DATA_SIZE_64B) |-> 
		(flit_op_data.addr[5:0] == '0));

	typedef enum logic [3:0] {S_ADDR, S_SRC, S_DATA, S_SEND, S_RECV} recv_state_t;

	wire flit_in_hs = flit_in_val & flit_in_rdy;

	// for the data flits only
	flit_num_t flit_ctr_r, flit_ctr_n;
	recv_state_t state_r, state_n;
	flit_op_t flit_op_r, flit_op_n;

	assign flit_op_data = flit_op_r;
	
	always_ff @(posedge clk) begin : proc_msg_state
		if(~rst_n) begin
			state_r    <= S_ADDR;
			flit_ctr_r <= 'X;
			flit_op_r  <= 'X;
		end else begin
			state_r    <= state_n;
			flit_ctr_r <= flit_ctr_n;
			flit_op_r  <= flit_op_n;
		end
	end

	assign flit_in_rdy = (state_r != S_SEND);
	assign flit_op_vld = (state_r == S_SEND);

	always_comb begin : proc_state_transition
		state_n    = state_r;
		flit_op_n  = flit_op_r;
		flit_ctr_n = flit_ctr_r;
		unique case (state_r)
			S_ADDR : begin
				if (flit_in_hs) begin
					assert(flit_in_data >= 3);
					state_n             = S_SRC;
					flit_op_n.head_flit = flit_in_data;
					// ASSERT: is write => flit != 0
					flit_op_n.num_flit  = flit_in_data[`MSG_LENGTH] - 2;
					flit_op_n.is_write = flit_in_data[`MSG_LENGTH] != 2;
					flit_op_n.uncachable = (flit_in_data[`MSG_TYPE] == `MSG_TYPE_NC_STORE_REQ) || (flit_in_data[`MSG_TYPE] == `MSG_TYPE_NC_LOAD_REQ);
					flit_ctr_n          = 0;
					flit_op_n.data_flits = 'X; // poison the data flits
				end
			end
			S_SRC : begin
				if (flit_in_hs) begin
					flit_op_n.addr_flit = flit_in_data;
					flit_op_n.addr = '0;
					flit_op_n.addr[`PHY_ADDR_WIDTH-1:0] = flit_in_data[`MSG_ADDR_];
					flit_op_n.size = flit_in_data[`MSG_DATA_SIZE_];
					state_n             = S_DATA;
				end
			end
			S_DATA : begin
				if (flit_in_hs) begin
					flit_op_n.src_flit = flit_in_data;
					// if there's no data, jump directly into the next phase. Otherwise, wait to receive data
					state_n            = flit_op_r.num_flit == 0 ? S_SEND : S_RECV;
				end
			end
			S_RECV : begin
				if (flit_in_hs) begin
					flit_ctr_n = flit_ctr_r + 1;
					flit_op_n.data_flits[7 - flit_ctr_r] = flit_in_data;
					if (flit_ctr_r == flit_op_r.num_flit - 1) begin
						state_n = S_SEND;
					end
				end
			end
			S_SEND : begin
				if (flit_op_vld & flit_op_rdy) begin
					state_n = S_ADDR;
				end
			end
		endcase
	end

endmodule : noc_axi4_decoder
