module noc_axi4_encoder
import noc_axi4_pkg::*; 
(
	input clk,    // Clock
	input rst_n,  // Asynchronous reset active low

	input flit_op_t flit_op_data,
	input logic flit_op_vld,
	output logic flit_op_rdy,

	output flit_t flit_out,
	output logic flit_out_val,
	input logic flit_out_rdy
);

typedef enum logic [2:0] {S_HEADER, S_DATA} state_t;

state_t state_r, state_n;
flit_num_t remaining_flit_r, remaining_flit_n;
flit_t [7:0] data_flits_r ;
flit_t [7:0] data_flits_n ;

always_ff @(posedge clk) begin : proc_state_r
	if(~rst_n) begin
		state_r <= S_HEADER;
		remaining_flit_r <= 'X;
		data_flits_r <= {'X};
	end else begin
		state_r <= state_n;
		remaining_flit_r <= remaining_flit_n;
		data_flits_r <= data_flits_n;
	end
end

// header is always in IDLE, but for read data, it is in a separate stage

assign flit_out_val = state_r == S_HEADER ? flit_op_vld : 1'b1;
assign flit_op_rdy = state_r == S_HEADER ? flit_out_rdy : 1'b0;


flit_t data_flit_output;

always_comb begin : proc_state	
	state_n = state_r;
	remaining_flit_n = remaining_flit_r;
	data_flits_n = data_flits_r;
	data_flit_output = 'X;
	unique case (state_r)
	S_HEADER: begin
		if (flit_op_vld & flit_op_rdy & ~flit_op_data.is_write) begin
			remaining_flit_n = `PAYLOAD_LEN - flit_op_data.num_flit - 1;
			data_flits_n = flit_op_data.data_flits;
		end
	end
	S_DATA: begin
		data_flit_output = data_flits_r[remaining_flit_r];
		if (flit_out_val & flit_out_rdy) begin
			remaining_flit_n = remaining_flit_r - 1;
			if (remaining_flit_r == 1) begin
				state_n = S_HEADER;
			end
		end
	end
	endcase
end


endmodule : noc_axi4_encoder
