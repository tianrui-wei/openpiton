module noc_axi4_encoder_comb
import noc_axi4_pkg::*;
(

	input flit_op_t flit_op_data,
	output flit_t flit_out
);

always_comb begin : proc_flit_out
	flit_out                 = flit_op_data.head_flit;
	// write response does not carry payload
	flit_out[63:30] = flit_op_data.src_flit[63:30];
	flit_out[`MSG_OPTIONS_1] = '0;
	//FIXME: this doesn't work for NC
	flit_out[`MSG_LENGTH] = flit_op_data.is_write ? '0 : `PAYLOAD_LEN;
	unique case (flit_op_data.head_flit[`MSG_TYPE])
		`MSG_TYPE_LOAD_MEM : begin
			flit_out[`MSG_TYPE] = `MSG_TYPE_LOAD_MEM_ACK;
		end
		`MSG_TYPE_STORE_MEM : begin
			flit_out[`MSG_TYPE] = `MSG_TYPE_STORE_MEM_ACK;
		end
		`MSG_TYPE_NC_LOAD_REQ : begin
			flit_out[`MSG_TYPE] = `MSG_TYPE_NC_LOAD_MEM_ACK;
		end
		`MSG_TYPE_NC_STORE_REQ : begin
			flit_out[`MSG_TYPE] = `MSG_TYPE_NC_STORE_MEM_ACK;
		end
	endcase
end

endmodule : noc_axi4_encoder_comb
