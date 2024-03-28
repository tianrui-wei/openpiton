module noc_axi4_addr_aligner #(parameter int addr_width = 64, parameter int mask_width = addr_width / 8) (
	input              clk   , // Clock
	input              rst_n , // Asynchronous reset active low
	input logic [addr_width-1:0] addr_i,
	input logic [mask_width-1:0] mask_i,
	output logic [addr_width-1:0] addr_o,
	output logic [mask_width-1:0] mask_o

);

endmodule : noc_axi4_addr_aligner
