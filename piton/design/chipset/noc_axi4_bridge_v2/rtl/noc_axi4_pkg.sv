package noc_axi4_pkg;
	typedef logic [4:0] flit_num_t;
	typedef logic [`NOC_DATA_WIDTH-1:0] flit_t;
	typedef logic [2:0] size_t; // hardcoded for noc
	typedef logic [`PHY_ADDR_WIDTH-1:0] addr_t;

	typedef struct packed {
		flit_num_t num_flit;
		logic is_write;
		logic uncachable;
		size_t size;
		addr_t addr;
		flit_t head_flit;
		flit_t addr_flit;
		flit_t src_flit;
		flit_t [7:0] data_flits;
	} flit_op_t;	
endpackage : noc_axi4_pkg
