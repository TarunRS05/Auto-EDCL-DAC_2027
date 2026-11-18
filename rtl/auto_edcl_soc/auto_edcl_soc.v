`timescale 1ns / 1ps

module auto_edcl_soc #(
    parameter ENABLE_FAST_MUL = 0
) (
    input clk,
    input reset,

    input               sample_valid,
    input  signed [7:0] sample_in,
    output              sample_ready,

    output trap,

    output mem_valid,
    output mem_instr,
    output [31:0] mem_addr,
    output [31:0] mem_wdata,
    output [3:0]  mem_wstrb,
    input  [31:0] ext_rdata,
    input         ext_ready,

    output soc_irq,
    output [31:0] soc_rdata,
    output [31:0] psum_0,
    output [31:0] psum_1,
    output [31:0] psum_2,
    output [31:0] psum_3,
    output [1:0]  prec_tag,
    output        event_alert
);

    wire resetn = ~reset;

    wire mem_ready;
    wire [31:0] mem_rdata;

    wire accel_irq;

    wire [127:0] coproc_psum;
    wire         coproc_event;
    wire [1:0]   coproc_prec;

    assign psum_0      = coproc_psum[31:0];
    assign psum_1      = coproc_psum[63:32];
    assign psum_2      = coproc_psum[95:64];
    assign psum_3      = coproc_psum[127:96];
    assign prec_tag    = coproc_prec;
    assign event_alert = coproc_event;

    assign soc_irq   = accel_irq | coproc_event;
    assign soc_rdata = mem_rdata;

    picorv32 #(
        .ENABLE_COUNTERS(0),
        .ENABLE_COUNTERS64(0),
        .ENABLE_REGS_16_31(0),
        .ENABLE_REGS_DUALPORT(0),
        .TWO_STAGE_SHIFT(0),
        .BARREL_SHIFTER(0),
        .TWO_CYCLE_COMPARE(0),
        .TWO_CYCLE_ALU(0),
        .CATCH_MISALIGN(0),
        .CATCH_ILLINSN(0),
        .ENABLE_IRQ(0),
        .ENABLE_IRQ_QREGS(0),
        .ENABLE_IRQ_TIMER(0),
        .ENABLE_FAST_MUL(ENABLE_FAST_MUL),
        .PROGADDR_RESET(32'h0000_0000),
        .STACKADDR(32'h0001_0000)
    ) cpu (
        .clk(clk),
        .resetn(resetn),
        .trap(trap),
        .mem_valid(mem_valid),
        .mem_instr(mem_instr),
        .mem_ready(mem_ready),
        .mem_addr(mem_addr),
        .mem_wdata(mem_wdata),
        .mem_wstrb(mem_wstrb),
        .mem_rdata(mem_rdata),
        .irq(32'h0),
        .eoi(),
        .mem_la_read(),
        .mem_la_write(),
        .mem_la_addr(),
        .mem_la_wdata(),
        .mem_la_wstrb(),
        .pcpi_valid(),
        .pcpi_insn(),
        .pcpi_rs1(),
        .pcpi_rs2(),
        .pcpi_wr(1'b0),
        .pcpi_rd(32'h0),
        .pcpi_wait(1'b0),
        .pcpi_ready(1'b0),
        .trace_valid(),
        .trace_data()
    );

    wire is_ram = (mem_addr[31:28] == 4'h0);
    wire is_acc = (mem_addr[31:28] == 4'h1);
    wire is_sys = (mem_addr[31:28] == 4'h2);
    wire is_uart = (mem_addr[31:28] == 4'h3);

    wire mem_valid_ram = mem_valid && is_ram;
    wire mem_valid_acc = mem_valid && is_acc;

    wire mem_ready_ram;
    wire mem_ready_acc;

    wire [31:0] mem_rdata_ram;
    wire [31:0] mem_rdata_acc;

    assign mem_ready = is_ram ? mem_ready_ram :
                       is_acc ? mem_ready_acc :
                       is_sys ? 1'b1 :
                       is_uart ? 1'b1 :
                       1'b0;

    assign mem_rdata = is_ram ? mem_rdata_ram :
                       is_acc ? mem_rdata_acc :
                       32'h0;

    reg mem_ready_ram_q;
    assign mem_ready_ram = (mem_addr[31:5] == 0) ? mem_ready_ram_q : ext_ready;

    wire [31:0] boot_rom [0:7];
    assign boot_rom[0] = 32'h100002b7;
    assign boot_rom[1] = 32'h0060f337;
    assign boot_rom[2] = 32'h1c130313;
    assign boot_rom[3] = 32'h0062a023;
    assign boot_rom[4] = 32'h51338393;
    assign boot_rom[5] = 32'h0072a623;
    assign boot_rom[6] = 32'h0102a403;
    assign boot_rom[7] = 32'hff5ff06f;

    wire [2:0] rom_addr = mem_addr[4:2];
    assign mem_rdata_ram = (mem_addr[31:5] == 0) ? boot_rom[rom_addr] : ext_rdata;

    always @(posedge clk) begin
        if (reset)
            mem_ready_ram_q <= 1'b0;
        else
            mem_ready_ram_q <= mem_valid_ram && !mem_ready_ram_q;
    end

    wire        axi_awvalid;
    wire        axi_awready;
    wire [31:0] axi_awaddr;
    wire [ 2:0] axi_awprot;
    wire        axi_wvalid;
    wire        axi_wready;
    wire [31:0] axi_wdata;
    wire [ 3:0] axi_wstrb;
    wire        axi_bvalid;
    wire        axi_bready;
    wire        axi_arvalid;
    wire        axi_arready;
    wire [31:0] axi_araddr;
    wire [ 2:0] axi_arprot;
    wire        axi_rvalid;
    wire        axi_rready;
    wire [31:0] axi_rdata;

    picorv32_axi_adapter axi_adapter (
        .clk(clk),
        .resetn(resetn),

        .mem_valid(mem_valid_acc),
        .mem_instr(1'b0),
        .mem_ready(mem_ready_acc),
        .mem_addr(mem_addr),
        .mem_wdata(mem_wdata),
        .mem_wstrb(mem_wstrb),
        .mem_rdata(mem_rdata_acc),

        .mem_axi_awvalid(axi_awvalid),
        .mem_axi_awready(axi_awready),
        .mem_axi_awaddr(axi_awaddr),
        .mem_axi_awprot(axi_awprot),
        .mem_axi_wvalid(axi_wvalid),
        .mem_axi_wready(axi_wready),
        .mem_axi_wdata(axi_wdata),
        .mem_axi_wstrb(axi_wstrb),
        .mem_axi_bvalid(axi_bvalid),
        .mem_axi_bready(axi_bready),
        .mem_axi_arvalid(axi_arvalid),
        .mem_axi_arready(axi_arready),
        .mem_axi_araddr(axi_araddr),
        .mem_axi_arprot(axi_arprot),
        .mem_axi_rvalid(axi_rvalid),
        .mem_axi_rready(axi_rready),
        .mem_axi_rdata(axi_rdata)
    );

    axi_wrapper #(
        .DATA_WIDTH(8),
        .ACCUM_WIDTH(32),
        .C_S_AXI_DATA_WIDTH(32),
        .C_S_AXI_ADDR_WIDTH(6)
    ) accelerator (
        .S_AXI_ACLK(clk),
        .S_AXI_ARESETN(resetn),

        .ext_sample_valid(sample_valid),
        .ext_sample_in(sample_in),
        .sample_ready(sample_ready),

        .S_AXI_AWADDR(axi_awaddr[5:0]),
        .S_AXI_AWVALID(axi_awvalid),
        .S_AXI_AWREADY(axi_awready),

        .S_AXI_WDATA(axi_wdata),
        .S_AXI_WSTRB(axi_wstrb),
        .S_AXI_WVALID(axi_wvalid),
        .S_AXI_WREADY(axi_wready),

        .S_AXI_BRESP(),
        .S_AXI_BVALID(axi_bvalid),
        .S_AXI_BREADY(axi_bready),

        .S_AXI_ARADDR(axi_araddr[5:0]),
        .S_AXI_ARVALID(axi_arvalid),
        .S_AXI_ARREADY(axi_arready),

        .S_AXI_RDATA(axi_rdata),
        .S_AXI_RRESP(),
        .S_AXI_RVALID(axi_rvalid),
        .S_AXI_RREADY(axi_rready),

        .interrupt_out(accel_irq),
        .coproc_psum_out(coproc_psum),
        .coproc_event_out(coproc_event),
        .coproc_prec_out(coproc_prec)
    );

endmodule

module picorv32 #(
	parameter [ 0:0] ENABLE_COUNTERS = 1,
	parameter [ 0:0] ENABLE_COUNTERS64 = 1,
	parameter [ 0:0] ENABLE_REGS_16_31 = 1,
	parameter [ 0:0] ENABLE_REGS_DUALPORT = 1,
	parameter [ 0:0] LATCHED_MEM_RDATA = 0,
	parameter [ 0:0] TWO_STAGE_SHIFT = 1,
	parameter [ 0:0] BARREL_SHIFTER = 0,
	parameter [ 0:0] TWO_CYCLE_COMPARE = 0,
	parameter [ 0:0] TWO_CYCLE_ALU = 0,
	parameter [ 0:0] COMPRESSED_ISA = 0,
	parameter [ 0:0] CATCH_MISALIGN = 1,
	parameter [ 0:0] CATCH_ILLINSN = 1,
	parameter [ 0:0] ENABLE_PCPI = 0,
	parameter [ 0:0] ENABLE_MUL = 0,
	parameter [ 0:0] ENABLE_FAST_MUL = 0,
	parameter [ 0:0] ENABLE_DIV = 0,
	parameter [ 0:0] ENABLE_IRQ = 0,
	parameter [ 0:0] ENABLE_IRQ_QREGS = 1,
	parameter [ 0:0] ENABLE_IRQ_TIMER = 1,
	parameter [ 0:0] ENABLE_TRACE = 0,
	parameter [ 0:0] REGS_INIT_ZERO = 0,
	parameter [31:0] MASKED_IRQ = 32'h 0000_0000,
	parameter [31:0] LATCHED_IRQ = 32'h ffff_ffff,
	parameter [31:0] PROGADDR_RESET = 32'h 0000_0000,
	parameter [31:0] PROGADDR_IRQ = 32'h 0000_0010,
	parameter [31:0] STACKADDR = 32'h ffff_ffff
) (

	input clk, resetn,
	output reg trap,

	output reg        mem_valid,
	output reg        mem_instr,
	input             mem_ready,

	output reg [31:0] mem_addr,
	output reg [31:0] mem_wdata,
	output reg [ 3:0] mem_wstrb,
	input      [31:0] mem_rdata,

	output            mem_la_read,
	output            mem_la_write,
	output     [31:0] mem_la_addr,
	output reg [31:0] mem_la_wdata,
	output reg [ 3:0] mem_la_wstrb,

	output reg        pcpi_valid,
	output reg [31:0] pcpi_insn,
	output     [31:0] pcpi_rs1,
	output     [31:0] pcpi_rs2,

	input             pcpi_wr,
	input      [31:0] pcpi_rd,
	input             pcpi_wait,
	input             pcpi_ready,

	input      [31:0] irq,

	output reg [31:0] eoi,

	output reg        trace_valid,
	output reg [35:0] trace_data
);
	localparam integer regfile_size  = ENABLE_REGS_16_31 ? 32 : 16;
	localparam integer regindex_bits = ENABLE_REGS_16_31 ? 5 : 4;

	localparam [2:0] S_FETCH  = 3'd0;
	localparam [2:0] S_DECODE = 3'd1;
	localparam [2:0] S_SERIAL = 3'd2;
	localparam [2:0] S_SHIFT  = 3'd3;
	localparam [2:0] S_LOAD   = 3'd4;
	localparam [2:0] S_STORE  = 3'd5;
	localparam [2:0] S_WB     = 3'd6;

	localparam [6:0] OP_LUI    = 7'b0110111;
	localparam [6:0] OP_AUIPC  = 7'b0010111;
	localparam [6:0] OP_JAL    = 7'b1101111;
	localparam [6:0] OP_JALR   = 7'b1100111;
	localparam [6:0] OP_BRANCH = 7'b1100011;
	localparam [6:0] OP_LOAD   = 7'b0000011;
	localparam [6:0] OP_STORE  = 7'b0100011;
	localparam [6:0] OP_IMM    = 7'b0010011;
	localparam [6:0] OP_REG    = 7'b0110011;
	localparam [6:0] OP_FENCE  = 7'b0001111;
	localparam [6:0] OP_SYSTEM = 7'b1110011;

	reg [2:0]  state;
	reg [4:0]  bit_cnt;
	reg [4:0]  shamt;
	reg [31:0] pc;
	reg [31:0] ir;
	reg [31:0] buf_reg;
	reg [31:0] wdata_reg;
	reg        alu_cy;
	reg        pc4_cy;
	reg        pc_br_cy;
	reg        eq_acc;

	reg [31:0] cpuregs [1:regfile_size-1];

	wire [6:0] opcode = ir[6:0];
	wire [2:0] funct3 = ir[14:12];
	wire [regindex_bits-1:0] rd_idx  = ir[ 7 +: regindex_bits];
	wire [regindex_bits-1:0] rs1_idx = ir[15 +: regindex_bits];
	wire [regindex_bits-1:0] rs2_idx = ir[20 +: regindex_bits];
	wire rd_nz = |rd_idx;

	wire [31:0] imm_i = {{20{ir[31]}}, ir[31:20]};
	wire [31:0] imm_s = {{20{ir[31]}}, ir[31:25], ir[11:7]};
	wire [31:0] imm_b = {{19{ir[31]}}, ir[31], ir[7], ir[30:25], ir[11:8], 1'b0};
	wire [31:0] imm_u = {ir[31:12], 12'b0};
	wire [31:0] imm_j = {{11{ir[31]}}, ir[31], ir[19:12], ir[20], ir[30:21], 1'b0};

	wire is_sub_op   = (opcode == OP_REG) && (funct3 == 3'b000) && ir[30];
	wire is_slt_op   = ((opcode == OP_REG) || (opcode == OP_IMM)) &&
	                   ((funct3 == 3'b010) || (funct3 == 3'b011));
	wire is_shift_op = ((opcode == OP_REG) || (opcode == OP_IMM)) &&
	                   ((funct3 == 3'b001) || (funct3 == 3'b101));
	wire alu_sub     = (opcode == OP_BRANCH) || is_sub_op || is_slt_op;

	wire uses_rs1 = (opcode == OP_JALR) || (opcode == OP_BRANCH) ||
	                (opcode == OP_LOAD) || (opcode == OP_STORE)  ||
	                (opcode == OP_IMM)  || (opcode == OP_REG);
	wire uses_rs2 = (opcode == OP_BRANCH) || (opcode == OP_STORE) || (opcode == OP_REG);

	wire serial_wb = rd_nz && (
		(opcode == OP_LUI)   ||
		(opcode == OP_AUIPC) ||
		(opcode == OP_JAL)   ||
		(opcode == OP_JALR)  ||
		((opcode == OP_SYSTEM) && (funct3 != 3'b000)) ||
		(((opcode == OP_IMM) || (opcode == OP_REG)) && !is_slt_op && !is_shift_op)
	);

	wire [regfile_size-1:0] rs1_dec = uses_rs1 ? ({{(regfile_size-1){1'b0}}, 1'b1} << rs1_idx) : {regfile_size{1'b0}};
	wire [regfile_size-1:0] rs2_dec = uses_rs2 ? ({{(regfile_size-1){1'b0}}, 1'b1} << rs2_idx) : {regfile_size{1'b0}};
	wire [regfile_size-1:0] rd_dec  = ({{(regfile_size-1){1'b0}}, 1'b1} << rd_idx);

	wire [regfile_size-1:1] rs1_oh = rs1_dec[regfile_size-1:1];
	wire [regfile_size-1:1] rs2_oh = rs2_dec[regfile_size-1:1];
	wire [regfile_size-1:1] rd_oh  = rd_dec[regfile_size-1:1];
	wire [regfile_size-1:1] reg_lsb;

	genvar g;
	generate
		for (g = 1; g < regfile_size; g = g + 1) begin : gen_rf_lsb
			assign reg_lsb[g] = cpuregs[g][0];
		end
	endgenerate

	wire rs1_bit = |(rs1_oh & reg_lsb);
	wire rs2_bit = |(rs2_oh & reg_lsb);

	wire pc4_b        = (bit_cnt == 5'd2);
	wire pc_plus4_bit = pc[0] ^ pc4_b ^ pc4_cy;
	wire pc4_cy_next  = (pc[0] & pc4_b) | (pc[0] & pc4_cy) | (pc4_b & pc4_cy);

	wire pc_br_bit    = pc[0] ^ buf_reg[0] ^ pc_br_cy;
	wire pc_br_cy_nxt = (pc[0] & buf_reg[0]) | (pc[0] & pc_br_cy) | (buf_reg[0] & pc_br_cy);

	wire alu_a_bit     = (opcode == OP_LUI) ? 1'b0 :
	                     ((opcode == OP_AUIPC) || (opcode == OP_JAL)) ? pc[0] : rs1_bit;
	wire alu_b_bit_raw = ((opcode == OP_REG) || (opcode == OP_BRANCH)) ? rs2_bit : buf_reg[0];
	wire alu_b_bit     = alu_sub ? ~alu_b_bit_raw : alu_b_bit_raw;
	wire alu_sum_bit   = alu_a_bit ^ alu_b_bit ^ alu_cy;
	wire alu_cy_next   = (alu_a_bit & alu_b_bit) | (alu_a_bit & alu_cy) | (alu_b_bit & alu_cy);

	wire eq_now  = eq_acc & (alu_a_bit == alu_b_bit_raw);
	wire cmp_ltu = ~alu_cy_next;
	wire cmp_lts = (alu_a_bit != alu_b_bit_raw) ? alu_a_bit : alu_sum_bit;

	reg branch_cond;
	always @* begin
		case (funct3)
			3'b000:  branch_cond =  eq_now;
			3'b001:  branch_cond = ~eq_now;
			3'b100:  branch_cond =  cmp_lts;
			3'b101:  branch_cond = ~cmp_lts;
			3'b110:  branch_cond =  cmp_ltu;
			3'b111:  branch_cond = ~cmp_ltu;
			default: branch_cond = 1'b0;
		endcase
	end

	reg serial_rd_bit;
	always @* begin
		if ((opcode == OP_JAL) || (opcode == OP_JALR)) begin
			serial_rd_bit = pc_plus4_bit;
		end else if ((opcode == OP_SYSTEM) || (opcode == OP_FENCE)) begin
			serial_rd_bit = 1'b0;
		end else if ((opcode == OP_LUI) || (opcode == OP_AUIPC)) begin
			serial_rd_bit = alu_sum_bit;
		end else begin
			case (funct3)
				3'b000:  serial_rd_bit = alu_sum_bit;
				3'b100:  serial_rd_bit = alu_a_bit ^ alu_b_bit_raw;
				3'b110:  serial_rd_bit = alu_a_bit | alu_b_bit_raw;
				3'b111:  serial_rd_bit = alu_a_bit & alu_b_bit_raw;
				default: serial_rd_bit = alu_sum_bit;
			endcase
		end
	end

	wire wb_active = ((state == S_SERIAL) && serial_wb) || (state == S_WB);
	wire wb_bit_in = (state == S_WB) ? buf_reg[0] : serial_rd_bit;
	wire [regfile_size-1:1] reg_wb_en  = wb_active ? rd_oh : {regfile_size-1{1'b0}};
	wire [regfile_size-1:1] reg_rot_en = (state == S_SERIAL) ? (rs1_oh | rs2_oh) : {regfile_size-1{1'b0}};
	wire [regfile_size-1:1] reg_en     = reg_wb_en | reg_rot_en;

	wire [31:0] serial_word = {alu_sum_bit, buf_reg[31:1]};
	wire [31:0] rs2_word    = {rs2_bit, wdata_reg[31:1]};

	reg [7:0] load_byte;
	always @* begin
		case (buf_reg[1:0])
			2'b00: load_byte = mem_rdata[ 7: 0];
			2'b01: load_byte = mem_rdata[15: 8];
			2'b10: load_byte = mem_rdata[23:16];
			2'b11: load_byte = mem_rdata[31:24];
		endcase
	end

	wire [15:0] load_half = buf_reg[1] ? mem_rdata[31:16] : mem_rdata[15:0];

	reg [31:0] load_val;
	always @* begin
		case (funct3)
			3'b000:  load_val = {{24{load_byte[7]}}, load_byte};
			3'b001:  load_val = {{16{load_half[15]}}, load_half};
			3'b010:  load_val = mem_rdata;
			3'b100:  load_val = {24'd0, load_byte};
			3'b101:  load_val = {16'd0, load_half};
			default: load_val = mem_rdata;
		endcase
	end

	reg [31:0] store_wdata;
	reg [ 3:0] store_wstrb;
	always @* begin
		case (funct3[1:0])
			2'b00: begin
				store_wdata = {4{rs2_word[7:0]}};
				store_wstrb = 4'b0001 << serial_word[1:0];
			end
			2'b01: begin
				store_wdata = {2{rs2_word[15:0]}};
				store_wstrb = serial_word[1] ? 4'b1100 : 4'b0011;
			end
			default: begin
				store_wdata = rs2_word;
				store_wstrb = 4'b1111;
			end
		endcase
	end

	integer k;
	always @(posedge clk) begin
		if (!resetn) begin
			state      <= S_FETCH;
			bit_cnt    <= 5'd0;
			shamt      <= 5'd0;
			pc         <= PROGADDR_RESET;
			ir         <= 32'h0000_0013;
			buf_reg    <= 32'h0;
			wdata_reg  <= 32'h0;
			alu_cy     <= 1'b0;
			pc4_cy     <= 1'b0;
			pc_br_cy   <= 1'b0;
			eq_acc     <= 1'b1;
			mem_valid  <= 1'b0;
			mem_wstrb  <= 4'b0000;
			trap       <= 1'b0;
			for (k = 1; k < regfile_size; k = k + 1)
				cpuregs[k] <= (k == 2) ? (REGS_INIT_ZERO ? 32'h0 : STACKADDR) : 32'h0;
		end else begin
			for (k = 1; k < regfile_size; k = k + 1) begin
				if (reg_en[k])
					cpuregs[k] <= {reg_wb_en[k] ? wb_bit_in : cpuregs[k][0], cpuregs[k][31:1]};
			end

			case (state)
				S_FETCH: begin
					mem_wstrb <= 4'b0000;
					if (!trap) begin
						if (!mem_valid) begin
							mem_valid <= 1'b1;
						end else if (mem_ready) begin
							mem_valid <= 1'b0;
							ir        <= mem_rdata;
							state     <= S_DECODE;
						end
					end
				end

				S_DECODE: begin
					bit_cnt  <= 5'd0;
					alu_cy   <= alu_sub;
					pc4_cy   <= 1'b0;
					pc_br_cy <= 1'b0;
					eq_acc   <= 1'b1;
					shamt    <= ir[24:20];

					if ((opcode == OP_LUI) || (opcode == OP_AUIPC))
						buf_reg <= imm_u;
					else if (opcode == OP_JAL)
						buf_reg <= imm_j;
					else if (opcode == OP_BRANCH)
						buf_reg <= imm_b;
					else if (opcode == OP_STORE)
						buf_reg <= imm_s;
					else
						buf_reg <= imm_i;

					if ((opcode == OP_SYSTEM) && (funct3 == 3'b000)) begin
						trap  <= 1'b1;
						state <= S_FETCH;
					end else begin
						state <= S_SERIAL;
					end
				end

				S_SERIAL: begin
					bit_cnt  <= bit_cnt + 5'd1;
					alu_cy   <= alu_cy_next;
					pc4_cy   <= pc4_cy_next;
					pc_br_cy <= pc_br_cy_nxt;
					eq_acc   <= eq_now;

					if ((opcode == OP_REG) && is_shift_op && (bit_cnt < 5'd5))
						shamt <= {rs2_bit, shamt[4:1]};

					if (opcode == OP_STORE)
						wdata_reg <= {rs2_bit, wdata_reg[31:1]};

					if (bit_cnt != 5'd31) begin
						pc <= {pc_plus4_bit, pc[31:1]};
						if (opcode == OP_BRANCH)
							buf_reg <= {pc_br_bit, buf_reg[31:1]};
						else if (is_shift_op)
							buf_reg <= {rs1_bit, buf_reg[31:1]};
						else
							buf_reg <= {alu_sum_bit, buf_reg[31:1]};
					end else begin
						if ((opcode == OP_JAL) || (opcode == OP_JALR))
							pc <= {serial_word[31:1], (opcode == OP_JALR) ? 1'b0 : serial_word[0]};
						else if ((opcode == OP_BRANCH) && branch_cond)
							pc <= {pc_br_bit, buf_reg[31:1]};
						else
							pc <= {pc_plus4_bit, pc[31:1]};

						if (is_slt_op) begin
							buf_reg <= {31'd0, (funct3 == 3'b011) ? cmp_ltu : cmp_lts};
							state   <= rd_nz ? S_WB : S_FETCH;
						end else if (is_shift_op) begin
							buf_reg <= {rs1_bit, buf_reg[31:1]};
							state   <= S_SHIFT;
						end else if (opcode == OP_LOAD) begin
							buf_reg   <= serial_word;
							mem_wstrb <= 4'b0000;
							mem_valid <= 1'b1;
							state     <= S_LOAD;
						end else if (opcode == OP_STORE) begin
							buf_reg   <= serial_word;
							wdata_reg <= store_wdata;
							mem_wstrb <= store_wstrb;
							mem_valid <= 1'b1;
							state     <= S_STORE;
						end else begin
							state <= S_FETCH;
						end
					end
				end

				S_SHIFT: begin
					if (shamt != 5'd0) begin
						if (funct3 == 3'b001)
							buf_reg <= {buf_reg[30:0], 1'b0};
						else
							buf_reg <= {ir[30] & buf_reg[31], buf_reg[31:1]};
						shamt <= shamt - 5'd1;
					end else begin
						bit_cnt <= 5'd0;
						state   <= rd_nz ? S_WB : S_FETCH;
					end
				end

				S_LOAD: begin
					if (mem_valid && mem_ready) begin
						mem_valid <= 1'b0;
						buf_reg   <= load_val;
						bit_cnt   <= 5'd0;
						state     <= rd_nz ? S_WB : S_FETCH;
					end
				end

				S_STORE: begin
					if (mem_valid && mem_ready) begin
						mem_valid <= 1'b0;
						mem_wstrb <= 4'b0000;
						state     <= S_FETCH;
					end
				end

				S_WB: begin
					bit_cnt <= bit_cnt + 5'd1;
					buf_reg <= {1'b0, buf_reg[31:1]};
					if (bit_cnt == 5'd31)
						state <= S_FETCH;
				end

				default: state <= S_FETCH;
			endcase
		end
	end

	always @* begin
		mem_instr    = (state == S_FETCH);
		mem_addr     = (state == S_FETCH) ? {pc[31:2], 2'b00} : {buf_reg[31:2], 2'b00};
		mem_wdata    = wdata_reg;
		mem_la_wdata = 32'h0;
		mem_la_wstrb = 4'h0;
		pcpi_valid   = 1'b0;
		pcpi_insn    = 32'h0;
		eoi          = 32'h0;
		trace_valid  = 1'b0;
		trace_data   = 36'h0;
	end

	assign mem_la_read  = 1'b0;
	assign mem_la_write = 1'b0;
	assign mem_la_addr  = 32'h0;
	assign pcpi_rs1     = 32'h0;
	assign pcpi_rs2     = 32'h0;

endmodule

module picorv32_axi_adapter (
	input clk, resetn,

	output        mem_axi_awvalid,
	input         mem_axi_awready,
	output [31:0] mem_axi_awaddr,
	output [ 2:0] mem_axi_awprot,

	output        mem_axi_wvalid,
	input         mem_axi_wready,
	output [31:0] mem_axi_wdata,
	output [ 3:0] mem_axi_wstrb,

	input         mem_axi_bvalid,
	output        mem_axi_bready,

	output        mem_axi_arvalid,
	input         mem_axi_arready,
	output [31:0] mem_axi_araddr,
	output [ 2:0] mem_axi_arprot,

	input         mem_axi_rvalid,
	output        mem_axi_rready,
	input  [31:0] mem_axi_rdata,

	input         mem_valid,
	input         mem_instr,
	output        mem_ready,
	input  [31:0] mem_addr,
	input  [31:0] mem_wdata,
	input  [ 3:0] mem_wstrb,
	output [31:0] mem_rdata
);
	reg ack_awvalid;
	reg ack_arvalid;
	reg ack_wvalid;
	reg xfer_done;

	assign mem_axi_awvalid = mem_valid && |mem_wstrb && !ack_awvalid;
	assign mem_axi_awaddr = mem_addr;
	assign mem_axi_awprot = 0;

	assign mem_axi_arvalid = mem_valid && !mem_wstrb && !ack_arvalid;
	assign mem_axi_araddr = mem_addr;
	assign mem_axi_arprot = mem_instr ? 3'b100 : 3'b000;

	assign mem_axi_wvalid = mem_valid && |mem_wstrb && !ack_wvalid;
	assign mem_axi_wdata = mem_wdata;
	assign mem_axi_wstrb = mem_wstrb;

	assign mem_ready = mem_axi_bvalid || mem_axi_rvalid;
	assign mem_axi_bready = mem_valid && |mem_wstrb;
	assign mem_axi_rready = mem_valid && !mem_wstrb;
	assign mem_rdata = mem_axi_rdata;

	always @(posedge clk) begin
		if (!resetn) begin
			ack_awvalid <= 1'b0;
			ack_arvalid <= 1'b0;
			ack_wvalid  <= 1'b0;
			xfer_done   <= 1'b0;
		end else begin
			xfer_done <= mem_valid && mem_ready;
			if (mem_axi_awready && mem_axi_awvalid)
				ack_awvalid <= 1;
			if (mem_axi_arready && mem_axi_arvalid)
				ack_arvalid <= 1;
			if (mem_axi_wready && mem_axi_wvalid)
				ack_wvalid <= 1;
			if (xfer_done || !mem_valid) begin
				ack_awvalid <= 0;
				ack_arvalid <= 0;
				ack_wvalid <= 0;
			end
		end
	end
endmodule

module axi_wrapper #(
    parameter DATA_WIDTH = 8,
    parameter ACCUM_WIDTH = 32,
    parameter C_S_AXI_DATA_WIDTH = 32,
    parameter C_S_AXI_ADDR_WIDTH = 6
)(

    input  wire                          S_AXI_ACLK,
    input  wire                          S_AXI_ARESETN,

    input  wire [C_S_AXI_ADDR_WIDTH-1:0] S_AXI_AWADDR,

    input  wire                          S_AXI_AWVALID,
    output wire                          S_AXI_AWREADY,

    input  wire [C_S_AXI_DATA_WIDTH-1:0] S_AXI_WDATA,

    input  wire [(C_S_AXI_DATA_WIDTH/8)-1:0] S_AXI_WSTRB,

    input  wire                          S_AXI_WVALID,
    output wire                          S_AXI_WREADY,

    output wire [1:0]                    S_AXI_BRESP,
    output wire                          S_AXI_BVALID,
    input  wire                          S_AXI_BREADY,

    input  wire [C_S_AXI_ADDR_WIDTH-1:0] S_AXI_ARADDR,

    input  wire                          S_AXI_ARVALID,
    output wire                          S_AXI_ARREADY,

    output wire [C_S_AXI_DATA_WIDTH-1:0] S_AXI_RDATA,
    output wire [1:0]                    S_AXI_RRESP,
    output wire                          S_AXI_RVALID,
    input  wire                          S_AXI_RREADY,

    output wire                          interrupt_out,

    output wire [127:0]                  coproc_psum_out,
    output wire                          coproc_event_out,
    output wire [1:0]                    coproc_prec_out,

    input  wire                          ext_sample_valid,
    input  wire signed [DATA_WIDTH-1:0]  ext_sample_in,
    output wire                          sample_ready
);

    reg axi_awready;
    reg axi_wready;
    reg axi_bvalid;
    reg axi_arready;
    reg [C_S_AXI_DATA_WIDTH-1:0] axi_rdata;
    reg axi_rvalid;

    assign S_AXI_AWREADY = axi_awready;
    assign S_AXI_WREADY  = axi_wready;
    assign S_AXI_BRESP   = 2'b00;
    assign S_AXI_BVALID  = axi_bvalid;
    assign S_AXI_ARREADY = axi_arready;
    assign S_AXI_RDATA   = axi_rdata;
    assign S_AXI_RRESP   = 2'b00;
    assign S_AXI_RVALID  = axi_rvalid;

    reg [1:0]  ctrl_low_reg;
    reg [7:0]  ctrl_margin_reg;
    reg [3:0]  ctrl_row_en_reg;
    reg [2:0]  ctrl_mode_reg;
    reg [11:0] uadt_cfg_reg;
    reg [15:0] emrs_cfg_reg;
    reg [10:0] learn_ctrl_reg;
    reg        status_event_sticky;

    wire [31:0] ctrl_reg_readback = {9'd0, ctrl_mode_reg, 4'd0, ctrl_row_en_reg, ctrl_margin_reg, 2'd0, ctrl_low_reg};

    reg        valid_in_pulse;
    reg [7:0]  sample_in_reg;
    reg        weight_load_pulse;
    reg [7:0]  weight_data_reg;
    reg [1:0]  weight_col_reg;
    reg [1:0]  weight_row_reg;

    wire        raw_event;
    wire        event_detected;
    wire [1:0]  current_prec_tag;
    wire [6:0]  uadt_thresh;
    wire [15:0] inter_event_cycles;
    wire [15:0] active_int8_cycles;
    wire [(4*ACCUM_WIDTH)-1:0] psum_out_raw;

    always @(posedge S_AXI_ACLK) begin
        if (S_AXI_ARESETN == 1'b0) begin
            axi_awready       <= 1'b0;
            axi_wready        <= 1'b0;
            axi_bvalid        <= 1'b0;
            ctrl_low_reg      <= 2'b00;
            ctrl_margin_reg   <= 8'd28;
            ctrl_row_en_reg   <= 4'hF;
            ctrl_mode_reg     <= 3'd6;
            uadt_cfg_reg      <= {2'b01, 3'd4, 7'd32};
            emrs_cfg_reg      <= 16'h0;
            learn_ctrl_reg    <= 11'h0;
            sample_in_reg     <= 8'd0;
            weight_data_reg   <= 8'd0;
            weight_col_reg    <= 2'd0;
            weight_row_reg    <= 2'd0;
            valid_in_pulse    <= 1'b0;
            weight_load_pulse <= 1'b0;
        end else begin
            valid_in_pulse    <= 1'b0;
            weight_load_pulse <= 1'b0;

            if (~axi_awready && S_AXI_AWVALID && S_AXI_WVALID) begin
                axi_awready <= 1'b1;
                axi_wready  <= 1'b1;
            end else begin
                axi_awready <= 1'b0;
                axi_wready  <= 1'b0;
            end

            if (axi_wready && S_AXI_WVALID && axi_awready && S_AXI_AWVALID) begin
                case (S_AXI_AWADDR[5:2])
                    4'd0: begin
                        ctrl_low_reg    <= S_AXI_WDATA[1:0];
                        ctrl_margin_reg <= S_AXI_WDATA[11:4];
                        ctrl_row_en_reg <= S_AXI_WDATA[15:12];
                        ctrl_mode_reg   <= S_AXI_WDATA[22:20];
                    end
                    4'd2: begin
                        sample_in_reg  <= S_AXI_WDATA[7:0];
                        valid_in_pulse <= 1'b1;
                    end
                    4'd3: begin
                        weight_data_reg   <= S_AXI_WDATA[7:0];
                        weight_col_reg    <= S_AXI_WDATA[9:8];
                        weight_row_reg    <= S_AXI_WDATA[12:11];
                        weight_load_pulse <= 1'b1;
                    end
                    4'd12: uadt_cfg_reg   <= S_AXI_WDATA[11:0];
                    4'd13: emrs_cfg_reg   <= S_AXI_WDATA[15:0];
                    4'd14: learn_ctrl_reg <= S_AXI_WDATA[10:0];
                    default: ;
                endcase
                axi_bvalid <= 1'b1;
            end else if (S_AXI_BREADY && axi_bvalid) begin
                axi_bvalid <= 1'b0;
            end
        end
    end

    always @(posedge S_AXI_ACLK) begin
        if (S_AXI_ARESETN == 1'b0) begin
            axi_arready         <= 1'b0;
            axi_rvalid          <= 1'b0;
            axi_rdata           <= 32'h0;
            status_event_sticky <= 1'b0;
        end else begin
            if (~axi_arready && S_AXI_ARVALID) begin
                axi_arready <= 1'b1;
                axi_rvalid  <= 1'b1;
                case (S_AXI_ARADDR[5:2])
                    4'd0:  axi_rdata <= ctrl_reg_readback;
                    4'd1: begin
                        axi_rdata           <= {22'd0, uadt_thresh, current_prec_tag, status_event_sticky};
                        status_event_sticky <= 1'b0;
                    end
                    4'd4:  axi_rdata <= psum_out_raw[31:0];
                    4'd5:  axi_rdata <= psum_out_raw[63:32];
                    4'd6:  axi_rdata <= psum_out_raw[95:64];
                    4'd7:  axi_rdata <= psum_out_raw[127:96];
                    4'd12: axi_rdata <= {20'd0, uadt_cfg_reg};
                    4'd13: axi_rdata <= {16'd0, emrs_cfg_reg};
                    4'd14: axi_rdata <= {21'd0, learn_ctrl_reg};
                    4'd15: axi_rdata <= {active_int8_cycles, inter_event_cycles};
                    default: axi_rdata <= 32'h0;
                endcase
            end else if (axi_rvalid && S_AXI_RREADY) begin
                axi_rvalid  <= 1'b0;
                axi_arready <= 1'b0;
            end

            if (event_detected) begin
                status_event_sticky <= 1'b1;
            end
        end
    end

    wire clk = S_AXI_ACLK;
    wire rst = (~S_AXI_ARESETN) | ctrl_low_reg[1];
    wire start_enable = ctrl_low_reg[0];

    reg prev_event;
    always @(posedge clk) begin
        if (rst) prev_event <= 1'b0;
        else     prev_event <= raw_event;
    end
    assign event_detected = raw_event & (~prev_event);
    assign interrupt_out  = status_event_sticky;

    wire [2:0] op_mode = ctrl_mode_reg;

    wire eff_valid_in = (valid_in_pulse & start_enable) | ext_sample_valid;
    wire signed [DATA_WIDTH-1:0] eff_sample_in = ext_sample_valid ? ext_sample_in : sample_in_reg;
    wire stream_fifo_full_sig;
    assign sample_ready = ~stream_fifo_full_sig;

    dpw_top #(
        .DATA_WIDTH(DATA_WIDTH),
        .ACCUM_WIDTH(ACCUM_WIDTH),
        .ROWS(4),
        .COLS(4),
        .NOISE_FLOOR(7'd32),
        .DECAY_SHIFT(4),
        .PE_TYPE(1)
    ) u_dpw_top (
        .clk(clk),
        .reset(rst),
        .op_mode(op_mode),
        .streamer_en(emrs_cfg_reg[0]),
        .sample_rate_div(emrs_cfg_reg[12:1]),
        .emrs_boxcar_en(emrs_cfg_reg[13]),
        .emrs_boxcar_shift(emrs_cfg_reg[15:14]),
        .margin_config(ctrl_margin_reg),
        .uadt_noise_floor(uadt_cfg_reg[6:0]),
        .uadt_decay_shift(uadt_cfg_reg[9:7]),
        .uadt_peak_mode(uadt_cfg_reg[11:10]),
        .valid_in(eff_valid_in),
        .sample_in(eff_sample_in),
        .row_enable(ctrl_row_en_reg),
        .weight_load_en(weight_load_pulse),
        .weight_in(weight_data_reg),
        .weight_row_addr(weight_row_reg),
        .weight_col_addr(weight_col_reg),
        .mode_bwd(learn_ctrl_reg[0]),
        .bwd_submode(learn_ctrl_reg[1]),
        .adapt_en(learn_ctrl_reg[2]),
        .error_sign_vec(learn_ctrl_reg[6:3]),
        .error_valid_vec(learn_ctrl_reg[10:7]),
        .event_detected(raw_event),
        .current_prec_tag(current_prec_tag),
        .uadt_thresh(uadt_thresh),
        .stream_fifo_full(stream_fifo_full_sig),
        .stream_fifo_empty(),
        .inter_event_cycles(inter_event_cycles),
        .active_int8_cycles(active_int8_cycles),
        .psum_out(psum_out_raw),
        .psum_valid()
    );

    assign coproc_psum_out  = psum_out_raw;
    assign coproc_event_out = raw_event;
    assign coproc_prec_out  = current_prec_tag;

endmodule

module dpw_top #(
    parameter DATA_WIDTH  = 8,
    parameter ACCUM_WIDTH = 32,
    parameter ROWS        = 4,
    parameter COLS        = 4,
    parameter NOISE_FLOOR = 7'd32,
    parameter DECAY_SHIFT = 4,
    parameter PE_TYPE     = 1
)(
    input  wire                         clk,
    input  wire                         reset,

    input  wire [2:0]                   op_mode,

    input  wire                         streamer_en,
    input  wire [11:0]                  sample_rate_div,
    input  wire                         emrs_boxcar_en,
    input  wire [1:0]                   emrs_boxcar_shift,
    input  wire [7:0]                   margin_config,
    input  wire [6:0]                   uadt_noise_floor,
    input  wire [2:0]                   uadt_decay_shift,
    input  wire [1:0]                   uadt_peak_mode,

    input  wire                         valid_in,
    input  wire signed [DATA_WIDTH-1:0] sample_in,

    input  wire [ROWS-1:0]              row_enable,

    input  wire                         weight_load_en,
    input  wire [DATA_WIDTH-1:0]        weight_in,
    input  wire [1:0]                   weight_row_addr,
    input  wire [1:0]                   weight_col_addr,

    input  wire                         mode_bwd,
    input  wire                         bwd_submode,
    input  wire                         adapt_en,
    input  wire [COLS-1:0]              error_sign_vec,
    input  wire [COLS-1:0]              error_valid_vec,

    output wire                         event_detected,
    output wire [1:0]                   current_prec_tag,
    output wire [6:0]                   uadt_thresh,
    output wire                         stream_fifo_full,
    output wire                         stream_fifo_empty,
    output reg  [15:0]                  inter_event_cycles,
    output reg  [15:0]                  active_int8_cycles,

    output wire [(COLS*ACCUM_WIDTH)-1:0] psum_out,
    output wire [COLS-1:0]              psum_valid
);

    wire signed [DATA_WIDTH-1:0] emrs_sample_out;
    wire                         emrs_valid_out;

    emrs_streamer #(
        .DATA_WIDTH(DATA_WIDTH),
        .FIFO_DEPTH(8),
        .PTR_WIDTH(3),
        .DIV_WIDTH(16),
        .ACC_WIDTH(11)
    ) emrs_inst (
        .clk(clk),
        .rst(reset),
        .enable(streamer_en),
        .sensor_valid_in(valid_in),
        .sensor_data_in(sample_in),
        .sample_rate_div({4'b0000, sample_rate_div}),
        .boxcar_en(emrs_boxcar_en),
        .boxcar_shift(emrs_boxcar_shift),
        .burst_mode_en(1'b0),
        .watermark_cfg(4'd4),
        .downstream_ready(1'b1),
        .stream_sample_out(emrs_sample_out),
        .stream_valid_out(emrs_valid_out),
        .core_clk_en_hint(),
        .fifo_count(),
        .fifo_empty(stream_fifo_empty),
        .fifo_full(stream_fifo_full),
        .fifo_overflow_sticky(),
        .watermark_reached()
    );

    wire signed [DATA_WIDTH-1:0] eff_sample = streamer_en ? emrs_sample_out : sample_in;
    wire                         eff_valid  = streamer_en ? emrs_valid_out  : valid_in;

    wire       uadt_event;

    wire [1:0] uadt_tag;

    wire [6:0] uadt_thresh_val;

    uadt_detector #(
        .DATA_WIDTH(DATA_WIDTH),
        .MAG_WIDTH(7),
        .NOISE_FLOOR(NOISE_FLOOR),
        .DECAY_SHIFT(DECAY_SHIFT)
    ) uadt_inst (
        .clk(clk),
        .rst(reset),
        .valid_in(eff_valid),
        .sample_in(eff_sample),
        .cfg_noise_floor(uadt_noise_floor),
        .cfg_decay_shift(uadt_decay_shift),
        .cfg_peak_mode(uadt_peak_mode),
        .lockout_config(margin_config),
        .is_event(uadt_event),
        .prec_tag_out(uadt_tag),
        .thresh_reg(uadt_thresh_val)
    );

    assign event_detected = uadt_event;
    assign uadt_thresh    = uadt_thresh_val;

    wire prec_tag_stretched;

    event_scheduler #(
        .MAX_MARGIN(64)
    ) sched_inst (
        .clk(clk),
        .reset(reset),
        .event_trigger_in(uadt_event),
        .margin_config(margin_config),
        .prec_tag_out(prec_tag_stretched)
    );

    reg [1:0] sample_prec_tag;
    always @(*) begin
        case (op_mode)
            3'b000: sample_prec_tag = 2'b11;
            3'b001: sample_prec_tag = 2'b01;
            3'b010: sample_prec_tag = prec_tag_stretched ? 2'b11 : 2'b10;
            3'b011: sample_prec_tag = prec_tag_stretched ? 2'b11 : 2'b01;
            3'b100: sample_prec_tag = prec_tag_stretched ? 2'b11 : 2'b00;
            3'b101: sample_prec_tag = prec_tag_stretched ? 2'b11 : 2'b10;
            3'b110: sample_prec_tag = prec_tag_stretched ? 2'b11 : 2'b01;
            3'b111: sample_prec_tag = prec_tag_stretched ? 2'b11 : 2'b00;
            default: sample_prec_tag = 2'b11;
        endcase
    end

    reg [1:0] prec_tag_shift [0:ROWS-1];
    integer k;
    always @(posedge clk) begin
        if (reset) begin
            for (k = 0; k < ROWS; k = k + 1)
                prec_tag_shift[k] <= 2'b11;
        end else if (eff_valid) begin
            prec_tag_shift[0] <= sample_prec_tag;
            for (k = 1; k < ROWS; k = k + 1)
                prec_tag_shift[k] <= prec_tag_shift[k-1];
        end
    end

    reg [1:0] global_prec_tag;
    always @(*) begin
        case (op_mode)
            3'b010: global_prec_tag = prec_tag_stretched ? 2'b11 : 2'b10;
            3'b011: global_prec_tag = prec_tag_stretched ? 2'b11 : 2'b01;
            3'b100: global_prec_tag = prec_tag_stretched ? 2'b11 : 2'b00;
            default: global_prec_tag = sample_prec_tag;
        endcase
    end

    wire [(ROWS*2)-1:0] prec_tag_array;
    genvar r_idx;
    generate
        for (r_idx = 0; r_idx < ROWS; r_idx = r_idx + 1) begin : tag_mux
            assign prec_tag_array[r_idx*2 +: 2] = (op_mode >= 3'b101) ? prec_tag_shift[r_idx] :
                                                  ((op_mode == 3'b000) ? 2'b11 :
                                                  ((op_mode == 3'b001) ? 2'b01 : global_prec_tag));
        end
    endgenerate

    assign current_prec_tag = (op_mode >= 3'b101) ? prec_tag_shift[0] :
                              ((op_mode >= 3'b010) ? global_prec_tag : sample_prec_tag);

    reg [15:0] gap_counter;
    reg        prev_uadt_event;

    always @(posedge clk) begin
        if (reset) begin
            gap_counter        <= 16'd0;
            inter_event_cycles <= 16'd0;
            active_int8_cycles <= 16'd0;
            prev_uadt_event    <= 1'b0;
        end else if (eff_valid) begin
            prev_uadt_event <= uadt_event;

            if (sample_prec_tag == 2'b11)
                active_int8_cycles <= active_int8_cycles + 16'd1;

            if (uadt_event && !prev_uadt_event) begin
                inter_event_cycles <= gap_counter;
                gap_counter        <= 16'd1;
            end else if (gap_counter != 16'hFFFF) begin
                gap_counter        <= gap_counter + 16'd1;
            end
        end
    end

    wire [(ROWS*DATA_WIDTH)-1:0] data_window;
    wire                         window_valid;

    temporal_window_buffer #(
        .DATA_WIDTH(DATA_WIDTH),
        .WINDOW_SIZE(ROWS)
    ) buffer_inst (
        .clk(clk),
        .reset(reset),
        .sample_in(eff_sample),
        .valid_in(eff_valid),
        .window_out(data_window),
        .window_valid(window_valid)
    );

    wire [ROWS-1:0] array_valid = {ROWS{window_valid & eff_valid}};
    wire [ROWS-1:0] eff_row_enable = row_enable & {ROWS{eff_valid | mode_bwd}};

    reg [ROWS+COLS-1:0] valid_pipe;
    always @(posedge clk) begin
        if (reset) begin
            valid_pipe <= {ROWS+COLS{1'b0}};
        end else if (eff_valid) begin
            valid_pipe <= {valid_pipe[ROWS+COLS-2:0], (window_valid & eff_valid)};
        end
    end

    assign psum_valid[0] = valid_pipe[ROWS-1];
    assign psum_valid[1] = valid_pipe[ROWS];
    assign psum_valid[2] = valid_pipe[ROWS+1];
    assign psum_valid[3] = valid_pipe[ROWS+2];

    systolic_array #(
        .DATA_WIDTH(DATA_WIDTH),
        .ACCUM_WIDTH(ACCUM_WIDTH),
        .ARCH_MODE(3),
        .ROWS(ROWS),
        .COLS(COLS),
        .PE_TYPE(PE_TYPE)
    ) systolic_inst (
        .clk(clk),
        .reset(reset),
        .row_enable(eff_row_enable),
        .weight_load_en(weight_load_en),
        .weight_in(weight_in),
        .weight_row_addr({1'b0, weight_row_addr}),
        .weight_col_addr({1'b0, weight_col_addr}),
        .mode_bwd(mode_bwd),
        .bwd_submode(bwd_submode),
        .adapt_en(adapt_en),
        .error_sign_vec(error_sign_vec),
        .error_valid_vec(error_valid_vec),
        .data_in_valid(array_valid),
        .data_in(data_window),
        .prec_tag_in(prec_tag_array),
        .psum_out(psum_out)
    );

endmodule

module emrs_streamer #(
    parameter DATA_WIDTH = 8,
    parameter FIFO_DEPTH = 8,
    parameter PTR_WIDTH  = 3,
    parameter DIV_WIDTH  = 16,
    parameter ACC_WIDTH  = 11
)(
    input  wire                   clk,
    input  wire                   rst,

    input  wire                   enable,
    input  wire                   sensor_valid_in,
    input  wire signed [DATA_WIDTH-1:0] sensor_data_in,

    input  wire [DIV_WIDTH-1:0]   sample_rate_div,
    input  wire                   boxcar_en,
    input  wire [1:0]             boxcar_shift,
    input  wire                   burst_mode_en,
    input  wire [PTR_WIDTH:0]     watermark_cfg,

    input  wire                   downstream_ready,
    output reg  signed [DATA_WIDTH-1:0] stream_sample_out,
    output reg                    stream_valid_out,
    output wire                   core_clk_en_hint,

    output wire [PTR_WIDTH:0]     fifo_count,
    output wire                   fifo_empty,
    output wire                   fifo_full,
    output reg                    fifo_overflow_sticky,
    output wire                   watermark_reached
);

    reg  [DIV_WIDTH-1:0]        decim_cnt;
    reg                         win_first_sample;
    reg  signed [ACC_WIDTH-1:0] boxcar_acc;

    wire is_1x_mode     = (sample_rate_div == {DIV_WIDTH{1'b0}});
    wire decim_terminal = is_1x_mode | (~win_first_sample & (decim_cnt == {DIV_WIDTH{1'b0}}));
    wire decim_fire     = enable & sensor_valid_in & decim_terminal;

    wire signed [ACC_WIDTH-1:0] sample_ext  = {{(ACC_WIDTH-DATA_WIDTH){sensor_data_in[DATA_WIDTH-1]}}, sensor_data_in};
    wire signed [ACC_WIDTH-1:0] current_sum = win_first_sample ? sample_ext : (boxcar_acc + sample_ext);

    reg signed [ACC_WIDTH-1:0] shifted_sum;
    always @(*) begin
        case (boxcar_shift[1:0])
            2'd0: shifted_sum = current_sum;
            2'd1: shifted_sum = current_sum >>> 1;
            2'd2: shifted_sum = current_sum >>> 2;
            2'd3: shifted_sum = current_sum >>> 3;
        endcase
    end

    localparam signed [ACC_WIDTH-1:0] MAX_POS =  11'sd127;
    localparam signed [ACC_WIDTH-1:0] MIN_NEG = -11'sd128;

    wire signed [DATA_WIDTH-1:0] boxcar_saturated =
        (shifted_sum > MAX_POS) ? 8'sd127 :
        (shifted_sum < MIN_NEG) ? -8'sd128 :
                                  shifted_sum[DATA_WIDTH-1:0];

    wire signed [DATA_WIDTH-1:0] decimated_sample = boxcar_en ? boxcar_saturated : sensor_data_in;

    always @(posedge clk) begin
        if (rst || !enable) begin
            decim_cnt        <= {DIV_WIDTH{1'b0}};
            win_first_sample <= 1'b1;
            boxcar_acc       <= {ACC_WIDTH{1'b0}};
        end else if (sensor_valid_in) begin
            if (decim_terminal) begin
                win_first_sample <= 1'b1;
            end else begin
                if (win_first_sample) begin
                    decim_cnt        <= sample_rate_div - 1'b1;
                    win_first_sample <= 1'b0;
                end else begin
                    decim_cnt        <= decim_cnt - 1'b1;
                end
                if (boxcar_en) begin
                    boxcar_acc <= current_sum;
                end
            end
        end
    end

    reg signed [DATA_WIDTH-1:0] fifo_mem [0:FIFO_DEPTH-1];
    reg [PTR_WIDTH-1:0]         wr_ptr;
    reg [PTR_WIDTH-1:0]         rd_ptr;
    reg [PTR_WIDTH:0]           count_reg;
    reg                         burst_active;

    assign fifo_count        = count_reg;
    assign fifo_empty        = (count_reg == {(PTR_WIDTH+1){1'b0}});
    assign fifo_full         = count_reg[PTR_WIDTH];

    assign watermark_reached = (count_reg >= watermark_cfg) && (watermark_cfg != 0);

    wire can_drain = burst_mode_en ? (burst_active || watermark_reached) : (~fifo_empty);
    wire push_en   = decim_fire & (~fifo_full | (can_drain & downstream_ready));
    wire pop_en    = enable & can_drain & (~fifo_empty) & downstream_ready;

    assign core_clk_en_hint = enable & (can_drain | stream_valid_out);

    always @(posedge clk) begin
        if (rst) begin
            wr_ptr               <= {PTR_WIDTH{1'b0}};
            rd_ptr               <= {PTR_WIDTH{1'b0}};
            count_reg            <= {(PTR_WIDTH+1){1'b0}};
            burst_active         <= 1'b0;
            stream_sample_out    <= {DATA_WIDTH{1'b0}};
            stream_valid_out     <= 1'b0;
            fifo_overflow_sticky <= 1'b0;
        end else begin
            if (decim_fire && fifo_full && !pop_en) begin
                fifo_overflow_sticky <= 1'b1;
            end

            if (push_en) begin
                fifo_mem[wr_ptr] <= decimated_sample;
                wr_ptr           <= wr_ptr + 1'b1;
            end

            if (pop_en) begin
                stream_sample_out <= fifo_mem[rd_ptr];
                rd_ptr            <= rd_ptr + 1'b1;
                stream_valid_out  <= 1'b1;
            end else begin
                stream_valid_out  <= 1'b0;
            end

            case ({push_en, pop_en})
                2'b10:   count_reg <= count_reg + 1'b1;
                2'b01:   count_reg <= count_reg - 1'b1;
                default: count_reg <= count_reg;
            endcase

            if (!enable || !burst_mode_en) begin
                burst_active <= 1'b0;
            end else if (watermark_reached) begin
                burst_active <= 1'b1;
            end else if (pop_en && (count_reg == 4'd1) && !push_en) begin
                burst_active <= 1'b0;
            end
        end
    end

endmodule

module uadt_detector #(
    parameter DATA_WIDTH    = 8,
    parameter MAG_WIDTH     = 7,
    parameter NOISE_FLOOR   = 7'd32,
    parameter DECAY_SHIFT   = 4
)(
    input  wire                         clk,
    input  wire                         rst,
    input  wire                         valid_in,
    input  wire signed [DATA_WIDTH-1:0] sample_in,

    input  wire [MAG_WIDTH-1:0]         cfg_noise_floor,
    input  wire [2:0]                   cfg_decay_shift,
    input  wire [1:0]                   cfg_peak_mode,
    input  wire [7:0]                   lockout_config,

    output reg                          is_event,
    output reg  [1:0]                   prec_tag_out,
    output reg  [MAG_WIDTH-1:0]         thresh_reg
);

    wire [MAG_WIDTH-1:0] eff_floor = (cfg_noise_floor != {MAG_WIDTH{1'b0}}) ? cfg_noise_floor : NOISE_FLOOR;
    wire [2:0]           eff_shift = (cfg_decay_shift != 3'd0) ? cfg_decay_shift : DECAY_SHIFT;

    wire [MAG_WIDTH-1:0] abs_mag = sample_in[MAG_WIDTH-1:0] ^ {MAG_WIDTH{sample_in[DATA_WIDTH-1]}};

    reg [MAG_WIDTH-1:0] peak_jump;
    always @(*) begin
        case (cfg_peak_mode)
            2'b00:   peak_jump = (abs_mag >> 1);
            2'b01:   peak_jump = (abs_mag >> 1) + (abs_mag >> 2);
            2'b10:   peak_jump = (abs_mag >> 1) + (abs_mag >> 2) + (abs_mag >> 3);
            2'b11:   peak_jump = abs_mag;
            default: peak_jump = (abs_mag >> 1) + (abs_mag >> 2);
        endcase
    end

    wire [MAG_WIDTH-1:0] raw_decay  = thresh_reg >> eff_shift;
    wire [MAG_WIDTH-1:0] decay_step = (raw_decay == {MAG_WIDTH{1'b0}}) ? 7'd1 : raw_decay;
    wire [MAG_WIDTH:0]   decayed    = {1'b0, thresh_reg} - {1'b0, decay_step};

    wire                 above_thresh = (abs_mag > thresh_reg);
    wire                 at_floor     = (decayed[MAG_WIDTH] || (decayed[MAG_WIDTH-1:0] <= eff_floor));

    reg [7:0] lockout_cnt;

    always @(posedge clk) begin
        if (rst) begin
            thresh_reg   <= eff_floor;
            is_event     <= 1'b0;
            prec_tag_out <= 2'b00;
            lockout_cnt  <= 8'd0;
        end else if (valid_in) begin
            if (lockout_cnt > 8'd0) begin
                lockout_cnt <= lockout_cnt - 8'd1;
                is_event    <= 1'b0;

                if (peak_jump > thresh_reg) begin
                    thresh_reg <= peak_jump;
                end else if (!at_floor) begin
                    thresh_reg <= decayed[MAG_WIDTH-1:0];
                end
                prec_tag_out <= 2'b11;
            end else if (above_thresh) begin
                is_event     <= 1'b1;
                prec_tag_out <= 2'b11;
                thresh_reg   <= (peak_jump > eff_floor) ? peak_jump : eff_floor;
                lockout_cnt  <= (lockout_config > 8'd1) ? (lockout_config - 8'd1) : 8'd0;
            end else begin
                is_event     <= 1'b0;
                if (at_floor) begin
                    thresh_reg   <= eff_floor;
                    prec_tag_out <= 2'b00;
                end else begin
                    thresh_reg   <= decayed[MAG_WIDTH-1:0];
                    prec_tag_out <= 2'b01;
                end
            end
        end
    end

endmodule

module event_scheduler #(
    parameter MAX_MARGIN = 32
)(
    input wire clk,
    input wire reset,

    input wire event_trigger_in,

    input wire [7:0] margin_config,

    output reg prec_tag_out
);

    reg [7:0] active_counter;

    always @(posedge clk) begin
        if (reset) begin
            active_counter <= 8'd0;
            prec_tag_out <= 1'b0;
        end else begin
            if (event_trigger_in && (active_counter == 8'd0)) begin

                active_counter <= margin_config;
                prec_tag_out <= 1'b1;
            end else if (active_counter > 8'd0) begin

                active_counter <= active_counter - 8'd1;
                prec_tag_out <= 1'b1;
            end else begin

                prec_tag_out <= 1'b0;
            end
        end
    end

endmodule

module temporal_window_buffer #(
    parameter DATA_WIDTH = 8,
    parameter WINDOW_SIZE = 8
)(
    input wire clk,
    input wire reset,

    input wire [DATA_WIDTH-1:0] sample_in,
    input wire valid_in,

    output wire [(WINDOW_SIZE*DATA_WIDTH)-1:0] window_out,
    output reg window_valid
);

    reg [DATA_WIDTH-1:0] shift_reg [0:WINDOW_SIZE-1];
    reg [3:0] count;

    integer i;

    always @(posedge clk) begin
        if (reset) begin
            for (i = 0; i < WINDOW_SIZE; i = i + 1) begin
                shift_reg[i] <= 0;
            end
            count <= 0;
            window_valid <= 0;
        end else if (valid_in) begin

            for (i = WINDOW_SIZE-1; i > 0; i = i - 1) begin
                shift_reg[i] <= shift_reg[i-1];
            end

            shift_reg[0] <= sample_in;

            if (count < WINDOW_SIZE) begin
                count <= count + 1;
            end
            if (count >= WINDOW_SIZE - 1) begin
                window_valid <= 1;
            end
        end
    end

    genvar j;
    generate
        for (j = 0; j < WINDOW_SIZE; j = j + 1) begin : pack_window
            assign window_out[(j*DATA_WIDTH) +: DATA_WIDTH] = shift_reg[j];
        end
    endgenerate

endmodule

module systolic_array #(
    parameter DATA_WIDTH  = 8,
    parameter ACCUM_WIDTH = 32,
    parameter ROWS        = 4,
    parameter COLS        = 4,
    parameter ARCH_MODE   = 3,
    parameter PE_TYPE     = 1
)(
    input  wire                         clk,
    input  wire                         reset,

    input  wire [ROWS-1:0]              row_enable,

    input  wire                         weight_load_en,
    input  wire [DATA_WIDTH-1:0]        weight_in,
    input  wire [2:0]                   weight_row_addr,
    input  wire [2:0]                   weight_col_addr,

    input  wire                         mode_bwd,
    input  wire                         bwd_submode,
    input  wire                         adapt_en,
    input  wire [COLS-1:0]              error_sign_vec,
    input  wire [COLS-1:0]              error_valid_vec,

    input  wire [ROWS-1:0]              data_in_valid,
    input  wire [(ROWS*DATA_WIDTH)-1:0] data_in,
    input  wire [(ROWS*2)-1:0]          prec_tag_in,

    output wire [(COLS*ACCUM_WIDTH)-1:0] psum_out
);

    wire            safe_mode_bwd    = (mode_bwd    === 1'b1) ? 1'b1 : 1'b0;
    wire            safe_bwd_submode = (bwd_submode === 1'b1) ? 1'b1 : 1'b0;
    wire            safe_adapt_en    = (adapt_en    === 1'b1) ? 1'b1 : 1'b0;
    wire [COLS-1:0] safe_err_sign;
    wire [COLS-1:0] safe_err_valid;

    genvar gc;
    generate
        for (gc = 0; gc < COLS; gc = gc + 1) begin : gen_safe_bwd
            assign safe_err_sign[gc]  = (error_sign_vec[gc]  === 1'b1) ? 1'b1 : 1'b0;
            assign safe_err_valid[gc] = (error_valid_vec[gc] === 1'b1) ? 1'b1 : 1'b0;
        end
    endgenerate

    wire [DATA_WIDTH-1:0]  data_wire     [0:ROWS-1][0:COLS];
    wire [1:0]             prec_tag_wire [0:ROWS-1][0:COLS];
    wire [ACCUM_WIDTH-1:0] psum_wire     [0:ROWS][0:COLS-1];

    genvar r, c;
    generate
        for (r = 0; r < ROWS; r = r + 1) begin : row_in
            assign data_wire[r][0]     = data_in_valid[r] ? data_in[(r*DATA_WIDTH) +: DATA_WIDTH] : 0;
            assign prec_tag_wire[r][0] = prec_tag_in[r*2 +: 2];
        end
    endgenerate

    generate
        for (c = 0; c < COLS; c = c + 1) begin : col_in
            assign psum_wire[0][c] = 0;
            assign psum_out[(c*ACCUM_WIDTH) +: ACCUM_WIDTH] = psum_wire[ROWS][c];
        end
    endgenerate

    generate
        for (r = 0; r < ROWS; r = r + 1) begin : row
            for (c = 0; c < COLS; c = c + 1) begin : col

                wire [1:0] current_prec_tag;
`ifdef MODE_INT8
                assign current_prec_tag = 2'b11;
`elsif MODE_INT6
                assign current_prec_tag = 2'b10;
`elsif MODE_INT4
                assign current_prec_tag = 2'b01;
`elsif MODE_INT2
                assign current_prec_tag = 2'b00;
`else

                assign current_prec_tag = prec_tag_wire[r][c];
`endif

                wire weight_load = (weight_load_en && (weight_row_addr == r) && (weight_col_addr == c));

                if (PE_TYPE == 0) begin : gen_pe_base
                    pe #(
                        .DATA_WIDTH(DATA_WIDTH),
                        .ACCUM_WIDTH(ACCUM_WIDTH)
                    ) pe_inst (
                        .clk(clk),
                        .reset(reset),
                        .row_enable(row_enable[r]),
                        .weight_load(weight_load),
                        .weight_in(weight_in),
                        .data_in(data_wire[r][c]),
                        .prec_tag_in(current_prec_tag),
                        .psum_in(psum_wire[r][c]),
                        .data_out(data_wire[r][c+1]),
                        .prec_tag_out(prec_tag_wire[r][c+1]),
                        .psum_out(psum_wire[r+1][c])
                    );
`ifdef ENABLE_SOTA_DROPIN
                end else if (PE_TYPE == 2) begin : gen_pe_hybmed
                    pe_sota_hybmed_cdfa #(
                        .DATA_WIDTH(DATA_WIDTH),
                        .ACCUM_WIDTH(ACCUM_WIDTH)
                    ) pe_inst (
                        .clk(clk),
                        .reset(reset),
                        .row_enable(row_enable[r]),
                        .weight_load(weight_load),
                        .weight_in(weight_in),
                        .weight_out(),
                        .mode_bwd(safe_mode_bwd),
                        .bwd_submode(safe_bwd_submode),
                        .adapt_en(safe_adapt_en),
                        .error_sign_in(safe_err_sign[c]),
                        .error_valid_in(safe_err_valid[c]),
                        .bwd_act_in(data_in[(r*DATA_WIDTH) +: DATA_WIDTH]),
                        .data_in(data_wire[r][c]),
                        .prec_tag_in(current_prec_tag),
                        .psum_in(psum_wire[r][c]),
                        .data_out(data_wire[r][c+1]),
                        .prec_tag_out(prec_tag_wire[r][c+1]),
                        .psum_out(psum_wire[r+1][c])
                    );
                end else if (PE_TYPE == 3) begin : gen_pe_isscc25
                    pe_sota_isscc25_kws #(
                        .DATA_WIDTH(DATA_WIDTH),
                        .ACCUM_WIDTH(ACCUM_WIDTH)
                    ) pe_inst (
                        .clk(clk),
                        .reset(reset),
                        .row_enable(row_enable[r]),
                        .weight_load(weight_load),
                        .weight_in(weight_in),
                        .weight_out(),
                        .mode_bwd(safe_mode_bwd),
                        .bwd_submode(safe_bwd_submode),
                        .adapt_en(safe_adapt_en),
                        .error_sign_in(safe_err_sign[c]),
                        .error_valid_in(safe_err_valid[c]),
                        .bwd_act_in(data_in[(r*DATA_WIDTH) +: DATA_WIDTH]),
                        .data_in(data_wire[r][c]),
                        .prec_tag_in(current_prec_tag),
                        .psum_in(psum_wire[r][c]),
                        .data_out(data_wire[r][c+1]),
                        .prec_tag_out(prec_tag_wire[r][c+1]),
                        .psum_out(psum_wire[r+1][c])
                    );
                end else if (PE_TYPE == 4) begin : gen_pe_bols
                    pe_sota_bols_dftt #(
                        .DATA_WIDTH(DATA_WIDTH),
                        .ACCUM_WIDTH(ACCUM_WIDTH)
                    ) pe_inst (
                        .clk(clk),
                        .reset(reset),
                        .row_enable(row_enable[r]),
                        .weight_load(weight_load),
                        .weight_in(weight_in),
                        .weight_out(),
                        .mode_bwd(safe_mode_bwd),
                        .bwd_submode(safe_bwd_submode),
                        .adapt_en(safe_adapt_en),
                        .error_sign_in(safe_err_sign[c]),
                        .error_valid_in(safe_err_valid[c]),
                        .bwd_act_in(data_in[(r*DATA_WIDTH) +: DATA_WIDTH]),
                        .data_in(data_wire[r][c]),
                        .prec_tag_in(current_prec_tag),
                        .psum_in(psum_wire[r][c]),
                        .data_out(data_wire[r][c+1]),
                        .prec_tag_out(prec_tag_wire[r][c+1]),
                        .psum_out(psum_wire[r+1][c])
                    );
                end else if (PE_TYPE == 5) begin : gen_pe_fsl_hdnn
                    pe_sota_fsl_hdnn #(
                        .DATA_WIDTH(DATA_WIDTH),
                        .ACCUM_WIDTH(ACCUM_WIDTH)
                    ) pe_inst (
                        .clk(clk),
                        .reset(reset),
                        .row_enable(row_enable[r]),
                        .weight_load(weight_load),
                        .weight_in(weight_in),
                        .weight_out(),
                        .mode_bwd(safe_mode_bwd),
                        .bwd_submode(safe_bwd_submode),
                        .adapt_en(safe_adapt_en),
                        .error_sign_in(safe_err_sign[c]),
                        .error_valid_in(safe_err_valid[c]),
                        .bwd_act_in(data_in[(r*DATA_WIDTH) +: DATA_WIDTH]),
                        .data_in(data_wire[r][c]),
                        .prec_tag_in(current_prec_tag),
                        .psum_in(psum_wire[r][c]),
                        .data_out(data_wire[r][c+1]),
                        .prec_tag_out(prec_tag_wire[r][c+1]),
                        .psum_out(psum_wire[r+1][c])
                    );
                end else if (PE_TYPE == 6) begin : gen_pe_hyblearn
                    pe_sota_hyblearn #(
                        .DATA_WIDTH(DATA_WIDTH),
                        .ACCUM_WIDTH(ACCUM_WIDTH)
                    ) pe_inst (
                        .clk(clk),
                        .reset(reset),
                        .row_enable(row_enable[r]),
                        .weight_load(weight_load),
                        .weight_in(weight_in),
                        .weight_out(),
                        .mode_bwd(safe_mode_bwd),
                        .bwd_submode(safe_bwd_submode),
                        .adapt_en(safe_adapt_en),
                        .error_sign_in(safe_err_sign[c]),
                        .error_valid_in(safe_err_valid[c]),
                        .bwd_act_in(data_in[(r*DATA_WIDTH) +: DATA_WIDTH]),
                        .data_in(data_wire[r][c]),
                        .prec_tag_in(current_prec_tag),
                        .psum_in(psum_wire[r][c]),
                        .data_out(data_wire[r][c+1]),
                        .prec_tag_out(prec_tag_wire[r][c+1]),
                        .psum_out(psum_wire[r+1][c])
                    );
`endif
                end else begin : gen_pe_dual

                    pe_dual_mode #(
                        .DATA_WIDTH(DATA_WIDTH),
                        .ACCUM_WIDTH(ACCUM_WIDTH)
                    ) pe_inst (
                        .clk(clk),
                        .reset(reset),
                        .row_enable(row_enable[r]),
                        .weight_load(weight_load),
                        .weight_in(weight_in),
                        .weight_out(),
                        .mode_bwd(safe_mode_bwd),
                        .bwd_submode(safe_bwd_submode),
                        .adapt_en(safe_adapt_en),
                        .error_sign_in(safe_err_sign[c]),
                        .error_valid_in(safe_err_valid[c]),
                        .bwd_act_in(data_in[(r*DATA_WIDTH) +: DATA_WIDTH]),
                        .data_in(data_wire[r][c]),
                        .prec_tag_in(current_prec_tag),
                        .psum_in(psum_wire[r][c]),
                        .data_out(data_wire[r][c+1]),
                        .prec_tag_out(prec_tag_wire[r][c+1]),
                        .psum_out(psum_wire[r+1][c])
                    );

                end
            end
        end
    endgenerate

endmodule

module pe_dual_mode #(
    parameter DATA_WIDTH  = 8,
    parameter ACCUM_WIDTH = 32
)(
    input  wire                         clk,
    input  wire                         reset,

    input  wire                         row_enable,

    input  wire                         weight_load,
    input  wire signed [DATA_WIDTH-1:0] weight_in,
    output wire signed [DATA_WIDTH-1:0] weight_out,

    input  wire                         mode_bwd,
    input  wire                         bwd_submode,
    input  wire                         adapt_en,
    input  wire                         error_sign_in,
    input  wire                         error_valid_in,
    input  wire signed [DATA_WIDTH-1:0] bwd_act_in,

    input  wire signed [DATA_WIDTH-1:0] data_in,
    input  wire [1:0]                   prec_tag_in,
    input  wire signed [ACCUM_WIDTH-1:0] psum_in,

    output reg  signed [DATA_WIDTH-1:0] data_out,
    output reg  [1:0]                   prec_tag_out,
    output reg  signed [ACCUM_WIDTH-1:0] psum_out
);

    reg signed [DATA_WIDTH-1:0] weight_reg;
    assign weight_out = weight_reg;

    wire sign_sgd_dec = (error_sign_in == bwd_act_in[DATA_WIDTH-1]);

    wire proto_dec    = ($signed(bwd_act_in[DATA_WIDTH-1:4]) < $signed(weight_reg[DATA_WIDTH-1:4]));

    wire dec_enable   = bwd_submode ? sign_sgd_dec : proto_dec;

    wire [DATA_WIDTH-1:0] w_folded = weight_reg ^ {DATA_WIDTH{dec_enable}};
    wire is_saturated = (~w_folded[DATA_WIDTH-1]) & (&w_folded[DATA_WIDTH-2:0]);
    wire bwd_step_en  = mode_bwd & adapt_en & error_valid_in & (~is_saturated);

    always @(posedge clk) begin
        if (reset) begin
            weight_reg <= {DATA_WIDTH{1'b0}};
        end else if (weight_load) begin
            weight_reg <= weight_in;
        end else if (bwd_step_en) begin
            weight_reg <= weight_reg + {{(DATA_WIDTH-1){dec_enable}}, 1'b1};
        end
    end

    wire [DATA_WIDTH-1:0] data_gated;
    assign data_gated = (prec_tag_in == 2'b11) ? data_in :
                        (prec_tag_in == 2'b10) ? {data_in[7:2], 2'b00} :
                        (prec_tag_in == 2'b01) ? {data_in[7:4], 4'b0000} :
                                                 {data_in[7:6], 6'b000000};

    wire [DATA_WIDTH-1:0] weight_gated;
    assign weight_gated = (prec_tag_in == 2'b11) ? weight_reg :
                          (prec_tag_in == 2'b10) ? {weight_reg[7:2], 2'b00} :
                          (prec_tag_in == 2'b01) ? {weight_reg[7:4], 4'b0000} :
                                                   {weight_reg[7:6], 6'b000000};

    wire signed [DATA_WIDTH-1:0] signed_data   = data_gated;
    wire signed [DATA_WIDTH-1:0] signed_weight = weight_gated;

    (* use_dsp = "no" *) wire signed [ACCUM_WIDTH-1:0] product;
    assign product = signed_data * signed_weight;

    wire signed [ACCUM_WIDTH-1:0] next_psum;
    assign next_psum = $signed(psum_in) + product;

    wire fwd_row_en = row_enable & (~mode_bwd);

    always @(posedge clk) begin
        if (reset) begin
            data_out     <= 0;
            prec_tag_out <= 0;
            psum_out     <= 0;
        end else if (fwd_row_en) begin

            data_out[7:6]   <= data_in[7:6];
            prec_tag_out    <= prec_tag_in;
            psum_out[31:12] <= next_psum[31:12];

            if (prec_tag_in >= 2'b01) begin
                data_out[5:4]  <= data_in[5:4];
                psum_out[11:8] <= next_psum[11:8];
            end

            if (prec_tag_in >= 2'b10) begin
                data_out[3:2] <= data_in[3:2];
                psum_out[7:4] <= next_psum[7:4];
            end

            if (prec_tag_in == 2'b11) begin
                data_out[1:0] <= data_in[1:0];
                psum_out[3:0] <= next_psum[3:0];
            end
        end
    end

endmodule

module pe #(
    parameter DATA_WIDTH = 8,
    parameter ACCUM_WIDTH = 32
)(
    input wire clk,
    input wire reset,

    input wire row_enable,

    input wire weight_load,
    input wire [DATA_WIDTH-1:0] weight_in,

    input wire [DATA_WIDTH-1:0] data_in,
    input wire [1:0] prec_tag_in,
    input wire [ACCUM_WIDTH-1:0] psum_in,

    output reg [DATA_WIDTH-1:0] data_out,
    output reg [1:0] prec_tag_out,
    output reg [ACCUM_WIDTH-1:0] psum_out
);

    reg [DATA_WIDTH-1:0] weight_reg;
    always @(posedge clk) begin
        if (reset)
            weight_reg <= 0;
        else if (weight_load)
            weight_reg <= weight_in;
    end

    wire [DATA_WIDTH-1:0] data_gated;
    assign data_gated = (prec_tag_in == 2'b11) ? data_in :
                        (prec_tag_in == 2'b10) ? {data_in[7:2], 2'b00} :
                        (prec_tag_in == 2'b01) ? {data_in[7:4], 4'b0000} :
                                                 {data_in[7:6], 6'b000000};

    wire [DATA_WIDTH-1:0] weight_gated;
    assign weight_gated = (prec_tag_in == 2'b11) ? weight_reg :
                          (prec_tag_in == 2'b10) ? {weight_reg[7:2], 2'b00} :
                          (prec_tag_in == 2'b01) ? {weight_reg[7:4], 4'b0000} :
                                                   {weight_reg[7:6], 6'b000000};

    wire signed [DATA_WIDTH-1:0] signed_data = data_gated;
    wire signed [DATA_WIDTH-1:0] signed_weight = weight_gated;

    (* use_dsp = "no" *) wire signed [ACCUM_WIDTH-1:0] product;
    assign product = signed_data * signed_weight;

    wire signed [ACCUM_WIDTH-1:0] next_psum;
    assign next_psum = $signed(psum_in) + product;

    always @(posedge clk) begin
        if (reset) begin
            data_out <= 0;
            prec_tag_out <= 0;
            psum_out <= 0;
        end else if (row_enable) begin

            data_out[7:6] <= data_in[7:6];
            prec_tag_out <= prec_tag_in;
            psum_out[31:12] <= next_psum[31:12];

            if (prec_tag_in >= 2'b01) begin
                data_out[5:4] <= data_in[5:4];
                psum_out[11:8] <= next_psum[11:8];
            end

            if (prec_tag_in >= 2'b10) begin
                data_out[3:2] <= data_in[3:2];
                psum_out[7:4] <= next_psum[7:4];
            end

            if (prec_tag_in == 2'b11) begin
                data_out[1:0] <= data_in[1:0];
                psum_out[3:0] <= next_psum[3:0];
            end
        end
    end

endmodule

