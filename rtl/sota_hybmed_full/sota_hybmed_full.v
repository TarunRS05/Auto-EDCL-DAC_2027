`timescale 1ns / 1ps

module sota_hybmed_full #(
    parameter DATA_WIDTH   = 8,
    parameter NUM_PES      = 32,
    parameter CUS_PER_PE   = 4,
    parameter SRAM_DEPTH   = 256
)(
    input  wire                         clk,
    input  wire                         reset,

    input  wire                         start_cmd,
    input  wire [2:0]                   exec_mode,
    input  wire [63:0]                  prng_seed,
    output wire                         busy_status,
    output wire                         done_irq,

    input  wire                         in_valid,
    input  wire signed [DATA_WIDTH-1:0] in_data,
    output wire                         in_ready,

    input  wire [4:0]                   pe_sel_idx,
    output reg                          out_valid,
    output reg  signed [15:0]           out_data,
    output wire [NUM_PES-1:0]           pe_active_flags,
    output wire [15:0]                  array_reduction_sig
);

    wire rst_n = ~reset;
    wire [63:0] prng_rand_out;
    wire        prng_step;

    wire [(NUM_PES*DATA_WIDTH)-1:0] pe_inputs;
    wire [(NUM_PES*16)-1:0]         pe_accum_out;
    wire                             pe_accum_valid;

    wire [(NUM_PES*16)-1:0]         dfa_err_proj;
    wire               dfa_update_valid;

    wire pe_fwd_enable;
    wire dfa_enable;
    wire pingpong_swap;
    wire sram_wr_en;

    hybmed_fsm_ctrl u_fsm (
        .clk           (clk),
        .rst_n         (rst_n),
        .start_cmd     (start_cmd),
        .exec_mode     (exec_mode),
        .pe_done       (pe_accum_valid),
        .dfa_done      (dfa_update_valid),
        .pe_fwd_enable (pe_fwd_enable),
        .dfa_enable    (dfa_enable),
        .prng_step     (prng_step),
        .pingpong_swap (pingpong_swap),
        .sram_wr_en    (sram_wr_en),
        .in_ready      (in_ready),
        .busy_status   (busy_status),
        .done_irq      (done_irq)
    );

    hybmed_pingpong_sram #(
        .DATA_WIDTH (DATA_WIDTH),
        .NUM_PES    (NUM_PES),
        .SRAM_DEPTH (SRAM_DEPTH)
    ) u_sram_subsystem (
        .clk          (clk),
        .rst_n        (rst_n),
        .swap_bank    (pingpong_swap),
        .wr_en        (sram_wr_en),
        .in_valid     (in_valid),
        .in_data      (in_data),
        .pe_inputs_out(pe_inputs)
    );

    hybmed_pe_array_32 #(
        .DATA_WIDTH (DATA_WIDTH),
        .NUM_PES    (NUM_PES),
        .CUS_PER_PE (CUS_PER_PE)
    ) u_pe_array (
        .clk          (clk),
        .rst_n        (rst_n),
        .enable       (pe_fwd_enable),
        .inputs       (pe_inputs),
        .accum_valid  (pe_accum_valid),
        .accum_out    (pe_accum_out),
        .active_flags (pe_active_flags)
    );

    hybmed_xoroshiro_prng u_prng (
        .clk       (clk),
        .rst_n     (rst_n),
        .seed_load (start_cmd),
        .seed_in   (prng_seed),
        .step      (prng_step),
        .rand_out  (prng_rand_out)
    );

    hybmed_dfa_engine #(
        .NUM_PES (NUM_PES)
    ) u_dfa_engine (
        .clk            (clk),
        .rst_n          (rst_n),
        .enable         (dfa_enable),
        .rand_matrix_in (prng_rand_out),
        .pe_activations (pe_accum_out),
        .err_projected  (dfa_err_proj),
        .update_valid   (dfa_update_valid)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            out_valid <= 1'b0;
            out_data  <= 16'sh0;
        end else if (pe_accum_valid) begin
            out_valid <= 1'b1;
            out_data  <= pe_accum_out[pe_sel_idx*16 +: 16];
        end else begin
            out_valid <= 1'b0;
        end
    end

    reg [15:0] red_accum;
    integer p;
    always @(*) begin
        red_accum = 16'h0;
        for (p = 0; p < NUM_PES; p = p + 1) begin
            red_accum = red_accum ^ pe_accum_out[p*16 +: 16] ^ dfa_err_proj[p*16 +: 16];
        end
    end
    assign array_reduction_sig = red_accum;

endmodule

module hybmed_fsm_ctrl (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start_cmd,
    input  wire [2:0]  exec_mode,
    input  wire        pe_done,
    input  wire        dfa_done,
    output reg         pe_fwd_enable,
    output reg         dfa_enable,
    output reg         prng_step,
    output reg         pingpong_swap,
    output reg         sram_wr_en,
    output reg         in_ready,
    output reg         busy_status,
    output reg         done_irq
);
    localparam IDLE         = 3'd0;
    localparam BUFFER_LOAD  = 3'd1;
    localparam PE_FORWARD   = 3'd2;
    localparam DFA_GEN_B    = 3'd3;
    localparam DFA_UPDATE   = 3'd4;
    localparam FINISH       = 3'd5;

    reg [2:0] state, next_state;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            state <= IDLE;
        else
            state <= next_state;
    end

    always @(*) begin
        next_state    = state;
        pe_fwd_enable = 1'b0;
        dfa_enable    = 1'b0;
        prng_step     = 1'b0;
        pingpong_swap = 1'b0;
        sram_wr_en    = 1'b0;
        in_ready      = 1'b0;
        busy_status   = 1'b1;
        done_irq      = 1'b0;

        case (state)
            IDLE: begin
                busy_status = 1'b0;
                if (start_cmd)
                    next_state = BUFFER_LOAD;
            end

            BUFFER_LOAD: begin
                in_ready   = 1'b1;
                sram_wr_en = 1'b1;
                next_state = PE_FORWARD;
            end

            PE_FORWARD: begin
                pe_fwd_enable = 1'b1;
                if (pe_done) begin
                    if (exec_mode == 3'd3)
                        next_state = DFA_GEN_B;
                    else
                        next_state = FINISH;
                end
            end

            DFA_GEN_B: begin
                prng_step  = 1'b1;
                next_state = DFA_UPDATE;
            end

            DFA_UPDATE: begin
                dfa_enable = 1'b1;
                if (dfa_done) begin
                    pingpong_swap = 1'b1;
                    next_state    = FINISH;
                end
            end

            FINISH: begin
                busy_status = 1'b0;
                done_irq    = 1'b1;
                next_state  = IDLE;
            end
        endcase
    end
endmodule

module hybmed_pe_array_32 #(
    parameter DATA_WIDTH=8, NUM_PES=32, CUS_PER_PE=4
)(
    input wire clk, rst_n, enable,
    input wire [NUM_PES*DATA_WIDTH-1:0] inputs,
    output reg accum_valid,
    output reg [NUM_PES*16-1:0] accum_out,
    output wire [NUM_PES-1:0] active_flags
);
    reg [5:0] group_idx;
    reg completed;
    assign active_flags = {NUM_PES{enable}};
    genvar lane;
    generate
        for (lane=0; lane<4; lane=lane+1) begin : gen_lane
            wire signed [7:0] W0;
            wire signed [7:0] W1;
            wire signed [7:0] W2;
            wire signed [7:0] W3;
            assign W0 = (lane%4==0) ? 8'sd14 : -8'sd3;
            assign W1 = (lane%4==1) ? 8'sd14 : -8'sd3;
            assign W2 = (lane%4==2) ? 8'sd14 : -8'sd3;
            assign W3 = (lane%4==3) ? 8'sd14 : -8'sd3;
            wire [7:0] pe_index;
            wire signed [19:0] mac;
            wire signed [19:0] accumulated;
            assign pe_index = group_idx*4 + lane;
            assign mac =
                inputs[((pe_index+0)%NUM_PES)*DATA_WIDTH +: DATA_WIDTH]*W0 +
                inputs[((pe_index+1)%NUM_PES)*DATA_WIDTH +: DATA_WIDTH]*W1 +
                inputs[((pe_index+2)%NUM_PES)*DATA_WIDTH +: DATA_WIDTH]*W2 +
                inputs[((pe_index+3)%NUM_PES)*DATA_WIDTH +: DATA_WIDTH]*W3;
            assign accumulated = mac + mac + mac;
        end
    endgenerate

    genvar pe;
    generate
        for (pe=0; pe<NUM_PES; pe=pe+1) begin : gen_result
            wire signed [19:0] value = gen_lane[pe%4].accumulated;
            always @(posedge clk or negedge rst_n) begin
                if (!rst_n) accum_out[pe*16 +: 16]<=0;
                else if (enable && !completed && group_idx==pe/4)
                    accum_out[pe*16 +: 16] <= value>0 ? value[17:2] : 16'd0;
            end
        end
    endgenerate
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin group_idx<=0; completed<=0; accum_valid<=0; end
        else begin
            accum_valid<=0;
            if (!enable) begin group_idx<=0; completed<=0; end
            else if (!completed) begin
                if (group_idx==(NUM_PES+3)/4-1) begin
                    completed<=1; accum_valid<=1;
                end else group_idx<=group_idx+1'b1;
            end
        end
    end
endmodule

module hybmed_xoroshiro_prng (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        seed_load,
    input  wire [63:0] seed_in,
    input  wire        step,
    output wire [63:0] rand_out
);
    reg [63:0] s0, s1;

    function automatic [63:0] rotl;
        input [63:0] x;
        input [5:0]  k;
        begin
            rotl = (x << k) | (x >> (64 - k));
        end
    endfunction

    assign rand_out = s0 + s1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s0 <= 64'h19876543210ABCDE;
            s1 <= 64'hFEDCBA0987654321;
        end else if (seed_load) begin
            s0 <= seed_in;
            s1 <= seed_in ^ 64'h5555AAAA5555AAAA;
        end else if (step) begin
            s1 <= s0 ^ s1;
            s0 <= rotl(s0, 24) ^ (s1 ^ (s1 << 16));
        end
    end
endmodule

module hybmed_dfa_engine #(parameter NUM_PES=32)(
    input wire clk, rst_n, enable,
    input wire [63:0] rand_matrix_in,
    input wire [NUM_PES*16-1:0] pe_activations,
    output reg [NUM_PES*16-1:0] err_projected,
    output reg update_valid
);
    reg [5:0] group_idx;
    reg completed;
    genvar lane, pe;
    generate
        for (lane=0; lane<4; lane=lane+1) begin : gen_lane
            wire [7:0] pe_index=group_idx*4+lane;
            wire [31:0] product = $signed(rand_matrix_in[15:0]) *
                                 pe_activations[pe_index*16 +: 16];
            wire [15:0] projection=product[23:8];
        end
        for (pe=0; pe<NUM_PES; pe=pe+1) begin : gen_result
            always @(posedge clk or negedge rst_n) begin
                if (!rst_n) err_projected[pe*16 +: 16]<=0;
                else if (enable && !completed && group_idx==pe/4)
                    err_projected[pe*16 +: 16]<=gen_lane[pe%4].projection;
            end
        end
    endgenerate
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin group_idx<=0; completed<=0; update_valid<=0; end
        else begin
            update_valid<=0;
            if (!enable) begin group_idx<=0; completed<=0; end
            else if (!completed) begin
                if (group_idx==(NUM_PES+3)/4-1) begin
                    completed<=1; update_valid<=1;
                end else group_idx<=group_idx+1'b1;
            end
        end
    end
endmodule

module hybmed_pingpong_sram #(
    parameter DATA_WIDTH = 8,
    parameter NUM_PES    = 32,
    parameter SRAM_DEPTH = 256
)(
    input  wire                         clk,
    input  wire                         rst_n,
    input  wire                         swap_bank,
    input  wire                         wr_en,
    input  wire                         in_valid,
    input  wire signed [DATA_WIDTH-1:0] in_data,
    output reg  [(NUM_PES*DATA_WIDTH)-1:0] pe_inputs_out
);
    reg active_bank;
    reg [7:0] wr_addr;

    (* ram_style = "distributed" *)
    reg signed [DATA_WIDTH-1:0] bank0 [0:SRAM_DEPTH-1];
    (* ram_style = "distributed" *)
    reg signed [DATA_WIDTH-1:0] bank1 [0:SRAM_DEPTH-1];

    integer p;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (p=0; p<SRAM_DEPTH; p=p+1) begin
                bank0[p]<=0; bank1[p]<=0;
            end
            active_bank <= 1'b0;
            wr_addr     <= 8'd0;
            pe_inputs_out <= {(NUM_PES*DATA_WIDTH){1'b0}};
        end else begin
            if (swap_bank)
                active_bank <= ~active_bank;

            if (wr_en && in_valid) begin
                if (!active_bank)
                    bank0[wr_addr] <= in_data;
                else
                    bank1[wr_addr] <= in_data;
                wr_addr <= wr_addr + 1'b1;
            end

            for (p = 0; p < NUM_PES; p = p + 1) begin
                if (!active_bank)
                    pe_inputs_out[p*DATA_WIDTH +: DATA_WIDTH] <= bank1[p];
                else
                    pe_inputs_out[p*DATA_WIDTH +: DATA_WIDTH] <= bank0[p];
            end
        end
    end
endmodule

