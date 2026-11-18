`timescale 1ns / 1ps

module baseline_static_core #(
    parameter DATA_WIDTH  = 8,
    parameter ACCUM_WIDTH = 32,
    parameter ROWS        = 4,
    parameter COLS        = 4,
    parameter NOISE_FLOOR = 7'd32,
    parameter DECAY_SHIFT = 4,
    parameter PE_TYPE     = 0
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

