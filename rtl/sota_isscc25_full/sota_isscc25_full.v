`timescale 1ns / 1ps

module sota_isscc25_full #(
    parameter DATA_WIDTH     = 8,
    parameter WEIGHT_WIDTH   = 4,
    parameter FC_WIDTH       = 8,
    parameter CHANNELS       = 16,
    parameter NUM_CLASSES    = 35,
    parameter FEAT_DIM       = 16
)(
    input  wire                         clk,
    input  wire                         reset,

    input  wire                         start_cmd,
    input  wire [2:0]                   op_mode,
    input  wire [5:0]                   target_class,
    output wire                         busy_status,
    output wire                         done_irq,

    input  wire                         sample_valid,
    input  wire signed [DATA_WIDTH-1:0] sample_data,
    output wire                         sample_ready,

    output reg  [5:0]                   pred_class,
    output reg  signed [15:0]           pred_score,
    output wire [NUM_CLASSES-1:0]       class_match_flags,
    output wire signed [15:0]           serialized_logit_out
);

    wire rst_n = ~reset;
    wire fc_memory_ready;
    wire sequencer_busy;
    assign busy_status = !fc_memory_ready || sequencer_busy;
    wire [(CHANNELS*8)-1:0]  sinc_feat_out;
    wire                    sinc_feat_valid;

    wire [(FEAT_DIM*16)-1:0] conv_feat_out;
    wire               conv_feat_valid;

    wire signed [15:0] current_logit;
    wire [5:0]         current_class_idx;
    wire               fc_logit_valid;
    wire               fc_done_strobe;
    wire               fc_train_done;

    wire sinc_enable;
    wire conv_enable;
    wire fc_fwd_enable;
    wire fc_train_enable;
    wire enroll_enable;

    isscc25_micro_sequencer u_sequencer (
        .clk             (clk),
        .rst_n           (rst_n),
        .start_cmd       (start_cmd && fc_memory_ready),
        .op_mode         (op_mode),
        .sample_valid    (sample_valid),
        .sinc_done       (sinc_feat_valid),
        .conv_done       (conv_feat_valid),
        .fc_done         (fc_done_strobe),
        .train_done      (fc_train_done),
        .sinc_enable     (sinc_enable),
        .conv_enable     (conv_enable),
        .fc_fwd_enable   (fc_fwd_enable),
        .fc_train_enable (fc_train_enable),
        .enroll_enable   (enroll_enable),
        .sample_ready    (sample_ready),
        .busy_status     (sequencer_busy),
        .done_irq        (done_irq)
    );

    isscc25_sinc_fe #(
        .DATA_WIDTH (DATA_WIDTH),
        .CHANNELS   (CHANNELS)
    ) u_sinc_fe (
        .clk          (clk),
        .rst_n        (rst_n),
        .enable       (sinc_enable),
        .sample_valid (sample_valid),
        .sample_in    (sample_data),
        .feat_valid   (sinc_feat_valid),
        .feat_out     (sinc_feat_out)
    );

    isscc25_conv_array #(
        .DATA_WIDTH   (DATA_WIDTH),
        .WEIGHT_WIDTH (WEIGHT_WIDTH),
        .CHANNELS     (CHANNELS),
        .FEAT_DIM     (FEAT_DIM)
    ) u_conv_engine (
        .clk          (clk),
        .rst_n        (rst_n),
        .enable       (conv_enable),
        .feat_in_valid(sinc_feat_valid),
        .feat_in      (sinc_feat_out),
        .conv_valid   (conv_feat_valid),
        .conv_out     (conv_feat_out)
    );

    isscc25_fc_train_engine #(
        .FEAT_DIM    (FEAT_DIM),
        .NUM_CLASSES (NUM_CLASSES),
        .FC_WIDTH    (FC_WIDTH),

        .PROTO_WIDTH ((DATA_WIDTH == 8 && WEIGHT_WIDTH >= 3) ? 9 : 16)
    ) u_fc_engine (
        .clk              (clk),
        .rst_n            (rst_n),
        .fwd_enable       (fc_fwd_enable),
        .train_enable     (fc_train_enable),
        .enroll_enable    (enroll_enable),
        .target_class     (target_class),
        .feat_in_valid    (conv_feat_valid),
        .feat_in          (conv_feat_out),
        .logit_valid      (fc_logit_valid),
        .current_class_idx(current_class_idx),
        .current_logit    (current_logit),
        .fwd_done         (fc_done_strobe),
        .train_done       (fc_train_done),
        .memory_ready     (fc_memory_ready)
    );

    assign serialized_logit_out = current_logit;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pred_class <= 6'd0;
            pred_score <= -16'sh7FFF;
        end else if (fc_fwd_enable && conv_feat_valid) begin

            pred_score <= -16'sh7FFF;
            pred_class <= 6'd0;
        end else if (fc_logit_valid) begin

            if (current_logit > pred_score) begin
                pred_score <= current_logit;
                pred_class <= current_class_idx;
            end
        end
    end

    genvar g;
    generate
        for (g = 0; g < NUM_CLASSES; g = g + 1) begin : gen_match
            assign class_match_flags[g] = (pred_class == g[5:0]) && done_irq;
        end
    endgenerate

endmodule

module isscc25_micro_sequencer (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start_cmd,
    input  wire [2:0]  op_mode,
    input  wire        sample_valid,
    input  wire        sinc_done,
    input  wire        conv_done,
    input  wire        fc_done,
    input  wire        train_done,
    output reg         sinc_enable,
    output reg         conv_enable,
    output reg         fc_fwd_enable,
    output reg         fc_train_enable,
    output reg         enroll_enable,
    output reg         sample_ready,
    output reg         busy_status,
    output reg         done_irq
);
    localparam IDLE         = 3'd0;
    localparam RUN_SINC     = 3'd1;
    localparam RUN_CONV     = 3'd2;
    localparam RUN_FC_FWD   = 3'd3;
    localparam RUN_ENROLL   = 3'd4;
    localparam RUN_FC_TRAIN = 3'd5;
    localparam FINISH       = 3'd6;

    reg [2:0] state, next_state;
    reg [7:0] sample_counter;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state          <= IDLE;
            sample_counter <= 8'd0;
        end else begin
            state <= next_state;
            if (state == RUN_SINC && sample_valid)
                sample_counter <= sample_counter + 1'b1;
            else if (state == IDLE)
                sample_counter <= 8'd0;
        end
    end

    always @(*) begin
        next_state      = state;
        sinc_enable     = 1'b0;
        conv_enable     = 1'b0;
        fc_fwd_enable   = 1'b0;
        fc_train_enable = 1'b0;
        enroll_enable   = 1'b0;
        sample_ready    = 1'b0;
        busy_status     = 1'b1;
        done_irq        = 1'b0;

        case (state)
            IDLE: begin
                busy_status = 1'b0;
                if (start_cmd) begin
                    case (op_mode)
                        3'd1: next_state = RUN_SINC;
                        3'd2: next_state = RUN_ENROLL;
                        3'd3: next_state = RUN_FC_TRAIN;
                        default: next_state = IDLE;
                    endcase
                end
            end

            RUN_SINC: begin
                sinc_enable  = 1'b1;
                sample_ready = 1'b1;
                if (sinc_done || sample_counter >= 8'd128)
                    next_state = RUN_CONV;
            end

            RUN_CONV: begin
                conv_enable = 1'b1;
                if (conv_done) begin
                    if (op_mode == 3'd1)
                        next_state = RUN_FC_FWD;
                    else
                        next_state = RUN_ENROLL;
                end
            end

            RUN_FC_FWD: begin
                fc_fwd_enable = 1'b1;
                if (fc_done)
                    next_state = FINISH;
            end

            RUN_ENROLL: begin
                enroll_enable = 1'b1;
                if (train_done) next_state = FINISH;
            end

            RUN_FC_TRAIN: begin
                fc_train_enable = 1'b1;
                if (train_done)
                    next_state = FINISH;
            end

            FINISH: begin
                busy_status = 1'b0;
                done_irq    = 1'b1;
                next_state  = IDLE;
            end
        endcase
    end
endmodule

module isscc25_sinc_fe #(
    parameter DATA_WIDTH = 8,
    parameter CHANNELS   = 16
)(
    input  wire                           clk,
    input  wire                           rst_n,
    input  wire                           enable,
    input  wire                           sample_valid,
    input  wire signed [DATA_WIDTH-1:0]   sample_in,
    output reg                            feat_valid,
    output reg  [(CHANNELS*8)-1:0]        feat_out
);
    reg signed [DATA_WIDTH-1:0] shift_reg [0:7];
    reg [2:0] tap_idx;
    reg [7:0] stride_counter;

    localparam FILTER_WIDTH=(DATA_WIDTH==8 && CHANNELS<=16) ? 16 : 20;
    function automatic signed [7:0] approx_log;
        input signed [FILTER_WIDTH-5:0] val;
        reg [FILTER_WIDTH-5:0] abs_val;
        begin
            abs_val = (val < 0) ? -val : val;
            if (abs_val >= 16'd4096)
                approx_log = 8'd64 + (abs_val[11:4]);
            else if (abs_val >= 16'd1024)
                approx_log = 8'd48 + (abs_val[9:2]);
            else if (abs_val >= 16'd256)
                approx_log = 8'd32 + (abs_val[7:0]);
            else if (abs_val >= 16'd64)
                approx_log = 8'd16 + (abs_val[5:0]);
            else
                approx_log = abs_val[5:0];
        end
    endfunction

    integer ch, t;

    reg signed [DATA_WIDTH+7:0] base_accum;
    reg signed [DATA_WIDTH+2:0] sample_accum;
    wire signed [7:0] base_tap = (tap_idx == 3 || tap_idx == 4) ? 8'sd45 : -8'sd10;
    wire signed [15:0] base_product = sample_in * base_tap;
    wire signed [FILTER_WIDTH-1:0] channel_accum [0:CHANNELS-1];
    genvar channel;
    generate
        for (channel = 0; channel < CHANNELS; channel = channel + 1) begin : gen_sinc
            wire signed [FILTER_WIDTH-1:0] ch_idx;
            wire signed [FILTER_WIDTH-1:0] ch_offset;
            assign ch_idx = channel;
            assign ch_offset = ch_idx * sample_accum;
            assign channel_accum[channel] = base_accum - ch_offset;
        end
    endgenerate
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ch=0;
            feat_valid <= 0; stride_counter <= 0; tap_idx <= 0;
            feat_out <= 0; base_accum <= 0; sample_accum <= 0;
            for (t = 0; t < 8; t = t + 1) shift_reg[t] <= 0;
        end else if (enable && sample_valid) begin
            shift_reg[0] <= sample_in;
            for (t = 1; t < 8; t = t + 1) shift_reg[t] <= shift_reg[t-1];
            base_accum <= base_accum + base_product;
            sample_accum <= sample_accum + sample_in;
            tap_idx <= tap_idx + 1'b1;
            stride_counter <= stride_counter + 1'b1;
            if (stride_counter >= 7) begin
                stride_counter <= 0; feat_valid <= 1;
                for (ch = 0; ch < CHANNELS; ch = ch + 1)
                    feat_out[ch*8 +: 8] <= approx_log(channel_accum[ch][FILTER_WIDTH-1:4]);
                base_accum <= 0; sample_accum <= 0;
            end else feat_valid <= 0;
        end else feat_valid <= 0;
    end

endmodule

module isscc25_conv_array #(
    parameter DATA_WIDTH   = 8,
    parameter WEIGHT_WIDTH = 4,
    parameter CHANNELS     = 16,
    parameter FEAT_DIM     = 16
)(
    input  wire                           clk,
    input  wire                           rst_n,
    input  wire                           enable,
    input  wire                           feat_in_valid,
    input  wire [(CHANNELS*DATA_WIDTH)-1:0] feat_in,
    output reg                            conv_valid,
    output reg  [(FEAT_DIM*16)-1:0]       conv_out
);

    localparam [WEIGHT_WIDTH-1:0] DIAGONAL_WEIGHT=3;
    reg [3:0] feature_idx;
    reg completed;
    wire [DATA_WIDTH-1:0] value=feat_in[feature_idx*DATA_WIDTH +: DATA_WIDTH];
    wire [19:0] convolution=value * DIAGONAL_WEIGHT * 20'd7;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            conv_valid<=0; conv_out<=0; feature_idx<=0; completed<=0;
        end else begin
            conv_valid<=0;
            if (!enable) begin feature_idx<=0; completed<=0; end
            else if (!completed) begin
                conv_out[feature_idx*16 +: 16]<=convolution[19:4];
                if (feature_idx==FEAT_DIM-1) begin
                    completed<=1; conv_valid<=1;
                end else feature_idx<=feature_idx+1'b1;
            end
        end
    end

endmodule

module isscc25_fc_train_engine #(
    parameter FEAT_DIM = 16,
    parameter NUM_CLASSES = 35,
    parameter FC_WIDTH = 8,
    parameter PROTO_WIDTH = 16
)(
    input wire clk, rst_n,
    input wire fwd_enable, train_enable, enroll_enable,
    input wire [5:0] target_class,
    input wire feat_in_valid,
    input wire [(FEAT_DIM*16)-1:0] feat_in,
    output reg logit_valid,
    output reg [5:0] current_class_idx,
    output reg signed [15:0] current_logit,
    output reg fwd_done, train_done,
    output reg memory_ready
);
    localparam WORD_WIDTH = FC_WIDTH + PROTO_WIDTH;
    localparam WORDS = FEAT_DIM * NUM_CLASSES;
    localparam [FC_WIDTH-1:0] INITIAL_POS = 18, INITIAL_NEG = -4;

    reg [WORDS*WORD_WIDTH-1:0] state_ring;
    wire [WORD_WIDTH-1:0] head = state_ring[WORD_WIDTH-1:0];
    wire signed [FC_WIDTH-1:0] head_weight = head[FC_WIDTH-1:0];
    wire signed [PROTO_WIDTH-1:0] head_proto = head[WORD_WIDTH-1:FC_WIDTH];
    wire signed [15:0] proto_value = (PROTO_WIDTH == 16) ? $signed(head_proto) : $unsigned(head_proto);
    reg [5:0] ring_class;
    reg [3:0] ring_feature;
    reg [1:0] operation;
    localparam NONE=0, FORWARD=1, ENROLL=2, TRAIN=3;
    reg started, completed;
    reg [FEAT_DIM*16-1:0] features;
    reg signed [19:0] accumulator;
    wire signed [15:0] feature_value = features[ring_feature*16 +: 16];
    wire signed [19:0] product = feature_value * head_weight;
    wire signed [19:0] next_dot = accumulator + product;

    wire [7:0] proto_slice = proto_value[11:4];
    wire [7:0] gradient_scale = (ring_class == target_class) ? 8'd64 : -8'd2;
    wire [FC_WIDTH-1:0] gradient_product = gradient_scale * proto_slice;
    wire [FC_WIDTH-1:0] trained_weight = head_weight + (gradient_product >> 6);
    wire signed [15:0] enroll_feature = features[ring_feature*16 +: 16];
    wire signed [15:0] enroll_sum = proto_value + enroll_feature;
    wire signed [15:0] enrolled_proto = enroll_sum >>> 1;
    reg [WORD_WIDTH-1:0] tail;
    wire at_first = (ring_class == 0 && ring_feature == 0);
    wire at_last = (ring_class == NUM_CLASSES-1 && ring_feature == FEAT_DIM-1);
    wire active_word = memory_ready && !completed && operation != NONE &&
                       (started || (operation == ENROLL ?
                       (ring_class == target_class && ring_feature == 0) : at_first));
    always @(*) begin
        tail = head;
        if (!memory_ready)
            tail = {{PROTO_WIDTH{1'b0}}, ((ring_class % 16 == ring_feature) ? INITIAL_POS : INITIAL_NEG)};
        else if (active_word && operation == ENROLL)
            tail = {enrolled_proto[PROTO_WIDTH-1:0], head_weight};
        else if (active_word && operation == TRAIN)
            tail = {head_proto, trained_weight};
    end
    always @(posedge clk)
        state_ring <= {tail, state_ring[WORDS*WORD_WIDTH-1:WORD_WIDTH]};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ring_class <= 0; ring_feature <= 0; memory_ready <= 0;
            operation <= NONE; started <= 0; completed <= 0;
            features <= 0; accumulator <= 0;
            logit_valid <= 0; current_class_idx <= 0; current_logit <= 0;
            fwd_done <= 0; train_done <= 0;
        end else begin
            if (ring_feature == FEAT_DIM-1) begin
                ring_feature <= 0;
                if (ring_class == NUM_CLASSES-1) begin
                    ring_class <= 0;
                    memory_ready <= 1;
                end else ring_class <= ring_class + 1'b1;
            end else ring_feature <= ring_feature + 1'b1;
            logit_valid <= 0;
            fwd_done <= 0; train_done <= 0;
            if (!(fwd_enable || enroll_enable || train_enable)) begin
                operation <= NONE; started <= 0; completed <= 0;
            end else if (memory_ready && operation == NONE) begin
                features <= feat_in;
                operation <= enroll_enable ? ENROLL : train_enable ? TRAIN : FORWARD;
                accumulator <= 0;
                if (enroll_enable && target_class >= NUM_CLASSES) begin
                    completed <= 1;
                    train_done <= 1;
                end
            end else if (active_word) begin
                started <= 1;
                if (operation == FORWARD) begin
                    accumulator <= next_dot;
                    if (ring_feature == FEAT_DIM-1) begin
                        current_logit <= next_dot[19:4];
                        current_class_idx <= ring_class;
                        logit_valid <= 1;
                        accumulator <= 0;
                    end
                end
                if ((operation == ENROLL && ring_feature == FEAT_DIM-1) ||
                    (operation != ENROLL && at_last)) begin
                    completed <= 1;
                    if (operation == FORWARD) fwd_done <= 1;
                    else train_done <= 1;
                end
            end
            if (completed) begin
                if (operation == FORWARD) fwd_done <= 1;
                else train_done <= 1;
            end
        end
    end
endmodule

