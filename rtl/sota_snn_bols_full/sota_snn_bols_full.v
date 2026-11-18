`timescale 1ns / 1ps

module sota_snn_bols_full #(
    parameter DATA_WIDTH     = 8,
    parameter NUM_NEURONS    = 16,
    parameter NUM_TIMESTEPS  = 16,
    parameter NUM_CLASSES    = 4
)(
    input  wire                     clk,
    input  wire                     reset,

    input  wire                     start_cmd,
    input  wire [1:0]               snn_mode,
    input  wire [1:0]               target_label,
    output wire                     busy_status,
    output wire                     done_irq,

    input  wire                     sample_valid,
    input  wire signed [DATA_WIDTH-1:0] sample_in,
    output wire                     sample_ready,

    output reg  [1:0]               pred_class,
    output reg  [7:0]               spike_count_out,
    output wire [NUM_CLASSES-1:0]   class_match_flags,
    output wire [15:0]              weight_signature
);

    wire                    rst_n = ~reset;
    wire                    spike_event_in;
    wire                    encoder_valid;

    wire [NUM_NEURONS-1:0]  neuron_spikes_out;
    wire                    lif_step_valid;

    wire                    dftt_done;
    wire [1:0]              winner_class;
    wire [7:0]              best_spike_accum;

    wire encoder_enable;
    wire lif_enable;
    wire dftt_enable;
    wire router_reset;

    wire                    weight_update_en;
    wire [(NUM_NEURONS*8)-1:0] weight_deltas;
    wire [(NUM_NEURONS*8)-1:0] synaptic_weights_out;

    snn_fsm_ctrl u_fsm (
        .clk            (clk),
        .rst_n          (rst_n),
        .start_cmd      (start_cmd),
        .snn_mode       (snn_mode),
        .enc_done       (encoder_valid),
        .lif_done       (lif_step_valid),
        .dftt_done      (dftt_done),
        .encoder_enable (encoder_enable),
        .lif_enable     (lif_enable),
        .dftt_enable    (dftt_enable),
        .router_reset   (router_reset),
        .sample_ready   (sample_ready),
        .busy_status    (busy_status),
        .done_irq       (done_irq)
    );

    snn_spike_encoder #(
        .DATA_WIDTH (DATA_WIDTH)
    ) u_encoder (
        .clk          (clk),
        .rst_n        (rst_n),
        .enable       (encoder_enable),
        .sample_valid (sample_valid),
        .sample_in    (sample_in),
        .spike_out    (spike_event_in),
        .enc_valid    (encoder_valid)
    );

    snn_lif_array_16 #(
        .NUM_NEURONS   (NUM_NEURONS),
        .NUM_TIMESTEPS (NUM_TIMESTEPS)
    ) u_lif_array (
        .clk              (clk),
        .rst_n            (rst_n),
        .enable           (lif_enable),
        .spike_in         (spike_event_in),
        .weight_update_en (weight_update_en),
        .weight_deltas    (weight_deltas),
        .spikes_out       (neuron_spikes_out),
        .step_valid       (lif_step_valid),
        .weights_out      (synaptic_weights_out)
    );

    snn_spike_router #(
        .NUM_NEURONS   (NUM_NEURONS),
        .NUM_TIMESTEPS (NUM_TIMESTEPS),
        .NUM_CLASSES   (NUM_CLASSES)
    ) u_router (
        .clk          (clk),
        .rst_n        (rst_n),
        .reset_accum  (router_reset),
        .step_valid   (lif_step_valid),
        .spikes_in    (neuron_spikes_out),
        .winner_class (winner_class),
        .max_spikes   (best_spike_accum)
    );

    snn_dftt_engine #(
        .NUM_NEURONS (NUM_NEURONS),
        .NUM_CLASSES (NUM_CLASSES)
    ) u_dftt (
        .clk              (clk),
        .rst_n            (rst_n),
        .enable           (dftt_enable),
        .target_label     (target_label),
        .pred_label       (winner_class),
        .spikes_fired     (neuron_spikes_out),
        .weight_update_en (weight_update_en),
        .weight_deltas    (weight_deltas),
        .dftt_done        (dftt_done)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pred_class      <= 2'd0;
            spike_count_out <= 8'd0;
        end else if (done_irq) begin
            pred_class      <= winner_class;
            spike_count_out <= best_spike_accum;
        end
    end

    genvar g;
    generate
        for (g = 0; g < NUM_CLASSES; g = g + 1) begin : gen_match
            assign class_match_flags[g] = (pred_class == g[1:0]) && done_irq;
        end
    endgenerate

    reg [15:0] w_sig;
    integer nw;
    always @(*) begin
        w_sig = 16'h0;
        for (nw = 0; nw < NUM_NEURONS; nw = nw + 1) begin
            w_sig = w_sig ^ {synaptic_weights_out[nw*8 +: 8], synaptic_weights_out[nw*8 +: 8] ^ nw[7:0]};
        end
    end
    assign weight_signature = w_sig;

endmodule

module snn_fsm_ctrl (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       start_cmd,
    input  wire [1:0] snn_mode,
    input  wire       enc_done,
    input  wire       lif_done,
    input  wire       dftt_done,
    output reg        encoder_enable,
    output reg        lif_enable,
    output reg        dftt_enable,
    output reg        router_reset,
    output reg        sample_ready,
    output reg        busy_status,
    output reg        done_irq
);
    localparam IDLE         = 3'd0;
    localparam RUN_ENCODER  = 3'd1;
    localparam RUN_LIF      = 3'd2;
    localparam RUN_DFTT     = 3'd3;
    localparam FINISH       = 3'd4;

    reg [2:0] state, next_state;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            state <= IDLE;
        else
            state <= next_state;
    end

    always @(*) begin
        next_state     = state;
        encoder_enable = 1'b0;
        lif_enable     = 1'b0;
        dftt_enable    = 1'b0;
        router_reset   = 1'b0;
        sample_ready   = 1'b0;
        busy_status    = 1'b1;
        done_irq       = 1'b0;

        case (state)
            IDLE: begin
                busy_status  = 1'b0;
                router_reset = 1'b1;
                if (start_cmd)
                    next_state = RUN_ENCODER;
            end

            RUN_ENCODER: begin
                encoder_enable = 1'b1;
                sample_ready   = 1'b1;
                if (enc_done)
                    next_state = RUN_LIF;
            end

            RUN_LIF: begin
                lif_enable = 1'b1;
                if (lif_done) begin
                    if (snn_mode == 2'd2)
                        next_state = RUN_DFTT;
                    else
                        next_state = FINISH;
                end
            end

            RUN_DFTT: begin
                dftt_enable = 1'b1;
                if (dftt_done)
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

module snn_spike_encoder #(
    parameter DATA_WIDTH = 8
)(
    input  wire                     clk,
    input  wire                     rst_n,
    input  wire                     enable,
    input  wire                     sample_valid,
    input  wire signed [DATA_WIDTH-1:0] sample_in,
    output reg                      spike_out,
    output reg                      enc_valid
);
    reg signed [DATA_WIDTH-1:0] prev_sample;
    reg [4:0] enc_step;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            spike_out   <= 1'b0;
            enc_valid   <= 1'b0;
            prev_sample <= {DATA_WIDTH{1'b0}};
            enc_step    <= 5'd0;
        end else if (enable && sample_valid) begin

            if ((sample_in - prev_sample) > 8'sd12 || (prev_sample - sample_in) > 8'sd12)
                spike_out <= 1'b1;
            else
                spike_out <= 1'b0;

            prev_sample <= sample_in;
            enc_step    <= enc_step + 1'b1;

            if (enc_step >= 5'd15) begin
                enc_step  <= 5'd0;
                enc_valid <= 1'b1;
            end else begin
                enc_valid <= 1'b0;
            end
        end else begin
            enc_valid <= 1'b0;
        end
    end
endmodule

module snn_lif_array_16 #(
    parameter NUM_NEURONS   = 16,
    parameter NUM_TIMESTEPS = 16
)(
    input  wire                     clk,
    input  wire                     rst_n,
    input  wire                     enable,
    input  wire                     spike_in,
    input  wire                     weight_update_en,
    input  wire [(NUM_NEURONS*8)-1:0] weight_deltas,
    output reg  [NUM_NEURONS-1:0]     spikes_out,
    output reg                        step_valid,
    output wire [(NUM_NEURONS*8)-1:0] weights_out
);
    localparam signed [15:0] V_THRESH = 16'sd128;

    reg signed [15:0] v_mem [0:NUM_NEURONS-1];
    reg signed [7:0]  synaptic_weights [0:NUM_NEURONS-1];
    reg [4:0]         t_step;

    genvar g;
    generate
        for (g = 0; g < NUM_NEURONS; g = g + 1) begin : gen_w_out
            assign weights_out[g*8 +: 8] = synaptic_weights[g];
        end
    endgenerate

    integer n;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            spikes_out <= {NUM_NEURONS{1'b0}};
            step_valid <= 1'b0;
            t_step     <= 5'd0;
            for (n = 0; n < NUM_NEURONS; n = n + 1) begin
                v_mem[n]            <= 16'sd0;
                synaptic_weights[n] <= (n % 2 == 0) ? 8'sd35 : -8'sd15;
            end
        end else if (weight_update_en) begin

            for (n = 0; n < NUM_NEURONS; n = n + 1) begin
                synaptic_weights[n] <= synaptic_weights[n] + (weight_deltas[n*8 +: 8]);
            end
        end else if (enable) begin
            t_step <= t_step + 1'b1;

            for (n = 0; n < NUM_NEURONS; n = n + 1) begin

                spikes_out[n] <= (v_mem[n] >= V_THRESH);

                v_mem[n] <= v_mem[n] - (v_mem[n] >>> 2) +
                            (spike_in ? synaptic_weights[n] : 16'sd0) -
                            ((v_mem[n] >= V_THRESH) ? V_THRESH : 16'sd0);
            end

            if (t_step >= (NUM_TIMESTEPS - 1)) begin
                t_step     <= 5'd0;
                step_valid <= 1'b1;
            end else begin
                step_valid <= 1'b0;
            end
        end else begin
            step_valid <= 1'b0;
        end
    end
endmodule

module snn_spike_router #(
    parameter NUM_NEURONS   = 16,
    parameter NUM_TIMESTEPS = 16,
    parameter NUM_CLASSES   = 4
)(
    input  wire                     clk,
    input  wire                     rst_n,
    input  wire                     reset_accum,
    input  wire                     step_valid,
    input  wire [NUM_NEURONS-1:0]   spikes_in,
    output reg  [1:0]               winner_class,
    output reg  [7:0]               max_spikes
);

    reg [7:0] class_spike_accum [0:NUM_CLASSES-1];
    integer c;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            winner_class <= 2'd0;
            max_spikes   <= 8'd0;
            for (c = 0; c < NUM_CLASSES; c = c + 1)
                class_spike_accum[c] <= 8'd0;
        end else if (reset_accum) begin
            winner_class <= 0; max_spikes <= 0;
            for (c=0; c<NUM_CLASSES; c=c+1) class_spike_accum[c]<=0;
        end else if (step_valid) begin

            class_spike_accum[0] <= class_spike_accum[0] + spikes_in[0]  + spikes_in[1]  + spikes_in[2]  + spikes_in[3];
            class_spike_accum[1] <= class_spike_accum[1] + spikes_in[4]  + spikes_in[5]  + spikes_in[6]  + spikes_in[7];
            class_spike_accum[2] <= class_spike_accum[2] + spikes_in[8]  + spikes_in[9]  + spikes_in[10] + spikes_in[11];
            class_spike_accum[3] <= class_spike_accum[3] + spikes_in[12] + spikes_in[13] + spikes_in[14] + spikes_in[15];

            winner_class <= 2'd0;
            max_spikes   <= class_spike_accum[0];
            for (c = 1; c < NUM_CLASSES; c = c + 1) begin
                if (class_spike_accum[c] > max_spikes) begin
                    max_spikes   <= class_spike_accum[c];
                    winner_class <= c[1:0];
                end
            end
        end
    end
endmodule

module snn_dftt_engine #(
    parameter NUM_NEURONS = 16,
    parameter NUM_CLASSES = 4
)(
    input  wire                     clk,
    input  wire                     rst_n,
    input  wire                     enable,
    input  wire [1:0]               target_label,
    input  wire [1:0]               pred_label,
    input  wire [NUM_NEURONS-1:0]   spikes_fired,
    output reg                        weight_update_en,
    output reg  [(NUM_NEURONS*8)-1:0] weight_deltas,
    output reg                        dftt_done
);
    reg [2:0]  dftt_step;
    reg [15:0] lfsr_fb;
    integer n;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            n=0;
            dftt_done        <= 1'b0;
            weight_update_en <= 1'b0;
            dftt_step        <= 3'd0;
            lfsr_fb          <= 16'h5A5A;
            weight_deltas <= {(NUM_NEURONS*8){1'b0}};
        end else if (enable) begin
            dftt_step <= dftt_step + 1'b1;

            lfsr_fb <= {lfsr_fb[14:0], 1'b0} ^ (lfsr_fb[15] ? 16'h8016 : 16'h0000);

            if (dftt_step == 3'd1) begin

                weight_update_en <= 1'b1;
                for (n = 0; n < NUM_NEURONS; n = n + 1) begin
                    if (target_label != pred_label) begin

                        weight_deltas[n*8 +: 8] <= (lfsr_fb[n] ^ spikes_fired[n]) ? 8'sd2 : -8'sd2;
                    end else begin
                        weight_deltas[n*8 +: 8] <= 8'sd0;
                    end
                end
            end else if (dftt_step == 3'd2) begin
                weight_update_en <= 1'b0;
            end else if (dftt_step >= 3'd4) begin
                dftt_step <= 3'd0;
                dftt_done <= 1'b1;
            end else begin
                dftt_done <= 1'b0;
            end
        end else begin
            dftt_step        <= 3'd0;
            dftt_done        <= 1'b0;
            weight_update_en <= 1'b0;
        end
    end
endmodule

