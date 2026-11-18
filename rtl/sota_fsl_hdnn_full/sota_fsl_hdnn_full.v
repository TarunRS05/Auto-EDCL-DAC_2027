`timescale 1ns / 1ps

module sota_fsl_hdnn_full #(
    parameter FEAT_DIM    = 64,
    parameter HV_DIM      = 512,
    parameter NUM_CLASSES = 10
)(
    input  wire                   clk,
    input  wire                   reset,

    input  wire                   start_episode,
    input  wire [1:0]             phase_mode,
    input  wire [3:0]             target_class,
    output wire                   busy_status,
    output wire                   done_irq,

    input  wire                   sample_valid,
    input  wire signed [7:0]      sample_in,
    output wire                   sample_ready,

    output reg  [3:0]             pred_class,
    output reg  [9:0]             best_distance,
    output wire [NUM_CLASSES-1:0] class_match_flags,
    output wire [15:0]            class_memory_sig
);

    wire rst_n = ~reset;
    wire [(FEAT_DIM*8)-1:0] extracted_features;
    wire              fe_valid;

    wire [HV_DIM-1:0] bipolar_hv_out;
    wire              hv_valid;

    wire              bundle_done;
    wire              classify_done;
    wire [3:0]        matched_class;
    wire [9:0]        matched_dist;

    wire fe_enable;
    wire crp_enable;
    wire bundle_enable;
    wire classify_enable;

    wire memory_ready, episode_busy;
    assign busy_status = !memory_ready || episode_busy;

    fsl_episode_fsm u_fsm (
        .clk             (clk),
        .rst_n           (rst_n),
        .start_cmd       (start_episode && memory_ready),
        .phase_mode      (phase_mode),
        .fe_done         (fe_valid),
        .hv_done         (hv_valid),
        .bundle_done     (bundle_done),
        .classify_done   (classify_done),
        .fe_enable       (fe_enable),
        .crp_enable      (crp_enable),
        .bundle_enable   (bundle_enable),
        .classify_enable (classify_enable),
        .sample_ready    (sample_ready),
        .busy_status     (episode_busy),
        .done_irq        (done_irq)
    );

    fsl_clustered_fe #(
        .FEAT_DIM (FEAT_DIM)
    ) u_fe (
        .clk          (clk),
        .rst_n        (rst_n),
        .enable       (fe_enable),
        .sample_valid (sample_valid),
        .sample_in    (sample_in),
        .fe_valid     (fe_valid),
        .features_out (extracted_features)
    );

    fsl_crp_lfsr_engine #(
        .FEAT_DIM (FEAT_DIM),
        .HV_DIM   (HV_DIM)
    ) u_crp (
        .clk          (clk),
        .rst_n        (rst_n),
        .enable       (crp_enable),
        .features_in  (extracted_features),
        .hv_valid     (hv_valid),
        .bipolar_hv   (bipolar_hv_out)
    );

    fsl_serial_backend #(.HV_DIM(HV_DIM), .NUM_CLASSES(NUM_CLASSES)) u_backend (
        .clk(clk), .rst_n(rst_n), .bundle_enable(bundle_enable),
        .classify_enable(classify_enable), .target_class(target_class),
        .query_hv(bipolar_hv_out), .memory_ready(memory_ready),
        .bundle_done(bundle_done), .classify_done(classify_done),
        .pred_class(matched_class), .best_dist(matched_dist),
        .class_memory_sig(class_memory_sig)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pred_class    <= 4'd0;
            best_distance <= 10'd0;
        end else if (classify_done) begin
            pred_class    <= matched_class;
            best_distance <= matched_dist;
        end
    end

    genvar g;
    generate
        for (g = 0; g < NUM_CLASSES; g = g + 1) begin : gen_match
            assign class_match_flags[g] = (pred_class == g[3:0]) && done_irq;
        end
    endgenerate

endmodule

module fsl_episode_fsm (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       start_cmd,
    input  wire [1:0] phase_mode,
    input  wire       fe_done,
    input  wire       hv_done,
    input  wire       bundle_done,
    input  wire       classify_done,
    output reg        fe_enable,
    output reg        crp_enable,
    output reg        bundle_enable,
    output reg        classify_enable,
    output reg        sample_ready,
    output reg        busy_status,
    output reg        done_irq
);
    localparam IDLE       = 3'd0;
    localparam RUN_FE     = 3'd1;
    localparam RUN_CRP    = 3'd2;
    localparam RUN_BUNDLE = 3'd3;
    localparam RUN_CLASS  = 3'd4;
    localparam FINISH     = 3'd5;

    reg [2:0] state, next_state;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            state <= IDLE;
        else
            state <= next_state;
    end

    always @(*) begin
        next_state      = state;
        fe_enable       = 1'b0;
        crp_enable      = 1'b0;
        bundle_enable   = 1'b0;
        classify_enable = 1'b0;
        sample_ready    = 1'b0;
        busy_status     = 1'b1;
        done_irq        = 1'b0;

        case (state)
            IDLE: begin
                busy_status = 1'b0;
                if (start_cmd) begin
                    case (phase_mode)
                        2'd1: next_state = RUN_FE;
                        2'd2: next_state = RUN_FE;
                        2'd3: next_state = RUN_FE;
                        default: next_state = IDLE;
                    endcase
                end
            end

            RUN_FE: begin
                fe_enable    = 1'b1;
                sample_ready = 1'b1;
                if (fe_done) begin
                    if (phase_mode == 2'd1)
                        next_state = FINISH;
                    else
                        next_state = RUN_CRP;
                end
            end

            RUN_CRP: begin
                crp_enable = 1'b1;
                if (hv_done) begin
                    if (phase_mode == 2'd2)
                        next_state = RUN_BUNDLE;
                    else
                        next_state = RUN_CLASS;
                end
            end

            RUN_BUNDLE: begin
                bundle_enable = 1'b1;
                if (bundle_done)
                    next_state = FINISH;
            end

            RUN_CLASS: begin
                classify_enable = 1'b1;
                if (classify_done)
                    next_state = FINISH;
            end

            FINISH: begin
                busy_status = 1'b0;
                done_irq    = 1'b1;
                next_state  = IDLE;
            end

            default: begin
                next_state  = IDLE;
            end
        endcase
    end
endmodule

module fsl_clustered_fe #(
    parameter FEAT_DIM = 64
)(
    input  wire                     clk,
    input  wire                     rst_n,
    input  wire                     enable,
    input  wire                     sample_valid,
    input  wire signed [7:0]        sample_in,
    output reg                      fe_valid,
    output reg  [(FEAT_DIM*8)-1:0]  features_out
);

    function automatic signed [7:0] get_codebook;
        input [3:0] idx;
        begin
            case (idx)
                4'd0:  get_codebook = -8'sd64;
                4'd1:  get_codebook = -8'sd48;
                4'd2:  get_codebook = -8'sd32;
                4'd3:  get_codebook = -8'sd16;
                4'd4:  get_codebook = -8'sd8;
                4'd5:  get_codebook = -8'sd4;
                4'd6:  get_codebook = -8'sd2;
                4'd7:  get_codebook =  8'sd0;
                4'd8:  get_codebook =  8'sd2;
                4'd9:  get_codebook =  8'sd4;
                4'd10: get_codebook =  8'sd8;
                4'd11: get_codebook =  8'sd16;
                4'd12: get_codebook =  8'sd32;
                4'd13: get_codebook =  8'sd48;
                4'd14: get_codebook =  8'sd64;
                4'd15: get_codebook =  8'sd80;
            endcase
        end
    endfunction

    reg [5:0] sample_idx;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fe_valid     <= 1'b0;
            sample_idx   <= 6'd0;
            features_out <= {(FEAT_DIM*8){1'b0}};
        end else if (enable && sample_valid) begin

            features_out[sample_idx*8 +: 8] <= (sample_in + get_codebook(sample_idx[3:0])) >>> 1;
            sample_idx <= sample_idx + 1'b1;
            if (sample_idx >= 6'd63) begin
                sample_idx <= 6'd0;
                fe_valid   <= 1'b1;
            end else begin
                fe_valid   <= 1'b0;
            end
        end else begin
            fe_valid <= 1'b0;
        end
    end
endmodule

module fsl_crp_lfsr_engine #(
    parameter FEAT_DIM = 64,
    parameter HV_DIM   = 512
)(
    input  wire                     clk,
    input  wire                     rst_n,
    input  wire                     enable,
    input  wire [(FEAT_DIM*8)-1:0]  features_in,
    output reg                      hv_valid,
    output reg  [HV_DIM-1:0]        bipolar_hv
);
    reg [15:0] lfsr_state;
    reg [4:0]  block_step;
    reg        running;

    integer l;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            l = 0;
            lfsr_state  <= 16'hACE1;
            block_step  <= 5'd0;
            hv_valid    <= 1'b0;
            bipolar_hv  <= {HV_DIM{1'b0}};
            running     <= 1'b0;
        end else if (enable) begin
            if (!running) begin
                running    <= 1'b1;
                block_step <= 5'd0;
                hv_valid   <= 1'b0;
                lfsr_state <= 16'hACE1;
            end else begin

                lfsr_state <= {lfsr_state[14:0], 1'b0} ^ (lfsr_state[15] ? 16'h8016 : 16'h0000);

                for (l = 0; l < 16; l = l + 1) begin
                    bipolar_hv[{block_step, l[3:0]}] <=
                        ((features_in[{((block_step + l[4:0]) & 6'd63), 3'b000} +: 8] ^ {8{lfsr_state[l]}}) != 8'h0) ? 1'b1 : 1'b0;
                end

                block_step <= block_step + 1'b1;

                if (block_step >= 5'd31) begin
                    block_step <= 5'd0;
                    hv_valid   <= 1'b1;
                    running    <= 1'b0;
                end else begin
                    hv_valid   <= 1'b0;
                end
            end
        end else begin
            running  <= 1'b0;
            hv_valid <= 1'b0;
        end
    end
endmodule

module fsl_serial_backend #(
    parameter HV_DIM=512, NUM_CLASSES=10
)(
    input wire clk, rst_n, bundle_enable, classify_enable,
    input wire [3:0] target_class,
    input wire [HV_DIM-1:0] query_hv,
    output reg memory_ready, bundle_done, classify_done,
    output reg [3:0] pred_class,
    output reg [9:0] best_dist,
    output reg [15:0] class_memory_sig
);
    localparam CHUNKS = HV_DIM/16;
    reg [NUM_CLASSES*HV_DIM-1:0] prototype_ring;
    wire [15:0] head = prototype_ring[15:0];
    reg [3:0] ring_class;
    reg [8:0] ring_chunk;
    reg [1:0] operation;
    localparam NONE=0, BUNDLE=1, CLASSIFY=2;
    reg started, completed;
    reg [9:0] distance_accum;
    reg [3:0] selected_class;
    wire [15:0] query_word = query_hv[ring_chunk*16 +: 16];
    wire [15:0] difference = query_word ^ head;
    function automatic [4:0] popcount;
        input [15:0] value;
        integer b;
        begin
            popcount=0;
            for (b=0; b<16; b=b+1) popcount=popcount+value[b];
        end
    endfunction
    wire [9:0] distance_next = distance_accum + popcount(difference);
    wire active_word = memory_ready && !completed && operation != NONE &&
                       (started || (ring_chunk == 0 && ring_class ==
                       (operation == BUNDLE ? selected_class : 0)));
    wire [15:0] tail = !memory_ready ? (16'h5A5A ^ {12'd0, ring_class}) :
                       (active_word && operation == BUNDLE ? head ^ query_word : head);

    always @(posedge clk)
        prototype_ring <= {tail, prototype_ring[NUM_CLASSES*HV_DIM-1:16]};
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            memory_ready<=0; ring_class<=0; ring_chunk<=0;
            operation<=NONE; started<=0; completed<=0; selected_class<=0;
            distance_accum<=0; best_dist<=1023; pred_class<=0;
            bundle_done<=0; classify_done<=0; class_memory_sig<=0;
        end else begin
            if (ring_chunk == CHUNKS-1) begin
                ring_chunk<=0;
                if (ring_class == NUM_CLASSES-1) begin
                    ring_class<=0; memory_ready<=1;
                end else ring_class<=ring_class+1'b1;
            end else ring_chunk<=ring_chunk+1'b1;

            if (ring_chunk==0 || ring_chunk==15 || ring_chunk==31) begin
                if (!memory_ready) class_memory_sig<=class_memory_sig ^ tail;
                else if (active_word && operation==BUNDLE)
                    class_memory_sig<=class_memory_sig ^ head ^ tail;
            end
            bundle_done<=0; classify_done<=0;
            if (!(bundle_enable || classify_enable)) begin
                operation<=NONE; started<=0; completed<=0;
            end else if (memory_ready && operation==NONE) begin
                selected_class<=target_class;
                distance_accum<=0; best_dist<=1023; pred_class<=0;
                if (bundle_enable && target_class>=NUM_CLASSES) begin
                    bundle_done<=1; completed<=1; operation<=BUNDLE;
                end else operation<=bundle_enable ? BUNDLE : CLASSIFY;
            end else if (active_word) begin
                started<=1;
                if (operation==CLASSIFY) begin
                    distance_accum<=distance_next;
                    if (ring_chunk==CHUNKS-1) begin
                        if (distance_next<best_dist) begin
                            best_dist<=distance_next; pred_class<=ring_class;
                        end
                        distance_accum<=0;
                    end
                end
                if (ring_chunk==CHUNKS-1 &&
                    (operation==BUNDLE || ring_class==NUM_CLASSES-1)) begin
                    completed<=1;
                    if (operation==BUNDLE) bundle_done<=1;
                    else classify_done<=1;
                end
            end
            if (completed) begin
                if (operation==BUNDLE) bundle_done<=1;
                else classify_done<=1;
            end
        end
    end
endmodule

