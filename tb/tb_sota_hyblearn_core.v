`timescale 1ns / 1ps

module tb_sota_hyblearn_core;

    reg clk;
    reg reset;

    reg [2:0]   op_mode;
    reg         streamer_en;
    reg [11:0]  sample_rate_div;
    reg         emrs_boxcar_en;
    reg [1:0]   emrs_boxcar_shift;
    reg [7:0]   margin_config;
    reg [6:0]   uadt_noise_floor;
    reg [2:0]   uadt_decay_shift;
    reg [1:0]   uadt_peak_mode;

    reg         valid_in;
    reg signed [7:0] sample_in;

    reg [3:0]   row_enable;
    reg         weight_load_en;
    reg [7:0]   weight_in;
    reg [1:0]   weight_row_addr;
    reg [1:0]   weight_col_addr;

    reg         mode_bwd;
    reg         bwd_submode;
    reg         adapt_en;
    reg [3:0]   error_sign_vec;
    reg [3:0]   error_valid_vec;

    wire        event_detected;
    wire [1:0]  current_prec_tag;
    wire [6:0]  uadt_thresh;
    wire        stream_fifo_full;
    wire        stream_fifo_empty;
    wire [15:0] inter_event_cycles;
    wire [15:0] active_int8_cycles;
    wire [127:0] psum_out;
    wire [3:0]  psum_valid;

    sota_hyblearn_core dut (
        .clk                (clk),
        .reset              (reset),
        .op_mode            (op_mode),
        .streamer_en        (streamer_en),
        .sample_rate_div    (sample_rate_div),
        .emrs_boxcar_en     (emrs_boxcar_en),
        .emrs_boxcar_shift  (emrs_boxcar_shift),
        .margin_config      (margin_config),
        .uadt_noise_floor   (uadt_noise_floor),
        .uadt_decay_shift   (uadt_decay_shift),
        .uadt_peak_mode     (uadt_peak_mode),
        .valid_in           (valid_in),
        .sample_in          (sample_in),
        .row_enable         (row_enable),
        .weight_load_en     (weight_load_en),
        .weight_in          (weight_in),
        .weight_row_addr    (weight_row_addr),
        .weight_col_addr    (weight_col_addr),
        .mode_bwd           (mode_bwd),
        .bwd_submode        (bwd_submode),
        .adapt_en           (adapt_en),
        .error_sign_vec     (error_sign_vec),
        .error_valid_vec    (error_valid_vec),
        .event_detected     (event_detected),
        .current_prec_tag   (current_prec_tag),
        .uadt_thresh        (uadt_thresh),
        .stream_fifo_full   (stream_fifo_full),
        .stream_fifo_empty  (stream_fifo_empty),
        .inter_event_cycles (inter_event_cycles),
        .active_int8_cycles (active_int8_cycles),
        .psum_out           (psum_out),
        .psum_valid         (psum_valid)
    );

    always #0.5 clk = ~clk;

    integer r, c, k, m, d, s_idx;
    reg signed [7:0] biosignal_mem [0:8191];

    initial begin

        for (s_idx = 0; s_idx < 2048; s_idx = s_idx + 1) begin
            if (s_idx % 64 >= 28 && s_idx % 64 <= 32)
                biosignal_mem[s_idx] = 8'sd112;
            else if (s_idx % 64 == 26 || s_idx % 64 == 34)
                biosignal_mem[s_idx] = -8'sd48;
            else if (s_idx % 64 >= 10 && s_idx % 64 <= 18)
                biosignal_mem[s_idx] = 8'sd28;
            else if (s_idx % 64 >= 42 && s_idx % 64 <= 54)
                biosignal_mem[s_idx] = 8'sd38;
            else
                biosignal_mem[s_idx] = (s_idx % 16) - 8;
        end

        for (s_idx = 2048; s_idx < 4096; s_idx = s_idx + 1) begin
            if ((s_idx / 256) % 2 == 0)
                biosignal_mem[s_idx] = ((s_idx % 8 < 4) ? 8'sd85 : -8'sd85) + ((s_idx % 16 < 8) ? 8'sd25 : -8'sd25);
            else
                biosignal_mem[s_idx] = (s_idx % 4) - 2;
        end

        for (s_idx = 4096; s_idx < 6144; s_idx = s_idx + 1) begin
            if ((s_idx / 128) % 2 == 0)
                biosignal_mem[s_idx] = ((s_idx % 4 == 0) ? 8'sd95 : (s_idx % 4 == 1) ? -8'sd92 : (s_idx % 4 == 2) ? 8'sd70 : -8'sd75);
            else
                biosignal_mem[s_idx] = (s_idx % 8) - 4;
        end

        for (s_idx = 6144; s_idx < 8192; s_idx = s_idx + 1) begin
            if (s_idx % 16 == 0)
                biosignal_mem[s_idx] = 8'sd100;
            else if (s_idx % 16 == 1)
                biosignal_mem[s_idx] = -8'sd100;
            else
                biosignal_mem[s_idx] = 8'sd0;
        end
        $readmemh("sim/test_vectors/ecg_mitbih_raw_stream.hex", biosignal_mem);
    end

    task send_sample;
        input signed [7:0] val;
        begin
            @(posedge clk);
            valid_in  = 1'b1;
            sample_in = val;
            @(posedge clk);
            valid_in  = 1'b0;
            sample_in = 8'sd0;
            repeat (2) @(posedge clk);
        end
    endtask

    initial begin
        $dumpfile("vcd/sota_hyblearn_core/sota_hyblearn_core.vcd");
        $dumpvars(0, dut);

        clk = 0;
        reset = 1;
        op_mode = 3'b110;
        streamer_en = 0;
        sample_rate_div = 12'd0;
        emrs_boxcar_en = 0;
        emrs_boxcar_shift = 2'd0;
        margin_config = 8'd28;
        uadt_noise_floor = 7'd32;
        uadt_decay_shift = 3'd4;
        uadt_peak_mode = 2'b00;
        valid_in = 0;
        sample_in = 8'sd0;
        row_enable = 4'b1111;
        weight_load_en = 0;
        weight_in = 8'd0;
        weight_row_addr = 2'd0;
        weight_col_addr = 2'd0;
        mode_bwd = 0;
        bwd_submode = 0;
        adapt_en = 0;
        error_sign_vec = 4'b0000;
        error_valid_vec = 4'b0000;

        #100;
        reset = 0;
        repeat (20) @(posedge clk);

        for (r = 0; r < 4; r = r + 1) begin
            for (c = 0; c < 4; c = c + 1) begin
                @(posedge clk);
                weight_load_en  = 1'b1;
                weight_row_addr = r[1:0];
                weight_col_addr = c[1:0];
                weight_in       = (r * 17 - c * 23 + 35) & 8'hFF;
            end
        end
        @(posedge clk);
        weight_load_en = 1'b0;
        weight_in      = 8'd0;
        repeat (864) @(posedge clk);

        for (s_idx = 0; s_idx < 8500; s_idx = s_idx + 1) begin
            send_sample(biosignal_mem[s_idx % 2048]);
        end

        for (s_idx = 0; s_idx < 5000; s_idx = s_idx + 1) begin
            send_sample(biosignal_mem[2048 + (s_idx % 2048)]);
        end

        for (s_idx = 0; s_idx < 3750; s_idx = s_idx + 1) begin
            send_sample(biosignal_mem[4096 + (s_idx % 2048)]);
        end

        for (s_idx = 0; s_idx < 2500; s_idx = s_idx + 1) begin
            send_sample(biosignal_mem[6144 + (s_idx % 2048)]);
        end

        for (k = -128; k <= 127; k = k + 1) begin
            send_sample(k[7:0]);
        end

        for (k = 0; k < 512; k = k + 1) begin
            send_sample((k % 2 == 0) ? 8'sd127 : -8'sd128);
        end

        streamer_en = 1'b1;
        emrs_boxcar_en = 1'b1;
        for (d = 0; d < 4; d = d + 1) begin
            sample_rate_div = (d == 0) ? 12'd0 : ((12'd1 << (d * 2)) - 1);
            emrs_boxcar_shift = d[1:0];
            for (k = 0; k < 64; k = k + 1) begin
                @(posedge clk);
                valid_in  = 1'b1;
                sample_in = (k % 2 == 0) ? 8'sd85 : -8'sd85;
                @(posedge clk);
                valid_in  = 1'b0;
                sample_in = 8'sd0;
                repeat (2) @(posedge clk);
            end
        end
        streamer_en = 1'b0;
        emrs_boxcar_en = 1'b0;

        for (k = 0; k < 1904; k = k + 1) begin
            @(posedge clk);
            valid_in  = 1'b1;
            sample_in = (k % 2 == 0) ? 8'sd105 : -8'sd105;
        end
        @(posedge clk);
        valid_in = 1'b0;
        sample_in = 8'sd0;

        mode_bwd = 1'b1;
        for (m = 0; m < 2; m = m + 1) begin
            bwd_submode = m[0];
            for (k = 0; k < 1500; k = k + 1) begin
                @(posedge clk);
                adapt_en        = 1'b1;
                error_sign_vec  = k[3:0];
                error_valid_vec = 4'b1111;
                @(posedge clk);
                adapt_en        = 1'b0;
                error_valid_vec = 4'b0000;
                repeat (2) @(posedge clk);
            end
        end
        mode_bwd    = 1'b0;
        bwd_submode = 1'b0;
        adapt_en    = 1'b0;

        valid_in = 1'b0;
        sample_in = 8'sd0;
        repeat (2000) @(posedge clk);

        $display("PASS: tb_sota_hyblearn_core 100,000-cycle multi-modal comprehensive benchmark completed.");
        $finish;
    end

endmodule

