`timescale 1ns / 1ps

module tb_sota_fsl_hdnn_full;

    reg clk;
    reg reset;

    reg         start_episode;
    reg [1:0]   phase_mode;
    reg [3:0]   target_class;
    reg         sample_valid;
    reg signed [7:0] sample_in;

    wire        busy_status;
    wire        done_irq;
    wire        sample_ready;
    wire [3:0]  pred_class;
    wire [9:0]  best_distance;
    wire [9:0]  class_match_flags;
    wire [15:0] class_memory_sig;

    sota_fsl_hdnn_full dut (
        .clk              (clk),
        .reset            (reset),
        .start_episode    (start_episode),
        .phase_mode       (phase_mode),
        .target_class     (target_class),
        .sample_valid     (sample_valid),
        .sample_in        (sample_in),
        .busy_status      (busy_status),
        .done_irq         (done_irq),
        .sample_ready     (sample_ready),
        .pred_class       (pred_class),
        .best_distance    (best_distance),
        .class_match_flags(class_match_flags),
        .class_memory_sig (class_memory_sig)
    );

    always #0.5 clk = ~clk;

    integer sample_counter = 0, completed_commands = 0, c;
    reg signed [7:0] biosignal_mem [0:8191];

    initial begin
        for (c = 0; c < 2048; c = c + 1) begin
            if (c % 64 >= 28 && c % 64 <= 32)
                biosignal_mem[c] = 8'sd112;
            else if (c % 64 == 26 || c % 64 == 34)
                biosignal_mem[c] = -8'sd48;
            else
                biosignal_mem[c] = (c % 16) - 8;
        end
        $readmemh("sim/test_vectors/ecg_mitbih_raw_stream.hex", biosignal_mem);
    end

    always @(negedge clk) begin
        if (reset) begin
            sample_valid <= 0;
            sample_in    <= 0;
        end else begin
            sample_valid <= sample_ready;
            if (sample_ready) begin
                sample_in <= biosignal_mem[sample_counter % 2048];
                sample_counter <= sample_counter + 1;
            end else begin
                sample_in <= 8'sd0;
            end
        end
    end

    task command;
        input [1:0] mode;
        input [3:0] target;
        begin
            wait(!busy_status);
            @(negedge clk);
            phase_mode = mode;
            target_class = target;
            start_episode = 1;
            @(negedge clk);
            start_episode = 0;
            wait(done_irq);
            @(negedge clk);
            completed_commands = completed_commands + 1;
            repeat (3) @(negedge clk);
        end
    endtask

    initial begin
        $dumpfile("vcd/sota_fsl_hdnn_full/sota_fsl_hdnn_full.vcd");
        $dumpvars(0, dut);

        clk = 0;
        reset = 1;
        start_episode = 0;
        phase_mode = 0;
        sample_valid = 0;
        sample_in = 0;
        target_class = 0;
        repeat (50) @(negedge clk);
        reset = 0;
        wait(!busy_status);
        repeat (2) @(negedge clk);

        fork
            begin : CMD_LOOP
                integer cmd_idx;
                cmd_idx = 0;
                while (1) begin

                    command((cmd_idx % 2 == 0) ? 2'd2 : 2'd3, cmd_idx % 10);
                    cmd_idx = cmd_idx + 1;
                end
            end
            begin : TIMER
                repeat (100000) @(posedge clk);
                disable CMD_LOOP;
            end
        join

        $display("PASS: tb_sota_fsl_hdnn_full 100,000-cycle benchmark completed (%0d commands).", completed_commands);
        $finish;
    end

endmodule

