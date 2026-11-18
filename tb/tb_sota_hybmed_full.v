`timescale 1ns / 1ps

module tb_sota_hybmed_full;

    reg clk;
    reg reset;

    reg         start_cmd;
    reg [2:0]   exec_mode;
    reg [63:0]  prng_seed;
    reg         in_valid;
    reg signed [7:0] in_data;
    reg [4:0]   pe_sel_idx;

    wire        busy_status;
    wire        done_irq;
    wire        in_ready;
    wire        out_valid;
    wire signed [15:0] out_data;
    wire [31:0] pe_active_flags;
    wire [15:0] array_reduction_sig;

    sota_hybmed_full dut (
        .clk                 (clk),
        .reset               (reset),
        .start_cmd           (start_cmd),
        .exec_mode           (exec_mode),
        .prng_seed           (prng_seed),
        .in_valid            (in_valid),
        .in_data             (in_data),
        .pe_sel_idx          (pe_sel_idx),
        .busy_status         (busy_status),
        .done_irq            (done_irq),
        .in_ready            (in_ready),
        .out_valid           (out_valid),
        .out_data            (out_data),
        .pe_active_flags     (pe_active_flags),
        .array_reduction_sig (array_reduction_sig)
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
            in_valid <= 0;
            in_data  <= 0;
        end else begin
            in_valid <= in_ready;
            if (in_ready) begin
                in_data <= biosignal_mem[sample_counter % 2048];
                sample_counter <= sample_counter + 1;
            end else begin
                in_data <= 8'sd0;
            end
            pe_sel_idx <= pe_sel_idx + 1'b1;
        end
    end

    task command;
        input [2:0] mode;
        begin
            wait(!busy_status);
            @(negedge clk);
            exec_mode = mode;
            start_cmd = 1;
            @(negedge clk);
            start_cmd = 0;
            wait(done_irq);
            @(negedge clk);
            completed_commands = completed_commands + 1;
            repeat (3) @(negedge clk);
        end
    endtask

    initial begin
        $dumpfile("vcd/sota_hybmed_full/sota_hybmed_full.vcd");
        $dumpvars(0, dut);

        clk = 0;
        reset = 1;
        start_cmd = 0;
        exec_mode = 0;
        in_valid = 0;
        in_data = 0;
        prng_seed = 64'hA5A55A5A01234567;
        pe_sel_idx = 0;
        repeat (50) @(negedge clk);
        reset = 0;
        wait(!busy_status);
        repeat (2) @(negedge clk);

        fork
            begin : CMD_LOOP
                integer cmd_idx;
                cmd_idx = 0;
                while (1) begin

                    command((cmd_idx % 2 == 0) ? 3'd1 : 3'd3);
                    cmd_idx = cmd_idx + 1;
                end
            end
            begin : TIMER
                repeat (100000) @(posedge clk);
                disable CMD_LOOP;
            end
        join

        $display("PASS: tb_sota_hybmed_full 100,000-cycle benchmark completed (%0d commands).", completed_commands);
        $finish;
    end

endmodule

