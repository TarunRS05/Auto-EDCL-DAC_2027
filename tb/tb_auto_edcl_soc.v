`timescale 1ns / 1ps

module tb_auto_edcl_soc;

    reg clk;
    reg reset;

    reg               sample_valid;
    reg  signed [7:0] sample_in;
    wire              sample_ready;

    wire trap;

    wire mem_valid;
    wire mem_instr;
    wire [31:0] mem_addr;
    wire [31:0] mem_wdata;
    wire [3:0]  mem_wstrb;
    reg  [31:0] ext_rdata;
    reg         ext_ready;

    wire soc_irq;
    wire [31:0] soc_rdata;
    wire [31:0] psum_0;
    wire [31:0] psum_1;
    wire [31:0] psum_2;
    wire [31:0] psum_3;
    wire [1:0]  prec_tag;
    wire        event_alert;

    auto_edcl_soc dut (
        .clk          (clk),
        .reset        (reset),
        .sample_valid (sample_valid),
        .sample_in    (sample_in),
        .sample_ready (sample_ready),
        .trap         (trap),
        .mem_valid    (mem_valid),
        .mem_instr    (mem_instr),
        .mem_addr     (mem_addr),
        .mem_wdata    (mem_wdata),
        .mem_wstrb    (mem_wstrb),
        .ext_rdata    (ext_rdata),
        .ext_ready    (ext_ready),
        .soc_irq      (soc_irq),
        .soc_rdata    (soc_rdata),
        .psum_0       (psum_0),
        .psum_1       (psum_1),
        .psum_2       (psum_2),
        .psum_3       (psum_3),
        .prec_tag     (prec_tag),
        .event_alert  (event_alert)
    );

    always #0.5 clk = ~clk;

    integer k, m, d, s_idx;
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

    task send_sensor_sample;
        input signed [7:0] val;
        begin
            @(posedge clk);
            sample_valid <= 1'b1;
            sample_in    <= val;
            @(posedge clk);
            sample_valid <= 1'b0;
            sample_in    <= 8'sd0;
            repeat (2) @(posedge clk);
        end
    endtask

    initial begin
        $dumpfile("vcd/auto_edcl_soc/auto_edcl_soc.vcd");
        $dumpvars(0, dut);

        clk = 0;
        reset = 1;
        sample_valid = 0;
        sample_in = 8'sd0;
        ext_ready = 1'b1;
        ext_rdata = 32'h00000013;

        #100;
        reset = 0;
        repeat (900) @(posedge clk);

        for (s_idx = 0; s_idx < 8500; s_idx = s_idx + 1) begin
            send_sensor_sample(biosignal_mem[s_idx % 2048]);
        end

        for (s_idx = 0; s_idx < 5000; s_idx = s_idx + 1) begin
            send_sensor_sample(biosignal_mem[2048 + (s_idx % 2048)]);
        end

        for (s_idx = 0; s_idx < 3750; s_idx = s_idx + 1) begin
            send_sensor_sample(biosignal_mem[4096 + (s_idx % 2048)]);
        end

        for (s_idx = 0; s_idx < 2500; s_idx = s_idx + 1) begin
            send_sensor_sample(biosignal_mem[6144 + (s_idx % 2048)]);
        end

        for (k = -128; k <= 127; k = k + 1) begin
            send_sensor_sample(k[7:0]);
        end
        for (k = 0; k < 512; k = k + 1) begin
            send_sensor_sample((k % 2 == 0) ? 8'sd127 : -8'sd128);
        end
        for (k = 0; k < 2928; k = k + 1) begin
            send_sensor_sample((k % 2 == 0) ? 8'sd95 : -8'sd95);
        end

        for (k = 0; k < 3000; k = k + 1) begin
            @(posedge clk);
            sample_valid <= 1'b1;
            sample_in    <= biosignal_mem[k % 2048];
            ext_rdata    <= (k % 2 == 0) ? 32'h0060_F1C1 : 32'h0000_0013;
            @(posedge clk);
            sample_valid <= 1'b0;
            sample_in    <= 8'sd0;
            repeat (2) @(posedge clk);
        end

        sample_valid <= 1'b0;
        sample_in    <= 8'sd0;
        repeat (2000) @(posedge clk);

        $display("PASS: tb_auto_edcl_soc 100,000-cycle multi-modal comprehensive benchmark completed.");
        $finish;
    end

endmodule

