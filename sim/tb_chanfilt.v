// =============================================================================
// tb_chanfilt - skypluto_chanfilt against the bit-exact model of scripts/gen_chanfilt.py
//   Reads I/Q samples at 3.072 MS/s (hex, "IIII QQQQ" per line, two's complement) from +IN=<file>, holds each for 4 clocks (as the
//   modulator's 12.288 MS/s stream sampled by the core), runs with +MODE=<0..7>, and writes one output sample per 4 clocks to +OUT=<file>.
//   sim/chanfilt_check.py builds the input, runs this testbench and compares with the model (any decimation phase / delay).
// Run: iverilog -g2012 -o tb_chanfilt.vvp sim/tb_chanfilt.v hdl/library/skypluto_wfm/skypluto_chanfilt.v && vvp tb_chanfilt.vvp +IN=.. +OUT=.. +MODE=2
// =============================================================================
`timescale 1ns/1ps
module tb_chanfilt;
    reg clk = 0, rst = 1;
    always #40.69 clk = ~clk;
    reg [2:0] mode = 0;
    reg signed [15:0] xi = 0, xq = 0;
    wire signed [15:0] yi, yq;
    skypluto_chanfilt dut (.clk(clk), .rst(rst), .mode(mode), .i_in(xi), .q_in(xq), .i_out(yi), .q_out(yq));

    integer fi, fo, n, r, m;
    reg [15:0] a, b;
    reg [1023:0] fin, fout;
    initial begin
        if (!$value$plusargs("IN=%s", fin)) begin $display("need +IN"); $finish; end
        if (!$value$plusargs("OUT=%s", fout)) begin $display("need +OUT"); $finish; end
        if (!$value$plusargs("MODE=%d", m)) m = 2;
        mode = m[2:0];
        fi = $fopen(fin, "r"); fo = $fopen(fout, "w");
        repeat (8) @(posedge clk);
        rst = 0;
        n = 0;
        while (!$feof(fi)) begin
            r = $fscanf(fi, "%h %h\n", a, b);
            if (r == 2) begin
                xi = a; xq = b;
                repeat (2) @(posedge clk);
                $fwrite(fo, "%0d %0d\n", yi, yq);           // sampled mid-way through the 4-clock hold
                repeat (2) @(posedge clk);
                n = n + 1;
            end
        end
        $fclose(fi); $fclose(fo);
        $display("tb_chanfilt: %0d samples, mode %0d", n, mode);
        $finish;
    end
endmodule
