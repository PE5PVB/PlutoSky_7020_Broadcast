// SPDX-License-Identifier: Apache-2.0
// tb_dab_ofdm - skypluto_dab_ofdm against dab_ofdm_model.py (compared by check_hdl.py)
//   +WORDS=<file with one 32-bit hex phase word per line>  +NCLK=<clocks to run>  +X=<out file>  +Y=<out file>
// Writes the 2.048 MS/s samples (dbg_x, one line "re im" per sample) and the 3.072 MS/s output (sampled every 4 clocks).
`timescale 1ns/1ps
module tb_dab_ofdm;
    reg clk = 0, rst = 1;
    always #40.69 clk = ~clk;
    reg wr_en = 0; reg [31:0] wr_data = 0;
    wire [13:0] lvl; wire uf;
    wire signed [15:0] io, qo, xre, xim; wire xv;
    skypluto_dab_ofdm dut (.clk(clk), .rst(rst), .enable(1'b1), .wr_en(wr_en), .wr_data(wr_data), .fifo_level(lvl),
                           .underflow(uf), .i_out(io), .q_out(qo), .dbg_x_valid(xv), .dbg_x_re(xre), .dbg_x_im(xim));
    reg [31:0] words [0:200000];
    integer nw, n, fx, fy, nclk, i, r;
    reg [1023:0] fw, fxn, fyn;
    initial begin
        if (!$value$plusargs("WORDS=%s", fw) || !$value$plusargs("X=%s", fxn) || !$value$plusargs("Y=%s", fyn)) begin $display("args"); $finish; end
        if (!$value$plusargs("NCLK=%d", nclk)) nclk = 1300000;
        for (i = 0; i <= 200000; i = i + 1) words[i] = 32'hFFFFFFFF;
        $readmemh(fw, words);
        nw = 0; while (nw < 200000 && words[nw] !== 32'hFFFFFFFF) nw = nw + 1;   // (a real word of all ones would end the list)
        fx = $fopen(fxn, "w"); fy = $fopen(fyn, "w");
        repeat (5) @(posedge clk);
        rst = 0;
        n = 0;
        for (i = 0; i < nclk; i = i + 1) begin
            @(posedge clk);
            #1;
            wr_en = (n < nw) && (lvl < 14'd8000);
            wr_data = words[n];
            if (wr_en) n = n + 1;
            if (xv) $fwrite(fx, "%0d %0d\n", xre, xim);
            if (i % 4 == 0) $fwrite(fy, "%0d %0d\n", io, qo);
        end
        $fclose(fx); $fclose(fy);
        $display("tb_dab_ofdm: %0d clocks, %0d words written, underflow %0d", nclk, n, uf);
        $finish;
    end
endmodule
