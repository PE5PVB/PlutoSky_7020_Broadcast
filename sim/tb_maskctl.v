// tb_maskctl - skypluto_maskctl against its bit-exact model (sim/maskctl_model.py)
//   python3 sim/maskctl_model.py vectors <dir> [seconds]   writes mc_x.hex, mc_thr.hex, mc_q.txt, mc_cfg.txt into <dir>
//   iverilog -g2012 -I hdl/library/skypluto_wfm -o tb.vvp sim/tb_maskctl.v hdl/library/skypluto_wfm/skypluto_maskctl.v \
//            hdl/library/skypluto_wfm/skypluto_sincos.v && (cd <dir> && vvp -n tb.vvp)   (maskctl_sin.mem in <dir>)
// The samples arrive at 192 kHz against a 100 MHz clock (one every 520 clocks), as on the hardware.
// Prints PASS when every block's step equals the model's, and the largest number of clocks the engine needed per block.
`timescale 1ns/1ps
module tb_maskctl;
    reg clk = 0, rst = 1;
    always #5 clk = ~clk;
    localparam integer MAXN = 400000;
    reg [23:0] xs [0:MAXN-1];
    reg [27:0] th [0:15];
    integer qexp [0:8191];
    integer kdev, nsamp, nblk, fd, r;

    reg  [37:0] s_data; reg s_empty = 1; wire s_rd;
    wire [12:0] r_data; wire r_wr;
    reg  thr_we = 0; reg [3:0] thr_idx = 0; reg [27:0] thr_val = 0;
    wire [31:0] lim_blocks; wire [4:0] max_q, last_q; wire [15:0] overruns, resyncs;
    skypluto_maskctl dut (.clk(clk), .rst(rst), .s_data(s_data), .s_empty(s_empty), .s_rd(s_rd), .r_data(r_data), .r_wr(r_wr),
        .r_full(1'b0), .en(1'b1), .kdev(kdev[7:0]), .thr_we(thr_we), .thr_idx(thr_idx), .thr_val(thr_val), .clr(1'b0),
        .lim_blocks(lim_blocks), .max_q(max_q), .last_q(last_q), .overruns(overruns), .resyncs(resyncs));

    // the FWFT source: a new sample every 520 clocks
    integer n = 0, cnt = 0;
    always @(posedge clk) if (!rst) begin
        if (s_rd) s_empty <= 1'b1;
        cnt <= cnt + 1;
        if (cnt % 520 == 0 && n < nsamp) begin
            s_data <= {n[13:0], xs[n]}; s_empty <= 1'b0; n <= n + 1;
        end
    end

    // results
    integer got = 0, bad = 0, busy = 0, busymax = 0, i;
    always @(posedge clk) begin
        if (dut.est != 0) busy <= busy + 1; else begin if (busy > busymax) busymax <= busy; busy <= 0; end
        if (r_wr) begin
            if (r_data[12:5] != got[7:0] || r_data[4:0] != qexp[got]) begin
                if (bad < 10) $display("block %0d: got {%0d, %0d}, model %0d", got, r_data[12:5], r_data[4:0], qexp[got]);
                bad = bad + 1;
            end
            got = got + 1;
        end
    end

    initial begin
        fd = $fopen("mc_cfg.txt", "r"); r = $fscanf(fd, "%d %d %d", kdev, nsamp, nblk); $fclose(fd);
        $readmemh("mc_x.hex", xs);
        $readmemh("mc_thr.hex", th);
        fd = $fopen("mc_q.txt", "r"); for (i = 0; i < nblk; i = i + 1) r = $fscanf(fd, "%d", qexp[i]); $fclose(fd);
        repeat (5) @(posedge clk);
        for (i = 0; i < 16; i = i + 1) begin
            @(posedge clk); thr_we <= 1; thr_idx <= i; thr_val <= th[i];
        end
        @(posedge clk); thr_we <= 0;
        @(posedge clk); rst <= 0;
        wait (got >= nblk);
        repeat (10) @(posedge clk);
        $display("tb_maskctl: %0d blocks, %0d differ, blocks limited %0d, max step %0d, overruns %0d, resyncs %0d, engine max %0d clocks/block",
                 got, bad, lim_blocks, max_q, overruns, resyncs, busymax);
        if (bad == 0 && overruns == 0) $display("PASS"); else $display("FAIL");
        $finish;
    end
endmodule
