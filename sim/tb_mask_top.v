// tb_mask_top - the complete exciter with mask control on, against the model (sim/maskctl_model.py, check: sim/check_mask_top.py)
// A real I2S master plays mc_x.hex (24-bit words, both slots) at 192 kHz; mask control on with the thresholds of mc_thr.hex and
// kdev from mc_cfg.txt, the deviation limiter off. Written for the check:
//   mt_xf.txt    every sample the conditioner sends to the mask controller: tag, value (after the fade)
//   mt_res.txt   every result of the mask controller: block, step
//   mt_greq.txt  per sample at the limiter: its number (cidx), g_req
// +MS=<ms> run time (default 40 ms).
`timescale 1ns/1ps
module tb_mask_top;
    localparam real FS = 192187.5;
    localparam real TRD = 81.380;            // l_clk 12.288 MHz
    real bhalf; initial bhalf = 1e9 / (FS * 64.0) / 2.0;
    integer MS = 40;
    initial if ($value$plusargs("MS=%d", MS)) ;

    localparam integer MAXN = 400000;
    reg [23:0] xs [0:MAXN-1];
    reg [27:0] th [0:15];
    integer kdev, nsamp, nblk, fd, r, i;
    initial begin
        fd = $fopen("mc_cfg.txt", "r"); r = $fscanf(fd, "%d %d %d", kdev, nsamp, nblk); $fclose(fd);
        $readmemh("mc_x.hex", xs);
        $readmemh("mc_thr.hex", th);
    end

    reg l_clk = 0, bclk = 0, s_clk = 0;
    always #(TRD / 2.0) l_clk = ~l_clk;
    always #(bhalf) bclk = ~bclk;
    always #5 s_clk = ~s_clk;

    // ---- I2S master: 24 bit in a 32-bit slot, standard I2S, the same word in both slots ----
    reg lr = 1'b1, sd = 1'b0;
    integer bc = 0, fr = 0;
    reg [31:0] word;
    always @(negedge bclk) begin
        if (bc == 0) begin word = {(fr < nsamp) ? xs[fr] : 24'd0, 8'h00}; fr = fr + 1; end
        lr <= (bc >= 32);
        if ((bc % 32) == 0) sd <= 1'b0; else sd <= word[31 - ((bc % 32) - 1)];
        bc <= (bc == 63) ? 0 : bc + 1;
    end

    // ---- DUT ----
    reg rstn = 0;
    wire signed [15:0] i_out, q_out;
    wire awr, wr_, bv, arr, rv; wire [31:0] rd; wire [1:0] br, rr;
    skypluto_wfm_exciter dut (
        .l_clk(l_clk), .l_clk_resetn(rstn),
        .i2s_in_bclk(bclk), .i2s_in_lrclk(lr), .i2s_in_data(sd),
        .dac_enable_i0(1'b1), .i_out(i_out), .q_out(q_out),
        .s_axi_aclk(s_clk), .s_axi_aresetn(rstn),
        .s_axi_awaddr(7'd0), .s_axi_awvalid(1'b0), .s_axi_awready(awr),
        .s_axi_wdata(32'd0), .s_axi_wstrb(4'd0), .s_axi_wvalid(1'b0), .s_axi_wready(wr_),
        .s_axi_bresp(br), .s_axi_bvalid(bv), .s_axi_bready(1'b1),
        .s_axi_araddr(7'd0), .s_axi_arvalid(1'b0), .s_axi_arready(arr),
        .s_axi_rdata(rd), .s_axi_rresp(rr), .s_axi_rvalid(rv), .s_axi_rready(1'b1),
        .rx1_i_in(16'sd0), .rx1_q_in(16'sd0), .rx1_i_out(), .rx1_q_out());
    initial begin
        force dut.mask_en_a = 1'b1;
        force dut.lim_en_axi = 1'b0;
        #1; force dut.kdev_axi = kdev;
    end

    integer fx, fres, fg;
    initial begin fx = $fopen("mt_xf.txt", "w"); fres = $fopen("mt_res.txt", "w"); fg = $fopen("mt_greq.txt", "w"); end
    always @(posedge l_clk) begin
        if (dut.u_sc.ms_wr) $fdisplay(fx, "%0d %0d", dut.u_sc.ms_data[37:24], $signed(dut.u_sc.ms_data[23:0]));
        if (dut.u_sc.st == 5'd9) $fdisplay(fg, "%0d %0d", dut.u_sc.cidx, dut.u_sc.greq);       // S_BLK: g_req of c is final
    end
    always @(posedge s_clk) if (dut.u_mc.r_wr) $fdisplay(fres, "%0d %0d", dut.u_mc.r_data[12:5], dut.u_mc.r_data[4:0]);
    integer TRB = -1;
    initial if ($value$plusargs("TRB=%d", TRB)) ;
    always @(posedge s_clk)
        if (TRB >= 0 && dut.u_mc.ev_b14[13:6] >= TRB && dut.u_mc.ev_b14[13:6] <= TRB + 2 && dut.u_mc.est == 3'd6)
            $display("t=%0t blk=%0d EVDONE q_try=%0d fail=%b lo=%0d hi=%0d fr=%0d lag=%0d bin=%b", $time, dut.u_mc.ev_b14[13:6],
                     dut.u_mc.q_try, dut.u_mc.fail, dut.u_mc.lo, dut.u_mc.hi, dut.u_mc.fr, dut.u_mc.lag, dut.u_mc.bin_phase);
    always @(posedge l_clk)
        if (TRB >= 0 && dut.u_sc.ms_wr && dut.u_sc.fidx >= TRB * 64 && dut.u_sc.fidx < TRB * 64 + 200 && dut.u_sc.fidx % 16 == 0)
            $display("t=%0t push %0d", $time, dut.u_sc.fidx);

    initial begin
        #200 rstn = 1;
        #300;
        for (i = 0; i < 16; i = i + 1) dut.u_mc.thr[i] = th[i];
        #(MS * 1000000.0);
        $display("tb_mask_top: mask_miss=%0d overruns=%0d resyncs=%0d blocks limited=%0d max step=%0d, wd_fsm=%0d",
                 dut.u_sc.mask_miss, dut.u_mc.overruns, dut.u_mc.resyncs, dut.u_mc.lim_blocks, dut.u_mc.max_q, dut.u_sc.wd_fsm);
        $fclose(fx); $fclose(fres); $fclose(fg);
        $finish;
    end
endmodule
