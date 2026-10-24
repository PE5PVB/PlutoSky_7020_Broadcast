// SPDX-License-Identifier: Apache-2.0
// tb_dab_top - the complete skypluto_wfm_exciter in DAB mode, driven like the ARM would: AXI writes of the phase words to 0x68 with flow
// control on the status (0x68 read), DAB mode + gain in 0x64, carrier level in 0x08; and an I2S master (48 kHz stereo, 64 BCLK per frame)
// whose samples are read back through the audio FIFO (0x70 count, 0x6C pop).
//   +WORDS=<hex phase words>  +NCLK=<l_clk clocks>  +X=<file>  +Y=<file>   (X/Y as tb_dab_ofdm, compared by check_hdl.py TOP=1)
// Checks itself: i_out/q_out = the OFDM output x 2^gain x level / 2^16 (through the modulator's I/Q path, any fixed delay) and the
// audio words (left = counter, right = ~counter, no gaps). These two results are printed as MOD:/AUDIO: lines (with FAIL on an error);
// check_hdl.py's own PASS/FAIL covers only the X/Y streams.
`timescale 1ns/1ps
module tb_dab_top;
    localparam integer GSH = 1;              // DAB gain 2^1
    localparam integer LEVEL = 32768;        // carrier level 0.5
    reg l_clk = 0, s_clk = 0, bclk = 0, rstn = 0;
    always #40.69 l_clk = ~l_clk;
    always #5 s_clk = ~s_clk;
    always #162.76 bclk = ~bclk;             // 48 kHz x 64

    // ---- I2S master: standard I2S, 24 bit in a 32-bit slot; left = {k, 8'h5A}, right = {~k, 8'hA5} --------------------------------
    reg lr = 1'b1, sd = 1'b0; integer bc = 0; reg [15:0] k = 16'd0; reg [31:0] wl, wr;
    always @(negedge bclk) begin
        if (bc == 0) begin k = k + 16'd1; wl = {k, 8'h5A, 8'h00}; wr = {~k, 8'hA5, 8'h00}; end
        lr <= (bc >= 32);
        if ((bc % 32) == 0) sd <= 1'b0;
        else sd <= (bc < 32) ? wl[31 - ((bc % 32) - 1)] : wr[31 - ((bc % 32) - 1)];
        bc <= (bc == 63) ? 0 : bc + 1;
    end

    // ---- optional disturbances (+GLITCH=<ns between spikes>): 3 ns spikes on BCLK and DATA at pseudo-random moments, like ringing
    // or coupling on the header wires; the exciter's input filter has to ignore them (define NOFILT to see what they do without it)
    reg gb = 1'b0, gd = 1'b0; integer gper = 0, gseed = 7;
    initial if ($value$plusargs("GLITCH=%d", gper) && gper > 0) forever begin
        #(gper / 2 + ($unsigned($random(gseed)) % gper)); gb = 1'b1; #3 gb = 1'b0;
        #(gper / 3); gd = 1'b1; #3 gd = 1'b0;
    end
    wire bclk_p = bclk ^ gb, sd_p = sd ^ gd;
`ifdef NOFILT
    defparam dut.I2S_FILT = 0;
`endif

    // ---- DUT ----------------------------------------------------------------------------------------------------------------------
    reg [6:0] awaddr = 0, araddr = 0; reg awvalid = 0, wvalid = 0, arvalid = 0; reg [31:0] wdata = 0;
    wire awready, wready, bvalid, arready, rvalid; wire [1:0] bresp, rresp; wire [31:0] rdata;
    wire signed [15:0] i_out, q_out;
    skypluto_wfm_exciter #(.LUT_FILE("../../hdl/library/skypluto_wfm/data/sine_lut.mem")) dut (
        .l_clk(l_clk), .l_clk_resetn(rstn),
        .i2s_in_bclk(bclk_p), .i2s_in_lrclk(lr), .i2s_in_data(sd_p),
        .dac_enable_i0(1'b1), .i_out(i_out), .q_out(q_out),
        .s_axi_aclk(s_clk), .s_axi_aresetn(rstn),
        .s_axi_awaddr(awaddr), .s_axi_awvalid(awvalid), .s_axi_awready(awready),
        .s_axi_wdata(wdata), .s_axi_wstrb(4'hF), .s_axi_wvalid(wvalid), .s_axi_wready(wready),
        .s_axi_bresp(bresp), .s_axi_bvalid(bvalid), .s_axi_bready(1'b1),
        .s_axi_araddr(araddr), .s_axi_arvalid(arvalid), .s_axi_arready(arready),
        .s_axi_rdata(rdata), .s_axi_rresp(rresp), .s_axi_rvalid(rvalid), .s_axi_rready(1'b1),
        .rx1_i_in(16'sd0), .rx1_q_in(16'sd0), .rx1_i_out(), .rx1_q_out());
    defparam dut.u_ups.COEF_FILE = "../../hdl/library/skypluto_wfm/data/interp_coefs.mem";

    task axi_wr(input [6:0] a, input [31:0] d);
        begin
            @(posedge s_clk); #1 awaddr = a; wdata = d; awvalid = 1; wvalid = 1;
            @(posedge s_clk); while (!bvalid) @(posedge s_clk);
            #1 awvalid = 0; wvalid = 0;
            @(posedge s_clk);
        end
    endtask
    reg [31:0] rd_v;
    task axi_rd(input [6:0] a);
        begin
            @(posedge s_clk); #1 araddr = a; arvalid = 1;
            @(posedge s_clk); while (!rvalid) @(posedge s_clk);
            rd_v = rdata; #1 arvalid = 0;
            @(posedge s_clk);
        end
    endtask

    // ---- modulator check: i_out(t) == (sat(dab_i << GSH) * LEVEL) >>> 16, (t - D) for one fixed D -----------------------------------
    function signed [15:0] sat16(input signed [31:0] v);
        sat16 = (v > 32767) ? 16'sd32767 : (v < -32768) ? -16'sd32768 : v[15:0];
    endfunction
    reg signed [15:0] hi [0:15], hq [0:15];
    integer bad [0:15]; integer checked = 0, d, j, nmatch;
    initial for (d = 0; d < 16; d = d + 1) begin bad[d] = 0; hi[d] = 0; hq[d] = 0; end
    reg checking = 0;
    always @(posedge l_clk) if (checking) begin
        for (j = 15; j > 0; j = j - 1) begin hi[j] = hi[j-1]; hq[j] = hq[j-1]; end
        hi[0] = (sat16($signed(dut.dab_i) <<< GSH) * LEVEL) >>> 16;
        hq[0] = (sat16($signed(dut.dab_q) <<< GSH) * LEVEL) >>> 16;
        if (dut.dab_i != 0) begin
            checked = checked + 1;
            for (d = 0; d < 16; d = d + 1) if (i_out !== hi[d] || q_out !== hq[d]) bad[d] = bad[d] + 1;
        end
    end

    // ---- capture as tb_dab_ofdm (2.048 MS/s debug stream and the 3.072 MS/s OFDM output) -----------------------------------------------
    integer fx, fy, nclk, lc = 0;
    reg [1023:0] fw, fxn, fyn;
    always @(posedge l_clk) if (checking) begin
        if (dut.u_dab.dbg_x_valid) $fwrite(fx, "%0d %0d\n", dut.u_dab.dbg_x_re, dut.u_dab.dbg_x_im);
        if (lc % 4 == 0) $fwrite(fy, "%0d %0d\n", dut.dab_i, dut.dab_q);
        lc = lc + 1;
    end

    reg [31:0] words [0:200000];
    integer nw, n, i, burst, aud_n = 0, aud_bad = 0, cnt, maxlvl = 0, sat_seen = 0;
    reg [15:0] aud_prev; reg aud_first = 1;
    initial begin
        if (!$value$plusargs("WORDS=%s", fw) || !$value$plusargs("X=%s", fxn) || !$value$plusargs("Y=%s", fyn)) begin $display("args"); $finish; end
        if (!$value$plusargs("NCLK=%d", nclk)) nclk = 1300000;
        for (i = 0; i <= 200000; i = i + 1) words[i] = 32'hFFFFFFFF;
        $readmemh(fw, words);
        nw = 0; while (nw < 200000 && words[nw] !== 32'hFFFFFFFF) nw = nw + 1;
        fx = $fopen(fxn, "w"); fy = $fopen(fyn, "w");
        #300 rstn = 1;
        #2000;
        axi_wr(7'h08, LEVEL);                         // carrier level
        axi_wr(7'h00, 32'd1);                         // enable
        axi_wr(7'h64, 32'd1 | (GSH << 2));            // DAB mode, gain
        @(posedge l_clk); checking = 1;
        n = 0;
        while (lc < nclk) begin
            axi_rd(7'h68);                            // status: [13:0] OFDM FIFO level, [16] underflow, [17] CDC FIFO full
            if (rd_v[13:0] > maxlvl) maxlvl = rd_v[13:0];
            if (rd_v[17]) sat_seen = sat_seen + 1;
            burst = (8000 - rd_v[13:0] > 200) ? 200 : 8000 - rd_v[13:0];
            if (rd_v[13:0] > 8000) burst = 0;
            for (i = 0; i < burst && n < nw; i = i + 1) begin axi_wr(7'h68, words[n]); n = n + 1; end
            axi_rd(7'h70); cnt = rd_v;               // audio
            for (i = 0; i < cnt; i = i + 1) begin
                axi_rd(7'h6C);
                if (!aud_first && (rd_v[31:16] !== aud_prev + 16'd1 || rd_v[15:0] !== ~rd_v[31:16])) aud_bad = aud_bad + 1;
                aud_prev = rd_v[31:16]; aud_first = 0; aud_n = aud_n + 1;
            end
            if (burst == 0) #20000;
        end
        $fclose(fx); $fclose(fy);
        nmatch = 0;
        for (d = 0; d < 16; d = d + 1) if (bad[d] == 0) begin $display("MOD: delay %0d clocks, %0d samples checked, 0 errors", d, checked); nmatch = nmatch + 1; end
        if (nmatch == 0 || checked == 0) $display("MOD: FAIL (no fixed delay matches the modulator output, %0d samples checked)", checked);
        $display("tb_dab_top: %0d clocks, %0d words written, underflow %0d, max FIFO level %0d, CDC full seen %0d",
                 lc, n, dut.u_dab.underflow, maxlvl, sat_seen);
        $display("AUDIO: %0d samples read, %0d errors", aud_n, aud_bad);
        if (aud_bad != 0 || aud_n == 0) $display("AUDIO: FAIL");
        $finish;
    end
endmodule
