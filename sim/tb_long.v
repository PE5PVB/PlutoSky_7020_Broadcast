// Long-duration TB: complete exciter with a real I2S master, adjustable clock offset (+PPM=), AXI activity (+AXW=1: a clear write
// to LIM_CTRL every 20 ms, and a LIM_CEIL write every 100 ms), reset pulses (+RSTMS=<ms>), tone or square wave (+SQ=1 +A=), duration (+TEND=<ms>).
// Detects a STALL of the conditioner: dbg_pulls stops advancing within a 20 ms window.
`timescale 1ns/1ps
module tb_long;
    localparam real FS   = 192187.5;
    localparam real TRD  = 81.380;           // l_clk 12.288 MHz
    real A = 0.25, PPM = 0.0;
    integer SQ = 0, AXW = 0, TEND = 500, RSTMS = 0, RSTBURST = 0;
    initial begin
        if ($value$plusargs("A=%f", A)) ; if ($value$plusargs("SQ=%d", SQ)) ; if ($value$plusargs("PPM=%f", PPM));
        if ($value$plusargs("AXW=%d", AXW)); if ($value$plusargs("TEND=%d", TEND)); if ($value$plusargs("RSTMS=%d", RSTMS)); if ($value$plusargs("RSTBURST=%d", RSTBURST));
    end
    real pi; initial pi = 3.14159265358979;
    real bhalf; initial bhalf = 1e9/(FS*(1.0+PPM*1e-6)*64.0)/2.0;

    reg l_clk = 0, bclk = 0;
    always #(TRD/2.0) l_clk = ~l_clk;
    always #(bhalf)   bclk  = ~bclk;

    reg lr = 1'b1, sd = 1'b0;
    integer bc = 0, fr = 0;
    reg [31:0] word;
    real sv;
    always @(negedge bclk) begin
        if (bc == 0) begin
            if (SQ) sv = A * (((fr % 192) < 96) ? 1.0 : -1.0);
            else    sv = A * $sin(2.0*pi*1000.0*fr/FS) + 0.05 * $sin(2.0*pi*19000.0*fr/FS);
            word = {$rtoi(sv*8388607.0), 8'h00};
            fr = fr + 1;
        end
        lr <= (bc >= 32);
        if ((bc % 32) == 0) sd <= 1'b0; else sd <= word[31 - ((bc % 32) - 1)];
        bc <= (bc == 63) ? 0 : bc + 1;
    end

    reg rstn = 0;
    reg s_clk = 0; always #5 s_clk = ~s_clk;
    reg [5:0] awaddr = 0; reg awvalid = 0; reg [31:0] wdata = 0; reg wvalid = 0;
    wire signed [15:0] i_out, q_out;
    wire awr, wr_, bv, arr, rv;
    wire [31:0] rd; wire [1:0] br, rr;
    skypluto_wfm_exciter dut (
        .l_clk(l_clk), .l_clk_resetn(rstn),
        .i2s_in_bclk(bclk), .i2s_in_lrclk(lr), .i2s_in_data(sd),
        .dac_enable_i0(1'b1), .i_out(i_out), .q_out(q_out),
        .s_axi_aclk(s_clk), .s_axi_aresetn(rstn),
        .s_axi_awaddr(awaddr), .s_axi_awvalid(awvalid), .s_axi_awready(awr),
        .s_axi_wdata(wdata), .s_axi_wstrb(4'hF), .s_axi_wvalid(wvalid), .s_axi_wready(wr_),
        .s_axi_bresp(br), .s_axi_bvalid(bv), .s_axi_bready(1'b1),
        .s_axi_araddr(6'd0), .s_axi_arvalid(1'b0), .s_axi_arready(arr),
        .s_axi_rdata(rd), .s_axi_rresp(rr), .s_axi_rvalid(rv), .s_axi_rready(1'b1),
        .rx1_i_in(16'sd0), .rx1_q_in(16'sd0), .rx1_i_out(), .rx1_q_out());

    task axi_write(input [5:0] a, input [31:0] d);
        begin
            @(posedge s_clk); awaddr <= a; wdata <= d; awvalid <= 1; wvalid <= 1;
            @(posedge s_clk); while (!(awr && wr_)) @(posedge s_clk);
            awvalid <= 0; wvalid <= 0;
            repeat (4) @(posedge s_clk);
        end
    endtask

    integer ms = 0; integer last_pulls = 0, frozen = 0, cur;
    initial begin
        #200 rstn = 1;
        while (ms < TEND) begin
            #1000000; ms = ms + 1;
            if (AXW && (ms % 20) == 0) axi_write(6'h24, 32'h7);                 // clear pulse (as lim_poll)
            if (AXW && (ms % 100) == 0) axi_write(6'h20, 32'h7B0C05);           // ceil write (as lim_apply)
            if (RSTMS != 0 && ms == RSTMS) begin
                if (RSTBURST) begin                       // series of short resets with different intervals (also within the RAM clearing pass and in the middle of a pull)
                    rstn = 0; #100; rstn = 1; #3000;      // 3 us later: still within the clearing pass (256 clocks = 21 us)
                    rstn = 0; #100; rstn = 1; #9000;
                    rstn = 0; #40;  rstn = 1; #25000;     // just after the clearing pass
                    rstn = 0; #100; rstn = 1; #1234;
                    rstn = 0; #200; rstn = 1; #70000;     // in the middle of an FSM pull
                    rstn = 0; #100; rstn = 1;
                end else begin rstn = 0; #500; rstn = 1; end
            end
            if ((ms % 20) == 0) begin
                cur = dut.sc_dbg_pulls;
                if (cur == last_pulls && !frozen) begin
                    frozen = 1;
                    $display("*** STILSTAND bij t=%0d ms: pulls=%0d st=%0d q_empty=%b q_rd=%b up_empty=%b count=%0d state=%0d primed=%b bp=%0d wrfull=%b",
                             ms, cur, dut.u_sc.st, dut.sc_q_empty, dut.ups_fifo_rd, dut.fifo_empty, dut.fifo_count, dut.sc_state, dut.u_ups.primed, dut.u_ups.bp, dut.fifo_full);
                end
                last_pulls = cur;
            end
            if ((ms % 100) == 0)
                $display("t=%0d ms: pulls=%0d pops=%0d state=%0d count=%0d fade=%0d sat=%0d step=%0d wrfull=%b | venster %0d: piek comp=%0d q=%0d wd=%0d/%0d",
                         ms, dut.sc_dbg_pulls, dut.sc_dbg_pops, dut.sc_state, dut.fifo_count, dut.sc_fade, dut.ups_sat, dut.u_ups.step_dyn, dut.fifo_full,
                         dut.w_seq, dut.w_out_c, dut.w_out_q, dut.sc_dbg_wd[31:16], dut.sc_dbg_wd[15:0]);
        end
        if (!frozen) $display("GEEN STILSTAND in %0d ms (PPM=%f AXW=%0d SQ=%0d)", TEND, PPM, AXW, SQ);
        $finish;
    end
endmodule
