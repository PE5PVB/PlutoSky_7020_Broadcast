// Top TB: the complete skypluto_wfm_exciter with a real I2S master (like the Pico): BCLK 64 x fs, standard I2S,
// 24 bit left-aligned in a 32-bit slot, the same word in both slots. Tone 1 kHz, word peak A (+A=...).
// Checks: IN_PEAK in the conditioner and the modulator signal (comp at the FM modulator).
`timescale 1ns/1ps
module tb_top;
    localparam real FS   = 192187.5;
    localparam real TRD  = 81.380;           // l_clk 12.288 MHz
    real A = 0.25;
    integer SQ = 0, IMP = 0, SL = 32;       // SL = slot length in BCLKs: 32 = 24-bit in a 32-bit slot (BCLK 64 x fs); 16 = 16-bit in a 16-bit slot (BCLK 32 x fs)
    real bhalf;
    initial begin
        if ($value$plusargs("A=%f", A)) ; if ($value$plusargs("SQ=%d", SQ)) ; if ($value$plusargs("IMP=%d", IMP)) ; if ($value$plusargs("SL=%d", SL)) ;
        bhalf = 1e9/(FS*2.0*SL)/2.0;
    end
    real pi; initial pi = 3.14159265358979;

    reg l_clk = 0, bclk = 0;
    always #(TRD/2.0) l_clk = ~l_clk;
    always #(bhalf)   bclk  = ~bclk;

    // ---- I2S master ---------------------------------------------------------
    reg lr = 1'b1, sd = 1'b0;
    integer bc = 0, fr = 0;
    reg [31:0] word;
    reg [15:0] w16 = 0; reg lsb_prev = 0;
    real sv;
    always @(negedge bclk) begin
        // frame of 64 BCLKs: slot 0 (left, lr=0) bits 0..31, slot 1 (right, lr=1) bits 32..63
        if (bc == 0) begin
            if (IMP) sv = (fr == 3000 || fr == 3000 + 192187) ? 0.2 : 0.0;   // impulse measurement: one frame of 20 % FS, the rest silent
            else if (SQ) sv = A * (((fr % 192) < 96) ? 1.0 : -1.0);      // 1 kHz square wave: the FIR gets a large overshoot
            else    sv = A * $sin(2.0*pi*1000.0*fr/FS);
            lsb_prev = w16[0];
            word = {$rtoi(sv*8388607.0), 8'h00};
            w16  = $rtoi(sv*32767.0);
            fr = fr + 1;
        end
        lr <= (bc >= SL);
        // MSB 1 BCLK after the LR edge: data bit index = (bc - 1) within the slot
        if (SL == 16) begin                                    // 16-bit data in a 16-bit slot: the LSB coincides with the next WS edge
            if ((bc % 16) == 0) sd <= (bc == 0) ? lsb_prev : w16[0];
            else                sd <= w16[16 - (bc % 16)];
        end else begin
            if ((bc % 32) == 0) sd <= 1'b0;                     // 1 BCLK delay: first 0 (previous slot LSB)
            else                sd <= word[31 - ((bc % 32) - 1)];
        end
        bc <= (bc == 2*SL - 1) ? 0 : bc + 1;
    end

    // ---- DUT --------------------------------------------------------------
    reg rstn = 0;
    initial if (IMP) force dut.imp_en_axi = 1'b1;
    reg s_clk = 0; always #5 s_clk = ~s_clk;
    wire signed [15:0] i_out, q_out;
    wire awr, wr_, bv, arr, rv;
    wire [31:0] rd; wire [1:0] br, rr;
    skypluto_wfm_exciter dut (
        .l_clk(l_clk), .l_clk_resetn(rstn),
        .i2s_in_bclk(bclk), .i2s_in_lrclk(lr), .i2s_in_data(sd),
        .dac_enable_i0(1'b1), .i_out(i_out), .q_out(q_out),
        .s_axi_aclk(s_clk), .s_axi_aresetn(rstn),
        .s_axi_awaddr(6'd0), .s_axi_awvalid(1'b0), .s_axi_awready(awr),
        .s_axi_wdata(32'd0), .s_axi_wstrb(4'd0), .s_axi_wvalid(1'b0), .s_axi_wready(wr_),
        .s_axi_bresp(br), .s_axi_bvalid(bv), .s_axi_bready(1'b1),
        .s_axi_araddr(6'd0), .s_axi_arvalid(1'b0), .s_axi_arready(arr),
        .s_axi_rdata(rd), .s_axi_rresp(rr), .s_axi_rvalid(rv), .s_axi_rready(1'b1),
        .rx1_i_in(16'sd0), .rx1_q_in(16'sd0), .rx1_i_out(), .rx1_q_out());

    integer pk = 0, n = 0;
    integer cmax = 0;
    always @(posedge l_clk) begin
        if (dut.ups_comp > cmax) cmax = dut.ups_comp;
        if (-dut.ups_comp > cmax) cmax = -dut.ups_comp;
    end

    initial begin
        #200 rstn = 1;
        repeat (20) begin
            #1000000;                         // 1 ms
            if (dut.sc_state == 0 || $time > 19000000000)
            $display("t=%0t  left=%0d fifo=%0d state=%0d fade=%0d inpeak=%0d events=%0d comp_max=%0d wrfull=%b",
                     $time, dut.i2s_left, dut.fifo_count, dut.sc_state, dut.sc_fade, dut.sc_inpeak, dut.sc_events, cmax, dut.fifo_full);
        end
        $display("DBG woorden: 0={ws_edges,sd_edges}=%h 1=last_left=%h 2=frames=%0d 3=wcnt=%0d 4=peak_b=%0d 5=pk_rd=%0d 6=pulls=%0d 7=pops=%0d 8=nz=%0d 9=pk_q=%0d 10=pk_comp=%0d 11=sat=%0d 12=id=%h",
                 dut.dbg_flat[31:0], dut.dbg_flat[63:32], dut.dbg_flat[95:64], dut.dbg_flat[127:96], dut.dbg_flat[159:128], dut.dbg_flat[191:160],
                 dut.dbg_flat[223:192], dut.dbg_flat[255:224], dut.dbg_flat[287:256], dut.dbg_flat[319:288], dut.dbg_flat[351:320], dut.dbg_flat[383:352], dut.dbg_flat[415:384]);
        $display("DBG2: 13=hb=%0d 14=take=%0d 15=ovf=%0d 16=ups_state=%h 17=sc_state=%h(st=%0d) 18=wd={idle=%0d,fsm=%0d}",
                 dut.dbg_flat[447:416], dut.dbg_flat[479:448], dut.dbg_flat[511:480], dut.dbg_flat[543:512], dut.dbg_flat[575:544],
                 dut.dbg_flat[548:544], dut.dbg_flat[607:592], dut.dbg_flat[591:576]);
        $display("SUM: ssum=0x%h (verwacht bij geen begrenzing 0xf0000)  gs=%0d  init_busy=%b p=%0d", dut.u_sc.ssum, dut.u_sc.gs, dut.u_sc.init_busy, dut.u_sc.p);
        $display("comp_max (na FIR + sat) = %0d ; 2^23-1 = 8388607", cmax);
        $display("FMT: slot=%0d bits=%0d (verwacht SL=%0d)", dut.i2s_fmt_slot, dut.i2s_fmt_bits, SL);
        $display("IMP: count=%0d stamp=%0d (laag16=%0d) hb=%0d rx1_i=%0d rx1_q=%0d", dut.imp_count, dut.imp_stamp, dut.imp_stamp[15:0], dut.d_hb, dut.rx1_i_out, dut.rx1_q_out);
        $finish;
    end
endmodule
