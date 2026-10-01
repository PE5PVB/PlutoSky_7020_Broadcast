// Chain TB: async FIFO -> skypluto_sigcond -> skypluto_interp -> skypluto_fm_modulator, with the same tone as the
// mask test: 1 kHz + 19 kHz pilot 9 %, composite peak 97.4 % of full scale, kdev=100.
// Pico rate offset by PPM (see below), production settings of the interpolator.
// Writes I/Q (every 4th l_clk = 3.072 MSPS, as the AD9361 picks them up) to chain_iq.txt.
`timescale 1ns/1ps
module tb_chain;
    localparam DW=24, AW=6, FRAC=24;
    localparam real FS_NOM = 192187.5;
    localparam real PPM    = -23.0;            // Pico clock rate relative to nominal (measured from the tone frequency in the RX time)
    localparam real TRD    = 81.380;          // l_clk 12.288 MHz
    localparam real A_TONE = 0.8840;          // tone 88.4 %
    localparam real A_PIL  = 0.0900;          // pilot 9 %  (together 97.4 %)
    localparam real F_TONE = 1000.0;
    localparam real F_PIL  = 19000.0;
    localparam integer WARM = 8000000;         // l_clk cycles warm-up
    localparam integer NREC = 400000;         // l_clk cycles recording (32 ms)

    reg rd_clk=0, wr_clk=0, rd_rst=1, wr_rst=1;
    real wrhalf, pi, s, fs_wr;
    integer kk;
    initial begin pi=3.14159265358979; fs_wr=FS_NOM*(1.0+PPM/1e6); wrhalf=1e9/fs_wr/2.0; end
    always #(TRD/2.0) rd_clk = ~rd_clk;
    always #(wrhalf)  wr_clk = ~wr_clk;

    reg  signed [DW-1:0] wr_data; wire wr_full; reg wr_en=0;
    always @(posedge wr_clk or posedge wr_rst) begin
        if (wr_rst) begin kk<=0; wr_en<=0; wr_data<=0; end
        else begin
            wr_en <= 1'b1;
            s = A_TONE*$sin(2.0*pi*F_TONE*kk/FS_NOM) + A_PIL*$sin(2.0*pi*F_PIL*kk/FS_NOM);
            wr_data <= $rtoi(s*8388607.0);
            if (wr_en & ~wr_full) kk <= kk+1;
        end
    end

    wire signed [DW-1:0] fdata; wire fempty; wire [AW:0] fcount, fwrpos; wire sc_rd;
    wire signed [DW-1:0] comp_out; wire comp_valid;
    wire signed [DW-1:0] q_data; wire q_empty, frd;
    skypluto_async_fifo #(.DW(DW), .AW(AW)) u_fifo (
        .wr_clk(wr_clk), .wr_rst(wr_rst), .wr_en(wr_en), .wr_data(wr_data), .wr_full(wr_full),
        .rd_clk(rd_clk), .rd_rst(rd_rst), .rd_en(sc_rd), .rd_data(fdata),
        .rd_empty(fempty), .rd_count(fcount), .wr_pos(fwrpos));

    // conditioner (ceiling adjustable via +CEIL=<counts>; default almost full scale = transparent)
    reg [DW-1:0] ceil_r = 24'h7FFFFF;
    initial if ($value$plusargs("CEIL=%d", ceil_r)) ;
    wire [16:0] gmin, fade_o; wire [31:0] events, uf_events; wire [DW-1:0] inpeak; wire [1:0] scstate;
    skypluto_sigcond #(.DW(DW), .TGT(1 << (AW-1))) u_sc (
        .clk(rd_clk), .rst(rd_rst), .up_data(fdata), .up_empty(fempty), .up_count(fcount), .up_wrpos(fwrpos), .up_rd(sc_rd),
        .q_data(q_data), .q_empty(q_empty), .q_rd(frd), .ceil(ceil_r), .lim_en(1'b1), .fade_en(1'b1), .clr(1'b0),
        .gmin(gmin), .events(events), .inpeak(inpeak), .state(scstate), .uf_events(uf_events), .fade_o(fade_o));

    skypluto_interp #(.DW(DW), .FRAC(FRAC), .CNTW(AW+1), .STEP_NOM(262400),
                      .COEF_FILE("data/interp_coefs.mem")) u_int (
        .clk(rd_clk), .rst(rd_rst), .step(24'd262400),
        .fifo_data(q_data), .fifo_empty(q_empty), .fifo_count(fcount), .fifo_rd(frd),
        .comp_out(comp_out), .comp_valid(comp_valid));

    // Simulator only: at start the interpolator still delivers x with comp_valid=1; x + something stays x in the
    // phase accumulator. On the FPGA all registers are 0 after configuration, so this does not exist there.
    reg mod_rst = 1'b1;
    wire signed [15:0] i_out, q_out; wire iq_valid;
    skypluto_fm_modulator #(.COMP_W(DW), .KDEV_W(18), .KSHIFT(13), .PHASE_W(24),
                            .LUT_ADDR_W(12), .OUT_W(16), .LVL_W(16), .DC_W(12),
                            .LUT_FILE("data/sine_lut.mem")) u_mod (
        .clk(rd_clk), .rst(mod_rst), .en(1'b1),
        .kdev(18'sd100), .offset_inc(24'sd0), .level(16'd65535),
        .dc_i(12'sd0), .dc_q(12'sd0),
        .comp(comp_out), .comp_valid(comp_valid),
        .i_out(i_out), .q_out(q_out), .iq_valid(iq_valid));

    integer f, cnt, ph, pkmax, ovf_w, unf_r;
    initial begin
        pkmax=0; ovf_w=0; unf_r=0;
        repeat (60) @(posedge rd_clk); rd_rst=0;
        repeat (60) @(posedge wr_clk); wr_rst=0;
        repeat (WARM/2) @(posedge rd_clk); mod_rst = 1'b0;
        repeat (WARM/2) @(posedge rd_clk);
        f=$fopen("chain_iq.txt","w");
        ph=0;
        for (cnt=0; cnt<NREC; cnt=cnt+1) begin
            @(posedge rd_clk);
            if (comp_out > pkmax) pkmax = comp_out;
            if (wr_full) ovf_w=ovf_w+1;
            if (fempty) unf_r=unf_r+1;
            if ((cnt % 4)==0) $fwrite(f,"%0d %0d\n", i_out, q_out);
        end
        $fclose(f);
        $display("DBG comp_out=%0d comp_valid=%b | mod: prod_r=%0d phase_inc_r=%0d phase=%0d v1=%b v2=%b v3=%b sin_raw=%0d cos_raw=%0d iq_valid=%b",
                 comp_out, comp_valid, u_mod.prod_r, u_mod.phase_inc_r, u_mod.phase, u_mod.v1, u_mod.v2, u_mod.v3,
                 u_mod.sin_raw, u_mod.cos_raw, iq_valid);
        $display("LOOP: step_dyn=%0d fifo=%0d wr_full=%0d rd_empty=%0d | sigcond: state=%0d events=%0d gmin=%0d uf=%0d inpeak=%0d", u_int.step_dyn, fcount, ovf_w, unf_r, scstate, events, gmin, uf_events, inpeak);
        $display("CHAIN klaar: comp piek=%0d (%0f %% van 2^23), fifo=%0d", pkmax, 100.0*pkmax/8388608.0, fcount);
        $finish;
    end
endmodule
