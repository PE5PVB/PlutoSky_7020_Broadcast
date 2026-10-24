// SPDX-License-Identifier: Apache-2.0
// =============================================================================
// skypluto_dab_ofdm - OFDM part of the DAB transmitter (mode I), bit-exact to dab/hdl/dab_ofdm_model.py
// -----------------------------------------------------------------------------
// The ARM sends per transmission frame 76 symbols x 96 words; each word holds 16 carriers x 2-bit phase index p
// (carrier a in bits 2(a%16)+1..2(a%16); carrier order +1..+768, -768..-1). This block
//   - maps carrier a of symbol l to 45 degrees x ((2p + l) mod 8), amplitude 16384 (11585 on the diagonals),
//   - computes the 2048-point inverse FFT (radix-2, decimation in frequency, 18 bit, one butterfly per clock, three
//     memory banks: X and Y alternate as source/destination per stage, the third is being played out),
//   - plays out per frame a null symbol (2656 zeros) and 76 x (504 cyclic prefix + 2048) samples at 2.048 MS/s,
//   - interpolates 3/2 to 3.072 MS/s (144-tap polyphase FIR, 12 products per lane per clock over 4 clocks, pipelined),
//     flat to the outermost carriers and stopping from 900 kHz (no images, and the OFDM sidelobes around the ensemble cut).
// clk is the AD9361 data clock l_clk (12.288 MHz); the output changes every 4 clocks (3.072 MS/s), like the FM path.
// Per symbol: load ~1100 + 11 stages x 1031 clocks = ~12.5k clocks (1.02 ms) against a symbol period of 1.246 ms.
// This file exists twice, in hdl/library/skypluto_wfm/ and dab/hdl/; keep the two byte-identical (check_hdl.py TOP=1 checks it).
// =============================================================================
`timescale 1ns/1ps

module skypluto_dab_ofdm (
    input  wire               clk,
    input  wire               rst,              // also clears an underflow; the ARM then restarts at a frame boundary
    input  wire               enable,           // 0: output zeros (the frame does not start)
    input  wire               wr_en,            // phase word FIFO, write side (clk domain)
    input  wire        [31:0] wr_data,
    output wire        [13:0] fifo_level,       // words in the FIFO (8192 deep)
    output reg                underflow = 1'b0, // a symbol was due but not computed (sticky until rst)
    output reg  signed [15:0] i_out = 0,        // 3.072 MS/s, held 4 clocks
    output reg  signed [15:0] q_out = 0,
    output reg                dbg_x_valid = 1'b0, // the 2.048 MS/s samples before the interpolator (tests)
    output reg  signed [15:0] dbg_x_re = 0,
    output reg  signed [15:0] dbg_x_im = 0
);
    localparam integer N = 2048, TG = 504, TNULL = 2656;
    integer ii;

    function [10:0] bitrev11(input [10:0] v); integer k; begin for (k = 0; k < 11; k = k + 1) bitrev11[k] = v[10 - k]; end endfunction
    function signed [17:0] sat18(input signed [40:0] v); begin sat18 = (v > 41'sd131071) ? 18'sd131071 : (v < -41'sd131072) ? -18'sd131072 : v[17:0]; end endfunction
    function signed [15:0] sat16(input signed [40:0] v); begin sat16 = (v > 41'sd32767) ? 16'sd32767 : (v < -41'sd32768) ? -16'sd32768 : v[15:0]; end endfunction
    function signed [17:0] cre(input [2:0] q); begin case (q) 3'd0: cre = 16384; 3'd1: cre = 11585; 3'd2: cre = 0; 3'd3: cre = -11585; 3'd4: cre = -16384; 3'd5: cre = -11585; 3'd6: cre = 0; default: cre = 11585; endcase end endfunction
    function signed [17:0] cim(input [2:0] q); begin case (q) 3'd0: cim = 0; 3'd1: cim = 11585; 3'd2: cim = 16384; 3'd3: cim = 11585; 3'd4: cim = 0; 3'd5: cim = -11585; 3'd6: cim = -16384; default: cim = -11585; endcase end endfunction
    function [10:0] binof(input [10:0] a); begin binof = (a < 11'd768) ? a + 11'd1 : a + 11'd512; end endfunction

    // ---------------------------------------------------------------- phase word FIFO (8192 x 32)
    reg [31:0] fifo [0:8191];
    reg [12:0] f_wp = 0, f_rp = 0;
    reg [13:0] f_cnt = 0;
    reg        f_rd = 0, f_q_valid = 0;
    reg [31:0] f_q = 0;
    wire       f_wr = wr_en && (f_cnt != 14'd8192);
    assign fifo_level = f_cnt;
    always @(posedge clk) begin
        if (f_wr) fifo[f_wp] <= wr_data;
        if (f_rd) f_q <= fifo[f_rp];
        if (rst) begin f_wp <= 0; f_rp <= 0; f_cnt <= 0; f_q_valid <= 0; end
        else begin
            if (f_wr) f_wp <= f_wp + 13'd1;
            if (f_rd) f_rp <= f_rp + 13'd1;
            f_cnt <= f_cnt + (f_wr ? 14'd1 : 14'd0) - (f_rd ? 14'd1 : 14'd0);
            f_q_valid <= f_rd;
        end
    end

    // ---------------------------------------------------------------- three banks, 2048 x (18 + 18), two ports each
    reg        wa_en [0:2], wb_en [0:2];
    reg [10:0] a_adr [0:2], b_adr [0:2];
    reg signed [17:0] wa_re [0:2], wa_im [0:2], wb_re [0:2], wb_im [0:2];
    wire signed [17:0] ra_re0, ra_im0, rb_re0, rb_im0, ra_re1, ra_im1, rb_re1, rb_im1, ra_re2, ra_im2, rb_re2, rb_im2;
    dab_tdp_ram u_m0 (.clk(clk), .wea(wa_en[0]), .addra(a_adr[0]), .dia({wa_re[0], wa_im[0]}), .doa({ra_re0, ra_im0}),
                                 .web(wb_en[0]), .addrb(b_adr[0]), .dib({wb_re[0], wb_im[0]}), .dob({rb_re0, rb_im0}));
    dab_tdp_ram u_m1 (.clk(clk), .wea(wa_en[1]), .addra(a_adr[1]), .dia({wa_re[1], wa_im[1]}), .doa({ra_re1, ra_im1}),
                                 .web(wb_en[1]), .addrb(b_adr[1]), .dib({wb_re[1], wb_im[1]}), .dob({rb_re1, rb_im1}));
    dab_tdp_ram u_m2 (.clk(clk), .wea(wa_en[2]), .addra(a_adr[2]), .dia({wa_re[2], wa_im[2]}), .doa({ra_re2, ra_im2}),
                                 .web(wb_en[2]), .addrb(b_adr[2]), .dib({wb_re[2], wb_im[2]}), .dob({rb_re2, rb_im2}));
    function signed [17:0] sel3(input [1:0] k, input signed [17:0] v0, input signed [17:0] v1, input signed [17:0] v2);
        begin sel3 = (k == 2'd0) ? v0 : (k == 2'd1) ? v1 : v2; end endfunction

    // ---------------------------------------------------------------- twiddles e^{+j 2 pi m / 2048}, m = 0..1023, scale 2^16
    `include "skypluto_dab_twiddle.vh"            // function [35:0] c_tw(input [9:0] m) (generated ROM)

    // ---------------------------------------------------------------- bank roles and the symbol engine
    reg [1:0] bk_out = 2'd2, bk_x = 2'd0, bk_y = 2'd1;   // played out / load + even-stage source / odd-stage source
    localparam S_IDLE = 3'd0, S_ZERO = 3'd1, S_LOAD = 3'd2, S_STAGE = 3'd3, S_DRAIN = 3'd4, S_DONE = 3'd5;
    reg [2:0]  st = S_IDLE;
    reg [6:0]  sym_l = 0;                          // index of the symbol being computed (0..75)
    reg [9:0]  cnt = 0;
    reg [3:0]  stage = 0;
    reg [6:0]  wcnt = 0;                           // words taken (1..96)
    reg [2:0]  wsub = 0;                           // carrier pair within the word
    reg        have_word = 0, ready = 0, src_y = 0, scaled = 0;
    reg [31:0] word = 0;
    reg [3:0]  drain = 0;
    reg        take_req = 0;                       // from the play-out: it takes the computed symbol (bk_y)
    wire [10:0] h  = 11'd1024 >> stage;
    wire [10:0] bi = ({1'b0, cnt} >> (4'd10 - stage) << (4'd11 - stage)) | ({1'b0, cnt} & (h - 11'd1));
    wire [2:0] l8  = sym_l[2:0];

    // butterfly pipeline: T read address, T+1 sums/differences + twiddle, T+2 products, T+3 combine, T+4 write
    reg        q1_v = 0, q2_v = 0, q3_v = 0;
    reg [10:0] q0_i = 0, q0_j = 0, q1_i = 0, q1_j = 0, q2_i = 0, q2_j = 0, q3_i = 0, q3_j = 0;
    reg [9:0]  q0_m = 0;
    reg        q0_v = 0;
    reg signed [18:0] q1_sr = 0, q1_si = 0, q1_dr = 0, q1_di = 0, q2_sr = 0, q2_si = 0;
    reg signed [17:0] q1_wr = 0, q1_wi = 0;
    reg signed [36:0] q2_m1 = 0, q2_m2 = 0, q2_m3 = 0, q2_m4 = 0;
    reg signed [40:0] q3_tr = 0, q3_ti = 0, q3_sr = 0, q3_si = 0;
    wire [1:0] bsrc = src_y ? bk_y : bk_x, bdst = src_y ? bk_x : bk_y;

    // play-out address (port A of the played bank)
    reg [10:0] out_adr = 0;

    always @(*) begin : drive
        integer k;
        for (k = 0; k < 3; k = k + 1) begin
            wa_en[k] = 0; wb_en[k] = 0; a_adr[k] = 0; b_adr[k] = 0; wa_re[k] = 0; wa_im[k] = 0; wb_re[k] = 0; wb_im[k] = 0;
        end
        a_adr[bk_out] = out_adr;
        if (st == S_ZERO) begin                        // bins 0, 769..1279 (port A: 0, 769..1023; port B: 1024..1279)
            wa_en[bk_x] = 1; a_adr[bk_x] = (cnt == 10'd0) ? 11'd0 : 11'd768 + {1'b0, cnt};
            wb_en[bk_x] = 1; b_adr[bk_x] = 11'd1024 + {1'b0, cnt};
        end
        if (st == S_LOAD && have_word) begin           // carriers 16 (wcnt-1) + 2 wsub (+1)
            wa_en[bk_x] = 1; a_adr[bk_x] = binof({wcnt - 7'd1, wsub, 1'b0});
            wa_re[bk_x] = cre({word[4 * wsub +: 2], 1'b0} + l8); wa_im[bk_x] = cim({word[4 * wsub +: 2], 1'b0} + l8);
            wb_en[bk_x] = 1; b_adr[bk_x] = binof({wcnt - 7'd1, wsub, 1'b1});
            wb_re[bk_x] = cre({word[4 * wsub + 2 +: 2], 1'b0} + l8); wb_im[bk_x] = cim({word[4 * wsub + 2 +: 2], 1'b0} + l8);
        end
        if (st == S_STAGE) begin a_adr[bsrc] = bi; b_adr[bsrc] = bi + h; end
        if (q3_v) begin
            wa_en[bdst] = 1; a_adr[bdst] = q3_i; wa_re[bdst] = sat18(q3_sr); wa_im[bdst] = sat18(q3_si);
            wb_en[bdst] = 1; b_adr[bdst] = q3_j; wb_re[bdst] = sat18(q3_tr); wb_im[bdst] = sat18(q3_ti);
        end
    end

    always @(posedge clk) begin
        f_rd <= 0;
        if (rst) begin
            st <= S_IDLE; sym_l <= 0; ready <= 0; have_word <= 0; bk_out <= 2'd2; bk_x <= 2'd0; bk_y <= 2'd1;
        end else begin
            case (st)
            S_IDLE: if (!ready && f_cnt >= 14'd96) begin st <= S_ZERO; cnt <= 0; end
            S_ZERO: begin cnt <= cnt + 10'd1; if (cnt == 10'd255) begin st <= S_LOAD; wcnt <= 0; wsub <= 0; have_word <= 0; f_rd <= 1; end end
            S_LOAD: begin
                if (f_q_valid) begin word <= f_q; have_word <= 1; wcnt <= wcnt + 7'd1; wsub <= 0; end
                else if (have_word) begin
                    wsub <= wsub + 3'd1;
                    if (wsub == 3'd6 && wcnt != 7'd96) f_rd <= 1;
                    if (wsub == 3'd7) begin
                        have_word <= 0;
                        if (wcnt == 7'd96) begin st <= S_STAGE; stage <= 0; cnt <= 0; src_y <= 0; scaled <= 1; end
                    end
                end
            end
            S_STAGE: begin cnt <= cnt + 10'd1; if (cnt == 10'd1023) begin st <= S_DRAIN; drain <= 0; end end
            S_DRAIN: begin
                drain <= drain + 4'd1;
                if (drain == 4'd6) begin
                    if (stage == 4'd10) st <= S_DONE;
                    else begin stage <= stage + 4'd1; cnt <= 0; st <= S_STAGE; src_y <= ~src_y; scaled <= (stage + 4'd1) < 4'd6; end
                end
            end
            S_DONE: begin ready <= 1; st <= S_IDLE; sym_l <= (sym_l == 7'd75) ? 7'd0 : sym_l + 7'd1; end
            default: st <= S_IDLE;
            endcase
            if (take_req) begin ready <= 0; bk_out <= bk_y; bk_x <= bk_out; bk_y <= bk_x; end
        end
    end

    // butterfly cnt of stage s pairs bins bi and bi + h (h = 1024 >> s) with twiddle index m = (cnt mod h) << s. In stage 0, h[9:0]
    // is 0 (1024 does not fit in 10 bits), so h[9:0] - 1 wraps to 10'h3FF, which is exactly the mask needed there.
    always @(posedge clk) begin
        q0_v <= (st == S_STAGE); q0_i <= bi; q0_j <= bi + h; q0_m <= (cnt & (h[9:0] - 10'd1)) << stage;
        // T+1: RAM data of q0 has arrived
        q1_v <= q0_v; q1_i <= q0_i; q1_j <= q0_j;
        {q1_wr, q1_wi} <= c_tw(q0_m);
        q1_sr <= sel3(bsrc, ra_re0, ra_re1, ra_re2) + sel3(bsrc, rb_re0, rb_re1, rb_re2);
        q1_si <= sel3(bsrc, ra_im0, ra_im1, ra_im2) + sel3(bsrc, rb_im0, rb_im1, rb_im2);
        q1_dr <= sel3(bsrc, ra_re0, ra_re1, ra_re2) - sel3(bsrc, rb_re0, rb_re1, rb_re2);
        q1_di <= sel3(bsrc, ra_im0, ra_im1, ra_im2) - sel3(bsrc, rb_im0, rb_im1, rb_im2);
        // T+2: products
        q2_v <= q1_v; q2_i <= q1_i; q2_j <= q1_j; q2_sr <= q1_sr; q2_si <= q1_si;
        q2_m1 <= q1_dr * q1_wr; q2_m2 <= q1_di * q1_wi; q2_m3 <= q1_dr * q1_wi; q2_m4 <= q1_di * q1_wr;
        // T+3: combine, round, scale (the write follows combinationally from q3)
        q3_v <= q2_v; q3_i <= q2_i; q3_j <= q2_j;
        begin : comb
            reg signed [40:0] tr, ti, sr, si;
            tr = ($signed({{4{q2_m1[36]}}, q2_m1}) - $signed({{4{q2_m2[36]}}, q2_m2}) + 41'sd32768) >>> 16;
            ti = ($signed({{4{q2_m3[36]}}, q2_m3}) + $signed({{4{q2_m4[36]}}, q2_m4}) + 41'sd32768) >>> 16;
            sr = q2_sr; si = q2_si;
            if (scaled) begin tr = (tr + 41'sd1) >>> 1; ti = (ti + 41'sd1) >>> 1; sr = (sr + 41'sd1) >>> 1; si = (si + 41'sd1) >>> 1; end
            q3_tr <= tr; q3_ti <= ti; q3_sr <= sr; q3_si <= si;
        end
    end

    // ---------------------------------------------------------------- 12-clock schedule: 2 samples in (2.048 MS/s), 3 out (3.072 MS/s)
    reg [3:0] c12 = 0;
    always @(posedge clk) c12 <= rst ? 4'd0 : (c12 == 4'd11 ? 4'd0 : c12 + 4'd1);
    wire x_tick = (c12 == 4'd0) || (c12 == 4'd6);

    // ---------------------------------------------------------------- play-out at 2.048 MS/s
    reg        in_null = 1, running = 0;
    reg [11:0] pos = 0;
    reg [6:0]  out_l = 0;
    reg        x_fetch = 0, x_zero = 0, x_fetch_d = 0, x_zero_d = 0, x_new = 0;
    reg [1:0]  rd_bank = 0;
    reg signed [15:0] xr = 0, xi = 0;
    wire signed [17:0] o_re = sel3(rd_bank, ra_re0, ra_re1, ra_re2), o_im = sel3(rd_bank, ra_im0, ra_im1, ra_im2);
    always @(posedge clk) begin
        x_fetch <= 0; x_zero <= 0; take_req <= 0;
        if (rst) begin in_null <= 1; running <= 0; pos <= 0; out_l <= 0; underflow <= 0; end
        else if (x_tick) begin
            if (!running) begin
                x_zero <= 1;
                if (ready && enable) begin running <= 1; in_null <= 1; pos <= 0; end
            end else if (in_null) begin
                x_zero <= 1;
                if (pos == TNULL - 1) begin
                    pos <= 0; in_null <= 0; out_l <= 0;
                    if (ready) take_req <= 1; else underflow <= 1;
                end else pos <= pos + 12'd1;
            end else begin
                out_adr <= bitrev11(pos < TG ? pos + 12'd1544 : pos - 12'd504);
                x_fetch <= 1;
                if (pos == TG + N - 1) begin
                    pos <= 0;
                    if (out_l == 7'd75) in_null <= 1;
                    else begin out_l <= out_l + 7'd1; if (ready) take_req <= 1; else underflow <= 1; end
                end else pos <= pos + 12'd1;
            end
        end
    end
    always @(posedge clk) begin
        x_fetch_d <= x_fetch; x_zero_d <= x_zero; x_new <= x_fetch_d | x_zero_d;
        if (x_fetch) rd_bank <= bk_out;              // the bank of this read (the roles may rotate right after)
        if (x_zero_d) begin xr <= 0; xi <= 0; end
        else if (x_fetch_d) begin
            xr <= sat16(($signed({{23{o_re[17]}}, o_re}) + 41'sd2) >>> 2);
            xi <= sat16(($signed({{23{o_im[17]}}, o_im}) + 41'sd2) >>> 2);
        end
        dbg_x_valid <= x_new; dbg_x_re <= xr; dbg_x_im <= xi;
    end

    // ---------------------------------------------------------------- 3/2 interpolator (144 taps, 48 per phase)
    // The filter keeps the carriers flat to 768 kHz and stops from 900 kHz: it removes the images and also the sidelobes of the
    // rectangular OFDM symbols around the ensemble (the "shoulders").
    `include "skypluto_dab_interp_coefs.vh"        // function signed [17:0] c_int(input integer k) (generated)
    localparam integer TPP = 48;                    // taps per phase
    reg signed [15:0] dl_r [0:TPP-1], dl_i [0:TPP-1];   // the last 48 input samples, dl[0] newest
    reg signed [15:0] sn_r [0:TPP-1], sn_i [0:TPP-1];   // snapshot for the output being computed
    initial for (ii = 0; ii < TPP; ii = ii + 1) begin dl_r[ii] = 0; dl_i[ii] = 0; sn_r[ii] = 0; sn_i[ii] = 0; end
    always @(posedge clk) if (x_new) begin : shift
        integer k;
        for (k = TPP - 1; k > 0; k = k - 1) begin dl_r[k] <= dl_r[k - 1]; dl_i[k] <= dl_i[k - 1]; end
        dl_r[0] <= xr; dl_i[0] <= xi;
    end
    // the inputs enter dl at the end of c12 = 3 and 9; outputs r = 0, 1, 2 (phases 0, 2, 1) start at c12 = 4, 8, 0 with dl[0]
    // holding their base sample x[2p], x[2p], x[2p+1]; each takes 4 clocks with 12 taps per clock, as two registered sums of 6
    // (the same depth per clock as 6 taps) that are accumulated one clock later
    reg        mac_act = 0;
    reg [1:0]  mac_c = 0, mac_ph = 0;
    reg        p_val = 0, p_first = 0, p_last = 0;  // the pipeline stage: partial sums of this clock's 12 products
    reg signed [39:0] pa_r = 0, pb_r = 0, pa_i = 0, pb_i = 0;
    reg signed [39:0] acc_r = 0, acc_i = 0;
    always @(posedge clk) begin : mac
        integer k, t;
        reg signed [39:0] sar, sbr, sai, sbi, nr, ni;
        if (c12 == 4'd4 || c12 == 4'd8 || c12 == 4'd0) begin
            for (k = 0; k < TPP; k = k + 1) begin sn_r[k] <= dl_r[k]; sn_i[k] <= dl_i[k]; end
            mac_ph <= (c12 == 4'd4) ? 2'd0 : (c12 == 4'd8) ? 2'd2 : 2'd1;
            mac_act <= 1; mac_c <= 0;
        end
        // stage 1: 12 products of this clock, as two sums of 6
        p_val <= mac_act; p_first <= mac_act && mac_c == 2'd0; p_last <= mac_act && mac_c == 2'd3;
        if (mac_act) begin
            sar = 0; sbr = 0; sai = 0; sbi = 0;
            for (k = 0; k < 6; k = k + 1) begin
                t = 12 * mac_c + k;
                sar = sar + c_int(mac_ph + 3 * t) * sn_r[t];
                sai = sai + c_int(mac_ph + 3 * t) * sn_i[t];
                sbr = sbr + c_int(mac_ph + 3 * (t + 6)) * sn_r[t + 6];
                sbi = sbi + c_int(mac_ph + 3 * (t + 6)) * sn_i[t + 6];
            end
            pa_r <= sar; pb_r <= sbr; pa_i <= sai; pb_i <= sbi;
            mac_c <= mac_c + 2'd1;
            if (mac_c == 2'd3) mac_act <= (c12 == 4'd4 || c12 == 4'd8 || c12 == 4'd0);
        end
        // stage 2: accumulate; the 4th step of an output gives the sample
        if (p_val) begin
            nr = (p_first ? 40'sd0 : acc_r) + pa_r + pb_r;
            ni = (p_first ? 40'sd0 : acc_i) + pa_i + pb_i;
            acc_r <= nr; acc_i <= ni;
            if (p_last) begin
                i_out <= enable ? sat16((nr + 40'sd65536) >>> 17) : 16'sd0;
                q_out <= enable ? sat16((ni + 40'sd65536) >>> 17) : 16'sd0;
            end
        end
    end
endmodule

// true dual-port RAM, 2048 x 36, registered reads (Xilinx block RAM template: one always block per port)
module dab_tdp_ram (
    input  wire        clk,
    input  wire        wea, input wire [10:0] addra, input wire [35:0] dia, output reg [35:0] doa = 0,
    input  wire        web, input wire [10:0] addrb, input wire [35:0] dib, output reg [35:0] dob = 0
);
    reg [35:0] mem [0:2047];
    integer k;
    initial for (k = 0; k < 2048; k = k + 1) mem[k] = 36'd0;
    always @(posedge clk) begin if (wea) mem[addra] <= dia; doa <= mem[addra]; end
    always @(posedge clk) begin if (web) mem[addrb] <= dib; dob <= mem[addrb]; end
endmodule
