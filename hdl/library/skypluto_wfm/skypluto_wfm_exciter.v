// =============================================================================
// skypluto_wfm_exciter - WFM exciter with AXI-Lite control
// -----------------------------------------------------------------------------
//   I2S-RX (bclk) -> async FIFO (CDC) -> conditioner -> interpolator (l_clk) -> FM modulator
//                 -> i_out/q_out  (directly to axi_ad9361 dac_data_i0/q0)
//   + skypluto_axi_regs: enable / freq_offset (zero-IF) / carrier_level / kdev
//
// The carrier is ALWAYS on (even without I2S): at comp=0 the phase keeps running
// with freq_offset. Level and offset are adjustable at runtime via AXI (ssh/devmem).
//
// dac_valid_i0/dac_enable_i0 are OUTPUTS of the core -> not produced here.
// In 1R1T the modulator delivers IQ continuously on l_clk.
// =============================================================================
`timescale 1ns/1ps

module skypluto_wfm_exciter #(
    parameter integer DW         = 24,
    parameter integer PHASE_W    = 24,
    parameter integer LUT_ADDR_W = 14,
    parameter integer OUT_W      = 16,
    parameter integer KDEV_W     = 18,
    parameter integer KSHIFT     = 13,
    parameter integer LVL_W      = 16,
    parameter integer DC_W       = 12,
    parameter integer FRAC       = 24,
    parameter integer FIFO_AW    = 6,
    parameter        [FRAC-1:0] STEP = 24'd262400,    // 192,187.5 Hz -> l_clk. Confirmed on the radio: audio pitch
                                                       // is CORRECT with this value.
    parameter         LUT_FILE   = "sine_lut.mem"
)(
    input  wire                 l_clk,
    input  wire                 l_clk_resetn,
    // I2S-in (bank 13, 3.3V) - Pluto = slave
    input  wire                 i2s_in_bclk,
    input  wire                 i2s_in_lrclk,
    input  wire                 i2s_in_data,
    // DAC injection
    input  wire                 dac_enable_i0,
    output wire signed [OUT_W-1:0] i_out,
    output wire signed [OUT_W-1:0] q_out,
    // AXI4-Lite control (s_axi_aclk = sys_cpu_clk)
    input  wire                 s_axi_aclk,
    input  wire                 s_axi_aresetn,
    input  wire [ 6:0]          s_axi_awaddr,
    input  wire                 s_axi_awvalid,
    output wire                 s_axi_awready,
    input  wire [31:0]          s_axi_wdata,
    input  wire [ 3:0]          s_axi_wstrb,
    input  wire                 s_axi_wvalid,
    output wire                 s_axi_wready,
    output wire [ 1:0]          s_axi_bresp,
    output wire                 s_axi_bvalid,
    input  wire                 s_axi_bready,
    input  wire [ 6:0]          s_axi_araddr,
    input  wire                 s_axi_arvalid,
    output wire                 s_axi_arready,
    output wire [31:0]          s_axi_rdata,
    output wire [ 1:0]          s_axi_rresp,
    output wire                 s_axi_rvalid,
    input  wire                 s_axi_rready,
    // RX1 timestamp channels (impulse measurement): the RX1 output of the RX decimator is routed through here to the cpack. With IMP_CTRL[0] = 1 the RX1 channels carry
    // (I = l_clk counter low 16 bits at the moment of the RX sample, Q = l_clk counter low 16 bits of the last detected impulse); otherwise pass-through.
    input  wire signed [15:0]   rx1_i_in,
    input  wire signed [15:0]   rx1_q_in,
    output wire signed [15:0]   rx1_i_out,
    output wire signed [15:0]   rx1_q_out
);

    // ---- reset-synchronizers --------------------------------------------------
    reg [1:0] lrst_sync = 2'b11;
    always @(posedge l_clk or negedge l_clk_resetn)
        if (!l_clk_resetn) lrst_sync <= 2'b11; else lrst_sync <= {lrst_sync[0], 1'b0};
    wire l_rst = lrst_sync[1];

    reg [1:0] brst_sync = 2'b11;
    always @(posedge i2s_in_bclk or negedge l_clk_resetn)
        if (!l_clk_resetn) brst_sync <= 2'b11; else brst_sync <= {brst_sync[0], 1'b0};
    wire b_rst = brst_sync[1];

    // ---- AXI registers (s_axi_aclk domain) ------------------------------------
    wire                     en_axi;
    wire signed [PHASE_W-1:0] offset_axi;
    wire        [LVL_W-1:0]   level_axi;
    wire signed [KDEV_W-1:0]  kdev_axi;

    wire signed [DC_W-1:0] dci_axi, dcq_axi;
    // conditioner (limiter + fade): settings (axi domain) and status (synchronized into the axi domain)
    wire [23:0] lim_ceil_axi; wire lim_en_axi, fade_en_axi, clr_tog_axi;
    wire [31:0] st_gmin_a, st_events_a, st_inpeak_a, st_state_a, st_uf_a;
    wire [4:0]  dbg_sel_a;  wire [31:0] st_dbg_a;
    wire imp_en_axi; wire [23:0] imp_thr_axi; wire [31:0] st_imp_stamp_a, st_imp_count_a; wire [31:0] st_fmt_a;
    skypluto_axi_regs #(.PHASE_W(PHASE_W), .LVL_W(LVL_W), .KDEV_W(KDEV_W), .DC_W(DC_W)) u_regs (
        .s_axi_aclk(s_axi_aclk), .s_axi_aresetn(s_axi_aresetn),
        .s_axi_awaddr(s_axi_awaddr), .s_axi_awvalid(s_axi_awvalid), .s_axi_awready(s_axi_awready),
        .s_axi_wdata(s_axi_wdata), .s_axi_wstrb(s_axi_wstrb), .s_axi_wvalid(s_axi_wvalid), .s_axi_wready(s_axi_wready),
        .s_axi_bresp(s_axi_bresp), .s_axi_bvalid(s_axi_bvalid), .s_axi_bready(s_axi_bready),
        .s_axi_araddr(s_axi_araddr), .s_axi_arvalid(s_axi_arvalid), .s_axi_arready(s_axi_arready),
        .s_axi_rdata(s_axi_rdata), .s_axi_rresp(s_axi_rresp), .s_axi_rvalid(s_axi_rvalid), .s_axi_rready(s_axi_rready),
        .enable(en_axi), .freq_offset(offset_axi), .carrier_level(level_axi), .kdev(kdev_axi),
        .dc_i(dci_axi), .dc_q(dcq_axi),
        .status_overflow(1'b0), .status_underflow(1'b0),
        .lim_ceil(lim_ceil_axi), .lim_en(lim_en_axi), .fade_en(fade_en_axi), .lim_clr_tog(clr_tog_axi),
        .st_gmin(st_gmin_a), .st_events(st_events_a), .st_inpeak(st_inpeak_a), .st_state(st_state_a), .st_uf(st_uf_a),
        .dbg_sel(dbg_sel_a), .st_dbg(st_dbg_a),
        .i2s_ctrl(i2s_ctrl_axi), .imp_en(imp_en_axi), .imp_thr(imp_thr_axi), .st_imp_stamp(st_imp_stamp_a), .st_imp_count(st_imp_count_a), .st_fmt(st_fmt_a)
    );

    // ---- CDC of control to l_clk (quasi-static; double flop) ------------------
    (* ASYNC_REG = "TRUE" *) reg                      en_s0=0, en_s1=0;
    (* ASYNC_REG = "TRUE" *) reg signed [PHASE_W-1:0] off_s0=0, off_sr=0;  reg signed [PHASE_W-1:0] off_s1=0;
    // Multi-bit values (offset, level, kdev, dc, ceiling): s0 = first flop, sr = second, s1 = output. s1 only takes over a value when two
    // consecutive samples are equal (s0 == sr): a 'torn' value (bits of the previous and the next value due to routing skew) is present for only 1 sample
    // and is thus never passed on (e.g. a ceiling that is briefly 0 or full scale).
    (* ASYNC_REG = "TRUE" *) reg        [LVL_W-1:0]   lvl_s0=0, lvl_sr=0;  reg [LVL_W-1:0]   lvl_s1=0;
    (* ASYNC_REG = "TRUE" *) reg signed [KDEV_W-1:0]  kd_s0=0,  kd_sr=0;   reg signed [KDEV_W-1:0] kd_s1=0;
    (* ASYNC_REG = "TRUE" *) reg signed [DC_W-1:0]    dci_s0=0, dci_sr=0, dcq_s0=0, dcq_sr=0;  reg signed [DC_W-1:0] dci_s1=0, dcq_s1=0;
    (* ASYNC_REG = "TRUE" *) reg [23:0] lc_s0=24'h733333, lc_sr=24'h733333;  reg [23:0] lc_s1=24'h733333;
    (* ASYNC_REG = "TRUE" *) reg        le_s0=1, le_s1=1, fe_s0=1, fe_s1=1, ct_s0=0, ct_s1=0;
    reg ct_s2 = 0;
    wire lim_clr = ct_s1 ^ ct_s2;                    // pulse on every toggle of 'clear statistics'
    always @(posedge l_clk) begin
        lc_s0<=lim_ceil_axi; lc_sr<=lc_s0; if (lc_s0 == lc_sr) lc_s1<=lc_s0; le_s0<=lim_en_axi; le_s1<=le_s0; fe_s0<=fade_en_axi; fe_s1<=fe_s0;
        ct_s0<=clr_tog_axi; ct_s1<=ct_s0; ct_s2<=ct_s1;
    end
    always @(posedge l_clk) begin
        en_s0<=en_axi;   en_s1<=en_s0;
        off_s0<=offset_axi; off_sr<=off_s0; if (off_s0 == off_sr) off_s1<=off_s0;
        lvl_s0<=level_axi;  lvl_sr<=lvl_s0; if (lvl_s0 == lvl_sr) lvl_s1<=lvl_s0;
        kd_s0<=kdev_axi;    kd_sr<=kd_s0;   if (kd_s0  == kd_sr)  kd_s1<=kd_s0;
        dci_s0<=dci_axi; dci_sr<=dci_s0; if (dci_s0 == dci_sr) dci_s1<=dci_s0;
        dcq_s0<=dcq_axi; dcq_sr<=dcq_s0; if (dcq_s0 == dcq_sr) dcq_s1<=dcq_s0;
    end

    // ---- I2S-RX ---------------------------------------------------------------
    wire signed [DW-1:0] i2s_left, i2s_right;
    wire                 i2s_valid;
    wire [7:0] i2s_fmt_slot, i2s_fmt_bits; wire [1:0] i2s_fmt_mode;
    wire [15:0] i2s_ctrl_axi; reg [15:0] ic_b0 = 0, ic_b1 = 0;
    always @(posedge i2s_in_bclk) begin ic_b0 <= i2s_ctrl_axi; ic_b1 <= ic_b0; end    // static; 2 flip-flops suffice
    skypluto_i2s_rx #(.DATA_W(DW), .WS_TO_MSB(1)) u_i2s (
        .bclk(i2s_in_bclk), .ws(i2s_in_lrclk), .sd(i2s_in_data),
        .left(i2s_left), .right(i2s_right), .valid(i2s_valid),
        .man_en(ic_b1[0]), .man_mode(ic_b1[2:1]), .man_bits(ic_b1[15:8]),
        .fmt_slot(i2s_fmt_slot), .fmt_bits(i2s_fmt_bits), .fmt_mode(i2s_fmt_mode)
    );
    (* ASYNC_REG = "TRUE" *) reg [17:0] if0 = 0, if1 = 0;               // measured I2S format to the axi domain (static enough; status only)
    always @(posedge s_axi_aclk) begin if0 <= {i2s_fmt_mode, i2s_fmt_slot, i2s_fmt_bits}; if1 <= if0; end

    // ---- CDC-FIFO -------------------------------------------------------------
    wire                 fifo_full, fifo_empty, sc_up_rd, ups_fifo_rd;
    // USE_SC = 1: the conditioner (skypluto_sigcond: soft fade + limiter) sits between FIFO and interpolator.
    // USE_SC = 0 (fallback): conditioner bypassed, the interpolator reads directly from the FIFO; the status register SC_STATE (0x34) then reads
    // '3' in bits [1:0], so that the daemon knows there is NO limiter active (the saturation in the interpolator works independently of this).
    localparam integer USE_SC = 1;
    wire signed [DW-1:0] fifo_rdata;
    wire [FIFO_AW:0]     fifo_count, fifo_wrpos;
    skypluto_async_fifo #(.DW(DW), .AW(FIFO_AW)) u_fifo (
        .wr_clk(i2s_in_bclk), .wr_rst(b_rst), .wr_en(i2s_valid & ~fifo_full),
        .wr_data(i2s_left), .wr_full(fifo_full),
        .rd_clk(l_clk), .rd_rst(l_rst), .rd_en(USE_SC ? sc_up_rd : ups_fifo_rd),
        .rd_data(fifo_rdata), .rd_empty(fifo_empty), .rd_count(fifo_count), .wr_pos(fifo_wrpos)
    );

    // ---- conditioner: soft switch-on/off + look-ahead limiter (between FIFO and interpolator) -------------
    wire signed [DW-1:0] sc_q_data;
    wire                 sc_q_empty;
    wire [16:0]          sc_gmin, sc_fade;
    wire [31:0]          sc_events, sc_uf;
    wire [DW-1:0]        sc_inpeak;
    wire [1:0]           sc_state;
    wire [31:0]          sc_dbg_pulls, sc_dbg_pops, sc_dbg_nz, sc_dbg_state, sc_dbg_wd, sc_dbg_gain, sc_dbg_sum;
    skypluto_sigcond #(.DW(DW), .TGT(1 << (FIFO_AW - 1))) u_sc (
        .clk(l_clk), .rst(l_rst),
        .up_data(fifo_rdata), .up_empty(fifo_empty), .up_count(fifo_count), .up_wrpos(fifo_wrpos), .up_rd(sc_up_rd),
        .q_data(sc_q_data), .q_empty(sc_q_empty), .q_rd(USE_SC ? ups_fifo_rd : 1'b0),
        .ceil(lc_s1), .lim_en(le_s1), .fade_en(fe_s1), .clr(lim_clr),
        .gmin(sc_gmin), .events(sc_events), .inpeak(sc_inpeak), .state(sc_state), .uf_events(sc_uf), .fade_o(sc_fade),
        .dbg_pulls(sc_dbg_pulls), .dbg_pops(sc_dbg_pops), .dbg_nz(sc_dbg_nz), .dbg_state(sc_dbg_state), .dbg_wd(sc_dbg_wd), .dbg_gain(sc_dbg_gain), .dbg_sum(sc_dbg_sum)
    );
    // status to the axi domain (2-flop; software reads twice for counters)
    (* ASYNC_REG = "TRUE" *) reg [31:0] sg0=0, sg1=0, se0=0, se1=0, sp0=0, sp1=0, ss0=0, ss1=0, su0=0, su1=0;
    always @(posedge s_axi_aclk) begin
        sg0<={15'd0, sc_gmin};             sg1<=sg0;
        se0<=sc_events;                    se1<=se0;
        sp0<={8'd0, sc_inpeak};            sp1<=sp0;
        ss0<={sc_fade, 13'd0, (USE_SC ? sc_state : 2'd3)};   ss1<=ss0;
        su0<=sc_uf;                        su1<=su0;
    end
    assign st_gmin_a = sg1; assign st_events_a = se1; assign st_inpeak_a = sp1; assign st_state_a = ss1; assign st_uf_a = su1;

    // ---- interpolator (band-limiting polyphase + adaptive STEP) ---------------
    // image@116kHz -73 dB (SM.1268), no droop, rate tracking.
    // Pulls its samples from the conditioner (q_*) rather than directly from the FIFO; the FIFO occupancy
    // (fifo_count) is the input of the rate loop.
    wire signed [DW-1:0] ups_comp;
    wire                 ups_valid;
    wire [31:0]          ups_sat, ups_ovf, ups_take, ups_dbg_state;
    skypluto_interp #(.DW(DW), .FRAC(FRAC), .CNTW(FIFO_AW+1), .STEP_NOM(STEP)) u_ups (
        .clk(l_clk), .rst(l_rst), .step(STEP),
        .fifo_data(USE_SC ? sc_q_data : fifo_rdata), .fifo_empty(USE_SC ? sc_q_empty : fifo_empty), .fifo_count(fifo_count),
        .fifo_rd(ups_fifo_rd), .comp_out(ups_comp), .comp_valid(ups_valid), .sat_cnt(ups_sat),
        .dbg_ovf_cnt(ups_ovf), .dbg_take_cnt(ups_take), .dbg_state(ups_dbg_state)
    );
    reg [31:0] d_hb = 0;                                       // heartbeat: free-running counter on l_clk (two reads -> l_clk frequency, and proof that the clock is running)
    always @(posedge l_clk) d_hb <= d_hb + 1'b1;

    // ---- impulse measurement (end-to-end group delay) -------------------------------------------------------------------------------
    // Detector in the I2S domain: a word with |left| > threshold (IMP_THR) toggles imp_tog_b. In the l_clk domain that is synchronized and
    // the l_clk counter (d_hb, minus 3 for the synchronization) is latched as the impulse time. The RX1 channels of the capture carry that time and the
    // running counter (see rx1_*), so that the RX2 recording itself contains the time reference. IMP_CTRL[0] = 0: everything off (RX1 pass-through).
    (* ASYNC_REG = "TRUE" *) reg ie_l0 = 0, ie_l1 = 0, ie_b0 = 0, ie_b1 = 0;
    (* ASYNC_REG = "TRUE" *) reg [23:0] it_b0 = 24'h0F5C28, it_b1 = 24'h0F5C28;
    always @(posedge l_clk)       begin ie_l0 <= imp_en_axi; ie_l1 <= ie_l0; end
    always @(posedge i2s_in_bclk) begin ie_b0 <= imp_en_axi; ie_b1 <= ie_b0; it_b0 <= imp_thr_axi; it_b1 <= it_b0; end
    wire [23:0] imp_abs_left = i2s_left[DW-1] ? ~i2s_left[23:0] : i2s_left[23:0];
    reg imp_tog_b = 0;
    always @(posedge i2s_in_bclk) if (i2s_valid && ie_b1 && (imp_abs_left > it_b1)) imp_tog_b <= ~imp_tog_b;
    (* ASYNC_REG = "TRUE" *) reg ih0 = 0, ih1 = 0; reg ih2 = 0;
    reg [31:0] imp_stamp = 0, imp_count = 0;
    always @(posedge l_clk) begin
        ih0 <= imp_tog_b; ih1 <= ih0; ih2 <= ih1;
        if (ih1 ^ ih2) begin imp_stamp <= d_hb - 32'd3; imp_count <= imp_count + 1'b1; end
    end
    assign rx1_i_out = ie_l1 ? d_hb[15:0]         : rx1_i_in;
    assign rx1_q_out = ie_l1 ? imp_stamp[15:0]    : rx1_q_in;
    (* ASYNC_REG = "TRUE" *) reg [31:0] is0 = 0, is1 = 0, ic0 = 0, ic1 = 0;
    always @(posedge s_axi_aclk) begin is0 <= imp_stamp; is1 <= is0; ic0 <= imp_count; ic1 <= ic0; end
    assign st_imp_stamp_a = is1; assign st_imp_count_a = ic1;
    assign st_fmt_a = {14'd0, if1};

    // ---- diagnostics (read-only words via 0x3C, selected with a write to 0x3C) ------------------------------
    // bclk domain: SD/WS edges, last raw word, frame counter, FIFO write counter, peak of the raw word.
    // l_clk domain: peak at the FIFO output / conditioner output / interpolator output, pulls/pops/non-zero x, FIR saturations.
    // Brought to the axi domain with a 2-flop (static enough for diagnostics; counters may tear).
    reg         d_sd_q = 0, d_ws_q = 0;
    reg  [15:0] d_sd_edges = 0, d_ws_edges = 0;
    reg  [23:0] d_last_left = 0, d_peak_b = 0;
    reg  [31:0] d_frames = 0, d_wcnt = 0;
    wire [23:0] d_abs_left = i2s_left[DW-1] ? ~i2s_left[23:0] : i2s_left[23:0];
    always @(posedge i2s_in_bclk) begin
        d_sd_q <= i2s_in_data; d_ws_q <= i2s_in_lrclk;
        if (d_sd_q != i2s_in_data)   d_sd_edges <= d_sd_edges + 1'b1;
        if (d_ws_q != i2s_in_lrclk)  d_ws_edges <= d_ws_edges + 1'b1;
        if (i2s_valid) begin
            d_last_left <= i2s_left[23:0]; d_frames <= d_frames + 1'b1;
            if (d_abs_left > d_peak_b) d_peak_b <= d_abs_left;
            if (!fifo_full) d_wcnt <= d_wcnt + 1'b1;
        end
    end
    reg [23:0] d_pk_rd = 0, d_pk_q = 0, d_pk_comp = 0;
    // the FIFO is first-word-fall-through (asynchronous read): register first for the diagnostic peak, otherwise the path (64:1 mux + abs + comparator) is too long
    reg signed [DW-1:0] fifo_rdata_q = 0;
    always @(posedge l_clk) fifo_rdata_q <= fifo_rdata;
    wire [23:0] d_abs_rd   = fifo_rdata_q[DW-1] ? ~fifo_rdata_q[23:0] : fifo_rdata_q[23:0];
    wire [23:0] d_abs_q    = sc_q_data[DW-1]   ? ~sc_q_data[23:0]   : sc_q_data[23:0];
    wire [23:0] d_abs_comp = ups_comp[DW-1]    ? ~ups_comp[23:0]    : ups_comp[23:0];
    always @(posedge l_clk) begin
        if (d_abs_rd   > d_pk_rd)   d_pk_rd   <= d_abs_rd;
        if (d_abs_q    > d_pk_q)    d_pk_q    <= d_abs_q;
        if (d_abs_comp > d_pk_comp) d_pk_comp <= d_abs_comp;
    end
    // Peak deviation after the limiter/interpolator: peak of |comp| over windows of 20 ms (245,760 l_clk at 12.288 MHz).
    // wr[0..7] = the 8 most recently closed comp peaks (wr[0] = newest) for the deviation meter (?G), words 24..31; 8 x 20 ms = 160 ms of history.
    // w_out_* = the closed window, w_seq = window sequence number. comp at the modulator input = what actually goes on the air
    // (including the FIR overshoot of the interpolator); q = output of the conditioner (after the limiter, before the interpolator).
    reg [21:0] w_cnt = 0;
    reg [23:0] w_acc_c = 0, w_out_c = 0, w_acc_q = 0, w_out_q = 0;
    reg [31:0] w_seq = 0;
    reg [23:0] wr0 = 0, wr1 = 0, wr2 = 0, wr3 = 0, wr4 = 0, wr5 = 0, wr6 = 0, wr7 = 0;
    always @(posedge l_clk) begin
        if (w_cnt == 22'd245759) begin
            wr0 <= w_acc_c; wr1 <= wr0; wr2 <= wr1; wr3 <= wr2; wr4 <= wr3; wr5 <= wr4; wr6 <= wr5; wr7 <= wr6;
            w_cnt <= 0; w_out_c <= w_acc_c; w_out_q <= w_acc_q; w_acc_c <= d_abs_comp; w_acc_q <= d_abs_q; w_seq <= w_seq + 1'b1;
        end else begin
            w_cnt <= w_cnt + 1'b1;
            if (d_abs_comp > w_acc_c) w_acc_c <= d_abs_comp;
            if (d_abs_q    > w_acc_q) w_acc_q <= d_abs_q;
        end
    end
    wire [1023:0] dbg_flat = {
        {8'd0, wr7}, {8'd0, wr6}, {8'd0, wr5}, {8'd0, wr4}, {8'd0, wr3}, {8'd0, wr2}, {8'd0, wr1}, {8'd0, wr0},   // 31..24 = comp peaks per 20 ms window (24 = newest)
        sc_dbg_sum, sc_dbg_gain,                                             // 23 = running sum, 22 = instantaneous gain gs (Q16)
        w_seq, {8'd0, w_out_q}, {8'd0, w_out_c},                             // 21 = window sequence number, 20 = peak after limiter (q), 19 = peak at modulator input (comp), 20 ms
        sc_dbg_wd, sc_dbg_state, ups_dbg_state,                              // 18 = watchdog {wd_idle,wd_fsm}, 17 = conditioner state, 16 = interp state
        ups_ovf, ups_take, d_hb,                                             // 15 = interp overflow pulses, 14 = samples taken, 13 = l_clk heartbeat
        32'hB1D00017,                                                        // 12 = build id (identifies the bitstream)
        ups_sat,                {8'd0, d_pk_comp},                           // 11 = FIR saturations, 10 = peak |interp output|
        {8'd0, d_pk_q},         sc_dbg_nz,                                   // 9 = peak |conditioner output|, 8 = x_new != 0 (count)
        sc_dbg_pops,            sc_dbg_pulls,                                // 7 = FIFO pops, 6 = pulls
        {8'd0, d_pk_rd},        {8'd0, d_peak_b},                            // 5 = peak |FIFO output| (l_clk), 4 = peak |raw I2S word| (bclk)
        d_wcnt,                 d_frames,                                    // 3 = FIFO write counter, 2 = frames
        {8'd0, d_last_left},    {d_ws_edges, d_sd_edges}                     // 1 = last raw word, 0 = {WS edges, SD edges}
    };
    (* ASYNC_REG = "TRUE" *) reg [1023:0] dbg_s0 = 0, dbg_s1 = 0;
    always @(posedge s_axi_aclk) begin dbg_s0 <= dbg_flat; dbg_s1 <= dbg_s0; end
    assign st_dbg_a = dbg_s1[{dbg_sel_a, 5'd0} +: 32];

    // ---- FM-modulator ---------------------------------------------------------
    wire mod_en = en_s1 & dac_enable_i0;
    skypluto_fm_modulator #(
        .COMP_W(DW), .KDEV_W(KDEV_W), .KSHIFT(KSHIFT), .PHASE_W(PHASE_W),
        .LUT_ADDR_W(LUT_ADDR_W), .OUT_W(OUT_W), .LVL_W(LVL_W), .DC_W(DC_W), .LUT_FILE(LUT_FILE)
    ) u_mod (
        .clk(l_clk), .rst(l_rst), .en(mod_en),
        .kdev(kd_s1), .offset_inc(off_s1), .level(lvl_s1),
        .dc_i(dci_s1), .dc_q(dcq_s1),
        .comp(ups_comp), .comp_valid(ups_valid),
        .i_out(i_out), .q_out(q_out), .iq_valid()
    );

    wire _unused = &{1'b0, i2s_right, fifo_full};

endmodule
