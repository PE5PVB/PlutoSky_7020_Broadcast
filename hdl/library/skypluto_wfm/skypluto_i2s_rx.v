// =============================================================================
// skypluto_i2s_rx - I2S slave receiver with AUTOMATIC format detection
// -----------------------------------------------------------------------------
// Runs in the EXTERNAL BCLK domain (Pluto = slave). Delivers one left and one right sample per stereo frame (MSB-first). The WFM exciter uses 'left' as
// the real composite signal. WS=0 -> left channel, WS=1 -> right channel; WS changes on the falling BCLK edge, data is sampled on the rising edge.
//
// SUPPORTED VARIANTS (192 kHz; other rates too as long as the exciter's rate loop tracks it):
//   - alignment: standard I2S (MSB 1 BCLK after the WS edge), left-justified (MSB on the WS edge) and right-justified (LSB in the last BCLK of the slot)
//   - word width 8..32 bit (the top 24 bits are used); 16/20/24/32-bit data in slots of 16/24/32/48/64 BCLKs (BCLK = 32/48/64/96/128 x fs)
//   - slot length (BCLKs per WS half period) and alignment are detected AUTOMATICALLY; can be overridden manually via man_* (register I2S_CTRL).
//
// DETECTION (from bit activity only, no alignment information needed):
//   * Slot length L: number of BCLKs between two WS edges; two equal measurements in a row are accepted.
//   * For each BCLK position p in the slot (p = 0 on the cycle of the WS edge, then 1, 2, ...) it is tracked whether a '1' ever appeared on the data (act[p]).
//     Padding is always zero; the positions of the first and last active bit (s, e) give the alignment:
//        s = 1                      -> standard I2S, MSB at p = 1, N = e - s + 1 bits
//        s = 0, e < L-1             -> left-justified, MSB at p = 0, N = e + 1
//        s >= 2                     -> right-justified, MSB at p = s, N = L - s
//        s = 0, e = L-1 (slot full) -> I2S or left-justified without padding: decided by the agreement between the bit at p=0 and p=1 (sign extension in
//                                      LJ: MSB = next bit; in I2S p=0 is the LSB of the previous word: independent). Default: I2S.
//     RIGHT-JUSTIFIED WITH SIGN EXTENSION (upper bits of the slot = copy of the sign bit, data in the low bits): also s = 0 and e = L-1. Distinguished from
//        LJ via dis[]: position p 'ever differs' from the bit at p = 0. With RJ-with-extension the first deviation comes only after (L - D + 1) positions (R >= 6); with
//        LJ audio already after 1..4 positions. The word width is then D = the smallest standard width (16/20/24/32) >= L + 1 - R (the first data bits that are also
//        equal to the sign make R slightly larger; this holds down to about -24 dBFS signal). Only a completely filled slot is ambiguous in this way.
//     Every 2^EVAL_LOG2 slots (default 65536 = 170 ms at 192 kHz stereo) an evaluation is made; the format only changes after two equal
//     evaluations. On silence (all zero) the format is kept. The window must be long: a window in which every sample is POSITIVE has a sign
//     bit that is never 1, which looks exactly like a padded/right-justified word one or more bits later. A low bass tone (20-30 Hz) keeps
//     the composite positive for ~10 ms around each crest; with windows of 2048 slots (5.3 ms) that flipped the format to right-justified
//     and back, with gross errors on the samples in between. 170 ms contains a negative half period of anything above ~3 Hz.
//   * The word is LEFT-ALIGNED to 24 bit (16-bit data << 8): full scale, and hence deviation, stay the same.
// =============================================================================
`timescale 1ns/1ps

module skypluto_i2s_rx #(
    parameter integer DATA_W    = 24,  // output width per channel
    parameter integer WS_TO_MSB = 1,   // (initial format only) BCLKs from WS edge to MSB
    parameter integer EVAL_LOG2 = 16   // format evaluation window: 2^EVAL_LOG2 slots (11..16)
)(
    input  wire                      bclk,   // I2S bit clock (external) - clock
    input  wire                      ws,     // word select / LRCLK
    input  wire                      sd,     // serial data
    input  wire                      man_en,   // 1 = manual format (man_mode/man_bits) instead of automatic
    input  wire [1:0]                man_mode, // 0 = I2S, 1 = left-justified, 2 = right-justified
    input  wire [7:0]                man_bits, // word width in manual mode
    output reg  signed [DATA_W-1:0]  left,
    output reg  signed [DATA_W-1:0]  right,
    output reg                       valid,    // 1-BCLK pulse on a complete stereo frame
    output reg  [7:0]                fmt_slot, // measured slot length (BCLKs per WS half period); 0 = not yet measured
    output reg  [7:0]                fmt_bits, // word width in use
    output reg  [1:0]                fmt_mode  // alignment in use: 0 = I2S, 1 = left-justified, 2 = right-justified
);

    reg                     ws_d   = 1'b0;
    reg signed [DATA_W-1:0] shreg  = {DATA_W{1'b0}};
    reg [7:0]               bitpos = 8'd0;   // BCLKs since the last WS edge (at the next edge = the slot length)

    wire ws_edge = (ws != ws_d);

    // ---- slot length ----
    reg [7:0] l_prev = 8'd0;
    reg [7:0] slot_l = 8'd32;                // valid slot length (default 32)

    // ---- active format: position of the MSB (f_m) and number of bits (f_n) ----
    reg [7:0] f_m = WS_TO_MSB;
    reg [7:0] f_n = DATA_W;
    wire lsb_at_edge = (f_m >= 8'd1) && ((f_m + f_n) == (slot_l + 8'd1));    // the LSB falls on the WS edge of the next slot (standard I2S with full slots)
    wire signed [DATA_W-1:0] word_w = lsb_at_edge ? {shreg[DATA_W-2:0], sd} : shreg;
    wire signed [DATA_W-1:0] word_a = word_w <<< (DATA_W - f_n);              // left-align to DATA_W bit

    // ---- bit activity per position ----
    reg [63:0]  act     = 64'd0;
    reg [16:0]  a_slots = 17'd0;
    reg [16:0]  agree   = 17'd0;             // slots in which the bit at p=0 equalled the bit at p=1
    localparam [16:0] AG_HI = 17'd1700 << (EVAL_LOG2 - 11), AG_LO = 17'd1100 << (EVAL_LOG2 - 11);   // 83 % / 54 % of the window
    reg         p0b     = 1'b0;
    reg [63:0]  dis     = 64'd0;             // position p has ever had a different bit than position 0 (p0b)
    reg         rjx     = 1'b0;              // current format = right-justified with sign extension
    reg         cand_x  = 1'b0;
    reg [7:0]   cand_m  = 8'd1, cand_n = 8'd24;   // candidate from the previous evaluation

    // position of the first and last active bit (within the slot) and the decision
    reg [7:0]   s_f, e_f, m_c, n_c, r_f, dp_f;
    reg         any_f, x_c;
    integer     t_f;
    reg [63:0]  actm;
    integer     ii;
    always @* begin
        actm = act;
        for (ii = 0; ii < 64; ii = ii + 1) if (ii >= slot_l) actm[ii] = 1'b0;
        s_f = 8'd0; e_f = 8'd0; any_f = 1'b0;
        for (ii = 63; ii >= 0; ii = ii - 1) if (actm[ii]) begin s_f = ii; any_f = 1'b1; end
        for (ii = 0; ii < 64; ii = ii + 1) if (actm[ii]) e_f = ii;
        r_f = slot_l;
        for (ii = 63; ii >= 1; ii = ii - 1) if (ii < slot_l && dis[ii]) r_f = ii;
        t_f = slot_l + 1 - r_f;
        dp_f = (t_f <= 16) ? 8'd16 : (t_f <= 20) ? 8'd20 : (t_f <= 24) ? 8'd24 : 8'd32;
        if (dp_f > slot_l) dp_f = slot_l;
        x_c = 1'b0;
        m_c = f_m; n_c = f_n;
        if (any_f) begin
            if (s_f == 8'd0 && e_f == slot_l - 8'd1 && r_f >= (rjx ? 8'd4 : 8'd6) && r_f < slot_l) begin   // full slot, long sign run: RJ with sign extension
                x_c = 1'b1;
                m_c = slot_l - dp_f;
                n_c = (dp_f > DATA_W) ? DATA_W : dp_f;
            end else if (s_f == 8'd0 && e_f == slot_l - 8'd1) begin               // full slot: I2S or LJ?
                m_c = (agree > AG_HI) ? 8'd0 : (agree < AG_LO) ? 8'd1 : ((f_m == 8'd0) ? 8'd0 : 8'd1);   // >83 %: LJ, <54 %: I2S, in between: keep (hysteresis)
                n_c = (slot_l > DATA_W) ? DATA_W : slot_l;
            end else if (s_f == 8'd0) begin                             // left-justified with padding
                m_c = 8'd0;
                n_c = (e_f + 8'd1 > DATA_W) ? DATA_W : e_f + 8'd1;
            end else if (s_f == 8'd1) begin                             // standard I2S with padding
                m_c = 8'd1;
                n_c = (e_f > DATA_W) ? DATA_W : e_f;
            end else begin                                              // right-justified
                m_c = s_f;
                n_c = (slot_l - s_f > DATA_W) ? DATA_W : slot_l - s_f;
            end
        end
    end

    always @(posedge bclk) begin
        ws_d  <= ws;
        valid <= 1'b0;

        if (ws_edge) begin
            // The slot that has just finished belongs to channel 'ws_d'.
            if (ws_d == 1'b0) begin
                left  <= word_a;             // left channel done
            end else begin
                right <= word_a;             // right channel done
                valid <= 1'b1;               // complete L+R frame available
            end
            // update slot length
            l_prev <= bitpos;
            if (bitpos == l_prev && bitpos >= 8'd8) begin slot_l <= bitpos; fmt_slot <= bitpos; end
            // activity: position 0 = this cycle
            act[0] <= act[0] | sd; p0b <= sd;
            // format evaluation every 2^EVAL_LOG2 slots
            a_slots <= a_slots + 17'd1;
            if (a_slots == (17'd1 << EVAL_LOG2) - 17'd1) begin
                a_slots <= 17'd0; act <= {63'd0, sd}; agree <= 17'd0; dis <= 64'd0;
                if (!man_en && any_f) begin
                    cand_m <= m_c; cand_n <= n_c; cand_x <= x_c;
                    if (m_c == cand_m && n_c == cand_n && x_c == cand_x) begin f_m <= m_c; f_n <= n_c; rjx <= x_c; end   // only after two equal evaluations
                end
            end
            // manual format
            if (man_en) begin
                f_n <= (man_bits > DATA_W) ? DATA_W : (man_bits < 8'd4 ? DATA_W : man_bits);
                // right-justified: the word of man_bits bits ends at the slot's end, so it starts slot_l - man_bits bits after the edge
                // (also when it is wider than DATA_W: the top DATA_W bits are taken)
                f_m <= (man_mode == 2'd0) ? 8'd1 : (man_mode == 2'd1) ? 8'd0 :
                       (slot_l > ((man_bits < 8'd4) ? DATA_W : man_bits)) ? (slot_l - ((man_bits < 8'd4) ? DATA_W : man_bits)) : 8'd0;
            end
            // a word that starts on the WS edge (left-justified): the MSB is already on the data
            if (f_m == 8'd0) shreg <= {shreg[DATA_W-2:0], sd};
            bitpos <= 8'd1;                  // this cycle counts as "0 since edge"
        end else begin
            if (bitpos < 8'd64) act[bitpos[5:0]] <= act[bitpos[5:0]] | sd;
            if (bitpos == 8'd1) agree <= agree + ((sd == p0b) ? 17'd1 : 17'd0);
            if (bitpos < 8'd64 && sd != p0b) dis[bitpos[5:0]] <= 1'b1;
            // Shift in data bits within window [max(f_m,1), f_m + f_n)
            if (bitpos >= ((f_m == 8'd0) ? 8'd1 : f_m) && bitpos < (f_m + f_n))
                shreg <= {shreg[DATA_W-2:0], sd};   // MSB-first
            if (bitpos != 8'hFF)
                bitpos <= bitpos + 8'd1;
        end

        fmt_bits <= f_n;
        fmt_mode <= (f_m == 8'd1) ? 2'd0 : (f_m == 8'd0) ? 2'd1 : 2'd2;
    end

    initial begin fmt_slot = 8'd0; fmt_bits = 8'd24; fmt_mode = 2'd0; end

endmodule
