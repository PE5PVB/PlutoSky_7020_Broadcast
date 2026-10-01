// =============================================================================
// skypluto_sigcond - composite input conditioning: soft switch-on/off + look-ahead limiter
// -----------------------------------------------------------------------------
// Sits between the CDC FIFO and the interpolator and works at the INPUT RATE (192 kHz, ~64 l_clk per sample).
// The interpolator pulls a sample; the conditioner then fetches a new sample from the FIFO and returns the
// 256-sample (1.33 ms) delayed, limited sample (first-word-fall-through).
//
// SOFT SWITCH-ON/OFF (at the INPUT, before the delay, so that no step occurs)
//   xf[n] = x[n] * fade[n]. On loss of the I2S (HOLD) the last valid value is held and faded
//   to 0 (continuous in level, only the slope changes); on recovery the fade runs from 0 to 1.
//   RUN -> HOLD when the write side of the FIFO stalls (alive = 0) or the FIFO has been empty for UF_LIM consecutive pulls.
//   HOLD -> RUN only when the fade is at 0 AND the I2S is active again AND the FIFO is ~half full again. In HOLD, excess
//   (stale) FIFO data is discarded, so that the interpolator starts at the correct occupancy after recovery.
//   The fade runs at 1/2^FADE_SHIFT per sample (10 => ~5.3 ms).
//
// LIMITER (feed-forward, smooth, with exact guarantee), on xf
//   g_req[n] = min(1, CEIL/|xf[n]|)                            (Q16, restoring divider, 16 iterations)
//   bm[j]    = minimum of g_req over block j (16 samples);  W[j] = min(bm[j-15..j])  (256 samples, incl. the current block)
//   g_s      = average of W[j-15..j] (16 values), computed afresh once per block (no running sum, no gain memory)
//   y[n-256] = xf[n-256] * g_s   (g_s takes effect at a block boundary, after the multiplication of the last sample of the block)
//   Guarantee: for m in block B, W[j] <= bm[B] <= g_req[m] for all j in [B, B+15], hence g_s <= g_req[m] and |y| <= CEIL.
//   Below the ceiling g_s is exactly 1.0 and (at fade = 1) the output is bit-exact the delayed input.
//   Note: changing ceil/lim_en while the 256-sample delay line is filled gives no guarantee for up to 512 samples for the samples already in the line.
// =============================================================================
`timescale 1ns/1ps

module skypluto_sigcond #(
    parameter integer DW         = 24,
    parameter integer UF_LIM     = 64,      // consecutive empty pulls (even without the write side stopping) -> HOLD
    parameter integer TGT        = 32,      // target FIFO occupancy (half full)
    parameter integer FADE_SHIFT = 10       // fade step 2^-10 per sample -> ~5.3 ms from 0 to 1
)(
    input  wire                  clk,
    input  wire                  rst,
    // upstream: CDC-FIFO (first-word-fall-through)
    input  wire signed [DW-1:0]  up_data,
    input  wire                  up_empty,
    input  wire [6:0]            up_count,   // FIFO occupancy
    input  wire [6:0]            up_wrpos,   // write pointer (moves as long as the I2S is writing)
    output reg                   up_rd,
    // downstream to the interpolator (first-word-fall-through)
    output reg  signed [DW-1:0]  q_data,
    output reg                   q_empty,
    input  wire                  q_rd,      // 1-clock pulse: the interpolator has taken q_data
    // control (l_clk domain)
    input  wire [DW-1:0]         ceil,      // peak ceiling in counts (composite full scale = 2^23)
    input  wire                  lim_en,
    input  wire                  fade_en,
    input  wire                  clr,       // pulse: clear statistics
    // status
    output reg  [16:0]           gmin,      // smallest g_req since clr (65536 = no limiting)
    output reg  [31:0]           events,    // number of samples with g_req < 1
    output reg  [DW-1:0]         inpeak,    // largest |x| (after fade) since clr
    output reg  [1:0]            state,     // 0 = HOLD, 1 = RUN
    output reg  [31:0]           dbg_pulls, // diagnostics: number of pulls (FETCH), FIFO pops and x_new != 0
    output reg  [31:0]           dbg_pops,
    output reg  [31:0]           dbg_nz,
    output wire [31:0]           dbg_state, // instantaneous state (FSM, handshake, occupancy) for diagnostics on the hardware
    output wire [31:0]           dbg_wd,    // watchdog counters {wd_idle[15:0], wd_fsm[15:0]}
    output wire [31:0]           dbg_gain,  // instantaneous gain gs (Q16) and
    output wire [31:0]           dbg_sum,   // the running sum (diagnostics of the gain computation)
    output reg  [31:0]           uf_events, // number of RUN -> HOLD transitions
    output wire [16:0]           fade_o
);
    // ---- memories -------------------------------------------------------------
    // GAIN COMPUTATION WITHOUT RUNNING SUM AND WITHOUT RAM HISTORY (an incremental sum drifts permanently out of
    // balance on the hardware -> gain 1.9 or 0.1). Blocks of 16 samples:
    //   bm[j]  = minimum of g_req over block j
    //   W[j]   = min(bm[j-15 .. j])            (window of the last 16 blocks = 256 samples)
    //   gs     = average of W[j-15 .. j]       (16 values), once per block, computed AFRESH from 16 registers (no memory, no drift)
    // Guarantee: output sample m (block B) is output 256 samples later with gs = average of W[B .. B+15]; every W[j] with j in
    // [B, B+15] contains block B, so every term <= bm[B] <= g_req[m] => gs <= g_req[m] => |y| <= ceil.
    // gs only changes at a block boundary (after the multiplication of that sample), so it is constant per block.
    (* ram_style = "distributed" *) reg signed [DW-1:0] xd  [0:255];        // only the delay line (256 samples)
    integer ii;
    initial begin
        for (ii = 0; ii < 256; ii = ii + 1) xd[ii] = 0;
    end
    reg [7:0] p;                             // ring pointer of the delay line
    reg signed [DW-1:0] x_old;

    reg [16:0] bm [0:15];                    // last 16 completed block minima
    reg [16:0] wv [0:15];                    // last 16 window minima W
    reg [16:0] cm;                           // minimum of the current block
    reg [3:0]  sc, cb;                       // sc = position in the block, cb = oldest ring position (overwritten at the next block completion)
    reg [16:0] gs_next;                      // next gain, takes effect after the multiplication of the last sample of the block
    reg        blk_done;
    reg        bg_start;
    reg        bg_run;                       // background: P15 = min of the 15 newest bm, S15 = sum of the 15 newest W (oldest excluded)
    reg [4:0]  bi;
    reg [16:0] pmin, pacc, cmf;
    reg [20:0] ssum, sacc;

    // ---- state -----------------------------------------------------------------
    reg [16:0] fade;
    assign fade_o = fade;
    reg [7:0]  ufc;                          // consecutive empty pulls
    reg [6:0]  wr_prev;                      // previous write pointer
    reg [6:0]  alive;                        // write-side activity (0..64)
    reg signed [DW-1:0] x_last;              // last valid input (to hold on loss)
    (* dont_touch = "true" *) reg signed [DW-1:0] x_new;               // input chosen for this pull (dont_touch: do not absorb into the DSP A register; the FIFO memory path to it would otherwise be too long)
    reg signed [DW+17:0] xfp;                // x_new * fade
    reg signed [DW-1:0] xf;                  // after fade
    reg [DW:0]  ax;                          // |xf|
    reg [16:0]  greq;
    reg [16:0]  gs;                          // gain applied to x_old at this moment (<= 65536 = 1.0)
    reg signed [DW+17:0] yprod;

    // ---- divider ---------------------------------------------------------------
    reg [25:0] rem;
    reg [15:0] quo;
    reg [4:0]  it;

    localparam [6:0] TGT7 = TGT;
    localparam [4:0] S_IDLE = 0, S_FETCH = 1, S_FM0 = 2, S_FM1 = 3, S_FM2 = 4, S_ABS = 5, S_CMP = 6, S_DIV = 7, S_GREQ = 8,
                     S_BLK = 9, S_AVG0 = 13,
                     S_MUL0 = 15, S_MUL1 = 16, S_MUL2 = 17, S_OUT = 18;
    reg [4:0]  st;

    // ---- watchdog + diagnostics ------------------------------------------------------------------------------------
    // wdc  : number of clocks the FSM spends outside S_IDLE; a normal pull takes < ~90 clocks. At > 400 the FSM is reset
    //        to S_IDLE (self-heal) and wd_fsm is counted.
    // idc  : number of clocks in S_IDLE without q_rd (the interpolator normally asks for a sample every ~64 clocks). After ~10 ms of silence
    //        wd_idle is counted (once per stall; the interpolator itself cannot recover the conditioner).
    // After a reset the delay line xd is FIRST cleared (256 clocks, during init_busy; pulls are ignored then). The bm/wv rings and
    // ssum are set to 1.0 in the reset, so that no stale gain state can carry over after a reset.
    reg        init_busy;
    reg [8:0]  ic;
    reg [8:0]  wdc;
    reg [17:0] idc;
    reg        idc_flag;
    reg [15:0] wd_fsm, wd_idle;
    assign dbg_wd    = {wd_idle, wd_fsm};
    assign dbg_gain  = {15'd0, gs};                          // instantaneous gain (Q16, 65536 = 1.0)
    assign dbg_sum   = {11'd0, ssum};                        // S15 = sum of the 15 newest W (without limiting 15 x 65536 = 0xF0000)
    assign dbg_state = {6'b0, up_empty, q_empty, state, alive, up_count, 3'b0, st};

    wire run_state = (state == 2'd1);

    always @(posedge clk) begin
        if (rst) begin
            up_rd <= 1'b0; q_data <= 0; q_empty <= 1'b0;    // first word = zero, so that the interpolator can start
            gmin <= 17'd65536; events <= 0; inpeak <= 0; state <= 2'd0; uf_events <= 0;
            dbg_pulls <= 0; dbg_pops <= 0; dbg_nz <= 0;
            p <= 0; x_old <= 0;
            for (ii = 0; ii < 16; ii = ii + 1) begin bm[ii] <= 17'd65536; wv[ii] <= 17'd65536; end
            cm <= 17'd65536; cmf <= 17'd65536; sc <= 0; cb <= 0; gs_next <= 17'd65536; blk_done <= 1'b0;
            bg_start <= 1'b0; bg_run <= 1'b0; bi <= 0; pmin <= 17'd65536; pacc <= 17'd65536; ssum <= 21'hF0000; sacc <= 0;
            fade <= 0; ufc <= 0; wr_prev <= 0; alive <= 0; x_last <= 0; x_new <= 0; xfp <= 0; xf <= 0; ax <= 0;
            greq <= 17'd65536; gs <= 17'd65536; yprod <= 0;
            rem <= 0; quo <= 0; it <= 0; st <= S_IDLE;
            wdc <= 0; idc <= 0; idc_flag <= 1'b0; wd_fsm <= 0; wd_idle <= 0;
            init_busy <= 1'b1; ic <= 9'd0;
        end else if (init_busy) begin
            // RAM clearing pass: delay line to 0.
            // Same address line p as in normal operation (a RAM with ONE address for read and write: this is reliably synthesized
            // as RAM; a second write address (such as 'ic') gave a gain of ~0.1 on the hardware). p runs 256 steps and is back at 0 afterwards.
            xd[p]  <= {DW{1'b0}};
            p  <= p + 1'b1;
            ic <= ic + 1'b1;
            if (ic == 9'd255) init_busy <= 1'b0;
        end else begin
            up_rd <= 1'b0;
            if (up_rd) dbg_pops <= dbg_pops + 1'b1;                        // every FIFO pop (up_rd is high for 1 clock)
            if (clr) begin gmin <= 17'd65536; inpeak <= 0; end

            case (st)
            S_IDLE: begin
                if (q_rd) begin q_empty <= 1'b1; st <= S_FETCH; end
                else      q_empty <= 1'b0;                  // in IDLE a valid word is always ready: a 'stuck' q_empty recovers by itself
            end

            // --- choose the next sample (or held value) and update the fade ---
            S_FETCH: begin
                // track the activity of the I2S write side (1x per pull)
                dbg_pulls <= dbg_pulls + 1'b1;
                wr_prev <= up_wrpos;
                if (up_wrpos != wr_prev) alive <= (alive >= 7'd62) ? 7'd64 : alive + 7'd2;
                else                     alive <= (alive >= 7'd6)  ? alive - 7'd6 : 7'd0;
                if (run_state) begin
                    if (!up_empty) begin
                        x_new <= up_data; x_last <= up_data; up_rd <= 1'b1; ufc <= 0;
                    end else begin
                        x_new <= x_last;                                   // short underrun: hold the last value
                        ufc <= ufc + 1'b1;
                    end
                    if (alive == 7'd0 || (up_empty && ufc >= UF_LIM - 1)) begin   // I2S gone -> HOLD
                        state <= 2'd0; uf_events <= uf_events + 1'b1; x_new <= x_last; up_rd <= 1'b0;   // do not pop: x_new is the held value
                    end
                end else begin                                             // HOLD: hold the last value, FIFO at ~half full
                    x_new <= x_last;
                    if (up_count > TGT7 && !up_empty) up_rd <= 1'b1;       // discard excess (stale) data
                    if (alive >= 7'd48 && up_count >= TGT7 - 7'd2 && fade == 17'd0) begin state <= 2'd1; ufc <= 0; end
                end
                // fade: towards 1 in RUN, towards 0 in HOLD (immediate when fade is off)
                if (!fade_en) fade <= run_state ? 17'd65536 : 17'd0;
                else if (run_state) fade <= (fade > 17'd65536 - (17'd65536 >> FADE_SHIFT)) ? 17'd65536 : fade + (17'd65536 >> FADE_SHIFT);
                else                fade <= (fade < (17'd65536 >> FADE_SHIFT)) ? 17'd0 : fade - (17'd65536 >> FADE_SHIFT);
                st <= S_FM0;
            end
            S_FM0: begin xfp <= x_new * $signed({1'b0, fade}); st <= S_FM1; if (x_new != 0) dbg_nz <= dbg_nz + 1'b1; end     // x_new and fade are valid here
            S_FM1: st <= S_FM2;
            S_FM2: begin xf <= xfp[DW+15:16]; st <= S_ABS; end
            S_ABS: begin
                ax <= xf[DW-1] ? (~{xf[DW-1], xf} + 1'b1) : {1'b0, xf};       // |xf| in DW+1 bits
                st <= S_CMP;
            end
            S_CMP: begin
                if (ax[DW-1:0] > inpeak) inpeak <= ax[DW-1:0];
                if (lim_en && (ax > {1'b0, ceil})) begin
                    rem <= {2'b00, ceil}; quo <= 0; it <= 5'd0; st <= S_DIV;
                end else begin
                    greq <= 17'd65536; st <= S_BLK;
                end
            end
            // --- restoring divider: q = floor(ceil * 65536 / |x|) (ceil < |x|) ---
            S_DIV: begin
                if ({rem[24:0], 1'b0} >= {1'b0, ax}) begin
                    rem <= {rem[24:0], 1'b0} - {1'b0, ax}; quo <= {quo[14:0], 1'b1};
                end else begin
                    rem <= {rem[24:0], 1'b0}; quo <= {quo[14:0], 1'b0};
                end
                it <= it + 1'b1;
                if (it == 5'd15) st <= S_GREQ;
            end
            S_GREQ: begin greq <= {1'b0, quo}; st <= S_BLK; end
            S_BLK: begin
                if (greq < 17'd65536) events <= events + 1'b1;
                if (greq < gmin) gmin <= greq;
                cm  <= (greq < cm) ? greq : cm;
                cmf <= (greq < cm) ? greq : cm;
                x_old <= xd[p];                  // sample from 256 pulls ago (read before write)
                st <= S_AVG0;
            end
            // --- block boundary: update bm/W and prepare the next gain (takes effect only after the multiplication, in S_OUT) ---
            S_AVG0: begin
                xd[p] <= xf;
                p <= p + 1'b1;
                if (sc == 4'd15) begin
                    bm[cb]  <= cmf;
                    wv[cb]  <= (cmf < pmin) ? cmf : pmin;
                    gs_next <= ((ssum + {4'd0, ((cmf < pmin) ? cmf : pmin)}) >> 4) > 21'd65536 ? 17'd65536
                               : ((ssum + {4'd0, ((cmf < pmin) ? cmf : pmin)}) >> 4);
                    blk_done <= 1'b1; bg_start <= 1'b1;
                    cb <= cb + 1'b1; cm <= 17'd65536; sc <= 0;
                end else sc <= sc + 1'b1;
                st <= S_MUL0;
            end
            S_MUL0: begin yprod <= x_old * $signed({1'b0, gs}); st <= S_MUL1; end
            S_MUL1: st <= S_MUL2;
            S_MUL2: begin q_data <= yprod[DW+15:16]; st <= S_OUT; end
            S_OUT: begin q_empty <= 1'b0; if (blk_done) begin gs <= gs_next; blk_done <= 1'b0; end st <= S_IDLE; end
            default: st <= S_IDLE;
            endcase

            // ---- background: P15 = min of the 15 newest bm, S15 = sum of the 15 newest W (oldest = index cb excluded) ----
            if (bg_start) begin bg_start <= 1'b0; bg_run <= 1'b1; bi <= 0; pacc <= 17'd65536; sacc <= 0; end
            else if (bg_run) begin
                if (bi == 5'd16) begin pmin <= pacc; ssum <= sacc; bg_run <= 1'b0; end
                else begin
                    if (bi[3:0] != cb) begin
                        if (bm[bi[3:0]] < pacc) pacc <= bm[bi[3:0]];
                        sacc <= sacc + {4'd0, wv[bi[3:0]]};
                    end
                    bi <= bi + 1'b1;
                end
            end

            // ---- watchdog (placed AFTER the case, so it wins on conflicting assignments) ----
            if (st != S_IDLE) begin
                if (wdc == 9'd400) begin st <= S_IDLE; q_empty <= 1'b0; wd_fsm <= wd_fsm + 1'b1; wdc <= 0; end
                else wdc <= wdc + 1'b1;
            end else wdc <= 0;
            if (st == S_IDLE && !q_rd) begin
                if (!idc[17]) idc <= idc + 1'b1;
                else if (!idc_flag) begin idc_flag <= 1'b1; wd_idle <= wd_idle + 1'b1; end
            end else begin idc <= 0; idc_flag <= 1'b0; end
        end
    end
endmodule
