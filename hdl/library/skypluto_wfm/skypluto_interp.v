// =============================================================================
// skypluto_interp - band-limiting polyfase interpolator
// -----------------------------------------------------------------------------
// Composite 192,187.5 Hz (async FIFO) -> l_clk, 2 stages:
//   Stage 1: x4 polyfase FIR (96 taps, 24/branch, 18-bit) -> 768,750 Hz stream.
//            Flat +-0.002 dB up to 76 kHz; image >=116 kHz -73 dB. HARD <=76 kHz.
//   Stage 2: fractional LINEAR interpolation -> l_clk.
//
// PIPELINED for the 250 MHz l_clk constraint:
//   - MAC: 4 parallel lanes, registered products -> registered
//     accumulation -> 2-stage final sum. 6 feed clocks + ~4 latency << 16 (the
//     fractional overflow interval), so mac_done is ready well in time.
//   - linear interp: registered multiply (2 stages) for comp_out.
// Single stream: branch outputs y[m] (m%4=branch); 4-deep pipeline
// s0/s1/s2/mac_done shifts on every overflow; interp between s0 and s1.
// =============================================================================
`timescale 1ns/1ps

module skypluto_interp #(
    parameter integer DW         = 24,
    parameter integer FRAC       = 24,
    parameter integer TAPS       = 24,
    parameter integer NB         = 4,
    parameter integer COEFW      = 18,
    parameter integer COEF_SHIFT = 15,
    parameter integer CNTW       = 7,            // FIFO occupancy width (AW+1)
    parameter [FRAC-1:0] STEP_NOM = 24'd262400,  // nominal FIFO rate increment
    parameter integer RANGE      = 512,          // max STEP deviation (~2000 ppm)
    parameter integer LP_SHIFT   = 16,           // loop update every 2^LP_SHIFT l_clks
    parameter         COEF_FILE  = "interp_coefs.mem"
)(
    input  wire                 clk,
    input  wire                 rst,
    input  wire [FRAC-1:0]      step,            // nominal rate (loop center)
    input  wire signed [DW-1:0] fifo_data,
    input  wire                 fifo_empty,
    input  wire [CNTW-1:0]      fifo_count,      // FIFO occupancy (read domain)
    output reg                  fifo_rd,
    output reg  signed [DW-1:0] comp_out,
    output reg                  comp_valid,
    output reg  [31:0]          sat_cnt,         // number of FIR outputs that were clamped (saturated) instead of wrapping
    // diagnostics (read-only): counters of overflow pulses ('ovf') and samples actually taken (take), plus the instantaneous state
    output reg  [31:0]          dbg_ovf_cnt,
    output reg  [31:0]          dbg_take_cnt,
    output wire [31:0]          dbg_state        // {primed, bp[2:0], 4'b0, step_dyn[23:0]}
);
    localparam integer TGT = (1 << (CNTW-2));    // half full (DEPTH/2)
    localparam integer DB  = 2;                  // deadband
    localparam integer PRODW = DW + COEFW;      // 42
    localparam integer ACCMW = PRODW + 5;       // 47

    (* rom_style = "distributed" *) reg signed [COEFW-1:0] coef [0:NB*TAPS-1];
    initial $readmemh(COEF_FILE, coef);

    reg signed [DW-1:0] x [0:TAPS-1];           // delay line, x[0]=newest
    integer di;

    // ---- adaptive STEP: slow control loop on the FIFO occupancy -------------
    // Keeps the FIFO around half full -> tracks the exact Pico rate, zero drift,
    // regardless of crystal difference/temperature. Loop bandwidth << audio (sub-Hz).
    reg [FRAC-1:0]      step_dyn;
    reg [LP_SHIFT-1:0]  lp;
    reg                 primed;                        // only read once the FIFO is half full
    reg signed [23:0]   acc_i;                         // PI-integrator
    localparam integer  KP_SH = 3;                     // proportional: err*8
    localparam integer  KI_SH = 6;                     // integral: acc_i/64
    localparam integer  ACC_CLAMP = RANGE << KI_SH;    // limits acc_i to ±RANGE effect
    // Pipelined loop arithmetic (every clock, registered single-op stages)
    // -> short paths for the 250 MHz rx_clk constraint. The update itself is slow.
    reg [CNTW-1:0]      fcnt_r;                         // FIFO occupancy (registered)
    reg signed [8:0]    ferr_r;                         // count - TGT
    reg signed [23:0]   isum_r;                         // acc_i + ferr_r (integrator candidate)
    reg signed [FRAC:0] corr_r;                         // Kp*err + Ki*acc_i
    reg signed [FRAC:0] sdcand_r;                       // step + PI correction (step_dyn candidate)

    // ---- fractional phase (Stage 2) -----------------------------------------
    reg  [FRAC-1:0] acc2;
    reg  [FRAC-1:0] step4_r;                     // step_dyn<<2, registered
    // -> splits the long (routing-dominated) step_dyn->adder path. step_dyn is
    //    quasi-static, so this 1-clock delay is functionally irrelevant.
    wire [FRAC:0]   acc2_sum = {1'b0,acc2} + {1'b0,step4_r};
    wire            ovf = acc2_sum[FRAC];
    reg  [2:0]      bp;                          // branch of the NEXT compute
    reg signed [DW-1:0] s0r, s1r, s2r;

    // ---- MAC engine: 4 lanes, pipelined -------------------------------------
    reg               macgo;                    // feed phase active
    reg [5:0]         ke;                        // 0,4,...,20
    reg [2:0]         cbr;
    // read stage: register coef/x first -> breaks the ke/cbr->mux->DSP path (250 MHz)
    reg signed [COEFW-1:0] rc0,rc1,rc2,rc3;
    reg signed [DW-1:0]    rx0,rx1,rx2,rx3;
    reg                    rvld, rvlast;
    // multiply stage (registered products)
    reg signed [PRODW-1:0] p0,p1,p2,p3;
    reg                    mv, mvlast;
    // accumulation
    reg signed [ACCMW-1:0] a0,a1,a2,a3;
    reg                    acclast;
    // final sum (2 stages)
    reg signed [ACCMW-1:0] sumAB, sumCD;
    reg                    sumv, sumv2;
    localparam integer     FULLW = ACCMW - COEF_SHIFT;                 // width of the FIR sum after the scaling shift (32)
    reg signed [FULLW-1:0] mac_full;                                   // full (not yet clamped) sum
    localparam signed [FULLW-1:0] SAT_MAX =  (1 <<< (DW-1)) - 1;
    localparam signed [FULLW-1:0] SAT_MIN = -(1 <<< (DW-1));
    reg signed [DW-1:0]    mac_done;

    // ---- linear interp pipeline (Stage 2 output) ----------------------------
    // 17-bit fraction -> the multiply fits in 1 DSP48E1 (25x18) instead of a 2-DSP
    // cascade (which does not meet the 250 MHz rx_clk). 17-bit interp precision is ample.
    localparam integer      FW = 17;
    wire signed [DW:0]      sdiff = s1r - s0r;
    reg  signed [DW:0]      id;
    reg  [FW-1:0]           ifr;
    reg  signed [DW-1:0]    is0, is0b;
    reg  signed [DW+FW+1:0] iprod;

    always @(posedge clk) begin
        if (rst) begin
            fifo_rd<=1'b0; comp_valid<=1'b0; comp_out<=0;
            for (di=0; di<TAPS; di=di+1) x[di]<=0;
            acc2<=0; bp<=0; s0r<=0; s1r<=0; s2r<=0;
            macgo<=0; ke<=0; cbr<=0; p0<=0;p1<=0;p2<=0;p3<=0; mv<=0; mvlast<=0;
            rc0<=0;rc1<=0;rc2<=0;rc3<=0; rx0<=0;rx1<=0;rx2<=0;rx3<=0; rvld<=0; rvlast<=0;
            a0<=0;a1<=0;a2<=0;a3<=0; acclast<=0; sumAB<=0;sumCD<=0; sumv<=0; sumv2<=0; mac_full<=0; sat_cnt<=0;
            dbg_ovf_cnt<=0; dbg_take_cnt<=0;
            mac_done<=0; id<=0; ifr<=0; is0<=0; is0b<=0; iprod<=0;
            step_dyn<=STEP_NOM; lp<=0; acc_i<=0; primed<=1'b0; step4_r<=STEP_NOM<<2;
            fcnt_r<=0; ferr_r<=0; isum_r<=0; corr_r<=0; sdcand_r<=0;
        end else if (!primed) begin
            // ===== prime phase: wait until the FIFO is half full =====
            // Carrier is on (comp_out=0 -> pure carrier), no audio yet.
            fifo_rd<=1'b0; comp_valid<=1'b1; comp_out<={DW{1'b0}};
            acc2<=0; bp<=0; step_dyn<=STEP_NOM; acc_i<=0; lp<=0; macgo<=0;
            if (fifo_count >= TGT) primed <= 1'b1;
        end else begin
            fifo_rd<=1'b0; comp_valid<=1'b1;

            // ===== adaptive STEP loop: damped PI on FIFO occupancy =====
            // Pipeline (every clock, short paths): count->ferr->isum/corr/sdcand.
            fcnt_r   <= fifo_count;
            ferr_r   <= $signed({1'b0, fcnt_r}) - TGT;
            isum_r   <= acc_i + ferr_r;                          // integrator candidate
            corr_r   <= (ferr_r <<< KP_SH) + (acc_i >>> KI_SH);  // Kp*err + Ki*acc_i
            sdcand_r <= $signed({1'b0, step}) + ((ferr_r <<< KP_SH) + (acc_i >>> KI_SH));
            // Slow update (1x per 2^LP_SHIFT): only compare+mux (short path).
            lp <= lp + 1'b1;
            if (lp == {LP_SHIFT{1'b0}}) begin
                if (isum_r >  ACC_CLAMP)      acc_i <=  ACC_CLAMP;   // anti-windup
                else if (isum_r < -ACC_CLAMP) acc_i <= -ACC_CLAMP;
                else                          acc_i <=  isum_r;
                if (corr_r >  RANGE)          step_dyn <= step + RANGE[FRAC-1:0];
                else if (corr_r < -RANGE)     step_dyn <= step - RANGE[FRAC-1:0];
                else                          step_dyn <= sdcand_r[FRAC-1:0];
            end

            // ===== Stage 2: fractional phase =====
            acc2    <= acc2_sum[FRAC-1:0];
            step4_r <= step_dyn << 2;              // registered copy (short path)

            // linear interp (pipelined, constant latency 2):
            id  <= sdiff;  ifr <= acc2[FRAC-1 -: FW];  is0 <= s0r;  // stage 1: capture (top FW fraction bits)
            iprod <= id * $signed({1'b0, ifr});                     // stage 2: 25x18 -> 1 DSP48E1
            is0b  <= is0;
            comp_out <= is0b + (iprod >>> FW);                      // stage 3: sum

            if (ovf) begin
                dbg_ovf_cnt <= dbg_ovf_cnt + 1'b1;
                s0r <= s1r; s1r <= s2r; s2r <= mac_done;
                if (bp == 3'd0 && !fifo_empty) begin
                    fifo_rd <= 1'b1; dbg_take_cnt <= dbg_take_cnt + 1'b1;
                    for (di=TAPS-1; di>0; di=di-1) x[di] <= x[di-1];
                    x[0] <= fifo_data;
                end
                // start MAC feed for branch bp
                macgo<=1'b1; cbr<=bp; ke<=6'd0;
                a0<=0;a1<=0;a2<=0;a3<=0;
                bp <= (bp==NB-1) ? 3'd0 : bp + 3'd1;
            end

            // ===== Stage 1 MAC: read stage (register operands, 4 lanes) =====
            if (macgo) begin
                rc0<=coef[cbr*TAPS+ke  ]; rc1<=coef[cbr*TAPS+ke+1];
                rc2<=coef[cbr*TAPS+ke+2]; rc3<=coef[cbr*TAPS+ke+3];
                rx0<=x[ke  ]; rx1<=x[ke+1]; rx2<=x[ke+2]; rx3<=x[ke+3];
                rvld   <= 1'b1;
                rvlast <= (ke >= TAPS-4);
                if (ke >= TAPS-4) macgo <= 1'b0; else ke <= ke + 6'd4;
            end else begin
                rvld <= 1'b0;
            end

            // ===== multiply stage (registered operands -> DSP) =====
            p0<=rc0*rx0; p1<=rc1*rx1; p2<=rc2*rx2; p3<=rc3*rx3;
            mv     <= rvld;
            mvlast <= rvlast;

            // ===== accumulation =====
            if (mv) begin
                a0<=a0+p0; a1<=a1+p1; a2<=a2+p2; a3<=a3+p3;
            end
            acclast <= mv & mvlast;

            // ===== final sum (2 stages) =====
            if (acclast) begin sumAB <= a0+a1; sumCD <= a2+a3; sumv<=1'b1; end
            else sumv <= 1'b0;
            // SATURATE the FIR output instead of letting it wrap: the FIR has overshoot (typically +5..6 %, up to +25 % on flat peaks);
            // a wrap (sign flip) of one sample causes a ~4 us frequency dip from +58 to -60 kHz = 13-22 dB extra
            // energy at 130-170 kHz (spectral mask!). Saturating costs ~2 dB instead of 17 dB.  Extra pipeline stage (short path).
            sumv2 <= sumv;
            if (sumv) mac_full <= (sumAB + sumCD) >>> COEF_SHIFT;
            if (sumv2) begin
                if (mac_full > SAT_MAX)      begin mac_done <= SAT_MAX[DW-1:0]; sat_cnt <= sat_cnt + 1'b1; end
                else if (mac_full < SAT_MIN) begin mac_done <= SAT_MIN[DW-1:0]; sat_cnt <= sat_cnt + 1'b1; end
                else                              mac_done <= mac_full[DW-1:0];
            end
        end
    end
    assign dbg_state = {primed, bp, 4'b0000, step_dyn};
endmodule
