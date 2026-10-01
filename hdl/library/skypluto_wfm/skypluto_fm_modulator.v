// =============================================================================
// skypluto_fm_modulator - FM modulation with NCO offset, carrier level and DC nulling
// -----------------------------------------------------------------------------
// Runs on l_clk (timed as rx_clk = 250 MHz). Fully pipelined: 1 operation
// per register stage.
//
//   phase_inc = offset_inc + ((kdev * comp) >>> KSHIFT)
//   phase    += phase_inc            (also runs without audio -> always a carrier)
//   I = sat( (cos(phase)*level) >>> LVL_SHIFT + dc_i )
//   Q = sat( (sin(phase)*level) >>> LVL_SHIFT + dc_q )
//
// dc_i/dc_q: small digital DC offset to cancel the analog LO leakage.
// =============================================================================
`timescale 1ns/1ps

module skypluto_fm_modulator #(
    parameter integer COMP_W     = 24,
    parameter integer KDEV_W     = 18,
    parameter integer KSHIFT     = 8,
    parameter integer PHASE_W    = 24,
    parameter integer LUT_ADDR_W = 14,
    parameter integer OUT_W      = 16,
    parameter integer LVL_W      = 16,
    parameter integer LVL_SHIFT  = 16,
    parameter integer DC_W       = 12,
    parameter         LUT_FILE   = "sine_lut.mem"
)(
    input  wire                      clk,
    input  wire                      rst,
    input  wire                      en,
    input  wire signed [KDEV_W-1:0]  kdev,
    input  wire signed [PHASE_W-1:0] offset_inc,
    input  wire        [LVL_W-1:0]   level,
    input  wire signed [DC_W-1:0]    dc_i,
    input  wire signed [DC_W-1:0]    dc_q,
    input  wire signed [COMP_W-1:0]  comp,
    input  wire                      comp_valid,
    output reg  signed [OUT_W-1:0]   i_out,
    output reg  signed [OUT_W-1:0]   q_out,
    output reg                       iq_valid
);
    localparam integer PROD_W = KDEV_W + COMP_W;
    localparam signed [OUT_W:0] MAXO =  (1 << (OUT_W-1)) - 1;
    localparam signed [OUT_W:0] MINO = -(1 << (OUT_W-1));

    // s1: kdev*comp
    reg signed [PROD_W-1:0]  prod_r = 0;  reg v1 = 0;
    // s2: phase_inc
    reg signed [PHASE_W-1:0] phase_inc_r = 0; reg v2 = 0;
    wire signed [PROD_W-1:0] prod_sh = prod_r >>> KSHIFT;
    // s3: phase accumulator
    reg signed [PHASE_W-1:0] phase = 0; reg v3 = 0;
    // s4: sin/cos lookup (1 clock in the core)
    wire [LUT_ADDR_W-1:0] lut_phase = phase[PHASE_W-1 -: LUT_ADDR_W];
    wire signed [OUT_W-1:0] sin_raw, cos_raw;
    reg v4 = 0;
    // s4b: register sincos + level
    reg signed [OUT_W-1:0] cos_r = 0, sin_r = 0;
    reg        [LVL_W-1:0] level_r = 0; reg v4b = 0;
    // s5: level multiply -> registered, scaled
    wire signed [OUT_W+LVL_W:0] i_scaled = cos_r * $signed({1'b0, level_r});
    wire signed [OUT_W+LVL_W:0] q_scaled = sin_r * $signed({1'b0, level_r});
    reg signed [OUT_W:0] i_sc = 0, q_sc = 0; reg v5 = 0;
    // s6: + DC offset
    reg signed [OUT_W+1:0] i_sum = 0, q_sum = 0; reg v6 = 0;

    always @(posedge clk) begin
        if (rst) begin
            prod_r<=0; phase_inc_r<=0; phase<=0; cos_r<=0; sin_r<=0; level_r<=0;
            i_sc<=0; q_sc<=0; i_sum<=0; q_sum<=0;
            v1<=0; v2<=0; v3<=0; v4<=0; v4b<=0; v5<=0; v6<=0;
            i_out<=0; q_out<=0; iq_valid<=0;
        end else begin
            prod_r      <= kdev * comp;                       v1  <= comp_valid & en;
            phase_inc_r <= prod_sh[PHASE_W-1:0] + offset_inc; v2  <= v1;
            if (v2) phase <= phase + phase_inc_r;             v3  <= v2;
            v4  <= v3;
            cos_r <= cos_raw; sin_r <= sin_raw; level_r <= level; v4b <= v4;
            i_sc <= i_scaled >>> LVL_SHIFT;  q_sc <= q_scaled >>> LVL_SHIFT;  v5 <= v4b;
            i_sum <= i_sc + dc_i;            q_sum <= q_sc + dc_q;            v6 <= v5;
            i_out <= (i_sum > MAXO) ? MAXO[OUT_W-1:0] : (i_sum < MINO) ? MINO[OUT_W-1:0] : i_sum[OUT_W-1:0];
            q_out <= (q_sum > MAXO) ? MAXO[OUT_W-1:0] : (q_sum < MINO) ? MINO[OUT_W-1:0] : q_sum[OUT_W-1:0];
            iq_valid <= v6;
        end
    end

    skypluto_sincos #(
        .ADDR_W(LUT_ADDR_W), .DATA_W(OUT_W), .LUT_FILE(LUT_FILE)
    ) u_sincos (
        .clk(clk), .phase(lut_phase), .sin_out(sin_raw), .cos_out(cos_raw)
    );

endmodule
