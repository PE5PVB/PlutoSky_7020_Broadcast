// =============================================================================
// tb_fm_modulator - test of the FM modulator with NCO offset and carrier level
// Checks: constant envelope, rotation with DC audio, AND carrier rotation with
// NO audio but an offset (requirement: always a carrier), and level=0 -> silent.
//
// Run (from the repo root):
//   iverilog -g2012 -o sim/tb_fm.vvp sim/tb_fm_modulator.v \
//       hdl/library/skypluto_wfm/skypluto_fm_modulator.v \
//       hdl/library/skypluto_wfm/skypluto_sincos.v
//   vvp sim/tb_fm.vvp
// =============================================================================
`timescale 1ns/1ps

module tb_fm_modulator;

    localparam integer COMP_W  = 24;
    localparam integer OUT_W   = 16;
    localparam integer PHASE_W = 24;
    localparam integer LUTAW   = 12;
    localparam integer KDEV_W  = 18;
    localparam integer LVL_W   = 16;

    reg                        clk = 1'b0;
    reg                        rst = 1'b1;
    reg                        en  = 1'b0;
    reg  signed [KDEV_W-1:0]   kdev = 18'sd50;
    reg  signed [PHASE_W-1:0]  offset_inc = 0;
    reg  [LVL_W-1:0]           level = 16'hFFFF;
    reg  signed [COMP_W-1:0]   comp = 0;
    reg                        comp_valid = 1'b0;

    wire signed [OUT_W-1:0]    i_out, q_out;
    wire                       iq_valid;

    integer errors = 0, nsamp = 0, i_changed = 0;
    reg signed [OUT_W-1:0] prev_i = 0;

    always #5 clk = ~clk;

    skypluto_fm_modulator #(
        .COMP_W(COMP_W), .KDEV_W(KDEV_W), .PHASE_W(PHASE_W),
        .LUT_ADDR_W(LUTAW), .OUT_W(OUT_W), .LVL_W(LVL_W),
        .LUT_FILE("hdl/library/skypluto_wfm/data/sine_lut.mem")
    ) dut (
        .clk(clk), .rst(rst), .en(en), .kdev(kdev),
        .offset_inc(offset_inc), .level(level),
        .dc_i(12'sd0), .dc_q(12'sd0),
        .comp(comp), .comp_valid(comp_valid),
        .i_out(i_out), .q_out(q_out), .iq_valid(iq_valid)
    );

    // envelope-check (level=max -> ~amp)
    localparam real AMP = 32767.0;
    localparam real MAG2_EXP = AMP*AMP;
    real mag2, err_frac;
    always @(posedge clk) begin
        if (iq_valid && !rst && level == 16'hFFFF) begin
            mag2 = $itor(i_out)*$itor(i_out) + $itor(q_out)*$itor(q_out);
            err_frac = (mag2 - MAG2_EXP) / MAG2_EXP;
            if (err_frac < 0) err_frac = -err_frac;
            if (err_frac > 0.02) begin
                $display("FOUT envelope: I=%0d Q=%0d mag2=%.0f (afw %.2f%%)",
                         i_out, q_out, mag2, err_frac*100.0);
                errors = errors + 1;
            end
            if (i_out !== prev_i) i_changed = i_changed + 1;
            prev_i <= i_out;
            nsamp = nsamp + 1;
        end
    end

    initial begin
        $dumpfile("tb_fm_modulator.vcd");
        $dumpvars(0, tb_fm_modulator);
        repeat (4) @(posedge clk);
        rst <= 1'b0;
        comp_valid <= 1'b1;
        en         <= 1'b1;

        // --- Test 1: NO audio (comp=0) + offset -> carrier must rotate -----------
        comp       <= 0;
        offset_inc <= 24'sd50000;   // NCO-offset
        i_changed   = 0;
        repeat (2000) @(posedge clk);
        if (i_changed < 10) begin
            $display("FOUT: geen carrier-rotatie bij offset zonder audio (%0d)", i_changed);
            errors = errors + 1;
        end else
            $display("OK: draaggolf roteert zonder audio (offset), i_changed=%0d", i_changed);

        // --- Test 2: DC audio on top of offset -> envelope still constant --------
        comp <= 24'sd100000;
        repeat (2000) @(posedge clk);

        // --- Test 3: level=0 -> silent (I=Q=0) -----------------------------------
        level <= 16'd0;
        repeat (20) @(posedge clk);
        if (i_out !== 0 || q_out !== 0) begin
            $display("FOUT: level=0 maar output niet nul (I=%0d Q=%0d)", i_out, q_out);
            errors = errors + 1;
        end else
            $display("OK: level=0 -> carrier uit");

        if (errors == 0) $display("PASS: FM-modulator (offset+level) - %0d monsters.", nsamp);
        else             $display("FAIL: %0d fouten.", errors);
        $finish;
    end

    initial begin #2000000 $display("TIMEOUT"); $finish; end

endmodule
