// =============================================================================
// tb_i2s_rx - testbench for skypluto_i2s_rx: sweeps over ALL formats
//   alignment  : standard I2S, left-justified, right-justified (zero-padded and with sign extension)
//   word width : 16, 20, 24, 32 bit
//   slot length: 16, 24, 32, 64 BCLKs (only combinations where word width <= slot)
// The formats follow one another without reset; for each format the receiver must recognize itself and return the last frames
// exactly (left-aligned to 24 bit). Afterwards: manual override.
// Run: iverilog -g2012 -o tb_i2s_rx.vvp sim/tb_i2s_rx.v hdl/library/skypluto_wfm/skypluto_i2s_rx.v && vvp tb_i2s_rx.vvp
// =============================================================================
`timescale 1ns/1ps

module tb_i2s_rx;

    localparam integer DATA_W = 24;
    localparam integer FRAMES = 3600;         // frames per format (3 evaluation windows of 1024 frames + margin)
    localparam integer CHECK  = 40;           // last frames that are checked

    reg  bclk = 1'b0;
    reg  ws   = 1'b1;
    reg  sd   = 1'b0;
    reg        man_en = 1'b0;
    reg  [1:0] man_mode = 2'd0;
    reg  [7:0] man_bits = 8'd24;

    wire signed [DATA_W-1:0] left, right;
    wire                     valid;
    wire [7:0] fmt_slot, fmt_bits;
    wire [1:0] fmt_mode;

    integer errors = 0;
    integer valid_count = 0;

    always #40.69 bclk = ~bclk;
    always @(posedge bclk) if (valid) valid_count = valid_count + 1;

    skypluto_i2s_rx #(.DATA_W(DATA_W), .WS_TO_MSB(1), .EVAL_LOG2(11)) dut (
        .bclk(bclk), .ws(ws), .sd(sd),
        .man_en(man_en), .man_mode(man_mode), .man_bits(man_bits),
        .left(left), .right(right), .valid(valid),
        .fmt_slot(fmt_slot), .fmt_bits(fmt_bits), .fmt_mode(fmt_mode)
    );

    real pi; initial pi = 3.14159265358979;

    // n-bit signed word for frame i, channel ch (sine 0.4 FS + second tone: audio-like, MSB = next bit almost always)
    function signed [63:0] samp(input integer i, input integer ch, input integer n);
        real v, fs;
        begin
            fs = 2.0 ** (n - 1) - 1.0;
            v  = 0.4 * $sin(2.0*pi*i/97.0 + ch) + 0.05 * $sin(2.0*pi*i/7.3);
            samp = $rtoi(v * fs);
        end
    endfunction

    // bit at BCLK position p of a slot (p = 0 : cycle of the WS edge); prev_d0 = LSB of the previous slot (I2S packing)
    function bitat(input integer p, input signed [63:0] d, input prev_d0, input integer n, input integer slot, input integer fmt);
        integer k;
        begin
            bitat = 1'b0;
            if (fmt == 0) begin
                if (p == 0) bitat = (n == slot) ? prev_d0 : 1'b0;
                else begin k = p - 1; if (k < n) bitat = d[n-1-k]; end
            end else if (fmt == 1) begin
                k = p; if (k < n) bitat = d[n-1-k];
            end else if (fmt == 2) begin
                k = p - (slot - n); if (k >= 0 && k < n) bitat = d[n-1-k];
            end else begin                                         // 3 = right-justified WITH sign extension
                k = p - (slot - n); bitat = (k < 0) ? d[n-1] : d[n-1-k];
            end
        end
    endfunction

    reg prev0 = 1'b0;

    task send_slot(input ws_val, input signed [63:0] d, input integer n, input integer slot, input integer fmt);
        integer p;
        begin
            for (p = 0; p < slot; p = p + 1) begin
                @(negedge bclk);
                if (p == 0) ws = ws_val;
                sd = bitat(p, d, prev0, n, slot, fmt);
            end
            prev0 = d[0];
        end
    endtask

    function signed [DATA_W-1:0] expect24(input signed [63:0] d, input integer n);
        begin
            if (n >= DATA_W) expect24 = d >>> (n - DATA_W);
            else             expect24 = d <<< (DATA_W - n);
        end
    endfunction

    integer fmt, slot, n, i, nerr_cfg, ncfg = 0, nfail = 0;
    reg signed [63:0] dl, dr, dl_prev, dr_prev;
    integer slots_l [0:3]; integer bits_l [0:3];

    task run_cfg(input integer fm, input integer sl, input integer nb);
        begin
            nerr_cfg = 0;
            for (i = 0; i < FRAMES; i = i + 1) begin
                dl = samp(i, 0, nb); dr = samp(i, 1, nb);
                send_slot(1'b0, dl, nb, sl, fm);
                // is left of the previous frame latched now? (latches at start of R) - checked below at the next L
                if (i >= FRAMES - CHECK && i > 0) begin
                    // at the start of this L slot, right of the previous frame has been latched; left of the previous one at the start of the previous R slot
                    if (left !== expect24(dl_prev, nb) || right !== expect24(dr_prev, nb)) begin
                        if (nerr_cfg < 2) $display("  FOUT fmt=%0d slot=%0d n=%0d i=%0d: L=%h (verw %h) R=%h (verw %h) herkend: mode=%0d bits=%0d slot=%0d",
                            fm, sl, nb, i, left, expect24(dl_prev, nb), right, expect24(dr_prev, nb), fmt_mode, fmt_bits, fmt_slot);
                        nerr_cfg = nerr_cfg + 1;
                    end
                end
                send_slot(1'b1, dr, nb, sl, fm);
                dl_prev = dl; dr_prev = dr;
            end
            ncfg = ncfg + 1;
            if (nerr_cfg != 0) begin nfail = nfail + 1; errors = errors + nerr_cfg; end
            else $display("  ok  fmt=%0d slot=%0d n=%0d -> herkend mode=%0d bits=%0d slot=%0d", fm, sl, nb, fmt_mode, fmt_bits, fmt_slot);
        end
    endtask

    integer si, ni;
    initial begin
        slots_l[0] = 16; slots_l[1] = 24; slots_l[2] = 32; slots_l[3] = 64;
        bits_l[0]  = 16; bits_l[1]  = 20; bits_l[2]  = 24; bits_l[3]  = 32;
        // standard 24 bit / 32 slot first (the production situation), then the rest
        for (fmt = 0; fmt < 4; fmt = fmt + 1)
            for (si = 0; si < 4; si = si + 1)
                for (ni = 0; ni < 4; ni = ni + 1)
                    if (bits_l[ni] <= slots_l[si])
                        run_cfg(fmt, slots_l[si], bits_l[ni]);

        // manual override: send RJ 20 bit in a 32 slot while auto is off and RJ/20 is selected manually -> must be received correctly
        man_en = 1'b1; man_mode = 2'd2; man_bits = 8'd20;
        run_cfg(2, 32, 20);

        if (valid_count < 1000) begin $display("FOUT: valid pulseerde te weinig (%0d)", valid_count); errors = errors + 1; end
        if (errors == 0) $display("PASS: %0d formaten automatisch herkend en correct ontvangen (valid x%0d).", ncfg, valid_count);
        else             $display("FAIL: %0d fouten in %0d van %0d formaten.", errors, nfail, ncfg);
        $finish;
    end

    initial begin
        #2000000000 $display("TIMEOUT"); $finish;
    end

endmodule
