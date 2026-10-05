// =============================================================================
// tb_i2s_lowfreq - format detection of skypluto_i2s_rx with a low-frequency composite
//   Standard I2S, 24 bit in 32-bit slots (the PicoAudio format). The composite is a 20 Hz tone at 0.2 FS plus a 19 kHz pilot at 0.09 FS
//   (192 kHz frames): for ~10 ms around each crest every sample is positive, so the sign bit stays 0 for several evaluation windows. The
//   receiver must keep the format (I2S 24/32) and return every frame exactly. Counts wrong frames and format changes over 0.6 s.
// Run: iverilog -g2012 -o tb_i2s_lowfreq.vvp sim/tb_i2s_lowfreq.v hdl/library/skypluto_wfm/skypluto_i2s_rx.v && vvp tb_i2s_lowfreq.vvp
// =============================================================================
`timescale 1ns/1ps

module tb_i2s_lowfreq;

    localparam integer DATA_W = 24;
    localparam integer FRAMES = 115200;       // 0.6 s at 192 kHz (12 periods of 20 Hz)
    localparam integer SKIP   = 1000;         // first frames are not checked

    reg  bclk = 1'b0;
    reg  ws   = 1'b1;
    reg  sd   = 1'b0;

    wire signed [DATA_W-1:0] left, right;
    wire                     valid;
    wire [7:0] fmt_slot, fmt_bits;
    wire [1:0] fmt_mode;

    always #40.69 bclk = ~bclk;

    skypluto_i2s_rx #(.DATA_W(DATA_W), .WS_TO_MSB(1)) dut (
        .bclk(bclk), .ws(ws), .sd(sd),
        .man_en(1'b0), .man_mode(2'd0), .man_bits(8'd24),
        .left(left), .right(right), .valid(valid),
        .fmt_slot(fmt_slot), .fmt_bits(fmt_bits), .fmt_mode(fmt_mode)
    );

    real pi; initial pi = 3.14159265358979;
    function signed [23:0] samp(input integer i);
        real v;
        begin
            v = 0.2 * $sin(2.0*pi*20.0*i/192000.0) + 0.09 * $sin(2.0*pi*19000.0*i/192000.0);
            samp = $rtoi(v * 8388607.0);
        end
    endfunction

    reg prev0 = 1'b0;
    task send_slot(input ws_val, input signed [23:0] d);
        integer p;
        begin
            for (p = 0; p < 32; p = p + 1) begin
                @(negedge bclk);
                if (p == 0) ws = ws_val;
                sd = (p == 0) ? 1'b0 : (p - 1 < 24) ? d[23 - (p - 1)] : 1'b0;   // p=0 is padding of the previous slot (24 bit in 32)
            end
            prev0 = d[0];
        end
    endtask

    integer i, nerr = 0, nfmt = 0, worst = 0, d;
    reg signed [23:0] dl, dl_prev;
    reg [1:0] mode_prev = 2'd0; reg [7:0] bits_prev = 8'd24;
    initial begin
        for (i = 0; i < FRAMES; i = i + 1) begin
            dl = samp(i);
            send_slot(1'b0, dl);
            if (i > SKIP && left !== dl_prev) begin
                d = left - dl_prev; if (d < 0) d = -d; if (d > worst) worst = d;
                if (nerr < 3) $display("  FOUT i=%0d: L=%0d verwacht %0d (mode=%0d bits=%0d)", i, left, dl_prev, fmt_mode, fmt_bits);
                nerr = nerr + 1;
            end
            if (i > SKIP && (fmt_mode != mode_prev || fmt_bits != bits_prev)) begin
                nfmt = nfmt + 1;
                if (nfmt < 5) $display("  formaatwissel i=%0d (%.1f ms): mode %0d bits %0d", i, i / 192.0, fmt_mode, fmt_bits);
            end
            mode_prev = fmt_mode; bits_prev = fmt_bits;
            send_slot(1'b1, dl);
            dl_prev = dl;
        end
        if (nerr == 0 && nfmt == 0) $display("PASS: %0d frames 20 Hz + piloot, formaat bleef I2S 24/32, alles exact.", FRAMES);
        else $display("FAIL: %0d foute frames (grootste afwijking %0d LSB = %.1f %% FS), %0d formaatwisselingen.", nerr, worst, worst * 100.0 / 8388608.0, nfmt);
        $finish;
    end
endmodule
