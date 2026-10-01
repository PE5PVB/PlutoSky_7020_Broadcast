// Unit TB for skypluto_sigcond. Upstream = simple FWFT FIFO (writes at 192.1875 kHz), downstream = 'interpolator'
// that pulls a sample every ~64 clocks. Scenarios (-P tb_sigcond.SCEN=n):
//   0 transparency: tone 0.9 FS, ceiling = full scale         -> output = delayed input, bit-exact
//   1 limiter:      tone 0.98 FS, ceiling 0.5 FS + sudden peak -> |y| <= ceiling, smooth
//   2 loss:         tone 0.5 FS, I2S gone for 2000 samples    -> fade to 0 and back, no steps, no missed pulls
`timescale 1ns/1ps
module tb_sigcond;
    parameter integer SCEN = 0;
    localparam integer DW = 24;
    localparam real    FS  = 192187.5;
    localparam real    TCK = 81.380;                  // 12.288 MHz
    localparam real    PI  = 3.14159265358979;

    reg clk = 0, rst = 1;
    always #(TCK/2.0) clk = ~clk;

    // ---- upstream FIFO-model ----
    reg signed [DW-1:0] mem [0:1023];
    reg [10:0] wp = 0, rp = 0;
    wire up_empty = (wp == rp);
    wire signed [DW-1:0] up_data = mem[rp[9:0]];
    wire [6:0] up_count = wp[6:0] - rp[6:0];
    wire [6:0] up_wrpos = wp[6:0];
    wire up_rd;
    always @(posedge clk) if (up_rd && !up_empty) rp <= rp + 1'b1;

    // ---- DUT ----
    wire signed [DW-1:0] q_data; wire q_empty; reg q_rd = 0;
    reg [DW-1:0] ceil = 24'h7FFFFF; reg lim_en = 1, fade_en = 1, clr = 0;
    wire [16:0] gmin; wire [31:0] events, uf_events; wire [DW-1:0] inpeak; wire [1:0] state; wire [16:0] fade_o;
    skypluto_sigcond #(.DW(DW)) dut (
        .clk(clk), .rst(rst), .up_data(up_data), .up_empty(up_empty), .up_count(up_count), .up_wrpos(up_wrpos), .up_rd(up_rd),
        .q_data(q_data), .q_empty(q_empty), .q_rd(q_rd), .ceil(ceil), .lim_en(lim_en), .fade_en(fade_en), .clr(clr),
        .gmin(gmin), .events(events), .inpeak(inpeak), .state(state), .uf_events(uf_events), .fade_o(fade_o));

    // ---- writer (I2S side): 192.1875 kHz with fractional phase ----
    reg [23:0] wacc = 0; wire [24:0] wsum = {1'b0, wacc} + 25'd262400;
    integer kk = 0;
    reg writing = 0;
    real amp = 0.9, s;
    integer fin, fout;
    always @(posedge clk) if (!rst) begin
        wacc <= wsum[23:0];
        if (wsum[24] && writing) begin
            s = amp * $sin(2.0 * PI * 1000.0 * kk / FS);
            if (SCEN == 1 && kk > 3000 && kk < 3020) s = 0.98 * ((kk & 1) ? 1.0 : -1.0);   // sharp peak
            mem[wp[9:0]] <= $rtoi(s * 8388607.0);
            wp <= wp + 1'b1;
            $fwrite(fin, "%0d %0d\n", kk, $rtoi(s * 8388607.0));
            kk <= kk + 1;
        end
    end

    // ---- 'interpolator': pulls a sample every ~64 clocks ----
    reg [23:0] pacc = 0; wire [24:0] psum = {1'b0, pacc} + 25'd262400;
    integer pk = 0, missed = 0;
    reg take = 0, pull_en = 0;
    always @(posedge clk) if (!rst && !pull_en && up_count >= 7'd32) pull_en <= 1'b1;   // the interpolator waits until the FIFO is half full
    always @(posedge clk) if (!rst) begin
        pacc <= psum[23:0];
        take <= 0; q_rd <= take;                    // q_rd = 1 clock AFTER the take (like the real interpolator)
        if (psum[24] && pull_en) begin
            if (q_empty) missed = missed + 1;
            else begin take <= 1; $fwrite(fout, "%0d %0d %0d %0d\n", pk, q_data, state, fade_o); pk = pk + 1; end
        end
    end

    integer i;
    initial begin
        fin  = $fopen("sc_in.txt",  "w");
        fout = $fopen("sc_out.txt", "w");
        for (i = 0; i < 1024; i = i + 1) mem[i] = 0;
        if (SCEN == 1) begin amp = 0.98; ceil = 24'd4194304; end          // ceiling 0.5 FS
        if (SCEN == 2) begin amp = 0.5;  ceil = 24'd7549747; end          // ceiling 0.9 FS
        repeat (50) @(posedge clk); rst = 0;
        // fill the FIFO to half first (as the interpolator waits until half full)
        writing = 1;
        if (SCEN == 2) begin
            wait (kk == 4000);
            writing = 0;                                                  // I2S gone
            wait (pk == 2000 + 2500);
            writing = 1;                                                  // I2S back
            wait (kk == 4000 + 1500);
            repeat (64 * 600) @(posedge clk);
        end else begin
            wait (kk == 5000);
            repeat (64 * 700) @(posedge clk);
        end
        $fclose(fin); $fclose(fout);
        $display("SCEN %0d: gemiste pulls=%0d, events=%0d, gmin=%0d, uf_events=%0d, inpeak=%0d, state=%0d", SCEN, missed, events, gmin, uf_events, inpeak, state);
        $finish;
    end
endmodule
