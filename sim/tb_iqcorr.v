// tb_iqcorr: the digital I/Q corrections of skypluto_fm_modulator. Two modulators get the same carrier (an NCO offset, no audio); the second one has a DC offset,
// a Q gain error and an I/Q skew set. For every sample it must hold (within 2 LSB):
//    I2 = I1 + dc_i
//    Q2 = Q1 + Q1*qgain/2^18 + I1*qskew/2^18 + dc_q
// and with all corrections 0 the two must be identical.
`timescale 1ns/1ps
module tb_iqcorr;
    reg clk = 0; always #5 clk = ~clk;
    reg rst = 1;
    wire signed [15:0] i1, q1, i2, q2;
    wire v1, v2;
    reg signed [11:0] dci = 0, dcq = 0;
    reg signed [17:0] qg = 0, qs = 0;

    skypluto_fm_modulator #(.KSHIFT(13)) m1 (
        .clk(clk), .rst(rst), .en(1'b1), .kdev(18'sd100), .offset_inc(24'sd136533), .level(16'd56000),
        .dc_i(12'sd0), .dc_q(12'sd0), .qgain(18'sd0), .qskew(18'sd0), .comp(24'sd0), .comp_valid(1'b1),
        .i_out(i1), .q_out(q1), .iq_valid(v1));
    skypluto_fm_modulator #(.KSHIFT(13)) m2 (
        .clk(clk), .rst(rst), .en(1'b1), .kdev(18'sd100), .offset_inc(24'sd136533), .level(16'd56000),
        .dc_i(dci), .dc_q(dcq), .qgain(qg), .qskew(qs), .comp(24'sd0), .comp_valid(1'b1),
        .i_out(i2), .q_out(q2), .iq_valid(v2));

    integer n = 0, bad = 0, checked = 0, maxerr = 0;
    real ei, eq, di, dq;
    integer phase_idx = 0;
    task check(input integer tag);
        begin
            repeat (3000) @(posedge clk);
            repeat (2000) begin
                @(posedge clk);
                if (v1 && v2) begin
                    ei = $itor(i1) + $itor(dci);
                    eq = $itor(q1) + $itor(q1) * $itor(qg) / 262144.0 + $itor(i1) * $itor(qs) / 262144.0 + $itor(dcq);
                    di = i2 - ei; dq = q2 - eq;
                    if (di < 0) di = -di; if (dq < 0) dq = -dq;
                    checked = checked + 1;
                    if (di > maxerr) maxerr = di; if (dq > maxerr) maxerr = dq;
                    if (di > 2.0 || dq > 2.0) begin
                        bad = bad + 1;
                        if (bad < 6) $display("  MISMATCH tag %0d: I1=%0d Q1=%0d  I2=%0d (expected %f)  Q2=%0d (expected %f)", tag, i1, q1, i2, ei, q2, eq);
                    end
                end
            end
        end
    endtask

    initial begin
        #100 rst = 0;
        check(0);                                           // no correction: identical
        dci = 12'sd37;  dcq = -12'sd21;  qg = 18'sd0;      qs = 18'sd0;      check(1);
        dci = 12'sd0;   dcq = 12'sd0;    qg = 18'sd2621;   qs = 18'sd0;      check(2);   // gain +1 %
        dci = 12'sd0;   dcq = 12'sd0;    qg = 18'sd0;      qs = -18'sd1311;  check(3);   // skew -0.5 %
        dci = -12'sd150; dcq = 12'sd88;  qg = -18'sd5243;  qs = 18'sd7864;   check(4);   // everything
        dci = 12'sd0;   dcq = 12'sd0;    qg = 18'sd0;      qs = 18'sd0;      check(5);   // back to none
        $display("tb_iqcorr: %0d samples checked, %0d out of tolerance, max error %0d LSB -> %s", checked, bad, maxerr, (bad == 0) ? "PASS" : "FAIL");
        $finish;
    end
endmodule
