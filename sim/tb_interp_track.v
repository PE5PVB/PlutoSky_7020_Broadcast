// Integration TB: async FIFO + skypluto_interp with adaptive STEP.
// Pico write rate deliberately offset by +PPM -> proves that the loop tracks:
// FIFO stays around half full, no over/underflow, output tone = input tone.
`timescale 1ns/1ps
module tb_interp_track;
    localparam DW=24, AW=6, FRAC=24;
    localparam real FS_NOM = 192187.5;
    localparam real PPM    = 1000.0;         // Pico faster than nominal (large, for fast sim)
    localparam real TRD    = 81.380;         // l_clk period (12.288 MHz), ns
    localparam real FTONE  = 10000.0;

    reg rd_clk=0, wr_clk=0, rd_rst=1, wr_rst=1;
    real fs_wr, wrhalf, pi;
    integer kk;

    // clocks
    initial begin pi=3.14159265358979; fs_wr=FS_NOM*(1.0+PPM/1e6); wrhalf=1e9/fs_wr/2.0; end
    always #(TRD/2.0) rd_clk = ~rd_clk;
    always #(wrhalf)  wr_clk = ~wr_clk;

    // ---- write side (Pico) ----
    reg  signed [DW-1:0] wr_data;
    wire wr_full;
    reg  wr_en=0;
    always @(posedge wr_clk or posedge wr_rst) begin
        if (wr_rst) begin kk<=0; wr_en<=0; wr_data<=0; end
        else begin
            wr_en <= 1'b1;
            wr_data <= $rtoi(0.30*8388607.0*$sin(2.0*pi*FTONE*kk/fs_wr));
            if (wr_en & ~wr_full) kk <= kk+1;
        end
    end

    // ---- FIFO + interpolator ----
    wire signed [DW-1:0] fdata; wire fempty; wire [AW:0] fcount; wire frd;
    wire signed [DW-1:0] comp_out; wire comp_valid;

    skypluto_async_fifo #(.DW(DW), .AW(AW)) u_fifo (
        .wr_clk(wr_clk), .wr_rst(wr_rst), .wr_en(wr_en), .wr_data(wr_data), .wr_full(wr_full),
        .rd_clk(rd_clk), .rd_rst(rd_rst), .rd_en(frd), .rd_data(fdata),
        .rd_empty(fempty), .rd_count(fcount));

    skypluto_interp #(.DW(DW), .FRAC(FRAC), .CNTW(AW+1), .LP_SHIFT(8), .RANGE(2048),
                      .COEF_FILE("data/interp_coefs.mem")) u_int (
        .clk(rd_clk), .rst(rd_rst), .step(24'd262400),
        .fifo_data(fdata), .fifo_empty(fempty), .fifo_count(fcount), .fifo_rd(frd),
        .comp_out(comp_out), .comp_valid(comp_valid));

    // ---- monitors + capture ----
    integer f, cnt, cmax, cmin, ovf_w, unf_r;
    initial begin
        cmax=0; cmin=999; ovf_w=0; unf_r=0;
        repeat (60) @(posedge rd_clk); rd_rst=0;
        repeat (60) @(posedge wr_clk); wr_rst=0;
        // let the loop converge (FIFO fill dynamics are slow)
        repeat (1000000) @(posedge rd_clk);
        // monitor + dump
        f=$fopen("track_out.txt","w");
        for (cnt=0; cnt<65536; cnt=cnt+1) begin
            @(posedge rd_clk);
            if (fcount>cmax) cmax=fcount;
            if (fcount<cmin) cmin=fcount;
            if (wr_full) ovf_w=ovf_w+1;
            if (fempty)  unf_r=unf_r+1;
            $fwrite(f,"%0d\n", comp_out);
        end
        $fclose(f);
        $display("TRACK: fifo_count min=%0d max=%0d  wr_full-events=%0d  rd_empty-events=%0d  step_dyn=%0d",
                 cmin, cmax, ovf_w, unf_r, u_int.step_dyn);
        $finish;
    end
endmodule
