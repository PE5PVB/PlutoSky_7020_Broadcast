// Testbench for skypluto_interp: feed 1 kHz + 76 kHz (via mock FIFO at 192,187.5 Hz),
// dump comp_out (on l_clk) to a file for Python FFT analysis.
`timescale 1ns/1ps
module tb_interp;
    localparam DW=24, FRAC=24;
    reg clk=0, rst=1;
    reg signed [DW-1:0] fifo_data;
    reg  fifo_empty=0;
    wire fifo_rd;
    wire signed [DW-1:0] comp_out;
    wire comp_valid;

    skypluto_interp #(.DW(DW), .FRAC(FRAC), .COEF_FILE("data/interp_coefs.mem")) dut (
        .clk(clk), .rst(rst), .step(24'd262400),
        .fifo_data(fifo_data), .fifo_empty(fifo_empty), .fifo_count(7'd32), .fifo_rd(fifo_rd),
        .comp_out(comp_out), .comp_valid(comp_valid));

    always #5 clk = ~clk;            // 100 MHz sim clock (the ratio matters, not the abs. freq)

    // mock FIFO: index n, generate 1kHz + 76kHz at fs_in=192187.5
    real fs_in; integer n; real pi;
    real s;
    initial begin fs_in=192187.5; n=0; pi=3.14159265358979; end
    task set_data; begin
        s = 0.40*$sin(2.0*pi*1000.0 *n/fs_in)
          + 0.40*$sin(2.0*pi*76000.0*n/fs_in);
        fifo_data = $rtoi(s*8388607.0);
    end endtask
    always @(posedge clk) if (fifo_rd) begin n = n+1; set_data; end

    integer f, cnt;
    initial begin
        set_data;
        f = $fopen("interp_out.txt","w");
        repeat (40) @(posedge clk); rst=0;   // reset
        // let it settle
        repeat (200000) @(posedge clk);
        // dump 65536 output samples
        for (cnt=0; cnt<65536; cnt=cnt+1) begin
            @(posedge clk);
            $fwrite(f, "%0d\n", comp_out);
        end
        $fclose(f);
        $display("TB klaar: 65536 samples gedumpt (n=%0d inputsamples verbruikt)", n);
        $finish;
    end
endmodule
