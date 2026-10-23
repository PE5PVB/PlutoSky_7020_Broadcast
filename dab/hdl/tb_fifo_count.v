// tb_fifo_count: skypluto_async_fifo_bram read the way skypluto-dab reads the audio FIFO: take rd_count, pop that many words,
// wait, again. The writer counts up at 48 kHz frame rate in its own clock. Every popped word must be the next number (no repeat,
// no gap) and rd_count may never exceed what is really in the FIFO, also across the pointer wraps (every 2048 words).
`timescale 1ns/1ps
module tb_fifo_count;
    localparam AW = 10;
    reg wclk = 0, rclk = 0;
    always #162.76 wclk = ~wclk;                 // BCLK 3.072 MHz
    always #5 rclk = ~rclk;                      // AXI clock 100 MHz
    reg rst = 1;
    reg wr_en = 0; reg [31:0] wr_data = 0;
    wire wr_full, rd_empty; wire [31:0] rd_data; wire [AW:0] rd_count, wr_pos;
    reg rd_en = 0;
    skypluto_async_fifo_bram #(.DW(32), .AW(AW)) dut (
        .wr_clk(wclk), .wr_rst(rst), .wr_en(wr_en), .wr_data(wr_data), .wr_full(wr_full),
        .rd_clk(rclk), .rd_rst(rst), .rd_en(rd_en), .rd_data(rd_data), .rd_empty(rd_empty), .rd_count(rd_count), .wr_pos(wr_pos));

    // writer: one word per 64 BCLK (one stereo frame)
    reg [5:0] div = 0; integer written = 0;
    always @(posedge wclk) begin
        div <= div + 1'b1; wr_en <= 1'b0;
        if (!rst && div == 0 && !wr_full) begin wr_en <= 1'b1; wr_data <= written; written <= written + 1; end
    end

    integer nexp = 0, errors = 0, maxc = 0, reads = 0, k, n, wraps;
    initial begin
        repeat (20) @(posedge rclk); rst = 0;
        // ~6000 words = about three pointer wraps; read rounds every ~1.1 ms of the writer
        while (nexp < 6000) begin
            repeat (110000) @(posedge rclk);
            @(posedge rclk); n = rd_count;
            if (n > maxc) maxc = n;
            if (n > written - nexp) begin errors = errors + 1; $display("count %0d but only %0d written and not read", n, written - nexp); end
            for (k = 0; k < n; k = k + 1) begin
                // an AXI read: data now, pop pulse one clock later, the next read a few clocks after that
                if (rd_data !== nexp) begin errors = errors + 1; if (errors < 10) $display("word %0d: got %0d", nexp, rd_data); end
                nexp = nexp + 1; reads = reads + 1;
                @(posedge rclk) rd_en <= 1'b1; @(posedge rclk) rd_en <= 1'b0;
                repeat (3) @(posedge rclk);
            end
        end
        wraps = nexp / (1 << (AW + 1));
        $display("%0d words read through %0d pointer wraps, largest rd_count %0d, errors %0d", reads, wraps, maxc, errors);
        if (errors == 0 && maxc < 200) $display("PASS"); else $display("FAIL");
        $finish;
    end
endmodule
