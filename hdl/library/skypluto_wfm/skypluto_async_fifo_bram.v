// =============================================================================
// skypluto_async_fifo_bram - asynchronous FIFO (CDC) in block RAM, first-word-fall-through
// -----------------------------------------------------------------------------
// Same ports and behaviour as skypluto_async_fifo, for deep FIFOs (the DAB phase words and the audio to the ARM). skypluto_async_fifo
// reads its memory asynchronously, which Vivado builds from flip-flops: fine at 64 words, thousands of slices at 1024. Here the
// memory is read synchronously (block RAM) into an output register; rd_data/rd_empty describe that register, so a pop (rd_en) may
// follow on every clock and the next word is on rd_data one clock after the pop, as before.
// Gray-code pointers (Cummings). Depth = 2^AW words in RAM + 1 in the output register.
// =============================================================================
`timescale 1ns/1ps

module skypluto_async_fifo_bram #(
    parameter integer DW = 32,
    parameter integer AW = 10
)(
    // write domain
    input  wire           wr_clk,
    input  wire           wr_rst,     // async, active high
    input  wire           wr_en,
    input  wire [DW-1:0]  wr_data,
    output wire           wr_full,
    // read domain
    input  wire           rd_clk,
    input  wire           rd_rst,     // async, active high
    input  wire           rd_en,
    output wire [DW-1:0]  rd_data,    // FWFT: the head of the FIFO, valid when !rd_empty; rd_en pops it
    output wire           rd_empty,
    output reg  [AW:0]    rd_count,   // words in RAM + output register (read domain, registered, saturates at 2^AW)
    output reg  [AW:0]    wr_pos      // write pointer as seen in the read domain (registered)
);

    (* ram_style = "block" *) reg [DW-1:0] mem [0:(1 << AW)-1];

    reg  [AW:0] wr_bin = 0, wr_gray = 0;
    reg  [AW:0] rd_bin = 0, rd_gray = 0;
    reg  [AW:0] wr_gray_rd0 = 0, wr_gray_rd1 = 0;  // wr_gray -> rd_clk
    reg  [AW:0] rd_gray_wr0 = 0, rd_gray_wr1 = 0;  // rd_gray -> wr_clk

    function [AW:0] bin2gray(input [AW:0] b); bin2gray = b ^ (b >> 1); endfunction
    function [AW:0] gray2bin(input [AW:0] g);
        integer i; begin
            gray2bin[AW] = g[AW];
            for (i=AW-1; i>=0; i=i-1) gray2bin[i] = gray2bin[i+1] ^ g[i];
        end
    endfunction

    // ---- write side (registered full, as skypluto_async_fifo) ------------------
    reg wr_full_r = 1'b0;
    assign wr_full = wr_full_r;
    wire [AW:0] wr_bin_nxt  = wr_bin + (wr_en & ~wr_full_r);
    wire [AW:0] wr_gray_nxt = bin2gray(wr_bin_nxt);
    wire        wr_full_val = (wr_gray_nxt == {~rd_gray_wr1[AW:AW-1], rd_gray_wr1[AW-2:0]});

    always @(posedge wr_clk) if (wr_en & ~wr_full_r) mem[wr_bin[AW-1:0]] <= wr_data;   // RAM write port (no reset)

    always @(posedge wr_clk or posedge wr_rst) begin
        if (wr_rst) begin
            wr_bin <= 0; wr_gray <= 0; rd_gray_wr0 <= 0; rd_gray_wr1 <= 0; wr_full_r <= 1'b0;
        end else begin
            wr_bin <= wr_bin_nxt; wr_gray <= wr_gray_nxt;
            rd_gray_wr0 <= rd_gray; rd_gray_wr1 <= rd_gray_wr0;
            wr_full_r <= wr_full_val;
        end
    end

    // ---- read side: RAM -> output register ------------------------------------
    wire [AW:0] wr_bin_rd = gray2bin(wr_gray_rd1);
    wire        ram_empty = (wr_gray_rd1 == rd_gray);
    // words in RAM, modulo the pointer width: the difference must be taken in AW+1 bits (in the 32-bit context of a comparison it
    // goes negative, i.e. huge, each time the write pointer has wrapped and the read pointer not yet)
    wire [AW:0] ram_fill  = wr_bin_rd - rd_bin;
    reg         out_valid = 1'b0;
    reg [DW-1:0] out_q;
    wire        fetch = ~ram_empty & (~out_valid | rd_en);
    assign rd_data  = out_q;
    assign rd_empty = ~out_valid;

    always @(posedge rd_clk) if (fetch) out_q <= mem[rd_bin[AW-1:0]];                 // RAM read port (synchronous: block RAM)

    always @(posedge rd_clk or posedge rd_rst) begin
        if (rd_rst) begin
            rd_bin <= 0; rd_gray <= 0; wr_gray_rd0 <= 0; wr_gray_rd1 <= 0;
            out_valid <= 1'b0; rd_count <= 0; wr_pos <= 0;
        end else begin
            wr_gray_rd0 <= wr_gray; wr_gray_rd1 <= wr_gray_rd0;
            if (fetch) begin rd_bin <= rd_bin + 1'b1; rd_gray <= bin2gray(rd_bin + 1'b1); end
            if (fetch) out_valid <= 1'b1; else if (rd_en) out_valid <= 1'b0;
            rd_count <= (ram_fill + out_valid > (1 << AW)) ? (1 << AW) : ram_fill + out_valid;
            wr_pos <= wr_bin_rd;
        end
    end

endmodule
