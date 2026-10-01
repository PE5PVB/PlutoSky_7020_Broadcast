// =============================================================================
// skypluto_async_fifo - asynchronous FIFO (CDC) with gray-code pointers
// -----------------------------------------------------------------------------
// Classic two-clock-domain design (Cummings). Used for the I2S BCLK -> l_clk crossing.
// Depth = 2^AW. Self-contained (no Xilinx IP needed).
// =============================================================================
`timescale 1ns/1ps

module skypluto_async_fifo #(
    parameter integer DW = 24,
    parameter integer AW = 6
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
    output wire [DW-1:0]  rd_data,    // FIRST-WORD-FALL-THROUGH: the head of the FIFO is always on rd_data (valid when !rd_empty); rd_en pops it
    output wire           rd_empty,   // REGISTERED (short path to the conditioner); goes 'not empty' at most 1 clock later than the pointers (safe)
    output reg  [AW:0]    rd_count,   // occupancy (read domain, REGISTERED - short path)
    output reg  [AW:0]    wr_pos      // write pointer as seen in the read domain (REGISTERED): keeps moving as long as the I2S is writing
);

    localparam integer DEPTH = (1 << AW);

    reg [DW-1:0] mem [0:DEPTH-1];

    // ---- pointers (binair + gray) --------------------------------------------
    reg  [AW:0] wr_bin = 0, wr_gray = 0;
    reg  [AW:0] rd_bin = 0, rd_gray = 0;

    // pointers synchronized into the other domain
    reg  [AW:0] wr_gray_rd0 = 0, wr_gray_rd1 = 0;  // wr_gray -> rd_clk
    reg  [AW:0] rd_gray_wr0 = 0, rd_gray_wr1 = 0;  // rd_gray -> wr_clk

    function [AW:0] bin2gray(input [AW:0] b); bin2gray = b ^ (b >> 1); endfunction
    function [AW:0] gray2bin(input [AW:0] g);
        integer i; begin
            gray2bin[AW] = g[AW];
            for (i=AW-1; i>=0; i=i-1) gray2bin[i] = gray2bin[i+1] ^ g[i];
        end
    endfunction

    // occupancy in the read domain: (synchronized wr pointer) - rd pointer.
    // REGISTERED (isolates the gray2bin+subtract chain -> short path for the rate loop).
    wire [AW:0] wr_bin_rd = gray2bin(wr_gray_rd1);

    // ---- write side -----------------------------------------------------------
    // wr_full is REGISTERED (Cummings). The counter uses the registered
    // full, not the combinational one -> no combinational loop (DRC LUTLP-1).
    reg wr_full_r = 1'b0;
    assign wr_full = wr_full_r;

    wire [AW:0] wr_bin_nxt  = wr_bin + (wr_en & ~wr_full_r);
    wire [AW:0] wr_gray_nxt = bin2gray(wr_bin_nxt);
    // full when next-wr-gray == rd-gray with the top 2 bits inverted
    wire        wr_full_val = (wr_gray_nxt == {~rd_gray_wr1[AW:AW-1], rd_gray_wr1[AW-2:0]});

    always @(posedge wr_clk or posedge wr_rst) begin
        if (wr_rst) begin
            wr_bin <= 0; wr_gray <= 0;
            rd_gray_wr0 <= 0; rd_gray_wr1 <= 0;
            wr_full_r <= 1'b0;
        end else begin
            if (wr_en & ~wr_full_r)
                mem[wr_bin[AW-1:0]] <= wr_data;
            wr_bin  <= wr_bin_nxt;
            wr_gray <= wr_gray_nxt;
            rd_gray_wr0 <= rd_gray;      // sync rd pointer in
            rd_gray_wr1 <= rd_gray_wr0;
            wr_full_r <= wr_full_val;
        end
    end

    // FWFT: rd_data = mem[rd_bin] (asynchronous read = distributed RAM). The gray-pointer synchronization guarantees that the head only counts as 'not empty'
    // once it has been written. No reset on the memory contents (an asynchronous reset on a RAM would turn it into 1536 separate registers).
    // synthesis translate_off
    integer mi; initial for (mi = 0; mi < DEPTH; mi = mi + 1) mem[mi] = 0;
    // synthesis translate_on
    assign rd_data = mem[rd_bin[AW-1:0]];

    // ---- read side -------------------------------------------------------------
    wire [AW:0] rd_bin_nxt  = rd_bin + (rd_en & ~rd_empty);
    wire [AW:0] rd_gray_nxt = bin2gray(rd_bin_nxt);

    // rd_empty is a register: empty when the next read pointer equals the (synchronized) write pointer. The write pointer only grows, so a
    // stale value can at worst report 'empty too long', never 'not empty' wrongly. After reset: empty.
    reg rd_empty_r = 1'b1;
    assign rd_empty = rd_empty_r;

    always @(posedge rd_clk or posedge rd_rst) begin
        if (rd_rst) begin
            rd_bin <= 0; rd_gray <= 0;
            wr_gray_rd0 <= 0; wr_gray_rd1 <= 0;
            rd_count <= 0; wr_pos <= 0; rd_empty_r <= 1'b1;
        end else begin
            rd_bin  <= rd_bin_nxt;
            rd_gray <= rd_gray_nxt;
            rd_empty_r <= (rd_gray_nxt == wr_gray_rd1);
            wr_gray_rd0 <= wr_gray;      // sync wr pointer in
            wr_gray_rd1 <= wr_gray_rd0;
            rd_count <= wr_bin_rd - rd_bin;
            wr_pos   <= wr_bin_rd;
        end
    end

endmodule
