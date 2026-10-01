// =============================================================================
// skypluto_sincos - simultaneous sin/cos via ROM LUT
// -----------------------------------------------------------------------------
// Full-wave sine table (2^ADDR_W entries) in BRAM, initialized from
// data/sine_lut.mem (generate with scripts/gen_sine_lut.py).
//   sin_out = sin(2*pi * phase / 2^ADDR_W)
//   cos_out = sin(phase + 90deg) = table[phase + 2^ADDR_W/4]
// 1 clock latency (registered outputs).
//
// BRAM cost: 2^ADDR_W * DATA_W bits, read twice -> 1 dual-port BRAM
// (e.g. 4096*16 = 64 kbit -> 2x 36k or 1x 36k dual-port). Ample room on the 7020.
// =============================================================================
`timescale 1ns/1ps

module skypluto_sincos #(
    parameter integer ADDR_W   = 14,               // phase address width (0..2pi); 14 bit = 16384 entries (8 BRAM36)
    parameter integer DATA_W   = 16,               // output width (signed)
    parameter         LUT_FILE = "sine_lut.mem"
)(
    input  wire                     clk,
    input  wire        [ADDR_W-1:0] phase,
    output reg  signed [DATA_W-1:0] sin_out,
    output reg  signed [DATA_W-1:0] cos_out
);

    (* rom_style = "block" *)
    reg signed [DATA_W-1:0] rom [0:(1<<ADDR_W)-1];

    initial begin
        $readmemh(LUT_FILE, rom);
    end

    // +90 degrees = +1/4 period
    wire [ADDR_W-1:0] cos_addr = phase + {2'b01, {(ADDR_W-2){1'b0}}};

    always @(posedge clk) begin
        sin_out <= rom[phase];
        cos_out <= rom[cos_addr];
    end

endmodule
