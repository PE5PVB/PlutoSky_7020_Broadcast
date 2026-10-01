// =============================================================================
// skypluto_axi_regs - AXI4-Lite control for the WFM exciter
// -----------------------------------------------------------------------------
// Register map (byte offset, 32-bit):
//   0x00 CTRL          [0]=enable (default 1)                            RW
//   0x04 FREQ_OFFSET   NCO offset relative to zero-IF (signed, low PHASE_W bits)  RW
//   0x08 CARRIER_LEVEL digital carrier amplitude (unsigned, low LVL_W bits) RW
//   0x0C KDEV          FM deviation gain (signed, low KDEV_W bits)       RW
//   0x10 STATUS        [0]=overflow [1]=underflow                        RO
//   0x14 MAGIC         0x57464D32 ("WFM2")                               RO
//   0x18 DC_I          digital DC offset on I (signed, LO-leakage nulling)  RW
//   0x1C DC_Q          digital DC offset on Q (signed)                   RW
//   0x20 LIM_CEIL      limiter peak ceiling (counts, full scale = 2^23)  RW  (default 0x733333 = 0.9 FS)
//   0x24 LIM_CTRL      [0]=limiter on [1]=fade on [2]=clear statistics   RW  (default 3)
//   0x28 LIM_GMIN      smallest g_req since clear (Q16, 65536 = none)    RO
//   0x2C LIM_EVENTS    number of samples with limiting                   RO
//   0x30 IN_PEAK       largest |x| since clear (counts)                  RO
//   0x34 SC_STATE      [1:0]=state (1=RUN) [31:15]=fade Q16 (65536=1.0)  RO
//   0x38 UF_EVENTS     number of times the I2S dropped out              RO
//   0x3C DBG           write: select word 0..31; read: that diagnostic word (see wfm_exciter)  RW
//   0x40 IMP_CTRL      [0]=impulse measurement on: RX1 channels carry timestamps instead of RX1 data, impulse detector active   RW (default 0)
//   0x44 IMP_THR       detection threshold |word| in counts (full scale = 2^23; default 12 % = 0x0F5C28)   RW
//   0x48 IMP_STAMP     l_clk counter value (32 bit) at the last detected impulse                          RO
//   0x4C IMP_COUNT     number of detected impulses                                                        RO
//   0x54 I2S_CTRL      [0]=manual format, [2:1]=alignment (0 I2S, 1 left-justified, 2 right-justified), [15:8]=word width  RW (default 0 = auto)
//   0x50 I2S_FMT       [15:8] measured slot length (BCLKs per WS half period), [7:0] word width in use  RO
// =============================================================================
`timescale 1ns/1ps

module skypluto_axi_regs #(
    parameter integer PHASE_W = 24,
    parameter integer LVL_W   = 16,
    parameter integer KDEV_W  = 18,
    parameter integer DC_W    = 12,
    parameter [PHASE_W-1:0] OFFSET_DEFAULT = 0,
    parameter [LVL_W-1:0]   LEVEL_DEFAULT  = 16'h4000,   // -12 dBFS: clean (little 3rd-order)
    parameter [KDEV_W-1:0]  KDEV_DEFAULT   = 18'sd100  // KSHIFT=13: 100 = +-75 kHz at full-scale MPX (~750 Hz/step)
)(
    input  wire        s_axi_aclk,
    input  wire        s_axi_aresetn,
    input  wire [ 6:0] s_axi_awaddr,
    input  wire        s_axi_awvalid,
    output reg         s_axi_awready,
    input  wire [31:0] s_axi_wdata,
    input  wire [ 3:0] s_axi_wstrb,
    input  wire        s_axi_wvalid,
    output reg         s_axi_wready,
    output reg  [ 1:0] s_axi_bresp,
    output reg         s_axi_bvalid,
    input  wire        s_axi_bready,
    input  wire [ 6:0] s_axi_araddr,
    input  wire        s_axi_arvalid,
    output reg         s_axi_arready,
    output reg  [31:0] s_axi_rdata,
    output reg  [ 1:0] s_axi_rresp,
    output reg         s_axi_rvalid,
    input  wire        s_axi_rready,
    // to fabric
    output wire                     enable,
    output wire signed [PHASE_W-1:0] freq_offset,
    output wire        [LVL_W-1:0]   carrier_level,
    output wire signed [KDEV_W-1:0]  kdev,
    output wire signed [DC_W-1:0]    dc_i,
    output wire signed [DC_W-1:0]    dc_q,
    // from fabric
    input  wire                     status_overflow,
    input  wire                     status_underflow,
    // conditioner (limiter + fade): settings and status
    output wire        [23:0]       lim_ceil,
    output wire                     lim_en,
    output wire                     fade_en,
    output wire                     lim_clr_tog,      // toggles on every 'clear statistics'
    input  wire        [31:0]       st_gmin,          // smallest g_req (Q16) since clear
    input  wire        [31:0]       st_events,        // samples with limiting
    input  wire        [31:0]       st_inpeak,        // largest |x| since clear
    input  wire        [31:0]       st_state,         // [1:0] state, [31:15] fade Q16
    input  wire        [31:0]       st_uf,            // number of times the I2S dropped out
    output wire                     imp_en,           // impulse measurement on
    output wire        [15:0]       i2s_ctrl,         // manual I2S format
    output wire        [23:0]       imp_thr,          // detection threshold (counts)
    input  wire        [31:0]       st_imp_stamp,     // l_clk counter at the last impulse
    input  wire        [31:0]       st_imp_count,     // number of impulses
    input  wire        [31:0]       st_fmt,           // I2S format (slot length, word width)
    output wire        [4:0]        dbg_sel,          // diagnostics: which word (0..31) is read at 0x3C
    input  wire        [31:0]       st_dbg            // the selected diagnostic word
);

    localparam [31:0] MAGIC = 32'h57464D32;

    reg [31:0] reg_ctrl, reg_offset, reg_level, reg_kdev, reg_dci, reg_dcq, reg_lceil, reg_lctrl;
    reg [4:0]  reg_dbgsel;
    reg [31:0] reg_impctrl, reg_impthr, reg_i2sctrl;
    assign i2s_ctrl = reg_i2sctrl[15:0];
    assign imp_en  = reg_impctrl[0];
    assign imp_thr = reg_impthr[23:0];
    reg        clr_tog;
    assign dbg_sel = reg_dbgsel;

    assign enable        = reg_ctrl[0];
    assign freq_offset   = reg_offset[PHASE_W-1:0];
    assign carrier_level = reg_level[LVL_W-1:0];
    assign kdev          = reg_kdev[KDEV_W-1:0];
    assign dc_i          = reg_dci[DC_W-1:0];
    assign dc_q          = reg_dcq[DC_W-1:0];
    assign lim_ceil      = reg_lceil[23:0];
    assign lim_en        = reg_lctrl[0];
    assign fade_en       = reg_lctrl[1];
    assign lim_clr_tog   = clr_tog;

    wire rst = ~s_axi_aresetn;
    wire wr_fire = s_axi_awvalid & s_axi_wvalid & ~s_axi_bvalid;

    always @(posedge s_axi_aclk) begin
        if (rst) begin
            s_axi_awready <= 1'b0; s_axi_wready <= 1'b0;
            s_axi_bvalid  <= 1'b0; s_axi_bresp  <= 2'b00;
            reg_ctrl   <= 32'h1;
            reg_offset <= {{(32-PHASE_W){1'b0}}, OFFSET_DEFAULT};
            reg_level  <= {{(32-LVL_W){1'b0}},   LEVEL_DEFAULT};
            reg_kdev   <= {{(32-KDEV_W){1'b0}},  KDEV_DEFAULT};
            reg_dci    <= 32'h0;
            reg_dcq    <= 32'h0;
            reg_lceil  <= 32'h733333; reg_lctrl <= 32'h3; clr_tog <= 1'b0; reg_dbgsel <= 5'd0;
            reg_impctrl <= 32'h0; reg_impthr <= 32'h0F5C28; reg_i2sctrl <= 32'h0;
        end else begin
            if (wr_fire) begin
                s_axi_awready <= 1'b1;
                s_axi_wready  <= 1'b1;
                case (s_axi_awaddr[6:2])
                    5'h00: reg_ctrl   <= s_axi_wdata;
                    5'h01: reg_offset <= s_axi_wdata;
                    5'h02: reg_level  <= s_axi_wdata;
                    5'h03: reg_kdev   <= s_axi_wdata;
                    5'h06: reg_dci    <= s_axi_wdata;
                    5'h07: reg_dcq    <= s_axi_wdata;
                    5'h08: reg_lceil  <= s_axi_wdata;
                    5'h10: reg_impctrl <= s_axi_wdata;
                    5'h11: reg_impthr  <= s_axi_wdata;
                    5'h15: reg_i2sctrl <= s_axi_wdata;
                    5'h09: begin reg_lctrl <= {s_axi_wdata[31:3], 1'b0, s_axi_wdata[1:0]};      // bit 2 = clear pulse (reads as 0)
                                if (s_axi_wdata[2]) clr_tog <= ~clr_tog; end
                    5'h0F: reg_dbgsel <= s_axi_wdata[4:0];
                    default: ;
                endcase
                s_axi_bvalid <= 1'b1;
                s_axi_bresp  <= 2'b00;
            end else begin
                s_axi_awready <= 1'b0;
                s_axi_wready  <= 1'b0;
                if (s_axi_bvalid & s_axi_bready)
                    s_axi_bvalid <= 1'b0;
            end
        end
    end

    always @(posedge s_axi_aclk) begin
        if (rst) begin
            s_axi_arready <= 1'b0; s_axi_rvalid <= 1'b0;
            s_axi_rresp   <= 2'b00; s_axi_rdata  <= 32'd0;
        end else begin
            if (s_axi_arvalid & ~s_axi_rvalid) begin
                s_axi_arready <= 1'b1;
                s_axi_rvalid  <= 1'b1;
                s_axi_rresp   <= 2'b00;
                case (s_axi_araddr[6:2])
                    5'h00: s_axi_rdata <= reg_ctrl;
                    5'h01: s_axi_rdata <= reg_offset;
                    5'h02: s_axi_rdata <= reg_level;
                    5'h03: s_axi_rdata <= reg_kdev;
                    5'h04: s_axi_rdata <= {30'd0, status_underflow, status_overflow};
                    5'h05: s_axi_rdata <= MAGIC;
                    5'h06: s_axi_rdata <= reg_dci;
                    5'h07: s_axi_rdata <= reg_dcq;
                    5'h08: s_axi_rdata <= reg_lceil;
                    5'h09: s_axi_rdata <= reg_lctrl;
                    5'h0A: s_axi_rdata <= st_gmin;
                    5'h0B: s_axi_rdata <= st_events;
                    5'h0C: s_axi_rdata <= st_inpeak;
                    5'h0D: s_axi_rdata <= st_state;
                    5'h0E: s_axi_rdata <= st_uf;
                    5'h0F: s_axi_rdata <= st_dbg;
                    5'h10: s_axi_rdata <= reg_impctrl;
                    5'h11: s_axi_rdata <= reg_impthr;
                    5'h12: s_axi_rdata <= st_imp_stamp;
                    5'h13: s_axi_rdata <= st_imp_count;
                    5'h14: s_axi_rdata <= st_fmt;
                    5'h15: s_axi_rdata <= reg_i2sctrl;
                    default: s_axi_rdata <= 32'd0;
                endcase
            end else begin
                s_axi_arready <= 1'b0;
                if (s_axi_rvalid & s_axi_rready)
                    s_axi_rvalid <= 1'b0;
            end
        end
    end

endmodule
