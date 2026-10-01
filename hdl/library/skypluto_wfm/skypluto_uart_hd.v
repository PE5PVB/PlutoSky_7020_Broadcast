// =============================================================================
// skypluto_uart_hd - single-wire half-duplex UART with AXI-Lite interface
// -----------------------------------------------------------------------------
// One wire (U9): open-drain/tri-state, idle-high with pull-up. OE=1 while we
// transmit (drive txd), otherwise Hi-Z -> listen. RX is gated during transmit
// (we do not hear ourselves). 8N1, baud via parameter. TX/RX FIFOs; the Linux daemon
// reads/writes via devmem/mmap.
//
// Register map (byte offset, AXI-Lite, 32-bit):
//   0x00 W : TXDATA  [7:0] -> push into TX FIFO (if not full)
//   0x04 R : RXDATA  {..., valid@8, byte[7:0]} - pops 1 byte (valid=0 if empty)
//   0x08 R : STATUS  bit0 rx_empty, bit1 tx_full, bit2 oe(transmitting), bit3 rx_ovf,
//                    [15:8] rx_count, [23:16] tx_count
//   0x0C W : CTRL    bit0 = clear rx_ovf flag (write-1)
// =============================================================================
`timescale 1ns/1ps

module skypluto_uart_hd #(
    parameter integer CLK_HZ   = 100000000,   // s_axi_aclk frequency
    parameter integer BAUD     = 115200,
    parameter integer FIFO_AW  = 4,           // FIFO depth = 2^AW
    parameter integer GUARD    = 2            // stop-bit guard after TX before release
)(
    input  wire        s_axi_aclk,
    input  wire        s_axi_aresetn,
    // AXI4-Lite
    input  wire [ 4:0] s_axi_awaddr,
    input  wire        s_axi_awvalid,
    output reg         s_axi_awready,
    input  wire [31:0] s_axi_wdata,
    input  wire [ 3:0] s_axi_wstrb,
    input  wire        s_axi_wvalid,
    output reg         s_axi_wready,
    output reg  [ 1:0] s_axi_bresp,
    output reg         s_axi_bvalid,
    input  wire        s_axi_bready,
    input  wire [ 4:0] s_axi_araddr,
    input  wire        s_axi_arvalid,
    output reg         s_axi_arready,
    output reg  [31:0] s_axi_rdata,
    output reg  [ 1:0] s_axi_rresp,
    output reg         s_axi_rvalid,
    input  wire        s_axi_rready,
    // single-wire pin side (to top-level IOBUF)
    output reg         txd,      // value to drive
    output reg         oe,       // 1 = drive txd, 0 = Hi-Z (listen)
    input  wire        rxd       // sampled line
);
    localparam integer DIV  = CLK_HZ / BAUD;
    localparam integer HALF = DIV/2;
    localparam integer DEPTH = (1<<FIFO_AW);

    // ---- TX FIFO -------------------------------------------------------------
    reg [7:0] txmem [0:DEPTH-1];
    reg [FIFO_AW:0] tx_wr, tx_rd;
    wire tx_empty = (tx_wr == tx_rd);
    wire tx_full  = (tx_wr[FIFO_AW-1:0]==tx_rd[FIFO_AW-1:0]) && (tx_wr[FIFO_AW]!=tx_rd[FIFO_AW]);
    wire [FIFO_AW:0] tx_count = tx_wr - tx_rd;

    // ---- RX FIFO -------------------------------------------------------------
    reg [7:0] rxmem [0:DEPTH-1];
    reg [FIFO_AW:0] rx_wr, rx_rd;
    wire rx_empty = (rx_wr == rx_rd);
    wire rx_full  = (rx_wr[FIFO_AW-1:0]==rx_rd[FIFO_AW-1:0]) && (rx_wr[FIFO_AW]!=rx_rd[FIFO_AW]);
    wire [FIFO_AW:0] rx_count = rx_wr - rx_rd;
    reg  rx_ovf;

    // ---- TX transmitter ------------------------------------------------------
    localparam TX_IDLE=0, TX_LOAD=1, TX_BIT=2, TX_GUARD=3;
    reg [1:0]  tx_st;
    reg [31:0] tx_div;
    reg [3:0]  tx_bit;        // 0=start,1..8=data,9=stop
    reg [7:0]  tx_sh;
    reg [7:0]  tx_guard_cnt;

    // ---- RX receiver ---------------------------------------------------------
    localparam RX_IDLE=0, RX_START=1, RX_BIT=2, RX_STOP=3;
    reg [1:0]  rx_st;
    reg [31:0] rx_div;
    reg [3:0]  rx_bit;
    reg [7:0]  rx_sh;
    reg        rxd_s0, rxd_s1;   // synchronizer + debouncing

    // ---- AXI write -----------------------------------------------------------
    wire wr_fire = s_axi_awvalid & s_axi_wvalid & ~s_axi_bvalid;
    wire rd_fire = s_axi_arvalid & ~s_axi_rvalid;
    reg  tx_push; reg [7:0] tx_pushbyte;
    reg  rx_pop;
    reg  clr_ovf;

    integer i;
    always @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            s_axi_awready<=0; s_axi_wready<=0; s_axi_bvalid<=0; s_axi_bresp<=0;
            s_axi_arready<=0; s_axi_rvalid<=0; s_axi_rresp<=0; s_axi_rdata<=0;
            tx_wr<=0; tx_rd<=0; rx_wr<=0; rx_rd<=0; rx_ovf<=0;
            tx_st<=TX_IDLE; tx_div<=0; tx_bit<=0; tx_sh<=0; tx_guard_cnt<=0;
            txd<=1'b1; oe<=1'b0;
            rx_st<=RX_IDLE; rx_div<=0; rx_bit<=0; rx_sh<=0; rxd_s0<=1; rxd_s1<=1;
        end else begin
            tx_push<=0; rx_pop<=0; clr_ovf<=0;

            // ===== AXI write channel =====
            if (wr_fire) begin
                s_axi_awready<=1; s_axi_wready<=1; s_axi_bvalid<=1; s_axi_bresp<=2'b00;
                case (s_axi_awaddr[4:2])
                    3'h0: if (!tx_full) begin tx_pushbyte<=s_axi_wdata[7:0]; tx_push<=1; end
                    3'h3: if (s_axi_wdata[0]) clr_ovf<=1;   // 0x0C CTRL
                    default: ;
                endcase
            end else begin
                s_axi_awready<=0; s_axi_wready<=0;
                if (s_axi_bvalid & s_axi_bready) s_axi_bvalid<=0;
            end

            // ===== AXI read channel =====
            if (rd_fire) begin
                s_axi_arready<=1; s_axi_rvalid<=1; s_axi_rresp<=2'b00;
                case (s_axi_araddr[4:2])
                    3'h1: begin  // 0x04 RXDATA (pop)
                        if (!rx_empty) begin
                            s_axi_rdata <= {23'b0, 1'b1, rxmem[rx_rd[FIFO_AW-1:0]]};
                            rx_pop<=1;
                        end else s_axi_rdata <= 32'b0;
                    end
                    3'h2:        // 0x08 STATUS: [23:16]=tx_count [15:8]=rx_count [3:0]=flags
                        s_axi_rdata <= {8'b0, 3'b0, tx_count, 3'b0, rx_count,
                                        4'b0, rx_ovf, oe, tx_full, rx_empty};
                    default: s_axi_rdata <= 32'b0;
                endcase
            end else begin
                s_axi_arready<=0;
                if (s_axi_rvalid & s_axi_rready) s_axi_rvalid<=0;
            end

            // ===== FIFO pointers =====
            if (tx_push) begin txmem[tx_wr[FIFO_AW-1:0]]<=tx_pushbyte; tx_wr<=tx_wr+1; end
            if (rx_pop) rx_rd<=rx_rd+1;
            if (clr_ovf) rx_ovf<=0;

            // ===== TX-FSM =====
            case (tx_st)
                TX_IDLE: begin
                    txd<=1'b1;
                    if (!tx_empty) begin
                        oe<=1'b1;                     // take over the line
                        tx_sh<=txmem[tx_rd[FIFO_AW-1:0]];
                        tx_rd<=tx_rd+1;
                        tx_bit<=0; tx_div<=0; txd<=1'b0;  // start bit
                        tx_st<=TX_BIT;
                    end else begin
                        oe<=1'b0;                     // release -> listen
                    end
                end
                TX_BIT: begin
                    if (tx_div==DIV-1) begin
                        tx_div<=0; tx_bit<=tx_bit+1;
                        case (tx_bit)
                            4'd0: txd<=tx_sh[0];
                            4'd1: txd<=tx_sh[1];
                            4'd2: txd<=tx_sh[2];
                            4'd3: txd<=tx_sh[3];
                            4'd4: txd<=tx_sh[4];
                            4'd5: txd<=tx_sh[5];
                            4'd6: txd<=tx_sh[6];
                            4'd7: txd<=tx_sh[7];
                            4'd8: txd<=1'b1;          // stop bit
                            default: begin end
                        endcase
                        if (tx_bit==4'd8) begin tx_guard_cnt<=0; tx_st<=TX_GUARD; end
                    end else tx_div<=tx_div+1;
                end
                TX_GUARD: begin                       // hold the stop bit + guard
                    if (tx_div==DIV-1) begin
                        tx_div<=0;
                        if (tx_guard_cnt>=GUARD) tx_st<=TX_IDLE;
                        else tx_guard_cnt<=tx_guard_cnt+1;
                    end else tx_div<=tx_div+1;
                end
                default: tx_st<=TX_IDLE;
            endcase

            // ===== RX FSM (off while transmitting) =====
            rxd_s0<=rxd; rxd_s1<=rxd_s0;
            if (oe) begin
                rx_st<=RX_IDLE; rx_div<=0;             // do not receive ourselves
            end else case (rx_st)
                RX_IDLE: if (!rxd_s1) begin rx_div<=0; rx_st<=RX_START; end  // start-bit edge
                RX_START: if (rx_div==HALF-1) begin     // middle of start bit
                                if (!rxd_s1) begin rx_div<=0; rx_bit<=0; rx_st<=RX_BIT; end
                                else rx_st<=RX_IDLE;    // false start
                          end else rx_div<=rx_div+1;
                RX_BIT: if (rx_div==DIV-1) begin
                            rx_div<=0; rx_sh<={rxd_s1, rx_sh[7:1]};
                            if (rx_bit==4'd7) rx_st<=RX_STOP; else rx_bit<=rx_bit+1;
                        end else rx_div<=rx_div+1;
                RX_STOP: if (rx_div==DIV-1) begin
                            rx_div<=0; rx_st<=RX_IDLE;
                            if (rxd_s1) begin            // valid stop bit
                                if (!rx_full) begin rxmem[rx_wr[FIFO_AW-1:0]]<=rx_sh; rx_wr<=rx_wr+1; end
                                else rx_ovf<=1;
                            end
                        end else rx_div<=rx_div+1;
                default: rx_st<=RX_IDLE;
            endcase
        end
    end
endmodule
