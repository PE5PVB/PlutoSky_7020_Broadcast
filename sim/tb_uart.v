// TB for skypluto_uart_hd: half-duplex on a shared pull-up line.
// Test 1: module transmits (via AXI TXDATA) -> external model receives.
// Test 2: external model transmits -> module RX FIFO -> read out via AXI.
`timescale 1ns/1ps
module tb_uart;
    localparam CLK_HZ=1843200, BAUD=115200;   // DIV=16
    localparam integer BITC=16;               // = DIV, sim clocks per bit
    reg clk=0, resetn=0;
    always #5 clk=~clk;

    // AXI
    reg [4:0] awaddr=0, araddr=0; reg awvalid=0, wvalid=0, arvalid=0, bready=0, rready=0;
    reg [31:0] wdata=0;
    wire awready,wready,bvalid,arready,rvalid; wire [1:0] bresp,rresp; wire [31:0] rdata;

    // shared single-wire line (pull-up)
    tri1 line;
    wire u_txd, u_oe;
    assign line = u_oe ? u_txd : 1'bz;
    reg ext_oe=0, ext_txd=1;
    assign line = ext_oe ? ext_txd : 1'bz;

    skypluto_uart_hd #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) dut (
        .s_axi_aclk(clk), .s_axi_aresetn(resetn),
        .s_axi_awaddr(awaddr), .s_axi_awvalid(awvalid), .s_axi_awready(awready),
        .s_axi_wdata(wdata), .s_axi_wstrb(4'hF), .s_axi_wvalid(wvalid), .s_axi_wready(wready),
        .s_axi_bresp(bresp), .s_axi_bvalid(bvalid), .s_axi_bready(bready),
        .s_axi_araddr(araddr), .s_axi_arvalid(arvalid), .s_axi_arready(arready),
        .s_axi_rdata(rdata), .s_axi_rresp(rresp), .s_axi_rvalid(rvalid), .s_axi_rready(rready),
        .txd(u_txd), .oe(u_oe), .rxd(line));

    task axi_write(input [4:0] a, input [31:0] d);
    begin
        @(posedge clk); awaddr<=a; wdata<=d; awvalid<=1; wvalid<=1; bready<=1;
        wait(bvalid); @(posedge clk); awvalid<=0; wvalid<=0;
        @(posedge clk); bready<=0;
    end endtask

    task axi_read(input [4:0] a, output [31:0] d);
    begin
        @(posedge clk); araddr<=a; arvalid<=1; rready<=1;
        wait(rvalid); d=rdata; @(posedge clk); arvalid<=0;
        @(posedge clk); rready<=0;
    end endtask

    task ext_send(input [7:0] b); integer i;
    begin
        ext_oe=1; ext_txd=0; repeat(BITC) @(posedge clk);          // start
        for (i=0;i<8;i=i+1) begin ext_txd=b[i]; repeat(BITC) @(posedge clk); end
        ext_txd=1; repeat(BITC) @(posedge clk);                    // stop
        ext_oe=0; repeat(BITC*2) @(posedge clk);                   // release
    end endtask

    // external receiver: captures frames that the DUT transmits
    reg [7:0] erx[0:15]; integer erxn=0; integer bi; reg [7:0] eb;
    always begin
        @(negedge line);
        if (!ext_oe) begin                                          // DUT transmits
            repeat(BITC + BITC/2) @(posedge clk);                   // to the middle of D0
            for (bi=0;bi<8;bi=bi+1) begin eb[bi]=line; repeat(BITC) @(posedge clk); end
            erx[erxn]=eb; erxn=erxn+1;
        end
    end

    integer k; reg [31:0] rv; integer ok;
    initial begin
        ok=1;
        repeat(5) @(posedge clk); resetn=1; repeat(5) @(posedge clk);

        // --- Test 1: DUT transmits "Hi" ---
        axi_write(5'h00, 8'h48);   // 'H'
        axi_write(5'h00, 8'h69);   // 'i'
        // wait until both frames are received
        repeat(BITC*40) @(posedge clk);
        $display("T1 extern ontvangen: %0d bytes: %02X %02X", erxn, erx[0], erx[1]);
        if (erxn!=2 || erx[0]!=8'h48 || erx[1]!=8'h69) begin ok=0; $display("  FOUT T1"); end

        // --- Test 2: external transmits 0x37, 0x39 -> DUT RX ---
        ext_send(8'h37); ext_send(8'h39);
        repeat(BITC*4) @(posedge clk);
        axi_read(5'h08, rv);
        $display("T2 STATUS=%08X  rx_count=%0d", rv, rv[15:8]);
        axi_read(5'h04, rv); $display("  RXDATA=%03X (valid=%b byte=%02X)", rv[8:0], rv[8], rv[7:0]);
        if (rv[8]!=1 || rv[7:0]!=8'h37) begin ok=0; $display("  FOUT T2a"); end
        axi_read(5'h04, rv); $display("  RXDATA=%03X (valid=%b byte=%02X)", rv[8:0], rv[8], rv[7:0]);
        if (rv[8]!=1 || rv[7:0]!=8'h39) begin ok=0; $display("  FOUT T2b"); end

        if (ok) $display("UART-TB: PASS"); else $display("UART-TB: FAIL");
        $finish;
    end
endmodule
