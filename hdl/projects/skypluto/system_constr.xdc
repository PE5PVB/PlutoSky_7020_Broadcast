# constraints
# ad9361 (SWAP == 0x1)

set_property  -dict {PACKAGE_PIN  U18  IOSTANDARD LVDS_25 DIFF_TERM TRUE} [get_ports rx_clk_in_p]        
set_property  -dict {PACKAGE_PIN  U19  IOSTANDARD LVDS_25 DIFF_TERM TRUE} [get_ports rx_clk_in_n]        
set_property  -dict {PACKAGE_PIN  Y16  IOSTANDARD LVDS_25 DIFF_TERM TRUE} [get_ports rx_frame_in_p]      
set_property  -dict {PACKAGE_PIN  Y17  IOSTANDARD LVDS_25 DIFF_TERM TRUE} [get_ports rx_frame_in_n]      
set_property  -dict {PACKAGE_PIN  Y18  IOSTANDARD LVDS_25 DIFF_TERM TRUE} [get_ports rx_data_in_p[0]]   
set_property  -dict {PACKAGE_PIN  Y19  IOSTANDARD LVDS_25 DIFF_TERM TRUE} [get_ports rx_data_in_n[0]]   
set_property  -dict {PACKAGE_PIN  T16  IOSTANDARD LVDS_25 DIFF_TERM TRUE} [get_ports rx_data_in_p[1]]   
set_property  -dict {PACKAGE_PIN  U17  IOSTANDARD LVDS_25 DIFF_TERM TRUE} [get_ports rx_data_in_n[1]]   
set_property  -dict {PACKAGE_PIN  V20  IOSTANDARD LVDS_25 DIFF_TERM TRUE} [get_ports rx_data_in_p[2]]   
set_property  -dict {PACKAGE_PIN  W20  IOSTANDARD LVDS_25 DIFF_TERM TRUE} [get_ports rx_data_in_n[2]]   
set_property  -dict {PACKAGE_PIN  T17  IOSTANDARD LVDS_25 DIFF_TERM TRUE} [get_ports rx_data_in_p[3]]   
set_property  -dict {PACKAGE_PIN  R18  IOSTANDARD LVDS_25 DIFF_TERM TRUE} [get_ports rx_data_in_n[3]]   
set_property  -dict {PACKAGE_PIN  T20  IOSTANDARD LVDS_25 DIFF_TERM TRUE} [get_ports rx_data_in_p[4]]   
set_property  -dict {PACKAGE_PIN  U20  IOSTANDARD LVDS_25 DIFF_TERM TRUE} [get_ports rx_data_in_n[4]]   
set_property  -dict {PACKAGE_PIN  W18  IOSTANDARD LVDS_25 DIFF_TERM TRUE} [get_ports rx_data_in_p[5]]   
set_property  -dict {PACKAGE_PIN  W19  IOSTANDARD LVDS_25 DIFF_TERM TRUE} [get_ports rx_data_in_n[5]]   
set_property  -dict {PACKAGE_PIN  U14  IOSTANDARD LVDS_25} [get_ports tx_clk_out_p]                     
set_property  -dict {PACKAGE_PIN  U15  IOSTANDARD LVDS_25} [get_ports tx_clk_out_n]                     
set_property  -dict {PACKAGE_PIN  V16  IOSTANDARD LVDS_25} [get_ports tx_frame_out_p]                   
set_property  -dict {PACKAGE_PIN  W16  IOSTANDARD LVDS_25} [get_ports tx_frame_out_n]                   
set_property  -dict {PACKAGE_PIN  V15  IOSTANDARD LVDS_25} [get_ports tx_data_out_p[0]]                 
set_property  -dict {PACKAGE_PIN  W15  IOSTANDARD LVDS_25} [get_ports tx_data_out_n[0]]                 
set_property  -dict {PACKAGE_PIN  V12  IOSTANDARD LVDS_25} [get_ports tx_data_out_p[1]]                 
set_property  -dict {PACKAGE_PIN  W13  IOSTANDARD LVDS_25} [get_ports tx_data_out_n[1]]                 
set_property  -dict {PACKAGE_PIN  W14  IOSTANDARD LVDS_25} [get_ports tx_data_out_p[2]]                 
set_property  -dict {PACKAGE_PIN  Y14  IOSTANDARD LVDS_25} [get_ports tx_data_out_n[2]]                 
set_property  -dict {PACKAGE_PIN  T12  IOSTANDARD LVDS_25} [get_ports tx_data_out_p[3]]                 
set_property  -dict {PACKAGE_PIN  U12  IOSTANDARD LVDS_25} [get_ports tx_data_out_n[3]]                 
set_property  -dict {PACKAGE_PIN  T11  IOSTANDARD LVDS_25} [get_ports tx_data_out_p[4]]                 
set_property  -dict {PACKAGE_PIN  T10  IOSTANDARD LVDS_25} [get_ports tx_data_out_n[4]]                 
set_property  -dict {PACKAGE_PIN  U13  IOSTANDARD LVDS_25} [get_ports tx_data_out_p[5]]                 
set_property  -dict {PACKAGE_PIN  V13  IOSTANDARD LVDS_25} [get_ports tx_data_out_n[5]]                  

set_property  -dict {PACKAGE_PIN  L20 IOSTANDARD LVCMOS25} [get_ports gpio_status[0]]                  
set_property  -dict {PACKAGE_PIN  L19 IOSTANDARD LVCMOS25} [get_ports gpio_status[1]]                  
set_property  -dict {PACKAGE_PIN  K19 IOSTANDARD LVCMOS25} [get_ports gpio_status[2]]                  
set_property  -dict {PACKAGE_PIN  T14 IOSTANDARD LVCMOS25} [get_ports gpio_status[3]]                  
set_property  -dict {PACKAGE_PIN  P15 IOSTANDARD LVCMOS25} [get_ports gpio_status[4]]                  
set_property  -dict {PACKAGE_PIN  M20 IOSTANDARD LVCMOS25} [get_ports gpio_status[5]]                  
set_property  -dict {PACKAGE_PIN  M19 IOSTANDARD LVCMOS25} [get_ports gpio_status[6]]                  
set_property  -dict {PACKAGE_PIN  N20 IOSTANDARD LVCMOS25} [get_ports gpio_status[7]]
                 
set_property  -dict {PACKAGE_PIN  J19 IOSTANDARD LVCMOS25} [get_ports gpio_ctl[0]]                     
set_property  -dict {PACKAGE_PIN  K14 IOSTANDARD LVCMOS25} [get_ports gpio_ctl[1]]                     
set_property  -dict {PACKAGE_PIN  L17 IOSTANDARD LVCMOS25} [get_ports gpio_ctl[2]]                     
set_property  -dict {PACKAGE_PIN  J20 IOSTANDARD LVCMOS25} [get_ports gpio_ctl[3]] 
set_property  -dict {PACKAGE_PIN  P20  IOSTANDARD LVCMOS25} [get_ports gpio_en_agc]
set_property  -dict {PACKAGE_PIN  R19  IOSTANDARD LVCMOS25} [get_ports gpio_resetb]

# Header GPIO, broken out on connector JP5 as 3V3_IO1..4 (pins 7, 9, 11, 13).
# Free in the stock design; driven here either by Linux (EMIO GPIO) or by the
# low nibble of the transmit sample.
#
# Balls read off the vendor schematic, sheet 5 (U1G, "PL端BANK13"):
#   3V3_IO1 = V10 (IO_L20N)   3V3_IO3 = U10 (IO_L12N)
#   3V3_IO2 = U9  (IO_L16P)   3V3_IO4 = T9  (IO_L12P)
# sample_gpio[n] is wired to 3V3_IO(n+1), so the bit number matches the silk.
#
# LVCMOS33 is right: sheet 1 ties VCCO_13_1..4 (T8, U11, W7, Y10) to VCC3V3.
# Note this is NOT the same as the rest of the design, which declares LVCMOS25
# and LVDS_25 on banks 34/35 that the same sheet supplies from VCC1V8 - an
# inconsistency inherited from ADI's stock Pluto constraints, left alone here.
#
# Do not guess these. V11, W9 and V7 are adjacent bank-13 balls and look like
# plausible candidates, but the schematic marks them "no connect" - and a wrong
# PACKAGE_PIN is not a build error, it is a bitstream that drives a pad wired
# to nothing.
#
# PULLDOWN gives the pins a DEFINED idle state. Each of these four nets appears
# exactly twice in the whole schematic - once at the FPGA ball, once at JP5 -
# so there is no external pull, series part or ESD diode anywhere on them, and
# nothing for an internal pull to fight. Undriven they would genuinely float,
# which for a line something downstream reads as a trigger is the wrong
# default: a floating input that drifts high looks asserted. Idle low is the
# de-asserted state for a sync or frame signal, so pull down rather than up.
#
# These four outputs are deliberately left UNCONSTRAINED, like every other
# LVCMOS output in this design. Skew between them is then whatever the router
# gives - a few hundred picoseconds, set by placement, not checked by timing
# analysis. That is far below one sample period (16 ns at 61.44 MSPS) and so
# irrelevant for the frame and sync signals this feature exists for. If you
# are doing something where it is not irrelevant, add a set_output_delay
# against the AD9361 datapath clock and re-run implementation; do not assume
# the numbers here have been verified, because they have not been.
# SkyPluto_WFM: I2S input on the 3.3V header pins (bank 13, LVCMOS33).
# Pluto = I2S slave; no level shifter needed for a 3.3V source.
set_property  -dict {PACKAGE_PIN  T9   IOSTANDARD LVCMOS33} [get_ports i2s_in_bclk]    ;# JP5-13
set_property  -dict {PACKAGE_PIN  U10  IOSTANDARD LVCMOS33} [get_ports i2s_in_lrclk]   ;# JP5-11
set_property  -dict {PACKAGE_PIN  V10  IOSTANDARD LVCMOS33} [get_ports i2s_in_data]    ;# JP5-7

# SkyPluto_WFM: single-wire half-duplex control UART (bank 13, 3.3V), bidirectional.
# PULLUP: keeps the line idle-high when nothing drives it (otherwise U9 floats -> false RX start bits).
set_property  -dict {PACKAGE_PIN  U9   IOSTANDARD LVCMOS33 PULLTYPE PULLUP} [get_ports uart_pin]  ;# JP5-9

# BCLK (12.288 MHz nominal) as clock; lands on a non-clock-capable pin ->
# disable the dedicated clock route (allowed at this low speed).
create_clock -period 81.380 -name i2s_bclk [get_ports i2s_in_bclk]
set_property CLOCK_DEDICATED_ROUTE FALSE [get_nets -of_objects [get_ports i2s_in_bclk]]

# The I2S clock domain is asynchronous to all other clocks (CDC via async FIFO).
set_clock_groups -asynchronous -group [get_clocks -include_generated_clocks i2s_bclk]

# The bit-map enable flag is written by software in the AXI clock domain and
# read in the AD9361 datapath domain, so it is a clock-domain crossing. That
# is exactly what the two flip-flops inside tx_gpio_bitmap are for. Timing it
# as an ordinary synchronous path is not conservative, it is meaningless: it
# gets 2 ns (the gap between the two clocks' edges) and can become the worst
# path in the whole design for no reason. Bound the crossing instead:
# -datapath_only keeps the two synchroniser flops close enough together for
# metastability to settle, without pretending the launch edge means anything.
#
# Both ends must be named. "set_max_delay -datapath_only" with
# only -to is an error (Constraints 18-540, "requires -from to be
# non-empty"), and in an .xdc that error drops the line without a word in the
# build log, so the crossing would silently be timed as a 2 ns path. To see
# that the limit is really applied, open the routed design and run
#   report_timing -to [get_cells -hier *flag_m_reg]
# The requirement must read "(MaxDelay Path 4.000ns)", not two clock edges.
#
# Written as a single command: an .xdc is a restricted Tcl dialect and "if" is
# NOT one of the commands it accepts - Vivado answers a guarded version with
# "Command 'if' is not supported in the xdc constraint file" and silently drops
# the whole block. (It says so in pluto.runs/*/runme.log, not in the top-level
# build log.) No guard is needed anyway: these lines only exist in the file
# once the bit-map patch is applied, and then the cell always exists.
set_max_delay -datapath_only 4.000 \
  -from [get_cells -quiet -hier -filter {NAME =~ *up_dac_gpio_out_int_reg*}] \
  -to   [get_cells -quiet -hier -filter {NAME =~ *tx_bitmap*flag_m_reg}]

set_property  -dict {PACKAGE_PIN  T15  IOSTANDARD LVCMOS25} [get_ports enable]
set_property  -dict {PACKAGE_PIN  P18  IOSTANDARD LVCMOS25} [get_ports txnrx]

set_property  -dict {PACKAGE_PIN  M14  IOSTANDARD LVCMOS25 PULLTYPE PULLUP} [get_ports iic_scl]
set_property  -dict {PACKAGE_PIN  M15  IOSTANDARD LVCMOS25 PULLTYPE PULLUP} [get_ports iic_sda]

set_property  -dict {PACKAGE_PIN  R17  IOSTANDARD LVCMOS25  PULLTYPE PULLUP} [get_ports spi_csn]
set_property  -dict {PACKAGE_PIN  V18  IOSTANDARD LVCMOS25} [get_ports spi_clk]
set_property  -dict {PACKAGE_PIN  P16  IOSTANDARD LVCMOS25} [get_ports spi_mosi]
set_property  -dict {PACKAGE_PIN  V17  IOSTANDARD LVCMOS25} [get_ports spi_miso]

set_property  -dict {PACKAGE_PIN  L14  IOSTANDARD LVCMOS25} [get_ports pl_spi_clk_o]
set_property  -dict {PACKAGE_PIN  N15  IOSTANDARD LVCMOS25} [get_ports pl_spi_miso]
set_property  -dict {PACKAGE_PIN  N16  IOSTANDARD LVCMOS25} [get_ports pl_spi_mosi]


create_clock -period 4.000 -name rx_clk [get_ports rx_clk_in_p]

# probably gone in 2016.4

create_clock -name clk_fpga_0 -period 10 [get_pins "i_system_wrapper/system_i/sys_ps7/inst/PS7_i/FCLKCLK[0]"]
create_clock -name clk_fpga_1 -period  5 [get_pins "i_system_wrapper/system_i/sys_ps7/inst/PS7_i/FCLKCLK[1]"]

create_clock -name spi0_clk      -period 40   [get_pins -hier */EMIOSPI0SCLKO]

set_input_jitter clk_fpga_0 0.3
set_input_jitter clk_fpga_1 0.15

# NB: the async clock group between clk_fpga_0 and rx_clk is in skypluto_late.xdc
# (PROCESSING_ORDER LATE) because clk_fpga_0 does not exist yet here.



