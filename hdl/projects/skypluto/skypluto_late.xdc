# =============================================================================
# skypluto_late.xdc - processed LATE (PROCESSING_ORDER LATE), so that the PS7 IP
# clocks (clk_fpga_0/1) and the AD9361 generated clocks already exist.
#
# sys_cpu_clk (clk_fpga_0/1, PS PLL) and rx_clk (AD9361 l_clk) are ASYNCHRONOUS.
# All crossings go through synchronizers (AD9361 core CDC, WFM control CDC with
# ASYNC_REG, DMA FIFO) -> declare async so that no false cross-domain
# setup violations are timed.
# =============================================================================
set_clock_groups -asynchronous \
  -group [get_clocks {clk_fpga_0 clk_fpga_1}] \
  -group [get_clocks -include_generated_clocks rx_clk]

# -----------------------------------------------------------------------------
# Multicycle on the interpolator-internal rx_clk paths.
# The wfm datapath logic runs on l_clk = PHYSICALLY ~12.288 MHz (81 ns) for the fixed
# 3.072 MSPS configuration, but rx_clk is pinned at 250 MHz (4 ns) worst case. The interp
# therefore has a few marginal paths (acc2->ovf->delay-line CE) against that 4 ns.
# A multicycle of 2 (8 ns budget) closes them comfortably and is safe: 8 ns << 81 ns
# actual l_clk period (valid as long as l_clk <= ~125 MHz -> all FM rates OK).
# -----------------------------------------------------------------------------
# The conditioner (u_sc: limiter + fade, 1 state per clock at the input rate, ~64 clocks per sample) is covered as well, and so is the
# FM modulator (u_mod): its I/Q correction multipliers (DSP -> DSP, 0 logic levels) missed the 4 ns by ~0.1 ns on placement alone; and the channel
# filter (u_chf), whose MAC pipelines run on the same l_clk.
set ups_cells [get_cells -hierarchical -filter {NAME =~ "*wfm/inst/u_ups/*" || NAME =~ "*wfm/inst/u_sc/*" || NAME =~ "*wfm/inst/u_mod/*" || NAME =~ "*wfm/inst/u_chf/*"}]
set_multicycle_path 2 -setup -from $ups_cells -to $ups_cells
set_multicycle_path 1 -hold  -from $ups_cells -to $ups_cells
