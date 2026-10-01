source ../../scripts/adi_env.tcl
source $ad_hdl_dir/projects/scripts/adi_project_xilinx.tcl
source $ad_hdl_dir/projects/scripts/adi_board.tcl

adi_project_create pluto 0 {} "xc7z020clg400-2"

adi_project_files pluto [list \
  "system_top.v" \
  "system_constr.xdc" \
  "skypluto_late.xdc" \
  "$ad_hdl_dir/library/common/ad_iobuf.v"]

# the async clock group refers to clk_fpga_0 (PS7 IP) -> process it LATE and
# not during synthesis (the PS clock does not exist there yet).
set_property PROCESSING_ORDER LATE   [get_files skypluto_late.xdc]
set_property used_in_synthesis false [get_files skypluto_late.xdc]

set_property is_enabled false [get_files  *system_sys_ps7_0.xdc]
adi_project_run pluto
source $ad_hdl_dir/library/axi_ad9361/axi_ad9361_delay.tcl

