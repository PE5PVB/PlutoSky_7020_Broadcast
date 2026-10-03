# hdl/projects/skypluto — integration of the WFM exciter into the Pluto project

These are the **files the build actually uses**. They form an overlay on `firmware/src/hdl/projects/pluto/` of the
fishball7020-fpga-devkit (base: devkit commit `edbde76`, Vivado 2022.2). The project's own HDL is in `hdl/library/skypluto_wfm/`.

| File | Purpose |
|---|---|
| `system_bd.tcl` | Block design: `skypluto_wfm_exciter` (`wfm`) on `axi_ad9361/l_clk`, UART `skypluto_uart_hd` (`uart`), AXI at 0x7C440000, output to `dac_data_*` |
| `system_top.v` | Top level: I2S input (JP5, bank 13/3.3 V: T9/U10/V10) and the half-duplex UART pin |
| `system_constr.xdc`, `skypluto_late.xdc` | Pins and clock constraints (CDC exceptions in the late XDC) |
| `system_project.tcl`, `build_hdl.tcl`, `set_bitstream_compress.tcl` | Project setup and bitstream options |

## Building

```
scripts/build/sync_hdl.sh        # copies hdl/library/skypluto_wfm/*.v + data to the devkit
cp hdl/projects/skypluto/{system_bd.tcl,system_constr.xdc,system_project.tcl,system_top.v,skypluto_late.xdc,build_hdl.tcl,set_bitstream_compress.tcl} \
   <devkit>/firmware/src/hdl/projects/pluto/
scripts/build/clean_rebuild.sh   # clean HDL build (--hdl-only); result: firmware/output/BOOT.bin
```

The firmware's buildroot changes are in `fw/buildroot-skypluto.patch` (`git apply` in `firmware/src`).
The scripts use `DK` for the devkit checkout (default `~/fishball7020-fpga-devkit`). Flashing the result: see the main README, "Installing a release".
