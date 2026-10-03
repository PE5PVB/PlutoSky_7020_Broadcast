# Simulation

Testbenches for the core HDL. Run with **Icarus Verilog** (free) or Vivado **xsim**.
Run from the **repo root** so that the path to `data/sine_lut.mem` is correct.

## Icarus Verilog

```bash
# generate the LUT first (if not yet present):
python3 scripts/gen_sine_lut.py --depth 16384 --width 16 \
        --out hdl/library/skypluto_wfm/data/sine_lut.mem

# I2S-RX
iverilog -g2012 -o sim/tb_i2s_rx.vvp \
    sim/tb_i2s_rx.v hdl/library/skypluto_wfm/skypluto_i2s_rx.v
vvp sim/tb_i2s_rx.vvp

# FM modulator (+ sincos)
iverilog -g2012 -o sim/tb_fm.vvp \
    sim/tb_fm_modulator.v \
    hdl/library/skypluto_wfm/skypluto_fm_modulator.v \
    hdl/library/skypluto_wfm/skypluto_sincos.v
vvp sim/tb_fm.vvp
```

Expected: `PASS: ...` on stdout. Open VCD waveforms (`*.vcd`) with GTKWave.

## Vivado xsim

```bash
xvlog sim/tb_fm_modulator.v hdl/library/skypluto_wfm/skypluto_fm_modulator.v \
      hdl/library/skypluto_wfm/skypluto_sincos.v
xelab tb_fm_modulator -s tb_fm
xsim tb_fm -runall
```

## What the tests check

- **tb_i2s_rx** — I2S master model (standard Philips, 24-bit in 32-bit slots) → checks that
  L/R are recovered correctly. Adjust `WS_TO_MSB`/`DATA_W` to your encoder if this fails.
- **tb_fm_modulator** — checks the FM property **constant envelope** (I²+Q² ≈ amp²) and that
  the IQ actually rotates with a DC composite; the audio tone test keeps the envelope check running continuously.
