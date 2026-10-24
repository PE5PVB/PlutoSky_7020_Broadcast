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

- **tb_chanfilt** (`python3 sim/chanfilt_check.py`) — the channel filter after the modulator against the bit-exact model of `scripts/gen_chanfilt.py`, all modes; off = unchanged.
- **tb_i2s_lowfreq** — format detection with a 20 Hz tone + pilot (the composite stays positive for >5 ms): the format must not flip.
- **tb_maskctl** — the mask controller (`skypluto_maskctl.v`) against its bit-exact model `maskctl_model.py`: the gain step of every
  block must match, with the samples arriving at 192 kHz against the 100 MHz clock (also reports the engine's clocks per block):
  ```bash
  python3 scripts/gen_maskctl.py                         # tables (only after a change of the model)
  python3 sim/maskctl_model.py vectors /tmp/mc 0.05      # a loud synthetic multiplex, kdev 133, margin 3 dB
  cp hdl/library/skypluto_wfm/data/maskctl_sin.mem /tmp/mc/
  iverilog -g2012 -I hdl/library/skypluto_wfm -o /tmp/mc/tb.vvp sim/tb_maskctl.v \
      hdl/library/skypluto_wfm/skypluto_maskctl.v hdl/library/skypluto_wfm/skypluto_sincos.v
  (cd /tmp/mc && vvp -n tb.vvp)                          # PASS
  ```
- **tb_mask_top** — the whole exciter with mask control on, fed over I2S, against the model (`check_mask_top.py`): the samples the
  conditioner sends, the controller's step per block and g_req per sample at the limiter. Same vectors as above, plus all
  `data/*.mem` in the run directory; compile with all `hdl/library/skypluto_wfm/*.v`, run `vvp -n tb.vvp +MS=40`, then
  `python3 <repo>/sim/check_mask_top.py` in that directory (PASS).

## Mask tool FFT (ARM only)

`bench_mask_fft.c` checks the NEON radix-4 FFT of `src/skypluto-mask.c` against the scalar one and times both. Build it with the ARM cross compiler (`-O3 -mcpu=cortex-a9 -mfpu=neon -ffast-math -lm -lpthread`) and run it on the Pluto: `bench_mask_fft 256 3 2` compares the bins 56..112 (expect a difference below 0.001 dB), `bench_mask_fft 256 3 0` / `1` time the scalar / fast max-hold over one 0.25 s window.
