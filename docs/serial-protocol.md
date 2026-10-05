# PlutoSky 7020 Broadcast v1.02 — serial control protocol

Control and telemetry link between the **PicoAudio encoder** (master) and the
**PlutoSky 7020 Broadcast exciter** (slave, "the Pluto"), alongside the I2S composite link.

The protocol is deliberately minimal: the encoder (PicoAudio) controls only **on/off, frequency
and output level (dBm)** and reads out status, temperature and the **mask monitor**. Everything else is fixed or
runs automatically on the Pluto.

---

## 1. Physical layer

**I2S input (MPX):** 192 kHz (the Pluto follows the source within ±1950 ppm around 192.1875 kHz), standard I2S, slave. The format is **detected automatically** from the bit activity: alignment standard I2S, left-justified or right-justified; word width 16/20/24/32 bit (the upper 24 bits are used); slot length 16/24/32/64 BCLKs per channel. Right-justified is recognised both with zero padding and with **sign extension** (with sign extension, from the length of the sign run; the word width then becomes the smallest standard width 16/20/24/32 that fits, reliable down to about −24 dBFS signal). Only a completely filled slot (e.g. 24 bit in 24 BCLKs) cannot be separated into I2S/LJ by position: there the sign agreement of the first and second bit decides (with hysteresis). Manual override via AXI register 0x54 (`devmem 0x7C440054`: bit0 enable, bits2:1 alignment, bits15:8 word width). The word is left-aligned to 24 bit so that full scale, and therefore the deviation, stays the same. The result is reported in `?E` (`i2s=`).

| Property | Value |
|---|---|
| Wires | **1**, half-duplex |
| Pluto pin | U9 = JP5-9 (bank 13, 3.3 V) — encoder GP26 |
| Level | 3.3 V, idle-high with an **external 4k7 pull-up to 3V3** |
| Baud rate | 115200, 8N1 |
| Arbitration | the encoder is master: it sends one line and waits at most 300 ms for the reply |

The Pluto does not hear its own reply (receiver blocked while transmitting). The encoder
does hear its own bytes (echo) and must discard them.

### Boot status (Pluto → encoder, unsolicited)

During start-up the Pluto sends lines `#B <pct 0-100> <text ≤24 characters>` for the status bar of the encoder's splash screen: `#B 10 Linux gestart` ('Linux started'),
`#B 30 Radio-chip klaar` ('radio chip ready'), `#B 45 Zender dicht` ('transmitter closed') (from `autorun.sh`), then from the daemon `#B 55 Besturing v1.02` ('control, version 1.02'), `#B 60 Wacht op tune` ('waiting for tune') and `#B 100 Gereed v1.02` ('ready, version 1.02'). The text of the 55 and 100 lines therefore carries the software version, so that the encoder's splash screen shows it without any extra query; `?V` (`ver=`) gives it at any time.
The daemon repeats the last line every second until it itself receives a byte from the encoder (the encoder is master; the line is half-duplex, so the Pluto falls silent
as soon as the encoder speaks) and it first listens for 0.4 s before sending the first line. The encoder stays silent during the boot phase until `#B 100` (or 45 s after its own start) and
then begins the normal sync. Lines starting with `#` do not count as "link is up". Before Linux is running (~15–20 s after power-on) the line is silent.
`?B` → `b=<pct> <text>` returns the latest boot step at any time.

### TCP console (manual use and tests)

Besides the serial line, the daemon listens on **127.0.0.1:5555** (local to the Pluto only; reachable via ssh). Same protocol, same replies, one client at a time; replies go only to that client. The Pluto has no `nc`; use the client mode of the daemon itself:

```
ssh root@<pluto> '/mnt/jffs2/skypluto-ctl -c "?M"'
ssh root@<pluto> '/mnt/jffs2/skypluto-ctl -c "H 60"'     # or: /mnt/jffs2/skypluto-cmd.sh H 60
```

Note: at every resync the encoder sends E/F/P/K/B/H again and thus overwrites manually set values. This also allows commands to be tried that the encoder cannot send (edge cases, `ERR range`/`ERR nolim`).

### Web interface

The same daemon serves a web interface on **port 80** (HTTP). Commands entered there go through exactly the same parser as the serial line. Endpoints: `GET /` (page), `POST /login`, `POST /api/cmd` (body = one command line; requires the header `X-Requested-With: skypluto`), `GET /api/state` (JSON with the answers to `?S ?T ?P ?E ?V ?M ?D ?B ?J ?R ?O`, plus link status), `GET /api/mask` (`?L ?W ?N`), `GET /api/g` (`?G`), `GET /api/tone` (`?A`) and `GET /api/ir.csv` (result of the impulse measurement). If `/mnt/jffs2/skypluto-web.conf` contains `password=<password>`, a login is required (session cookie); without this file or password the interface is open.

## 2. Framing

```
encoder → Pluto :  <cmd> [arg]<LF>         (LF only, no CR)
Pluto → encoder :  OK<LF> | ERR <reason><LF> | <key>=<val> …<LF>
```

One line per command, one line per reply. Unknown command → `ERR parse`; unknown query → `ERR query`.

**Reply before execution.** `E`, `F` and `P` are answered with `OK` immediately (within a few ms) and only then executed: the execution
(`iio_attr` to the AD9361) takes 130–280 ms, especially when the mask monitor loads the CPU, and that would otherwise approach the 300 ms timeout. Consequence:
an `OK` means *received and accepted*, not *already executed*. Whether the effect has taken place can be seen from `?S` (`att=`, `tx=`, `f=`) and `?P`. Only a
value out of range gets `ERR range`; a failed retune or attenuation change is not returned as an error but is written to the Pluto log and becomes visible in `?S`.

## 3. Control

**Boot behaviour:** after power-on the TX LO is off and the attenuation is at its maximum: **no RF** is produced until the encoder sends a valid `F` (tune). `E 1` without a preceding `F` keeps the transmitter closed; an `F` while `E` is not 0 opens the transmitter (first the LO, then the attenuation). `?S` shows `tx=off` as long as there is no tune. A restart of the daemon alone (an update, a crash) leaves an **open** transmitter open: the daemon adopts the current frequency from `/tmp/skypluto-tuned` and the attenuation on the chip, so the RF does not drop out; a power cycle always starts closed. The encoder detects a restart by `up=` in `?S` (it goes back to a small value) and should then send its settings again.

| Cmd | Arg | Function |
|-----|-----|---------|
| `E` | `0` / `1` | transmitter **off / on**. **Off = the RF is completely gone**: first the TX attenuation to the maximum (89.75 dB, TX1 and TX2), then the **AD9361 TX LO off** (`powerdown 1`; TX1 and TX2 share the LO), so there is no carrier leakage either. **On**: the LO on, the frequency reprogrammed (lets the PLL lock; the attenuation is still at the maximum at that point), and only then the attenuation back to the set level (no peak at a wrong level). |
| `F` | Hz | **tune**: carrier frequency (opens the transmitter after boot), 70e6 … 6e9 (e.g. `F 108000000`). Sending the same value again (±10 Hz) does nothing (no retune). While the transmitter is **off** the frequency is only remembered and applied at the next `E 1`. |
| `P` | dBm | **output level of TX1**, −84.75 … **−5.0** (e.g. `P -15.00`). Out of range → `ERR range`. The maximum of −5 dBm is fixed: above about −5 dBm the exciter no longer meets the SM.1268 mask (measured: −5 dBm +5.8 dB margin, 0 dBm −1.2 dB, +5 dBm −6.4 dB). |
| `B` | `0` / `1` | **FPGA limiter off / on** (optional command; menu option on the encoder). On (default) = the peak limiter in the FPGA conditioner is active (mask limit, see §5); off = no limiting by the Pluto (the soft fade-in/out on I2S loss always remains on). **Without a limiter in the bitstream** (bypassed) → `ERR nolim`: the encoder can then grey out the option (`bg=na` in `?E`). Off means that the Pluto itself no longer has mask protection; the encoder's composite limiter is then the only one. The mask monitor (§4a) keeps measuring. |
| `H` | `<kHz>` | **Limiter ceiling** of the FPGA limiter in kHz peak deviation (dot or comma as decimal, e.g. `H 63.0`), range 20–150, `ERR range` outside it; without a limiter `ERR nolim`. The encoder sends `H` at every link resync; `ceil_khz` in the conf file is only the fallback until the first `H`. Not stored in the conf file. Above 70 kHz the mask may be exceeded. `?E ... ceil=` returns the value. |
| `G` | `0` / `1` | **mask protection (guard)** off / on (default **off**; `guard=` in the conf file is the fallback). On: `H` is the *maximum*; the daemon lowers the limiter ceiling as soon as the 5-minute mask margin (measured only since the last change, after at least 20 s of data) drops below +3.5 dB, such that the expected margin becomes +4.5 dB (margin ≈ 0.9 dB per dB lower peak; basis is the peak actually transmitted during the last minute), by at most 25 % per step, never below 35 kHz. With ≥ 30 s of margin > +6.5 dB the ceiling goes back up as far as the margin allows (back to an expected +4.5 dB), at most 25 % per step, up to `H` (from 36 to 75 kHz in a few minutes). Without mask data (no signal from the coupler, transmitter closed) or with `G 0` the ceiling stays exactly `H`. A *different* `H` puts the protection back to the maximum (the same value again, as the encoder sends it at every resync, changes nothing: the guard keeps its ceiling). |
| `K` | `<n>` | **deviation range** (kdev): 0 dBFS on the I2S = 0.75 kHz × n (100 = 75 kHz), range 10–250, otherwise `ERR range`; above 100 without a limiter in the bitstream `ERR nolim`. The encoder sends `K` at every link resync; `kdev=` in the conf file is only the fallback until the first `K`. |
| `CAL` | — | **TX calibration now** (LO leakage, image of the AD9361), with the output muted (about 2 s). `ERR off` when the transmitter is closed, `ERR busy` during a `J`/`Y`/`Z` measurement; while a calibration is already pending or running `CAL` just answers `OK`. The `J`/`Y`/`Z` measurements in turn answer `ERR busy` while a calibration is pending or running. Progress and result: `cal=` in `?E`. |
| `OFS` | `<kHz>` | **low-IF offset** (-250..250, 0 = zero-IF, the default): the TX LO is programmed `<kHz>` below the carrier and the modulator's NCO is offset by the same amount, so the carrier stays on the tuned frequency while the LO leakage and the I/Q image move `<kHz>` and `2 x <kHz>` away. With the transmitter open the output is muted, both are shifted and the TX is calibrated at the new LO (like a tune). Meant for measuring and nulling the LO leakage and the image on a spectrum analyser; while an offset is set the mask monitor rejects its captures (the LO line counts as a second tone). Stored in `/mnt/jffs2/skypluto-iq.conf`. |
| `DC` | `<i> <q>` | **digital DC offset** on I and Q in 16-bit sample LSBs (-2047..2047): cancels the analog LO leakage of the transmitter. Stored. |
| `IQ` | `<gain> <skew>` | **digital I/Q imbalance correction** in parts per million: Q gets `<gain>` ppm of itself (gain error) and `<skew>` ppm of a radian of I (phase error) added (-400000..400000). Cancels the I/Q image. Needs bitstream B1D00019 or later, otherwise `ERR nobit`. Stored. The digital carrier level is lowered by the size of the corrections so that the corrected carrier never clips. |
| `LVL` | `<dB>` | **digital carrier level** (-20..0 dBFS, default -9): the modulator's level; the chip attenuation is lowered by the same amount so the output power stays the same. Below full scale the DAC/baseband stays linear, which keeps its distortion off the carrier at zero-IF. Stored. |
| `NULL` | — | **LO and image nulling now** through the directional coupler (TX1 → RX2, see `NULLRX`): about 5 s with a silent carrier moved 100 kHz from the LO by the NCO (the LO and the AD9361 calibration stay as in operation). Five-point fits of the LO line (stepping `DC`) and of the image line (stepping the gain/skew of `IQ`, bitstream B1D00019 or later) from the same capture; the results are applied and stored. A bad fit (a line in the way) is retried at 120, 80 and 140 kHz; a bad image fit keeps the old gain/skew; an image that would get worse is never taken. `ERR off` (closed), `ERR busy` (calibration or measurement running), `ERR level` (`NULLRX 1` above -15 dBm), `ERR ofs` (an `OFS` offset is set), `ERR nomask` (tool missing). Result in `?NL`. |
| `NULLAUTO` | `0` / `1` | run `NULL` automatically after every TX calibration (tune, opening, temperature drift, `CAL`); default **1**. The output stays muted / silent until it is done and `?E` reports `cal=run` meanwhile. Stored. |
| `NULLRX` | `1` / `2` | receiver of the nulling: **2** = a directional coupler on TX1 into RX2 (default, no level limit), 1 = a cable/attenuator TX1 → RX1 (only at ≤ -15 dBm). Stored. |
| `X` | — | **restart the mask monitor**: the 5-min max-hold and the statistics are cleared (`?W`/`?N` = `na`, `?M` counts from zero: `m=na n=0` until the first new measurement). Reply `OK`. |
| `Q` | `<dB>` | **power meter (RX1)**: value of the external attenuator between the TX1 output and RX1, 0…120 dB (stored). A measurement aid only: nothing is controlled with it. |
| `QK` | `<dBm>` | **calibrate**: the power at the TX1 output is NOW `<dBm>` (−60…+60); the meter sets its correction (`cal`) so that it shows this. Only on a settled reading (5 measurements, spread ≤ 0.5 dB): otherwise `ERR unstable`/`ERR nosig`. Stored. |
| `QM` | `0` / `1` | power meter off / on (stored). |
| `QG` | `<dB>` / `A` | hold the RX1 gain manually (0…70 dB) or `A` = switch the range automatically again (not stored; intended for investigation). |

`E`, `F`, `P` and `B` may be repeated every poll cycle: unchanged values are answered
with `OK` without effect. The encoder is in charge of the settings and sends `B` again at every link resync (like `E/F/P`); the Pluto does not remember it across a restart (default on).

**Encoder side:** the admin field "Output level" of the encoder is limited to −84.75 … −5.00 dBm, a stored value
above that is clamped, and the encoder never sends a `P` above −5.00. The encoder sends a `P` at every level change and after every link resync
(then again with the last confirmed value). The default value for a new board is −5.00 dBm.

**Absolute dBm:** the level is `P = 5.0 − attenuation`. That 5.0 dBm at 0 dB attenuation is
an **estimate** (not calibrated with a power meter); the absolute value may be a few dB
off. The **control** itself (see §5) is accurate in relative terms.

## 4. Read-out

| Query | Example reply |
|-------|-------------------|
| `?T` | `temp=42.1` — AD9361 die temperature in °C |
| `?S` | `en=1 f=107999998 p=-15.00 att=20.00 tx=on up=312 kdev=92` |
| `?P` | `set=-15.00 out=-15.10 att=20.00 trim=0.00 alc=hold` |
| `?E` | `unf=0 ovf=0 lim=0.0 limn=0 uf=0 pk=0 bg=on ceil=63.0 cmax=75.0 gd=act i2s=24/32 al=i2s cal=ok` — modulator and limiter status; `bg=` = FPGA limiter: `on` / `off` (by `B 0`) / `na` (not present in the bitstream); `ceil=` = limiter ceiling that applies NOW in kHz (or `off`); `cmax=` = the chosen maximum (`H`); `gd=` = mask protection `off` (also when the limiter is off: the guard only works with the limiter on) / `init` (initialising: still too little mask data, about 60 s after start, transmitter open, `X` or `G 1`; the guard does not make decisions yet) / `on` / `act` (ceiling has been lowered); `i2s=<bits>/<slot>` = automatically detected I2S format (word width / BCLKs per WS half period, e.g. `24/32`, `16/16`) and `al=i2s|lj|rj` = detected alignment. In full the reply ends with `... gd=<off|init|on|act> i2s=<bits>/<slot> al=<i2s|lj|rj> cal=<ok|run|fail>`; `cal=` is the TX calibration of the AD9361 (LO leakage, image): `run` while a tune, an opening of the transmitter or a temperature recalibration is in progress (the output is muted, about 2 s), and also during the automatic LO/image nulling that follows it (`NULLAUTO 1`, silent carrier, about 5 s), `fail` when it failed three times (the output stays muted and the daemon tries again after 30 s, or earlier on `F`, `E` or `CAL`). |
| `?IQ` | `dci=0 dcq=0 gain=0 skew=0 ofs=0.000 lvl=-9.00 lo=108000000 hw=1` — the digital corrections (`DC`, `IQ`), the low-IF offset (`OFS`), the digital level (`LVL`), the TX LO frequency and `hw=1` when the bitstream has the gain/skew correction |
| `?NL` | `NULL ok dci=60 dcq=-41 qg=-421 qs=-97 iq=ok before=-57.3 after=-71.2 imgb=-49.5 image=-77.5 floor=-80.0 curv=1.01,1.05 curvi=1.04,1.06 gain=44 x=100 auto=1 rx=2` — the last nulling: DC result, gain/skew in register units (2^-18), `iq=ok/fit/na`, LO line and image before/after and the measurement floor in dBc, the fit curvatures (≈ 1 = the model fits), RX gain, the offset used (kHz); then `NULLAUTO` and `NULLRX`. `nl=idle` / `nl=run` / `NULL err=<reason>` otherwise. |
| `?V` | `magic=57464D32 fw=PlutoSky_7020_Broadcast-1.02 proto=2 ver=1.02 bit=B1D0001A` — identification: `ver=` is the software version of the release (show this one), `bit=` the build id of the FPGA bitstream (`-` if the bitstream has no diagnostics) |
| `?C` | `cal=auto` |
| `?M` | `m=+7.3 w=+5.8 sh=-48.7 fl=-71.7 dev=72 n=1180 age=0 tw=0 c=1 seq=5231 ams=180 hit=-1 under=0.0 wf=+132` — mask monitor (see §4a) |
| `?L` | `l=<57 characters> q=<n>` — spectrum of the **latest** measurement (~250 ms); `q` = sequence number (equal to the previous reply = repeat), `l=na` without a measurement |
| `?D` | `d=<kHz> ceil=<kHz> q=<n> ams=<ms>` — measured peak deviation (kHz) from the FPGA conditioner (`pk/2^23 × 0.75 × kdev`), limiter ceiling, window sequence number and age; `d=na` without a measurement |
| `?B` | `b=100 Gereed` — latest boot step (see 'Boot status'), `b=0 -` if there has been none yet |
| `?A` | `a=19000.2 6.700 q=146 ams=176` — **tone measurement**: frequency (Hz) and deviation amplitude (kHz) of the strongest audio tone (100 Hz–25 kHz) in the own transmission (through the RX2 measurement path, FM demodulation, 16384-point FFT, power in the main lobe; accuracy < 0.005 dB offline), new every ~0.25 s (`q` = sequence number, `ams` = age in ms). The first `?A` switches the measurement on (valid for 10 s: keep polling); `a=na` without a fresh measurement. For audio linearity measurements (tone sweep). |
| `J <secs> [pct]` | **end-to-end impulse measurement** start (5..600 s, threshold `pct` % of full scale, 1..90, default 12). Meanwhile the encoder puts one impulse per second in the I2S stream (the rest silent or pilot only). The FPGA timestamps the impulses; the RX1 channels of the capture carry the timestamps; the mask monitor is paused. Reply `OK`; `ERR range` / `ERR off` (transmitter closed) / `ERR busy` / `ERR nomask`. Result: `/tmp/pluto_ir.csv` (f, magnitude relative to 1 kHz, phase, group delay from I2S frame to RX2 demodulation, time-gated). |
| `?J` | `j=run 40 n=1301` / `j=ok 0 n=..` / `j=err <rc> ..` / `j=idle n=<n>` / `j=busy` (an LO measurement `Y` is running) — status of the impulse measurement; `n` = number of impulses detected by the FPGA (counter). |
| `Y <secs>` | **LO leakage / fine spectrum** (5..600 s): per 0.5 s window the strongest audio tone (frequency, deviation), the level of the carrier bin (dBc relative to the total power) and absolute levels (RX2 measurement path). At a Bessel null of the carrier (tone f_m, deviation 2.405 × f_m) only LO leakage remains in the carrier bin. Files: `/tmp/pluto_lo_sweep.csv` (per window), `/tmp/pluto_lo_null.csv` (fine spectrum ±150 kHz, RBW 94 Hz, deepest carrier), `/tmp/pluto_lo_ref.csv` (same, first window). Replies as `J`. |
| `?Y` | `y=idle` / `y=run 40` / `y=ok 0` / `y=err <rc>` — status of the LO measurement. |
| `Z <secs>` | **write out the demodulated MPX** (5..300 s) for stereo linearity: repeatedly a window of 0.5 s of RX2 IQ (measurement path), FM demodulation (3.072 MSPS), decimation with a linear-phase FIR (351 taps, flat up to ~60 kHz) to exactly 192 kHz. Files `/tmp/pluto_mpx.f32` (float32, deviation in kHz, windows of 96000 samples) and `/tmp/pluto_mpx_index.csv` (window, t_s, UTC ms, strongest tone, deviation); convert with `scripts/mpx_to_npz.py`. Replies as `J`. |
| `?Z` | `z=idle` / `z=run 40` / `z=ok 0` / `z=err <rc>` — status. |
| `?R` | `r=ok g=11 fail=0` — **the RX2 measurement path** (the directional coupler on TX1, or the TX2 → RX2 cable with `twin=1`; needed for the mask monitor): `ok` = the mask measurement receives signal; `missing` = two consecutive measurements without signal, the RX gain at the maximum (≥ 70 dB with the coupler, ≥ 55 dB with the twin) or ≥ 4 windows in a row with noise instead of signal (coupler/cable loose or not connected): the mask values are then invalid; `na` = transmitter closed, mask monitor not on RX2 or no measurement yet. `g` = RX2 gain (dB), `fail` = consecutive failed measurements. The web interface then shows a warning. |
| `?O` | `o=-29.00 w=1.258e-06 in=-49.00 att=20.00 cal=-2.78 g=47 st=ok` — **power meter**: `o` = power at the TX1 output (dBm), `w` = same in watts, `in` = level at RX1, `att` = set attenuator, `cal` = calibration correction, `g` = RX1 gain; `st` = `ok` / `low` (below about −70.5 dBm at RX1: the board's own noise/crosstalk, about −71.5 dBm, dominates) / `high` (too strong, > −10 dBm at RX1 or the ADC compresses) / `nosig` / `off`. Without a valid measurement `o=na w=0 in=na ...`. Accuracy (measured with an analyser, 20 dB attenuator): ±0.1 dB from −40 to −10 dBm at the output, +0.4 dB at −50 dBm; below that not measurable. The meter switches by itself between RX1 gains of 27/37/47/57/67 dB and corrects the deviation of each setting. |
| `?G` | `g=61.8 62.1 59.0 ... q=<n>` — **swing meter**: peak deviation (kHz) per window of **20 ms**, the last 25 windows (0.5 s), **oldest first**, same measurement as `?D` (at the modulator input, after limiter + interpolator); `q` = sequence number of the newest window (equal to the previous reply = repeat). Ask it 5–10× per second (every 100 ms 5 new windows are added). `g=na` without a measurement. `?D` is the maximum of the last 12 windows (~0.25 s). |
| `?W` | `s=<57 characters>` — 5-minute max-hold spectrum, `s=na` without a measurement |
| `?N` | `f=<57 characters>` — the monitor's own noise floor, same encoding |

- `f` is the actual LO; the PLL rounds (±2 Hz relative to the requested `F`).
- `p` = requested level, `att` = actual attenuation (= 5.0 − p + trim); with `tx=off`, `att` is 89.75.
- `kdev` = the **deviation range currently in the FPGA**: deviation at full scale = `kdev × 0.75 kHz` (kdev 100 = ±75 kHz at 0 dBFS).
  For a deviation meter: deviation = composite peak (fraction of full scale) × 0.75 kHz × `kdev` × digital output level (linear).
- `out` = estimated actual level according to the TX_MONITOR measurement; `trim` = the correction
  of the level control (dB, + = more attenuation).
- `alc` = `wait` (no reference yet) · `hold` (at level) · `adj` (adjusting) ·
  `limit` (trim at the ±3 dB limit) · `low` (monitor signal too weak) · `off` (transmitter off).
- `?E`: `unf`/`ovf` = FIFO underflow/overflow of the modulator (bit 1/bit 0 of the status register). `lim` = largest gain reduction (dB) of the
  limiter in the last ~10 s, `limn` = number of limited samples since start, `uf` = number of times the I2S has dropped out (soft fade),
  `pk` = input peak in % of full scale (last ~10 s). **These four are 0 as long as the bitstream has no active limiter** (see §5).
- **Restart detection:** `up=` is the number of seconds the daemon has been running. If it goes
  back (or `en`/`f`/`p` differ from what the encoder sent), the Pluto has
  restarted and the encoder sends E/F/P again.

## 4a. Mask monitor (SM.1268-5) live

The Pluto itself measures the RF spectrum against the ITU-R SM.1268-5 mask (Hann FFT of 512 points @3.072 MSPS = 6 kHz/bin, RBW ~9 kHz, max-hold, 0 dB = peak)
on TX1 itself through a directional coupler into RX2 (the standard setup, `twin=0`; TX2 is then kept muted). The older alternative, `twin=1`, measures a copy of the
modulation on TX2 through a cable TX2 → RX2 (see `docs/twin-tx2.md`; the values are corrected to TX1).

**Continuous, ~4 spectra per second.** A mask process reads 25 s of uninterrupted IQ (gapless: a reader thread fills a ring of windows while the
FFT runs over both cores; if the processing falls behind, the oldest windows are skipped, no gaps arise within a window). Each
window of 0.25 s yields one spectrum with the max-hold of exactly that interval; the coverage of the FFT windows is 1.00. The process sets gain and
noise floor once per 25 s; between two processes there is a pause of ~2–3 s (ALC, gain, noise floor). A spectrum is ignored if the recording
is disturbed (phase jump between two samples or an impossible deviation > 150 kHz); the first window of each process (LO PLL) is skipped.
The **running max-hold over the last 300 s (about 1200 spectra, time-based)** is the number that counts.

- `?M`: `m` = margin of the latest measurement (dB, positive = within the mask); **`w` = margin of the 5-minute max-hold**;
  `sh` = mean shoulder level ±130…170 kHz (dB below the peak); `fl` = monitor noise floor (same scale);
  `dev` = peak deviation (kHz) of the 5-minute window, from a band-limited FM demodulation of the RF (±200 kHz channel, 90 kHz composite low-pass; 4 ms of every other window, so it can miss a rare peak: `?D`/`?G` from the FPGA see every sample); `n` = number of spectra in the
  window (max ~1200); `age` = seconds since the last spectrum; `tw` = 1 via the twin; `c` = 1 if the shoulder is ≥ 6 dB above the floor;
  `hit` = seconds since the last single spectrum (0.25 s) that was below the mask (-1 = none in the 5-minute window); `under` = % of the window's spectra below the mask;
  `wf` = offset (kHz) of the worst bin of the 5-minute max-hold (where `w` is measured)
  (0 = floor-limited, `w` is then a lower bound); `seq` = spectrum sequence number; `ams` = age of the latest spectrum in ms. Without a measurement: `m=na n=0`. No measurement is made while the transmitter is off (`E 0`).
- `?L` / `?W` / `?N`: 57 points, k = −28…+28, frequency = k × 6 kHz relative to the carrier (−168…+168 kHz). One character per point: level in dB below the peak
  in steps of 1.25 dB; `A`…`Z` = 0…25, `a`…`z` = 26…51, `0`…`9` = 52…61 (so dB = −1.25 × index; the 29th character is the carrier).
  `?L` = the latest single spectrum (live trace), `?W` = the max-hold over the 5-min window, `?N` = the own noise floor.
  Between two fresh spectra `?L` gives the same reply; polling at 1–4 Hz is sensible (each reply is ~60 characters = ~5 ms at 115200 baud).
- The mask itself (0 dB up to ±74 kHz, −15 dB at ±107.5, −30 dB at ±124, −40 dB at ±152.5 and beyond) is drawn by the encoder.
- Suggested status colour: green `w` ≥ +3, orange 0…+3, red < 0.

## 5. What is automatic and fixed (not in the protocol)

| Item | Behaviour |
|---|---|
| Deviation | `kdev` is set by the encoder (`K`); `kdev=` in `/mnt/jffs2/skypluto-lim.conf` (re-read every 10 s) is only the fallback until the first `K`, default 100 (0 dBFS = 75 kHz). Without an active limiter in the bitstream kdev is limited to 100 (75 kHz). The actual peak deviation is kdev × 0.75 kHz × the encoder's composite peak (e.g. kdev 92 and a ceiling of 90 % gives ~62 kHz). |
| Digital level | fixed full scale (65535, best DAC SNR) |
| Green LED | owned by the daemon (the kernel's `tx-active` trigger is switched off): off = transmitter closed, blinking 2 Hz = tuning/calibrating (output muted), on = on the air (open, not calibrating, not muted). A failed calibration (`cal=fail`) leaves it off. |
| Offset | fixed 0 (zero-IF, cleanest) |
| Calibration | temperature-following, **fixed** threshold of 8 °C with 60 s debounce; no protocol commands. Only on sustained drift (typically once, while warming up); a calibration interrupts the carrier very briefly. |
| **Level control (ALC)** | automatic. As soon as a level is set, the Pluto takes a **reference** via the internal TX_MONITOR and then corrects drift every ~20 s via the attenuation: max. ±3 dB, 0.25 dB steps, smooth (no carrier interruption). |
| **Interpolator saturation** | the FIR of the interpolator (192 kHz → 3.072 MHz) has overshoot (typically +5–6 %, up to +25 % with flat tops). The output is **saturated** instead of being allowed to wrap: a wrap (sign change) gives a ~4 µs frequency dip and 13–22 dB extra energy at 130–170 kHz (the cause of rare mask bursts). Register `DBG` word 11 counts the saturations. |
| **FPGA limiter / soft fade** | in the RTL (`skypluto_sigcond`, registers 0x20–0x38): a peak limiter at the ceiling set with `H` (default 67 kHz, derived from `kdev`) and a soft fade-in/out on I2S loss. The daemon detects the limiter by register read-back (`SC_STATE[1:0]` reads `3` in a bitstream with a bypassed conditioner: then no limiter is present, kdev ≤ 100, `bg=na`). |
| **Mask protection (guard)** | the daemon lowers the limiter ceiling when the 5-minute mask margin gets too small, and raises it again when the margin is ample; see command `G` (§3). Mask measurement itself is continuous, see §4a. |

The ALC keeps the level **constant relative to the reference** (tested: 2 dB of simulated drift
measured correctly, trimmed back within 0.3 dB). The TX_MONITOR is itself temperature-sensitive,
so this is a relative control, not an absolute dBm measurement.

### Configuration files on the Pluto (`/mnt/jffs2/`)

| File | Keys |
|---|---|
| `skypluto-lim.conf` | `kdev=` (deviation range), `ceil_khz=` (limiter ceiling, fallback until the first `H`), `ceil_max_khz=` (upper bound for the ceiling from the conf file, max. 70), `guard=0/1` (mask protection, fallback until the first `G`), `guard_low=` (margin in dB below which the guard acts, 3..12, default 3.5) |
| `skypluto-mask.conf` | `enable=1`, `rx=2` (RX2), `port=A_BALANCED`, `twin=0` (TX1 through the coupler; `twin=1` = TX2 follows TX1 for the measurement). Without the file: `enable=1 rx=2 twin=0` |
| `skypluto-iq.conf` | `dc_i=`, `dc_q=`, `qgain_ppm=`, `qskew_ppm=` (written by `DC`, `IQ` and the nulling), `lowif_khz=` (`OFS`), `level_db=` (`LVL`), `null_auto=` (default 1), `null_rx=` (default 2) |
| `skypluto-pwr.conf` | `enable=`, `atten_db=`, `cal_db=` (power meter; written by `Q`, `QK`, `QM`) |
| `skypluto-web.conf` | `password=` (optional password for the web interface) |

### Diagnostic registers (FPGA)

`0x3C` (reading gives the selected word): 0 = SD/WS edges, 1 = last raw I2S word, 2 = frames, 3 = FIFO write counter, 4 = peak raw word,
5 = peak FIFO output, 6/7/8 = pulls/pops/non-zero (conditioner), 9 = peak conditioner output, 10 = peak interpolator output, 11 = FIR saturations,
12 = build ID (identifies the bitstream; the impulse detector/timestamp registers are 0x40..0x4C), 13 = heartbeat, 14 = take, 15 = ovf, 16/17 = interpolator/conditioner state, 18 = watchdogs, 19/20/21 = window peak comp/q/seq (20 ms windows), 22 = gain `gs` (Q16, 65536 = 1.0), 23 = gain sum (15 × 65536 = 0xF0000 without limiting), 24..31 = the last 8 window maxima of 20 ms (24 = newest). Writing selects a word 0…31. For diagnostics only.

## 6. Example session

```
Encoder → ?V             Pluto → magic=57464D32 fw=PlutoSky_7020_Broadcast-1.02 proto=2
Encoder → F 108000000    Pluto → OK
Encoder → P -15.00       Pluto → OK
Encoder → E 1            Pluto → OK
Encoder → ?S             Pluto → en=1 f=107999998 p=-15.00 att=20.00 tx=on up=12 kdev=92
Encoder → ?T             Pluto → temp=42.1
Encoder → ?P             Pluto → set=-15.00 out=-15.02 att=20.00 trim=0.00 alc=hold
Encoder → ?L             Pluto → l=iiihhhgggebZXUSRONLIGB…   (spectrum of the latest window)
Encoder → ?M             Pluto → m=+11.4 w=+5.8 sh=-46.3 fl=-70.8 dev=72 n=1180 age=0 tw=1 c=1
Encoder → E 0            Pluto → OK          (RF completely off: attenuation max + TX LO off)
```

## 7. Reserved

RDS/RDS2 symbol streams (`D <stream> <hexbytes>`) once the composite generation
moves to the FPGA; well within 115200 baud.
