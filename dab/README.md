# PlutoSky DAB+ — plan and software

Status: under construction (2026-10-22). Software on the PC only; nothing has been sent to the Pluto or flashed yet.

*Update 2026-10-24 (translation into English and code review):* the status line above dates from 2026-10-22. The git history
records later work that is not in the phase list below: tests on the Pluto, a multiplex of up to 8 services (one encoded on
the Pluto, the others received from ODR-AudioEnc over ZeroMQ, `src/ext_stream.*`, `src/zmtp_sub.*`), a shared layout file
(`src/mux_conf.*`, also used by `skypluto-ctl`), multiplex reconfiguration, MOT slideshow (`src/dab_mot.*`), an own
MPEG-1 Layer II encoder for DAB (`src/mp2_enc.*`) and daemon/web-interface control. The current FPGA build id is B1D00021.

Goal: the PlutoSky 7020 as a **DAB+ transmitter** (one ensemble, one or a few services). The audio arrives as **stereo PCM over
I2S** (no longer as MPX), the Pluto encodes it to AAC itself, builds the DAB+ super frame, the multiplex and the OFDM signal
(mode I, Band III) and transmits it through the AD9361.

## 1. Licences (open-source project, Apache-2.0)

| Component | Choice | Why |
|---|---|---|
| AAC encoder | **own code** (this directory) | fdk-aac has its own Fraunhofer licence, which is not compatible with Apache/GPL and grants no patent rights; FFmpeg (LGPL) and faac (LGPL) do not fit an Apache project. |
| Huffman tables of the AAC standard | derived from **vo-aacenc** (VisualOn, Android, **Apache-2.0**) with attribution | the code books are normative data from ISO/IEC 14496-3; vo-aacenc provides them under the same licence as this project. They are converted and checked by a script (prefix-free, Kraft equality). |
| Reed-Solomon, CRCs, super frame | own code, following ETSI TS 102 563 | no code taken over; ODR-AudioEnc was only read as a reference. |
| Multiplex (FIC/MSC), channel coding, OFDM | own code, following ETSI EN 300 401 | ODR-DabMux/ODR-DabMod (GPL-3) only as a **test reference** outside the repository. |
| MPEG-1 Layer II encoder (DAB) | own code, following ISO/IEC 11172-3 and ETSI EN 300 401 *(added after 2026-10-22)* | the analysis window is an own Kaiser-windowed sinc, fitted by measurement to the standard synthesis filterbank. |
| Test tools | PyAV (FFmpeg decoder), reedsolo | only for testing, not part of the software. |

**Patents.** An own encoder solves the *copyright* question, not the patents. According to general information the basic
patents for AAC-LC have largely expired; for HE-AAC (SBR, 2001–2003) and HE-AAC v2 (PS, 2004) this is less certain, and a
licence pool (Via LA) still exists. Hence: **AAC-LC first** (allowed in DAB+), SBR/PS only later and only after a patent
check. This is not legal advice; have it checked before an official broadcast.

## 2. Chain

```
I2S stereo 48 kHz ─► FPGA I2S receiver ─► audio FIFO (1024 × {L16, R16}) ─► ARM (0x6C/0x70)
ARM (skypluto-dab): resampler 48 kHz × (1 ± ppm) to the transmitter clock ─► AAC-LC encoder (960, CPE with M/S)
      ─► DAB+ super frame (120 ms): header + fire code, AUs with CRC, RS(120,110), DLS text as X-PAD
      ─► multiplex: FIC + MSC sub-channel, energy dispersal, convolutional code + puncturing (EEP-A),
         time and frequency interleaving, DQPSK ─► per carrier a 2-bit phase index (16 per 32-bit word)
      ─► phase words (7296 per 96 ms) ─► FPGA (0x68, with a handshake on the bus)
FPGA: FIFO 8192 words ─► IFFT 2048 (1536 carriers) ─► cyclic prefix 504, null symbol 2656 ─► 2.048 MS/s
      ─► 3/2 interpolator (72 taps) ─► 3.072 MS/s ─► × 2^gain ─► the modulator's I/Q path (level, I/Q correction,
      DC nulling, as for FM; channel filter off) ─► AD9361 (Band III 174–240 MHz)
```

*(Update 2026-10-24: the encoder stage can also be the MPEG-1 Layer II encoder (DAB, 5 frames of 24 ms per 120 ms), and
the multiplex also supports EEP-B and up to 8 sub-channels; see `app/skypluto_dab.c`.)*

The split: everything that processes bits runs on the ARM (~1/10 of real time on the PC), the computationally heavy OFDM
(IFFT, 3/2 interpolation) in the FPGA. This keeps the AD9361 configuration and the mask monitoring the same as for FM.

### Registers (WFM block, 0x7C440000)

| Address | Read | Write |
|---|---|---|
| 0x64 | DAB control | [0] DAB mode (instead of FM), [1] reset of the OFDM block, [3:2] gain 2^n (saturating) |
| 0x68 | status: [13:0] OFDM FIFO level, [16] underflow (stays until reset), [17] CDC FIFO full | one phase word (waits on the bus while the CDC FIFO is full, at most 1024 clocks) |
| 0x6C | audio sample {L[15:0], R[15:0]} (a read pops it from the FIFO) | – |
| 0x70 | number of audio samples waiting | – |

The OFDM signal peaks around −11 dBFS (rms −22 dBFS); gain 1 (×2) gives −5 dBFS peak. The carrier level (0x08) and the
I/Q and DC correction work as for FM. After an underflow `skypluto-dab` puts the block in reset and restarts it on a frame
boundary.

## 3. AAC encoder (own design)

| | first version | later |
|---|---|---|
| Profile | AAC-LC, frame length **960** (mandatory in DAB+) | SBR (HE-AAC), PS (HE-AAC v2) after a patent check |
| Sampling | 48 kHz (6 AUs per super frame) and 32 kHz (4 AUs) | 24/16 kHz core with SBR |
| Channels | stereo as CPE with a common window, M/S per band | intensity stereo at low bitrates |
| Transform | MDCT 1920 → 960 lines, sine window (KBD as an option) | block switching with short windows (8 × 120) against pre-echo |
| Psychoacoustics | energy per scale factor band, spreading, tonality estimate, absolute threshold | refine with listening tests |
| Quantisation | x^(3/4), global gain + scale factors, noise-to-mask target, bit reservoir per super frame | TNS |
| Entropy coding | all 11 spectral code books + the scale factor code book, optimal sectioning | |
| Testing | decoding with FFmpeg (PyAV): no errors, SNR/NMR against the original, listening | ODR-AudioEnc as a quality reference |

*(Update 2026-10-24, checked in `src/aac_enc.c`: block switching with short windows and perceptual noise substitution (PNS)
are implemented; TNS, intensity stereo, SBR/PS and the KBD window are not (the window shape is always sine).)*

Bitrate advice: AAC-LC stereo **96–128 kbit/s** (48 kHz) or **80–96 kbit/s** with a 32 kHz core (15 kHz audio bandwidth,
the same as FM).

## 4. Phases

1. **Super frame + RS + CRC** (C), tested with reedsolo and an own decoder. ✔ *(2026-10-22: 24 configurations, RS corrects 5 byte errors)*
2. **AAC-LC encoder** 960, long windows, M/S; tested with FFmpeg. ✔ *first version (2026-10-22): FFmpeg decodes without errors,
   48 and 32 kHz, 80–128 kbit/s, exact level (gain 1.000), tones S/N 54–56 dB* ← *next: quality*
3. Block switching (short windows), bit reservoir, quality measurements. ✔ *block switching (2026-10-22): attack detector with
   one frame of look-ahead, LONG_START/EIGHT_SHORT/LONG_STOP, grouping around the attack; pre-echo −17.7 → −45.6 dB.*
   Still to do: noise substitution (PNS) against empty high bands with a tight budget, TNS, speed (now ~30× real time on one
   PC core; the Pluto's ARM is ~10–20× slower, so the rate control must become more efficient).
   *(Update 2026-10-24: PNS is implemented (`quantise_all()` in `src/aac_enc.c`); TNS is not; the speed was not re-measured
   in this review.)*
4. **Multiplex** (FIC/MSC) and **OFDM modulator in C** on the PC; IQ file; checked with a DAB+ receiver/decoder. ✔ *(2026-10-22)*
   welle-cli (welle.io, with FAAD2) receives the IQ file: OFDM synchronisation, FIC (ensemble, service, DAB+, sub-channel),
   super frame synchronisation, AAC-LC 48 kHz stereo, **DLS text** (X-PAD) – S/N the same as with the FFmpeg decoding.
   On the PC encoder + modulator take ~1/10 of real time; the OFDM is too heavy for the ARM → FPGA (phase 6).
5. Cross-compiling for the Pluto's ARM ✔ *(builds without warnings)*; measure the processor time on the Pluto. *(Requires permission.)*
6. OFDM in the **FPGA**, bit-exact against the model. ✔ *(2026-10-22)* `hdl/skypluto_dab_ofdm.v`: bit-exact over two frames,
   synthesised on its own 1875 LUT, 14 BRAM, 16 DSP. **Built into the exciter** (branch `dab-integration`, build id B1D0001E;
   *current build id B1D00021*): `hdl/tb_dab_top.v` drives the whole exciter over AXI as the ARM does: the modulator path is
   bit-exact (0 errors over 1.16 million samples), the audio reaches the ARM through the FIFO without errors.
   *(Flashing: requires permission.)*
7. **skypluto-dab** (ARM program) ✔ *first version*: audio from the FIFO, resampler with rate control on the total delay
   (PI, time constant ~1 minute, ±2000 ppm), encoder + multiplex in their own thread, phase words with flow control, restart
   after an underflow. Simulation mode without hardware (`-f`): +80 ppm clock difference, 60 s, no underflow, welle-cli
   decodes (tones S/N ~38 dB). Still to do: daemon/web interface, DAB mask and MER via RX2, Pico in stereo I2S mode.
   *(Update 2026-10-24: the daemon/web interface control exists (`?DAB`, `DCO`, `MXA`/`MXS`/`MXD` in `src/skypluto-ctl.c`)
   and the git history records the PicoAudio on the air over I2S; the DAB mask and MER via RX2 were not checked in this
   review.)*

## 5. Building and testing (PC)

```
make -C dab                        # builds the library and the test programs (gcc, WSL)
make -C dab test                   # MDCT test and super frame test (Reed-Solomon via reedsolo)
tests/check_chain.sh in.f32        # dab_tx_file → welle-cli
tests/check_app.sh [ppm] [s]       # skypluto-dab in simulation mode → FPGA model → welle-cli
python3 hdl/check_hdl.py build/ph.bin 2         # OFDM block against the model (iverilog)
TOP=1 python3 hdl/check_hdl.py build/ph.bin 2   # the whole exciter in DAB mode (tb_dab_top.v)
build/test_mdct                    # fast MDCT against the direct definition
python tests/check_aac.py build    # encoding + decoding with FFmpeg (PyAV), S/N and level
```

Test dependencies (on the PC only): Python with numpy, `reedsolo` and `av` (PyAV). For the chain test: welle-cli (welle.io),
built here with faad2, mpg123 and LAME under `~/dablocal` (no root needed).
`TONE=1000 tests/check_app.sh 0 20` tests the built-in test tone (`-T`).

### First test on the Pluto

```
dab/tools/install_dab_test.sh <ip> out/dab/BOOT.bin   # stages everything, changes nothing about what boots
ssh root@<ip> /mnt/jffs2/dab-boot.sh dab              # boot BOOT.bin.dab (FM keeps working, + DAB)
ssh root@<ip> /mnt/jffs2/dab-test.sh 174928000        # channel 5A, test tone 1 kHz left / 1.5 kHz right; Ctrl-C = back to FM
ssh root@<ip> /mnt/jffs2/dab-boot.sh fm               # back to the FM release (BOOT.bin.fm)
```

`dab-test.sh` sets the TX bandwidth to 3 MHz through the daemon and tunes (with calibration); afterwards the bandwidth is
restored. The DAB signal is ~16 dB below the FM carrier at the same P setting; the mask monitoring (FM mask) means nothing
during the test. `dab-test.sh <f> i2s` uses the I2S input (stereo 48 kHz) instead of the tone.

## 6. Files

| File | Contents |
|---|---|
| `src/dabplus_sf.*`, `src/dab_rs.*`, `src/dab_crc.*` | super frame, Reed-Solomon RS(120,110), fire code and CRC |
| `src/aac_enc.*` | AAC-LC encoder (960): psychoacoustics, block switching, M/S, PNS, quantisation with rate control, Huffman, fill |
| `src/aac_mdct.*` | MDCT via DCT-IV and a mixed-radix FFT (480 = 2⁵·3·5) |
| `src/aac_huff_tab.*` | code books (generated, see `NOTICE`) |
| `src/bitwriter.h` | MSB-first bit writer |
| `src/dabplus_enc.*` | PCM → super frames, distribution of the budget over the AUs; the same interface for MPEG-1 Layer II |
| `src/mp2_enc.*` | MPEG-1 Layer II encoder for DAB (48 kHz stereo; the DAB layout allows 112..192 kbit/s) *(added after 2026-10-22)* |
| `src/dab_pad.*` | Dynamic Label (DLS) in variable-size X-PAD, as a data stream element in the AU (or at the end of an MP2 frame) |
| `src/dab_mot.*` | MOT slideshow in X-PAD (one image at a time) *(added after 2026-10-22)* |
| `src/dab_mod.*` | DAB mode I: FIC (FIG 0/0, 0/1, 0/2, 0/9, 0/13, 1/0, 1/1), EEP-A and EEP-B, multiplex reconfiguration, energy dispersal, convolutional code, time and frequency interleaving, DQPSK, OFDM (IFFT 2048) → IQ 2.048 MS/s, or the phase words for the FPGA |
| `src/mux_conf.*` | the multiplex layout file (parser/writer), shared with `src/skypluto-ctl.c` *(added after 2026-10-22)* |
| `src/ext_stream.*`, `src/zmtp_sub.*` | services from ODR-AudioEnc over ZeroMQ: own ZMTP 3.0 SUB endpoint (no libzmq), jitter buffer *(added after 2026-10-22)* |
| `tests/dab_tx_file.c`, `tests/check_chain.sh` | complete transmitter on a file; chain test with welle-cli |
| `app/skypluto_dab.c` | the transmitter program on the ARM (registers, resampler, rate control, threads; `-f` = simulation) |
| `tests/check_app.sh` | skypluto-dab in simulation → FPGA model → welle-cli |
| `hdl/` | `skypluto_dab_ofdm.v` (copy in `hdl/library/skypluto_wfm`), model, generators, test benches, OOC synthesis |
| `tools/build_arm.sh` | cross build for the Pluto's ARM (build only) |
| `tools/install_dab_test.sh`, `tools/dab-boot.sh`, `tools/dab-test.sh` | DAB test on the Pluto: staging, boot choice FM/DAB, test with tone or I2S |
| `tools/gen_aac_huff.py` | generates and checks the code books |
| `tests/` | `test_sf`, `check_sf.py`, `test_mdct`, `enc_file`, `check_aac.py` |

## 7. Sources

- ETSI TS 102 563 — DAB+ audio (super frame, RS, fire code, CRC).
- ETSI EN 300 401 — DAB (FIC, MSC, channel coding, OFDM, PAD).
- ETSI EN 301 234, ETSI TS 101 499 — MOT and SlideShow.
- ISO/IEC 14496-3 — MPEG-4 Audio (AAC).
- ISO/IEC 11172-3 — MPEG-1 Audio (Layer II).
- RFC 23/ZMTP — ZeroMQ Message Transport Protocol 3.0.
- vo-aacenc (VisualOn/Android, Apache-2.0) — source of the Huffman tables, see `NOTICE`.
