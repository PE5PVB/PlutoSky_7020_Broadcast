#define _GNU_SOURCE
// =============================================================================
// skypluto-ctl.c - SkyPluto WFM control/telemetry daemon (Pluto side), protocol v2
// -----------------------------------------------------------------------------
// Half-duplex single-wire UART on U9 (fabric UART @ 0x7C450000) <-> PicoAudio
// (master). The protocol is deliberately minimal: the Pico only controls
//     E 0|1        transmitter on/off      (off = RF really off the air)
//     F <Hz>       frequency
//     P <dBm>      output level (setpoint)
//     ?T           temperature
// Everything else is fixed and/or runs automatically on the Pluto:
//     kdev=100 (+-75 kHz at 0 dBFS = 100 % deviation), digital level full, offset 0 (zero-IF),
//     TX calibration (LO leakage, image) after every tune and when the die temperature has drifted (done here, with the output muted; NO
//     protocol), and automatic level control (ALC) via TX_MONITOR.
// Obsolete commands (A K L O PT PB PE CT CE C) are answered with OK and ignored,
// so that an older Pico firmware does not keep repeating them. A is treated as an alias of P.
//
// ALC: a child process (--measure) sets the RX to TX_MONITOR1, measures the power (two
// floor measurements are subtracted) and reports the level in dB. The daemon takes a
// REFERENCE as soon as a setpoint is set and then trims drift away via the TX attenuation
// (max +-3 dB, 0.25 dB steps: smooth, no carrier interruption). It is a RELATIVE
// control loop: the absolute dBm rests on DBM_AT_0DB (an estimate, may be off by a few dB
// until measured once).
//
// Build (buildroot toolchain, dynamic):
//   arm-linux-gnueabihf-gcc -O2 -Wall -o skypluto-ctl skypluto-ctl.c -lm
// =============================================================================
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>
#include <sched.h>
#include <sys/mman.h>
#include <sys/prctl.h>
#include <sys/wait.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#define SELF       "/mnt/jffs2/skypluto-ctl"

// ---- physical addresses -------------------------------------------------------
#define WFM_BASE   0x7C440000UL
#define MAP_BASE   WFM_BASE
#define MAP_LEN    0x11000UL           // covers WFM (0..0x20) and UART (0x10000..0x1000C)

#define R_CTRL     0x00
#define R_OFFSET   0x04
#define R_LEVEL    0x08
#define R_KDEV     0x0C
#define R_STATUS   0x10
#define R_MAGIC    0x14

#define U_TX       0x10000
#define U_RX       0x10004
#define U_ST       0x10008
#define U_CLR      0x1000C

// ---- fixed settings -----------------------------------------------------
#define KDEV_FIXED   100               // default (without conf): deviation = kdev x 750 Hz at full scale: 100 = +-75 kHz at 0 dBFS
                                       // (PicoAudio: 100 % deviation = full scale); 200 would give +-150 kHz, which is wrong.
                                       // With -6 dB headroom on the Pico side (100 % = -6 dBFS), kdev 200 = +-75 kHz.

// Conditioner in the FPGA (limiter + soft fade-in/out), registers relative to WFM_BASE
#define R_LCEIL      0x20              // peak ceiling in counts (full scale 2^23)
#define R_LCTRL      0x24              // [0] limiter on [1] fade on [2] clear statistics (pulse)
#define R_LGMIN      0x28              // smallest g_req since clear (Q16, 65536 = no limiting)
#define R_LEVT       0x2C              // samples with limiting (counter)
#define R_LPEAK      0x30              // largest |x| since clear (counts)
#define R_LSTAT      0x34              // [1:0] state (1 = RUN), [31:15] fade Q16
#define R_LUF        0x38              // number of times I2S dropped out (counter)
#define R_IMPC       0x40              // impulse measurement: [0]=on (RX1 channels carry timestamps)
#define R_IMPT       0x44              // detection threshold (counts)
#define R_IMPS       0x48              // l_clk counter at the last impulse (ro)
#define R_IMPN       0x4C              // number of impulses (ro)
#define R_FMT        0x50              // I2S format (ro): [17:16] alignment (0 I2S, 1 LJ, 2 RJ), [15:8] slot length (BCLKs per WS half period), [7:0] word width
#define R_DCI        0x18              // digital DC offset on I (signed 12 bit, 16-bit sample LSBs): LO-leakage nulling
#define R_DCQ        0x1C              // digital DC offset on Q
#define R_QGAIN      0x58              // Q gain correction (signed 18 bit, units 2^-18) - bitstream B1D00019 and later
#define R_QSKEW      0x5C              // I/Q phase correction (signed 18 bit, 2^-18 rad)
#define R_I2SCTRL    0x54              // manual I2S format (rw): [0] on, [2:1] alignment, [15:8] word width; 0 = automatic
#define TCP_PORT     5555              // TCP console (localhost)
#define SW_VERSION   "1.02"            // software version of this release (also in ?V, the SD-card image and the README)
#define R_DBG        0x3C              // write: select diagnostic word 0..31; read: that word (only in bitstreams with a 6-bit register address)
#define LIMCONF      "/mnt/jffs2/skypluto-lim.conf"   // kdev=, ceil_khz=, ceil_max_khz=, guard=0|1
#define LIM_POLL     10.0              // s between readouts (and clears) of the limiter statistics
#define LEVEL_FIXED  65535             // digital full scale = best DAC SNR
#define DBM_AT_0DB   5.0               // ESTIMATE: output power at 0 dB attenuation
// Maximum settable level. Measured externally (2026-09-30): the spectral shoulder in the skirts grows ~1 dB per dB
// with RF power (the AD9361 RF output becomes non-linear at high drive): +5 dBm does NOT meet the
// SM.1268 mask, -5 dBm does. Anything above P_MAX_DBM gives `ERR range`. The value follows from the measurements (see
// docs/measurements). Fixed at -5 dBm (user, 2026-09-30): +5.8 dB margin to the mask measured.
#define P_MAX_DBM    -5.0
#define ATTEN_FLOOR  (DBM_AT_0DB - P_MAX_DBM)   // 10 dB: the ALC trim may NEVER go below this either
#define ATTEN_MAX    89.75
#define ATTEN_MUTE   89.75             // E 0: maximum attenuation = off the air
#define ALC_MAX_TRIM 3.0               // dB, max correction relative to the setpoint
#define ALC_DEADBAND 0.3               // dB, smaller deviations leave the ALC alone
#define ALC_PERIOD   20.0              // s between measurements (relaxed, hold)
#define ALC_SETTLE   2.5               // s to wait after a change before (re)measuring

static volatile uint8_t *g_map;
static inline uint32_t rd(unsigned off){ return *(volatile uint32_t*)(g_map+off); }
static inline void      wr(unsigned off,uint32_t v){ *(volatile uint32_t*)(g_map+off)=v; }

static double mono(void){ struct timespec ts; clock_gettime(CLOCK_MONOTONIC,&ts); return ts.tv_sec + ts.tv_nsec*1e-9; }

// ---- state ----------------------------------------------------------------
static long long last_f   = -1;        // the carrier frequency of the last tune (Hz); the TX LO sits lowif_khz below it
// Low-IF: the TX LO is programmed lowif_khz BELOW the carrier and the modulator's NCO is offset by the same amount, so the carrier stays on the tuned frequency while the LO leakage (and the
// I/Q image) move away from it. 0 = zero-IF. dc/gain/skew: digital I/Q corrections for the analog LO leakage and I/Q imbalance (gain/skew need bitstream B1D00019+). Kept in IQ_CONF.
#define IQ_CONF "/mnt/jffs2/skypluto-iq.conf"
static double lowif_khz = 0.0;
static int    iq_dc_i = 0, iq_dc_q = 0, iq_gain_ppm = 0, iq_skew_ppm = 0, has_iq = 0;
static double    applied_a= -1.0;      // attenuation that is on the chip NOW (dB)
static double    want_dbm = 0.0;       // requested setpoint
static double    nom_a    = 0.0;       // nominal attenuation for that setpoint
static double    trim     = 0.0;       // ALC correction (dB, positive = more attenuation)
static int       tx_off   = 1;         // transmitter closed (RF off). After boot always 1 until the Pico sends a tune (F); afterwards = !(want_on && tuned)
static int       want_on  = 1;         // last E request from the Pico (default on)
static int       tuned    = 0;         // 1 once the Pico has sent a valid F (tune) since daemon start

// ALC
static int       ref_valid = 0, ref_g = 0;
static double    ref_lvl = 0.0, last_lvl = 0.0;
static const char *alc_state = "wait";
static double    m_next = 0.0;
static int       m_pid = -1, m_fd = -1, m_len = 0, m_big = 0;
static char      m_buf[4096];               // room for several MASK/SPEC/FLOOR blocks (continuous mode delivers one every second)
static double    m_started = 0.0;

// Mask monitor (skypluto-mask): periodic measurement of the skirt shoulder; report only (log),
// no direct intervention. Config: /mnt/jffs2/skypluto-mask.conf with lines  enable=1  rx=1|2  port=A_BALANCED
#define MASKBIN      "/mnt/jffs2/skypluto-mask"
#define MASKCONF     "/mnt/jffs2/skypluto-mask.conf"
#define MASK_PERIOD  1.0               // s pause between measurements
#define MASK_WINDOW  300.0             // s: the max-hold covers all measurements of the last 5 minutes (SM.1268), time-based
#define MASK_KEEP    1400              // ring size (300 s at 4 spectra per second = 1200, with margin)
#define MASK_LOG_EVERY 120             // only every 120th spectrum goes to the log (= every ~30 s; saves /tmp), or always when margin < +3 dB
#define MASK_STREAM_INT "0.25"         // continuous mask mode: one spectrum per this many seconds (max-hold over exactly that interval)
#define MASK_STREAM_DUR "25"           // and this many seconds per process, after which the ALC gets its turn (gain + noise floor are set anew per process)
#define GRID         57                // grid k = -28..+28 -> -168..+168 kHz in steps of 6 kHz
// Stored measurements (spectrum and floor in dB relative to the peak) + the combined max-hold over the window
static struct { double t, margin, sho, flo, dev; int conc; double sp[GRID], fl[GRID]; } mk[MASK_KEEP];
static int    mk_n = 0, mk_i = 0;
static unsigned long mk_seq = 0;               // count of accepted spectra since daemon start (q= in ?L, seq= in ?M)
static double cmb_sp[GRID], cmb_fl[GRID], cmb_margin = 0, cmb_sho = 0, cmb_flo = 0, cmb_dev = 0, mk_last_t = 0, mk_last_margin = 0;
static int    cmb_conc = 0, cmb_n = 0;         // cmb_n = number of measurements in the 5-minute window
// Twin: via the FPGA, TX2 gets the same modulator output as TX1 and follows the TX1 attenuation; the mask monitor
// measures TX2 on RX2, so TX1 stays completely free of a measurement coupling. Config: twin=1 in MASKCONF.
// TX2 is connected DIRECTLY to RX2 (no attenuator): hence never louder than -15 dBm (TX2_MIN_ATTEN), and muted on E 0.
#define TX2_MIN_ATTEN   20.0
#define TWIN_OFFSET_DB  1.4            // simultaneous measurement at -15 dBm: TX2 has on average 1.4 dB LESS shoulder than TX1
static int    twin_enable = 0, twin_sel_done = 0, mask_rx_cfg = 1;
static double applied_a2 = -1.0;       // attenuation that is on TX2 NOW
static int    m_kind = 0;              // 0 = ALC measurement, 1 = mask measurement (kind of the active child process)
static double mask_next = 0.0;

// Limiter/conditioner: configuration and status
static int    lim_present = 0;                 // 1 if the bitstream has the conditioner (register readback matches)
static int    lim_bypass = 0;                  // 1 if present but bypassed (then it counts as absent)
static int    has_dbg = 0;                     // 1 if the bitstream has the register block with 6-bit address (diagnostics at 0x3C)
// peak deviation at the modulator input (after limiter and interpolator), measured in the FPGA over windows of 0.25 s (diagnostic words 19/21)
static uint32_t dev_seq_u = 0xFFFFFFFFu;
static double dev_hold = 0.0, dev_hold_t = 0.0;                    // largest peak deviation (kHz) of the last ~minute (maintained by dev_poll)
static int    dev_ok = 0; static double dev_pk_khz = 0.0, dev_last_t = 0.0; static unsigned long dev_seq = 0;
static int    ceil_from_pico = 0;              // 1 once the Pico has sent an 'H <kHz>': then that value applies and the daemon ignores ceil_khz/adapt from the conf file
static int    kdev_from_pico = 0;             // 1 once the Pico has sent a 'K <n>': then that value applies and the daemon ignores kdev= from the conf file
static int    lim_user = 1;                    // chosen by the Pico with 'B 0/1': limiter on (1, default) or off (0); the soft fade is always on
static int    kdev_cfg = KDEV_FIXED;           // desired kdev (from LIMCONF)
static double ceil_khz = 67.0;                 // peak deviation ceiling that the FPGA limiter enforces NOW (kHz)
static double ceil_max_khz = 67.5;             // upper bound for ceil_khz (also for the adaptive control)
// Mask protection (guard): the limit chosen by the Pico/conf is the MAXIMUM (ceil_user). ceil_khz is the limit the FPGA enforces now and is lowered by the
// protection as soon as the 5-minute mask margin drops below GUARD_LOW (and slowly restored when there is ample margin). Without mask data (loop missing,
// transmitter closed, no twin) or with guard off (G 0), ceil_khz stays = ceil_user.
static double guard_low = 3.5;         // dB: below this the limit is lowered (requirement: >= +3 dB); conf: guard_low= (test: set higher)
#define GUARD_TARGET (guard_low + 1.0)  // dB: the reduction aims at this margin
#define GUARD_HIGH   (guard_low + 4.5)  // dB: only above this (for at least 2 minutes) is the limit raised again
#define GUARD_LOW    guard_low
#define GUARD_MIN    35.0              // kHz: never lower
static double ceil_user = 67.0;        // maximum chosen by the user/Pico (kHz)
static int    guard_on = 1, guard_from_pico = 0, guard_act = 0;    // guard_act = 1: ceil_khz is below ceil_user because of the protection
static double guard_t = 0.0;           // time of the last change by the protection
static int    guard_init = 1;          // 1 = initialising: the mask window holds too little data (after start, TX open, mask reset or guard on) for the guard to act
#define GUARD_INIT_N 240               // spectra (~60 s at 4 per second) needed before the guard takes its first decision
static double ceil_tgt = 67.0;         // desired limit: ceil_khz ramps gradually (0.5 kHz per 100 ms) towards it, so that a change does not cause a gain jump (splatter) in the transmission
static double lim_gr_db = 0.0;                 // max gain reduction in the last window (dB, positive)
static unsigned lim_evt = 0, lim_uf = 0;       // counters (since FPGA start)
static int    lim_in_pct = 0;                  // largest input peak in the last window, in % of full scale
static double lim_next = 0.0;

static double t_start = 0.0;           // daemon start time (for up= in ?S: restart detection)
static double h_max = 0.0;             // longest processing time of a command (s)

// temperature (sysfs, cached)
static int    temp_mC = 0;
static double temp_last = -100.0;
static char   temp_path[128] = "";

// ---- helpers --------------------------------------------------------------
static void tx_byte(uint8_t c){
    unsigned guard = 2000000;
    while ((rd(U_ST) & 0x2) && --guard) { /* spin */ }
    wr(U_TX, c);
}
// Replies go to the source of the command: the UART (Pico) or, during a command from the TCP console, that connection (out_fd >= 0).
static int out_fd = -1;
static char cap_buf[2048]; static int cap_len = 0;        // out_fd == -2: collect replies (web interface)
static void tx_str(const char *s){
    if (out_fd == -2){ int n = (int)strlen(s); if (cap_len + n < (int)sizeof cap_buf){ memcpy(cap_buf + cap_len, s, (size_t)n); cap_len += n; cap_buf[cap_len] = 0; } return; }
    if (out_fd >= 0){ size_t n = strlen(s); while (n){ ssize_t w = send(out_fd, s, n, MSG_NOSIGNAL); if (w <= 0) break; s += w; n -= (size_t)w; } return; }
    while (*s) tx_byte((uint8_t)*s++);
}
// ---- boot status (Pluto -> Pico, unsolicited during startup) -------------------------------------------------------------------
// Lines '#B <pct 0-100> <text>' for the status bar of the Pico splash screen. The daemon repeats the last line every second until the Pico itself
// sends a byte (the Pico is master); anyone already using the line should not need these lines. '?B' returns the current step.
// ---- loop cable TX2 -> RX2: the mask monitor measures its own transmission via the twin. If the cable is missing, RX2 sees (almost) nothing: the mask tool reports
// 'geen_signaal' or the RX gain runs into the maximum. Two consecutive failed measurements or a gain >= 55 dB = 'missing' (?R, web interface).
static int lp_fail = 0, lp_g = -1, lp_jrun = 0; static double lp_t = -100.0;
static void lp_ok(int g){ lp_fail = 0; lp_g = g; lp_t = mono(); }
static void lp_err(const char *line){ if (strstr(line, "geen_signaal")){ lp_fail++; lp_t = mono(); } }
// ---- power meter: RX1 as measuring bridge (measurement only, no control) -------------------------------------------------------------------------------------------
// The transmitter output goes through an EXTERNAL attenuator (value entered by the user, Q <dB>) to RX1. RX1 is on the transmit frequency (RX-LO = TX-LO - 504 kHz);
// from the average AC power of the capture (counts^2) and the RX1 gain G follows: P_in(dBm) = 10 log10(p) - G + PW_K0 + cal, P_out = P_in + attenuation.
// PW_K0 is derived from the own loop (TX2 -15 dBm -> RX2, G = 11 dB, p = 3e5 counts^2); QK <dBm> (calibrate on a known power) sets 'cal'.
// The RX1 gain is chosen automatically from a ladder of 10 dB steps (27, 37, 47, 57, 67 dB) so that p stays between 1e4 and 1.6e6 counts^2: below that the receiver noise biases
// the reading, above it the ADC starts to compress (clipping from about 6e6). The AD9361 gain steps are not exactly 1 dB per dB: PW_GC lists the measured deviation of the
// reading per ladder gain (dB, interpolated in between), which is subtracted. NOTE: the AD9361 input may receive at most about +2.5 dBm (damage); P_in > -10 dBm gives 'high'.
#define PWRCONF "/mnt/jffs2/skypluto-pwr.conf"        // enable=, atten_db=, cal_db=
#define PW_K0   (-58.8)
static int    pw_en = 0, pw_gain = 37, pw_skip = 0, pw_ring_n = 0, pw_inited = 0;
static double pw_att = 0.0, pw_cal = 0.0, pw_p = 0.0, pw_pk = 0.0, pw_t = -100.0, pw_ring[5], pw_spread = 0.0;
static const int    pw_lad[5]  = { 27, 37, 47, 57, 67 };                // RX1 gain ladder (dB)
static const double pw_ladc[5] = { 0.0, 1.30, 1.17, -0.70, -1.40 };     // measured excess of the uncorrected reading at each ladder gain (dB, vs. the reference at 27..30 dB)
static double pw_gcorr(int g){                                          // correction (dB) to ADD to the reading: -excess, linear between the ladder gains, flat outside
    if (g <= pw_lad[0]) return 0.0;
    if (g >= pw_lad[4]) return -pw_ladc[4];
    if (g <= 30) return 0.0;                                            // the gain table is flat between 22 and 30 dB
    int k = 0; while (k < 3 && g > pw_lad[k + 1]) k++;
    double f = (double)(g - pw_lad[k]) / (double)(pw_lad[k + 1] - pw_lad[k]);
    return -(pw_ladc[k] + f * (pw_ladc[k + 1] - pw_ladc[k]));
}
static int    pw_man = -1;                // >= 0: RX1 gain held manually (QG), no automatic range switching; -1 = automatic
static pid_t  pw_pid = -1; static int pw_fd = -1; static double pw_next = 0.0, pw_started = 0.0; static char pw_buf[160]; static int pw_blen = 0;
static void pw_conf_read(void){
    FILE *f = fopen(PWRCONF, "r"); if (!f) return;
    char ln[96];
    while (fgets(ln, sizeof ln, f)){
        if (!strncmp(ln, "enable=", 7)) pw_en = atoi(ln + 7) ? 1 : 0;
        else if (!strncmp(ln, "atten_db=", 9)){ double v = atof(ln + 9); if (isfinite(v) && v >= 0.0 && v <= 120.0) pw_att = v; }
        else if (!strncmp(ln, "cal_db=", 7)){ double v = atof(ln + 7); if (isfinite(v) && fabs(v) <= 40.0) pw_cal = v; }
    }
    fclose(f);
}
static void pw_conf_write(void){
    FILE *f = fopen(PWRCONF, "w"); if (!f) return;
    fprintf(f, "enable=%d\natten_db=%.2f\ncal_db=%.2f\n", pw_en, pw_att, pw_cal);
    fclose(f);
}
static int phy_write(const char *attr, const char *val);                  // direct sysfs write (defined further down)
static void pw_set_gain(int g){
    if (g < 0) g = 0;
    if (g > 70) g = 70;
    char v[16]; snprintf(v, sizeof v, "%d", g);
    char c[160]; snprintf(c, sizeof c, "iio_attr -q -i -c ad9361-phy voltage0 hardwaregain %d >/dev/null 2>&1", g);
    if (phy_write("in_voltage0_hardwaregain", v) == 0 || system(c) == 0){ pw_gain = g; pw_ring_n = 0; pw_skip = 2; }     // sysfs (~1 ms) first: a system() call blocks the main loop for ~100 ms
}
static void pw_rx1_init(void){                          // RX1: port A, manual gain
    if ((phy_write("in_voltage0_rf_port_select", "A_BALANCED") == 0 && phy_write("in_voltage0_gain_control_mode", "manual") == 0) ||
        system("iio_attr -q -i -c ad9361-phy voltage0 rf_port_select A_BALANCED >/dev/null 2>&1; iio_attr -q -i -c ad9361-phy voltage0 gain_control_mode manual >/dev/null 2>&1") == 0) pw_inited = 1;
    pw_set_gain(pw_gain);
}
// process an incoming measurement (counts^2): average over ~1 s and adjust the gain
static void pw_feed(double p, double pk){
    pw_t = mono(); pw_pk = pk;
    if (pw_skip > 0){ pw_skip--; return; }
    pw_ring[pw_ring_n % 5] = p; if (pw_ring_n < 1000) pw_ring_n++;
    int n = pw_ring_n < 5 ? pw_ring_n : 5;
    double srt[5]; for (int i = 0; i < n; i++) srt[i] = pw_ring[i];
    for (int i = 1; i < n; i++){ double v = srt[i]; int j = i - 1; while (j >= 0 && srt[j] > v){ srt[j + 1] = srt[j]; j--; } srt[j + 1] = v; }
    pw_p = srt[n / 2];                                                 // median: single outliers (bursts in the capture) do not disturb the reading
    pw_spread = (n >= 3 && srt[0] > 0.0) ? 10.0 * log10(srt[n - 2] / srt[1]) : 99.0;   // spread (dB) of the middle values: stable reading = small
    if (pw_man >= 0) return;                                           // manual gain: no range switching
    int gi = 0; for (int k = 0; k < 5; k++) if (pw_lad[k] <= pw_gain) gi = k;
    if (p > 1.6e6 && gi > 0) pw_set_gain(pw_lad[gi - 1]);
    else if (p > 0 && p < 1.0e4 && gi < 4){                             // too weak: the ladder gain that brings p closest to ~3e5 (54.8 dB)
        int best = gi + 1; double bd = 1.0e9;
        for (int k = gi + 1; k < 5; k++){ double d = fabs(10.0 * log10(p) + (double)(pw_lad[k] - pw_gain) - 54.8); if (d < bd){ bd = d; best = k; } }
        pw_set_gain(pw_lad[best]);
    }
    else if (p <= 0 && gi < 4) pw_set_gain(pw_lad[gi + 1]);
}
static double pw_in_dbm(void){ return pw_p > 0 ? 10.0 * log10(pw_p) - pw_gain + PW_K0 + pw_cal + pw_gcorr(pw_gain) : -200.0; }
static double pico_last_rx = -1.0; static unsigned long pico_cmds = 0;   // last byte from the Pico (for the web interface) and number of commands
static int    bs_pct = 0;
static char   bs_txt[32] = "";
static int    bs_active = 1;                   // 1 = no byte seen from the Pico yet: transmit unsolicited
static double bs_next = 0.0, bs_t0 = 0.0;
static int    bs_phase = 0;
static void bs_send(void){
    char b[64]; snprintf(b, sizeof b, "#B %d %s\n", bs_pct, bs_txt);
    tx_str(b);
}
static void bs_set(int pct, const char *txt){ bs_pct = pct; snprintf(bs_txt, sizeof bs_txt, "%s", txt); }
static int rx_byte(void){ uint32_t r = rd(U_RX); return (r & 0x100) ? (int)(r & 0xFF) : -1; }

// first line of a command's output (without trailing whitespace); 0 = success
static int read_cmd_line(const char *cmd, char *out, size_t n){
    FILE *p = popen(cmd, "r");
    if (!p) return -1;
    out[0] = 0;
    if (fgets(out, (int)n, p) == NULL) { pclose(p); return -1; }
    size_t l = strlen(out);
    while (l && (out[l-1]=='\n'||out[l-1]=='\r'||out[l-1]==' '||out[l-1]=='\t')) out[--l]=0;
    return pclose(p)==0 ? 0 : 1;
}
static double quant(double db){                // 0.25 dB grid, clamped
    if (db < 0.0) db = 0.0;
    if (db > ATTEN_MAX) db = ATTEN_MAX;
    return floor(db*4.0 + 0.5)/4.0;
}
static void find_temp_path(void){
    for (int i = 0; i < 8; i++){
        char p[128];
        snprintf(p, sizeof p, "/sys/bus/iio/devices/iio:device%d/in_temp0_input", i);
        if (access(p, R_OK) == 0){ snprintf(temp_path, sizeof temp_path, "%s", p); return; }
    }
}
static void refresh_temp(void){
    if (!temp_path[0]) return;
    FILE *f = fopen(temp_path, "r"); if (!f) return;
    int v; if (fscanf(f, "%d", &v) == 1) temp_mC = v;
    fclose(f);
}

// Direct sysfs access to the ad9361-phy IIO device. An iio_attr process takes ~100 ms, a sysfs write ~1 ms: the mute that starts a tune and the retune + calibration use it,
// so the carrier that still leaks through the maximum attenuation is on the air for as short a time as possible.
static char phy_dir[80] = "";
static const char *phy_find(void){
    if (phy_dir[0]) return phy_dir;
    for (int i = 0; i < 8; i++){
        char pth[96], nm[32] = ""; snprintf(pth, sizeof pth, "/sys/bus/iio/devices/iio:device%d/name", i);
        FILE *f = fopen(pth, "r"); if (!f) continue;
        if (fgets(nm, sizeof nm, f)) nm[strcspn(nm, "\r\n")] = 0;
        fclose(f);
        if (!strcmp(nm, "ad9361-phy")){ snprintf(phy_dir, sizeof phy_dir, "/sys/bus/iio/devices/iio:device%d", i); break; }
    }
    return phy_dir;
}
static int phy_write(const char *attr, const char *val){
    if (!phy_find()[0]) return -1;
    char pth[140]; snprintf(pth, sizeof pth, "%s/%s", phy_dir, attr);
    FILE *f = fopen(pth, "w"); if (!f) return -1;
    int rc = (fputs(val, f) < 0) ? -1 : 0;
    if (fclose(f) != 0) rc = -1;
    return rc;
}
// ---- chip actions (NB: 'iio_attr -q' suppresses output; fine for writes) -------
static int hw_set_atten(double db){
    char v[16]; snprintf(v, sizeof v, "-%.2f", db);
    if (phy_write("out_voltage0_hardwaregain", v) == 0) return 1;     // sysfs: ~1 ms, checked; the iio_attr process below is the fallback
    char c[160];
    snprintf(c,sizeof c,"iio_attr -q -o -c ad9361-phy voltage0 hardwaregain -- -%.2f >/dev/null 2>&1", db);
    return system(c)==0;
}
// ---- limiter / conditioner ----------------------------------------------------------------
static void lim_conf_read(void){
    FILE *f = fopen(LIMCONF, "r");
    if (!f) return;
    char ln[96];
    while (fgets(ln, sizeof ln, f)){
        if      (!strncmp(ln, "kdev=", 5)){ if (!kdev_from_pico) kdev_cfg = atoi(ln + 5); }     // conf = fallback until the Pico sends its K
        else if (!strncmp(ln, "ceil_khz=", 9)){ if (!ceil_from_pico) ceil_user = atof(ln + 9); }   // conf = fallback until the Pico sends its H
        else if (!strncmp(ln, "ceil_max_khz=", 13)) ceil_max_khz = atof(ln + 13);
        else if (!strncmp(ln, "guard_low=", 10)){ double v = atof(ln + 10); if (v >= 3.0 && v <= 12.0) guard_low = v; }
        else if (!strncmp(ln, "guard=", 6)){ if (!guard_from_pico) guard_on = atoi(ln + 6) ? 1 : 0; }
    }
    fclose(f);
    if (!isfinite(ceil_user)) ceil_user = 67.0;
    if (!isfinite(ceil_max_khz)) ceil_max_khz = 70.0;
    if (ceil_max_khz > 70.0) ceil_max_khz = 70.0;             // hard upper bound for conf values (the Pico H may go up to 100)
    if (!ceil_from_pico){
        if (ceil_user > ceil_max_khz) ceil_user = ceil_max_khz;
        if (ceil_user < 30.0) ceil_user = 30.0;
        if (!guard_act || ceil_khz > ceil_user){ ceil_khz = ceil_user; ceil_tgt = ceil_user; }
    }
    if (kdev_cfg < 10) kdev_cfg = 10;
    if (kdev_cfg > 250) kdev_cfg = 250;
}
// Does the conditioner exist in the bitstream? NEVER write to detect it: a bitstream without conditioner has a 5-bit
// register address, so 0x20 folds back to 0x00 (CTRL) and 0x24 to 0x04 (OFFSET); a test write could
// switch the modulation off. READ only: in such a bitstream 0x20/0x24 return the same value as 0x00/0x04 (alias); with
// the conditioner they are separate registers (LIM_CEIL has its own default 0x733333, LIM_CTRL default 3).
static void lim_probe(void){
    uint32_t a = rd(R_LCEIL), b = rd(R_CTRL), c = rd(R_LCTRL), d = rd(R_OFFSET);
    lim_present = (a != b) || (c != d);
    has_dbg = lim_present;                                      // register block with 6-bit address (also if the conditioner is bypassed): 0x3C is then safe to use
    // Bitstream with bypassed conditioner (USE_SC=0): SC_STATE[1:0] reads 3 -> NO limiter active (but saturation in the interpolator).
    if (lim_present && (rd(R_LSTAT) & 3u) == 3u){ lim_present = 0; lim_bypass = 1; }
}
static uint32_t dbg_rd(int sel){ wr(R_DBG, (uint32_t)sel); return rd(R_DBG); }
// Read the peak deviation (kHz) of the last closed 0.25 s window at the modulator input (word 19; word 21 = window sequence number).
#define GR_N 64                                   // swing meter ring: last 64 windows of 20 ms (1.28 s)
static double gr[GR_N]; static int gr_cnt = 0, gr_head = 0;   // gr_head = next write position; gr_cnt = number of valid values
static int    gr_mode = 0;                        // 1 = bitstream with 20 ms windows + peak history (id >= B1D0000E)
static void gr_push(double v){ gr[gr_head] = v; gr_head = (gr_head + 1) % GR_N; if (gr_cnt < GR_N) gr_cnt++; }
// Swing meter: every closed 20 ms window (peak |comp| at the modulator input, after limiter + interpolator) as kHz.
// Bitstreams from B1D0000E on deliver the last 8 window maxima (words 24..31, 24 = newest) + sequence number (word 21); the daemon
// reads this at least every 100 ms (8 x 20 ms = 160 ms history) and builds the ring with absolute window position (missed = ring
// cleared). Bitstreams B1D00009..0D deliver one window of 0.25 s (words 19/21) instead, gr_mode = 0.
// Only in bitstreams with build id >= B1D00009 (word 12); otherwise dev_ok = 0 (?D then answers d=na).
static void dev_poll(void){
    if (!has_dbg){ dev_ok = 0; return; }
    { uint32_t id = dbg_rd(12); if (id < 0xB1D00009u || id > 0xB1D000FFu){ dev_ok = 0; return; } gr_mode = (id >= 0xB1D0000Eu); }
    int k = (int)(rd(R_KDEV) & 0x3FFFF);
    double f = 0.75 * (double)k / 8388608.0;
    uint32_t s1 = 0, s2 = 1, a = 0, w[8] = {0};
    if (!gr_mode){
        for (int t = 0; t < 3 && s1 != s2; t++){ s1 = dbg_rd(21); a = dbg_rd(19) & 0xFFFFFFu; s2 = dbg_rd(21); }   // avoid counter tearing
        dev_pk_khz = (double)a * f;
    } else {
        for (int t = 0; t < 4 && s1 != s2; t++){
            s1 = dbg_rd(21);
            for (int i = 0; i < 8; i++) w[i] = dbg_rd(24 + i) & 0xFFFFFFu;
            s2 = dbg_rd(21);
        }
        uint32_t n = s1 - dev_seq_u;                               // windows since the previous poll
        if (dev_seq_u == 0xFFFFFFFFu || n > 8){ gr_cnt = 0; gr_head = 0; n = (n > 8) ? 8 : n; }   // first poll or missed: restart the ring
        if (s1 != s2) n = 0;                                       // tearing: skip this poll
        else for (int i = (int)n - 1; i >= 0; i--) gr_push((double)w[i] * f);   // oldest first
        if (s1 == s2) dev_seq_u = s1;
        // ?D = largest of the last 12 windows (~0.25 s)
        double m = 0.0; int nn = gr_cnt < 12 ? gr_cnt : 12;
        for (int i = 1; i <= nn; i++){ double v = gr[(gr_head + GR_N - i) % GR_N]; if (v > m) m = v; }
        dev_pk_khz = m;
        if (m >= dev_hold || mono() - dev_hold_t > 60.0){ dev_hold = m; dev_hold_t = mono(); }
        s1 = dev_seq_u;
    }
    if (s1 != dev_seq){ dev_seq = s1; dev_last_t = mono(); }
    dev_ok = 1;
}
// kdev + limiter ceiling to the FPGA. Without conditioner never kdev > KDEV_FIXED (an excessive drive could then pass unchecked).
static void lim_apply(void){
    int k = kdev_cfg;
    if (!lim_present && k > KDEV_FIXED) k = KDEV_FIXED;
    if ((uint32_t)k != (rd(R_KDEV) & 0x3FFFF)) fprintf(stderr, "   -> kdev %d (conf %d, limiter %s)\n", k, kdev_cfg, lim_present ? "aanwezig" : "ONTBREEKT");
    wr(R_KDEV, (uint32_t)k);
    if (lim_present){
        // deviation = comp/2^23 x kdev x 750 Hz  =>  comp_ceil = dev_ceil / (kdev x 750) x 2^23, with 1 % margin (interpolation overshoot)
        double c = ceil_khz * 1000.0 / ((double)k * 750.0) * 8388608.0 * 0.99;
        if (c > 8388607.0) c = 8388607.0;
        if (c < 0.0) c = 0.0;
        wr(R_LCEIL, (uint32_t)c);
        wr(R_LCTRL, (lim_user ? 0x1u : 0x0u) | 0x2u);           // limiter (bit 0, by 'B') + soft fade (bit 1, always) on
    }
}
// Read out statistics (every LIM_POLL s) and clear; read counters twice against 'tearing' from the clock-domain crossing
static void lim_poll(void){
    if (!lim_present) return;
    uint32_t gmin, evt, pk, st, uf, a;
    do { a = rd(R_LEVT); evt = rd(R_LEVT); } while (a != evt);
    do { a = rd(R_LUF);  uf  = rd(R_LUF);  } while (a != uf);
    gmin = rd(R_LGMIN) & 0x1FFFF; pk = rd(R_LPEAK) & 0xFFFFFF; st = rd(R_LSTAT);
    wr(R_LCTRL, (lim_user ? 0x1u : 0x0u) | 0x2u | 0x4u);        // clear statistics (window starts anew); keeps the B choice
    lim_gr_db = (gmin > 0 && gmin < 65536) ? -20.0 * log10((double)gmin / 65536.0) : 0.0;
    lim_in_pct = (int)(100.0 * pk / 8388608.0 + 0.5);
    if (uf != lim_uf) fprintf(stderr, "   !! I2S is %u keer weggevallen (fade actief, toestand %u)\n", uf - lim_uf, st & 3);
    if (gmin < 65536) fprintf(stderr, "   .. begrenzer greep in: max %.1f dB gain-reductie, ingangspiek %d %%, %u samples\n", lim_gr_db, lim_in_pct, evt - lim_evt);
    lim_evt = evt; lim_uf = uf;
}

// Read config (MASKCONF): enable=, rx=, port=, twin=. Also sets the globals twin_enable/mask_rx_cfg.
static void read_conf(int *en, int *rx, char *port, size_t pn){
    *en = 0; *rx = 1; snprintf(port, pn, "A_BALANCED"); twin_enable = 0;
    FILE *f = fopen(MASKCONF, "r");
    if (!f) return;
    char ln[96];
    while (fgets(ln, sizeof ln, f)){
        if (!strncmp(ln, "enable=", 7)) *en = atoi(ln + 7);
        else if (!strncmp(ln, "rx=", 3)) *rx = atoi(ln + 3);
        else if (!strncmp(ln, "twin=", 5)) twin_enable = atoi(ln + 5);
        else if (!strncmp(ln, "port=", 5)){ snprintf(port, pn, "%s", ln + 5); char *e = strpbrk(port, "\r\n "); if (e) *e = 0; }
    }
    fclose(f);
    mask_rx_cfg = (*rx == 2) ? 2 : 1;
}
// cal_hold: the output stays muted (maximum attenuation, TX1 and TX2) while the transmitter retunes and the AD9361 calibrates for the new LO frequency, and is released afterwards.
static int    cal_hold = 0;
static double cal_hold_t = 0.0;
// TX2 (twin) follows TX1: same attenuation, never lower than TX2_MIN_ATTEN, and fully closed on E 0.
static void twin_apply(void){
    if (!twin_enable) return;
    static double twin_next_try = 0.0;
    if (!twin_sel_done && mono() >= twin_next_try){   // have channel 2 (TX2) select 'external data' = the modulator (DATA_SEL=2)
        if (system("iio_reg cf-ad9361-dds-core-lpc 0x0498 0x2 >/dev/null 2>&1; iio_reg cf-ad9361-dds-core-lpc 0x04D8 0x2 >/dev/null 2>&1") == 0)
            twin_sel_done = 1;
        else twin_next_try = mono() + 30.0;                                  // a system() call blocks the main loop: do not retry every second
    }
    // With the transmitter on, TX2 is always at -15 dBm (attenuation TX2_MIN_ATTEN), even if TX1 is lower: the mask shoulder grows with
    // power, so TX2 at -15 dBm is conservative for lower TX1 levels, and the measurement keeps a good signal-to-noise ratio. Mute on E 0.
    double a2 = (tx_off || cal_hold) ? ATTEN_MUTE : TX2_MIN_ATTEN;
    a2 = quant(a2);
    if (fabs(a2 - applied_a2) < 0.01) return;
    char v2[16]; snprintf(v2, sizeof v2, "-%.2f", a2);
    if (phy_write("out_voltage1_hardwaregain", v2) == 0){ applied_a2 = a2; return; }
    char c[170];
    snprintf(c, sizeof c, "iio_attr -q -o -c ad9361-phy voltage1 hardwaregain -- -%.2f >/dev/null 2>&1", a2);
    if (system(c) == 0) applied_a2 = a2;
}
static void apply_atten(double total){          // idempotent + smooth
    if (total < ATTEN_FLOOR) total = ATTEN_FLOOR;   // never more power than P_MAX_DBM (not even through ALC trim)
    if (cal_hold) total = ATTEN_MUTE;               // a retune/calibration holds the output muted: no level change may release it
    total = quant(total);
    if (fabs(total - applied_a) < 0.01){ twin_apply(); return; }
    if (hw_set_atten(total)) applied_a = total;
    twin_apply();
}
static void apply_current(void){                // what should be on the chip now
    apply_atten((tx_off || cal_hold) ? ATTEN_MUTE : nom_a + trim);
}
static long long lowif_hz(void){ return (long long)llround(lowif_khz * 1000.0); }
static long long lo_freq(void){ return last_f - lowif_hz(); }                   // what the TX LO has to be programmed to
static void nco_offset_apply(void){                                              // the NCO runs at 12.288 MHz with a 24-bit phase: inc = Hz * 2^24 / 12288000
    long long inc = (long long)llround(lowif_khz * 1000.0 * 16777216.0 / 12288000.0);
    wr(R_OFFSET, (uint32_t)(inc & 0xFFFFFF));
}
static uint32_t iq_level(void){                                                   // headroom for the corrections (a clipped constant-envelope carrier would make harmonics)
    double mag = (double)(abs(iq_dc_i) > abs(iq_dc_q) ? abs(iq_dc_i) : abs(iq_dc_q)) / 32767.0 + fabs(iq_gain_ppm) * 1e-6 + fabs(iq_skew_ppm) * 1e-6;
    if (mag == 0.0) return 65535;
    double lv = 65535.0 * (1.0 - 1.05 * mag - 0.002); if (lv > 65535.0) lv = 65535.0; if (lv < 50000.0) lv = 50000.0;
    return (uint32_t)lv;
}
static void iq_apply(void){
    wr(R_DCI, (uint32_t)(iq_dc_i & 0xFFF)); wr(R_DCQ, (uint32_t)(iq_dc_q & 0xFFF));
    if (has_iq){
        long long g = (long long)llround(iq_gain_ppm * 262144.0 / 1e6), q = (long long)llround(iq_skew_ppm * 262144.0 / 1e6);
        wr(R_QGAIN, (uint32_t)(g & 0x3FFFF)); wr(R_QSKEW, (uint32_t)(q & 0x3FFFF));
    }
    wr(R_LEVEL, iq_level());
}
static void iq_conf_write(void){
    FILE *f = fopen(IQ_CONF ".new", "w"); if (!f) return;
    fprintf(f, "# digital I/Q corrections and low-IF offset (written by the DC / IQ / OFS commands)\n");
    fprintf(f, "dc_i=%d\ndc_q=%d\nqgain_ppm=%d\nqskew_ppm=%d\nlowif_khz=%.3f\n", iq_dc_i, iq_dc_q, iq_gain_ppm, iq_skew_ppm, lowif_khz);
    fclose(f); rename(IQ_CONF ".new", IQ_CONF);
}
static void iq_conf_read(void){
    FILE *f = fopen(IQ_CONF, "r"); if (!f) return;
    char ln[96];
    while (fgets(ln, sizeof ln, f)){
        int v; double d;
        if (!strncmp(ln, "dc_i=", 5)){ v = atoi(ln + 5); if (v >= -2047 && v <= 2047) iq_dc_i = v; }
        else if (!strncmp(ln, "dc_q=", 5)){ v = atoi(ln + 5); if (v >= -2047 && v <= 2047) iq_dc_q = v; }
        else if (!strncmp(ln, "qgain_ppm=", 10)){ v = atoi(ln + 10); if (abs(v) <= 400000) iq_gain_ppm = v; }
        else if (!strncmp(ln, "qskew_ppm=", 10)){ v = atoi(ln + 10); if (abs(v) <= 400000) iq_skew_ppm = v; }
        else if (!strncmp(ln, "lowif_khz=", 10)){ d = atof(ln + 10); if (isfinite(d) && fabs(d) <= 250.0) lowif_khz = d; }
    }
    fclose(f);
}
// TX-LO (altvoltage1) of the AD9361 off/on: E 0 = really switch the synthesizer off (no carrier leakage; maximum attenuation alone
// still let a weak carrier through). TX1 and TX2 share the same TX-LO.  On: switch the LO on and reprogram the frequency
// (lets the PLL lock); the attenuation is then still at maximum and is only restored afterwards.
static int lo_pd = 0;
static void tx_lo_power(int on){
    if (on){
        if (phy_write("out_altvoltage1_TX_LO_powerdown", "0") == 0 || system("iio_attr -q -o -c ad9361-phy altvoltage1 powerdown 0 >/dev/null 2>&1") == 0) lo_pd = 0;
        if (last_f > 0){
            char c[160], fv[24]; snprintf(fv, sizeof fv, "%lld", lo_freq());
            snprintf(c, sizeof c, "iio_attr -q -o -c ad9361-phy altvoltage1 frequency %lld >/dev/null 2>&1", lo_freq());
            if (phy_write("out_altvoltage1_TX_LO_frequency", fv) != 0 && system(c) != 0){ fprintf(stderr, "   !! TX-LO hertune na E 1 MISLUKT (%lld)\n", lo_freq()); fflush(stderr); }
        }
    } else {
        if (lo_pd) return;
        if (phy_write("out_altvoltage1_TX_LO_powerdown", "1") == 0 || system("iio_attr -q -o -c ad9361-phy altvoltage1 powerdown 1 >/dev/null 2>&1") == 0) lo_pd = 1;
    }
}
static double lvl_change_t = 0.0;               // time of the last level/frequency/on-off change (the mask monitor waits a moment afterwards)
static void alc_reset(double delay){ ref_valid = 0; alc_state = "wait"; m_big = 0; m_next = mono() + delay; lvl_change_t = mono(); }

// Transmitter open (0) / closed (1): order ON = LO first, then attenuation; OFF = attenuation to maximum first, then LO off.
// ---- impulse measurement (end-to-end group delay): 'J <secs> [pct]' starts skypluto-mask --ir; output /tmp/pluto_ir.csv ------------------------------------
static pid_t  ir_pid = -1; static double ir_t0 = 0.0, ir_secs = 0.0; static int ir_rc = -1; static int ir_kind = 0;   // ir_kind: 1 = impulse measurement (J), 2 = LO/fine spectrum (Y)
static void ir_finish(void){
    if (ir_kind == 1) wr(R_IMPC, 0);                          // RX1 back to pass-through
    unlink("/tmp/skypluto.hold");
}
// TX calibration of the AD9361 (LO leakage = rf_dc_offs, image = tx_quad). The chip's calibration is only valid for the LO frequency it ran at, so it is repeated in a child process
// after every tune and every time the transmitter opens and when the die temperature has drifted by 3 degC (the output is muted during all of them).
static double cal_failed_t = 0.0;                          // when the calibration gave up: it is tried again after 30 s
static int    cal_fail = 0, cal_failed = 0, cal_try = 0;      // cal_fail: consecutive failed attempts; cal_failed: gave up (the output stays muted until the next F/E); cal_try: fork failures
static long long cal_expect_f = 0;                            // LO frequency the running child has to reach (0 = no retune)
static int    cal_temp_ref = 0, cal_drift = 0;                // die temperature (m degC) at the last calibration; consecutive 15 s checks with a drift
static double cal_last_end = 0.0, cal_temp_chk = 0.0;
static int    cal_pending = 0, cal_retune = 0;       // cal_retune: the calibration child also retunes the TX-LO to last_f (a tune while the transmitter is open)
static pid_t  cal_pid = -1;
static double cal_due = 0.0, cal_t0 = 0.0;
static void measure_finish(void);
static void tx_set_state(int off){
    if (off == tx_off) return;
    tx_off = off;
    if (!tx_off){
        cal_hold = 1; cal_hold_t = mono(); guard_init = 1; cal_pending = 1; cal_due = mono(); cal_failed = 0; cal_fail = 0;
        if (phy_find()[0]) cal_retune = 2;           // the child powers the LO up, tunes it and calibrates in one go
        else { tx_lo_power(1); cal_retune = 0; }
    }
    else {                                            // closing: stop a running calibration and measurement, nothing is held any more
        if (m_pid > 0) measure_finish();
        lp_fail = 0;
        if (cal_pid > 0){ int stc; kill(cal_pid, SIGKILL); waitpid(cal_pid, &stc, 0); cal_pid = -1; }
        cal_pending = 0; cal_retune = 0; cal_hold = 0; cal_failed = 0; cal_fail = 0;
    }
    apply_current();
    if (tx_off) tx_lo_power(0);
    // Remember whether the transmitter is open (in /tmp = RAM: gone after a power loss, so a cold start stays CLOSED until the tune); a restart of the daemon alone
    // (update, crash) then takes over the state and leaves the RF on.
    if (!tx_off && last_f > 0){ FILE *tf = fopen("/tmp/skypluto-tuned", "w"); if (tf){ fprintf(tf, "%lld\n", last_f); fclose(tf); } }
    else unlink("/tmp/skypluto-tuned");
    if (!tx_off) alc_reset(ALC_SETTLE); else alc_state = "off";
    fprintf(stderr, "   -> zender %s (atten TX1 %.2f dB, TX2 %.2f dB, TX-LO %s)\n", tx_off ? "DICHT" : "OPEN", applied_a, applied_a2, lo_pd ? "UIT" : "aan"); fflush(stderr);
}

// ---- ALC: measurement in a child process -----------------------------------------------
static void measure_start(void){
    if (m_pid > 0 || last_f <= 0) return;
    // An external RX measurement (e.g. the mask capture) is using the RX path: do not disturb it, and
    // re-reference afterwards (that measurement changes RX gain/LO, so a reference taken before it is unusable).
    { struct stat hs;
      if (stat("/tmp/skypluto.hold", &hs) == 0 && time(NULL) - hs.st_mtime < 600){ alc_reset(ALC_SETTLE); return; } }
    int pf[2]; if (pipe(pf) != 0) return;
    char a1[32], a2[16];
    snprintf(a1,sizeof a1,"%lld", last_f);
    snprintf(a2,sizeof a2,"%d", ref_valid ? ref_g : 0);
    pid_t p = fork();
    if (p < 0){ close(pf[0]); close(pf[1]); return; }
    if (p == 0){
        dup2(pf[1], 1); close(pf[0]); close(pf[1]);
        execl(SELF, SELF, "--measure", a1, a2, (char*)NULL);
        _exit(127);
    }
    close(pf[1]);
    fcntl(pf[0], F_SETFL, O_NONBLOCK);
    m_pid = p; m_fd = pf[0]; m_len = 0; m_started = mono(); m_kind = 0;
}
// start a mask measurement if the config enables it (enable=1); return 1 if a child was started
static int mask_start(void){
    if (m_pid > 0 || last_f <= 0) return 0;
    int en, rx; char port[40];
    read_conf(&en, &rx, port, sizeof port);
    twin_apply();                                  // (config may have changed) keep TX2 in step
    if (!en || access(MASKBIN, X_OK) != 0) return 0;
    { struct stat hs;
      if (stat("/tmp/skypluto.hold", &hs) == 0 && time(NULL) - hs.st_mtime < 600) return 0; }
    int pf[2]; if (pipe(pf) != 0) return 0;
    char a1[32], a2[8];
    snprintf(a1, sizeof a1, "%lld", last_f); snprintf(a2, sizeof a2, "%d", rx == 2 ? 2 : 1);
    pid_t p = fork();
    if (p < 0){ close(pf[0]); close(pf[1]); return 0; }
    if (p == 0){
        dup2(pf[1], 1); close(pf[0]); close(pf[1]);
        execl(MASKBIN, MASKBIN, a1, a2, port, "0", "0.5", MASK_STREAM_INT, MASK_STREAM_DUR, (pw_en && rx == 2) ? "1" : "0", (char*)NULL);   // 0.5 s for gain/noise floor, then continuous
        _exit(127);
    }
    close(pf[1]);
    fcntl(pf[0], F_SETFL, O_NONBLOCK);
    m_pid = p; m_fd = pf[0]; m_len = 0; m_started = mono(); m_kind = 1;
    return 1;
}
// SM.1268-5 mask (0 dB @74, -15 @107.5, -30 @124, -40 @152.5 kHz), d in kHz
static double mask_lvl(double d){
    d = fabs(d);
    if (d <= 74.0)  return 0.0;
    if (d <= 107.5) return -15.0 * (d - 74.0) / 33.5;
    if (d <= 124.0) return -15.0 - 15.0 * (d - 107.5) / 16.5;
    if (d <= 152.5) return -30.0 - 10.0 * (d - 124.0) / 28.5;
    return -40.0;
}
// 1 character per grid point: index v = round(-dB/1.25) clamped 0..61; 'A'..'Z' 0..25, 'a'..'z' 26..51, '0'..'9' 52..61
static char mk_enc(double db){
    int v = (int)floor(-db / 1.25 + 0.5); if (v < 0) v = 0; if (v > 61) v = 61;
    return v < 26 ? 'A' + v : v < 52 ? 'a' + (v - 26) : '0' + (v - 52);
}
static int mk_dec(char c, double *db){
    int v;
    if (c >= 'A' && c <= 'Z') v = c - 'A'; else if (c >= 'a' && c <= 'z') v = 26 + c - 'a'; else if (c >= '0' && c <= '9') v = 52 + c - '0'; else return 0;
    *db = -1.25 * v; return 1;
}
// Combine the stored measurements into a running max-hold (spectrum), max-hold floor and the margin derived from them
static void mask_combine(void){
    for (int k = 0; k < GRID; k++){ cmb_sp[k] = -99.0; cmb_fl[k] = -99.0; }
    cmb_dev = 0.0; cmb_n = 0;
    double tnow = mono();
    for (int i = 0; i < mk_n; i++){
        if (tnow - mk[i].t > MASK_WINDOW) continue;                 // older than 5 minutes: excluded
        cmb_n++;
        for (int k = 0; k < GRID; k++){
            if (mk[i].sp[k] > cmb_sp[k]) cmb_sp[k] = mk[i].sp[k];
            if (mk[i].fl[k] > cmb_fl[k]) cmb_fl[k] = mk[i].fl[k];
        }
        if (mk[i].dev > cmb_dev) cmb_dev = mk[i].dev;
    }
    double margin = 1e9, sh = 0, fl = 0; int n = 0;
    for (int k = 0; k < GRID; k++){
        double off = (k - GRID / 2) * 6.0, ad = fabs(off);
        if (ad < 80.0) continue;
        double m = mask_lvl(off) - cmb_sp[k]; if (m < margin) margin = m;
        if (ad >= 130.0){ sh += cmb_sp[k]; fl += cmb_fl[k]; n++; }
    }
    cmb_margin = margin; cmb_sho = n ? sh / n : 0; cmb_flo = n ? fl / n : 0; cmb_conc = ((cmb_sho - cmb_flo) >= 6.0);
}
// Mask protection: evaluate every ~1 s (after mask_combine). The 5-minute window is cleared after every change, so the margin below is the one SINCE the change.
//  - margin < GUARD_LOW (after at least 20 s of data): lower the limit so that the expected margin becomes GUARD_TARGET (the margin grows ~0.9 dB per dB lower peak). The basis is the
//    actually transmitted peak (dev_hold, last minute) if that is below the limit: a limit above the real peak does nothing.
//  - margin > GUARD_HIGH for at least 2 minutes: raise the limit by 0.5 kHz, up to the chosen maximum.
static void guard_step(void){
    if (!guard_on || !lim_present || !lim_user || tx_off) return;
    if (fabs(ceil_khz - ceil_tgt) > 0.01) return;                  // limit is still ramping to its target
    if (guard_init && cmb_n >= GUARD_INIT_N) guard_init = 0;
    if (lp_fail >= 2 || !cmb_conc || cmb_n < (guard_init ? GUARD_INIT_N : 80)) return;   // no reliable measurement (while initialising: wait for enough data)
    double now = mono();
    if (now - guard_t < 15.0) return;
    double w = cmb_margin, nc = ceil_khz;
    if (w < GUARD_LOW){
        double need = (GUARD_TARGET - w) / 0.9;                    // dB peak reduction
        double base = (dev_hold > 5.0 && dev_hold < ceil_khz) ? dev_hold : ceil_khz;
        nc = base * pow(10.0, -need / 20.0);
        if (nc > ceil_khz - 0.5) nc = ceil_khz - 0.5;
        if (nc < ceil_khz * 0.75) nc = ceil_khz * 0.75;             // at most 25 % per step
        if (nc < GUARD_MIN) nc = GUARD_MIN;
    } else if (guard_act && w > GUARD_HIGH && cmb_n >= 480){
        nc = ceil_khz + 0.5; if (nc > ceil_user) nc = ceil_user;
    }
    if (fabs(nc - ceil_khz) < 0.01) return;
    fprintf(stderr, "   -> maskerbescherming: begrenzergrens %.1f -> %.1f kHz (marge %+.1f dB, max %.1f kHz)\n", ceil_khz, nc, w, ceil_user); fflush(stderr);
    ceil_tgt = nc; guard_act = (nc < ceil_user - 0.01); guard_t = now;   // ceil_ramp() applies the change gradually and then clears the window
}
// Gradual change of the limiter ceiling (called from the main loop): 0.5 kHz per 100 ms. Afterwards clear the mask window (the margin then counts only the resulting state).
static void ceil_ramp(void){
    static double t_r = 0.0;
    double now = mono();
    if (now - t_r < 0.1 || fabs(ceil_khz - ceil_tgt) < 0.01) return;
    t_r = now;
    double d = ceil_tgt - ceil_khz;
    ceil_khz += (d > 0.5) ? 0.5 : (d < -0.5) ? -0.5 : d;
    lim_apply();
    if (fabs(ceil_khz - ceil_tgt) < 0.01){ ceil_khz = ceil_tgt; guard_t = mono(); mk_n = 0; mk_i = 0; cmb_conc = 0; cmb_n = 0; }
}
static void mask_done_blk(const char *mb){                 // processes a complete MASK/SPEC/FLOOR block (NUL-terminated)
    int conc, g; double margin, at, sho, flo, dev;
    if (sscanf(mb, "MASK conclusive=%d margin=%lf at=%lf shoulder=%lf floor=%lf dev=%lf g=%d", &conc, &margin, &at, &sho, &flo, &dev, &g) != 7){
        lp_err(mb); fprintf(stderr, "   .. mask: %.90s\n", mb); fflush(stderr); return;
    }
    lp_g = g;
    // A capture with phase jumps (dropped/corrupted samples in the RX capture itself; the transmitter cannot cause this) is not counted
    { const char *jp = strstr(mb, "jumps="); long jumps = jp ? atol(jp + 6) : 0;
      static long n_bad = 0, n_all = 0; n_all++;
      if (jumps >= 1000){ if (++lp_jrun >= 4){ lp_fail = 2; lp_t = mono(); } } else lp_jrun = 0;   // noise instead of signal (impossible deviation) in several consecutive windows = no loop
      if (jumps > 0){
          n_bad++;
          fprintf(stderr, "   .. mask: opname verstoord (%ld fasesprongen, meting had marge %+.1f dB @%+.0f kHz) -> genegeerd [%ld van %ld]\n",
                  jumps, margin, at, n_bad, n_all);
          fflush(stderr); return;
      } }
    lp_ok(lp_g);                                                       // window accepted: the loop carries signal
    const char *sp = strstr(mb, "SPEC "), *fp = strstr(mb, "FLOOR ");
    // Via the twin (RX2/TX2), TX2 counts slightly more favourably than TX1: the skirts (|d| >= 80 kHz) get the measured offset added
    int viatwin = (mask_rx_cfg == 2 && twin_enable);
    double off = viatwin ? TWIN_OFFSET_DB : 0.0;
    margin -= off;
    mk_last_margin = margin; mk_last_t = mono();
    double tsp[GRID], tfl[GRID];                                         // decoded first: a bad block must not touch the ring
    int okspec = 0;
    if (sp && fp){
        sp += 5; fp += 6; okspec = 1;
        for (int k = 0; k < GRID; k++){
            double a, b;
            if (!mk_dec(sp[k], &a) || !mk_dec(fp[k], &b)){ okspec = 0; break; }
            double ad = fabs((k - GRID / 2) * 6.0);
            if (ad >= 80.0){ a += off; if (a > 0.0) a = 0.0; }          // only the skirts are shifted; the peak stays at 0 dB
            tsp[k] = a; tfl[k] = b;
        }
    }
    if (!okspec){                                                        // no usable spectrum: do not count this block
        fprintf(stderr, "   .. mask: geen SPEC/FLOOR in de uitvoer\n"); fflush(stderr); return;
    }
    int slot = mk_i; mk_i = (mk_i + 1) % MASK_KEEP; if (mk_n < MASK_KEEP) mk_n++;
    mk[slot].t = mk_last_t; mk[slot].margin = margin; mk[slot].sho = sho + off; mk[slot].flo = flo; mk[slot].dev = dev; mk[slot].conc = conc;
    for (int k = 0; k < GRID; k++){ mk[slot].sp[k] = tsp[k]; mk[slot].fl[k] = tfl[k]; }
    mk_seq++;                                                            // an accepted spectrum (?L: q=, ?M: seq=)
    // the combination over the 5-min window takes a few ms at 4 spectra per second (1200 entries): compute at most once per second
    static double last_cmb = 0.0; static int lg = 0;
    int fresh_cmb = 0;
    if (mono() - last_cmb >= 1.0){ last_cmb = mono(); mask_combine(); fresh_cmb = 1; }
    { if (fresh_cmb && (lg++ % (MASK_LOG_EVERY / 4) == 0 || cmb_margin < 3.0)){
        const char *cp = strstr(mb, "cov="); double covv = cp ? atof(cp + 4) : 1.0;       // coverage of the FFT windows (1.00 = gapless)
        { const char *sk = strstr(mb, "skipped="); static long sk_prev = 0; long skv = sk ? atol(sk + 8) : 0;
          if (skv > sk_prev + 0) { fprintf(stderr, "   !! maskmeting: %ld venster(s) overgeslagen (verwerking te traag): de 5-minutenmarge mist die intervallen\n", skv - sk_prev); fflush(stderr); }
          sk_prev = skv; }
        fprintf(stderr, "   .. mask%s: marge %+.1f dB @%+.0f kHz | 5-min max-hold (%d metingen): marge %+.1f, schouder %.1f, vloer %.1f, dev %.0f kHz%s  cov=%.2f\n",
                viatwin ? "[TX2, -1,4 dB]" : "", margin, at, cmb_n, cmb_margin, cmb_sho, cmb_flo, cmb_dev, cmb_conc ? "" : "  [vloer-beperkt]", covv);
        fflush(stderr); } }

    if (fresh_cmb) guard_step();
}
static void measure_finish(void){
    int st; kill(m_pid, SIGKILL); waitpid(m_pid, &st, 0);
    close(m_fd); m_pid = -1; m_fd = -1;
    if (pw_en && (m_kind == 0 || mask_rx_cfg == 1)) pw_rx1_init();                  // the ALC measurement left RX1 on TX_MONITOR1 with its own gain: restore port, manual mode and the meter's gain
}
static void measure_done(void){
    m_buf[m_len] = 0;
    int g; double on, off, lvl;
    if (sscanf(m_buf, "OK g=%d on=%lf off=%lf lvl=%lf", &g, &on, &off, &lvl) != 4){
        if (!strncmp(m_buf,"LOW",3)) alc_state = "low";      // signal too weak relative to the floor
        m_next = mono() + ALC_PERIOD; return;
    }
    last_lvl = lvl;
    if (!ref_valid){
        ref_g = g; ref_lvl = lvl; ref_valid = 1; trim = 0.0;
        alc_state = "hold"; m_next = mono() + ALC_PERIOD; m_big = 0;
        fprintf(stderr,"   -> ALC referentie: g=%d lvl=%.2f dB (setpoint %.2f dBm)\n", g, lvl, want_dbm); fflush(stderr);
        return;
    }
    double err = lvl - ref_lvl;                             // + = too loud
    if (fabs(err) > 6.0){                                   // improbable: re-reference
        if (++m_big >= 3){ alc_reset(1.0); return; }
        m_next = mono() + 5.0; return;
    }
    m_big = 0;
    if (fabs(err) <= ALC_DEADBAND){ alc_state = (fabs(trim) >= ALC_MAX_TRIM-0.01) ? "limit" : "hold"; m_next = mono() + ALC_PERIOD; return; }
    // err > 0 = too loud -> MORE attenuation (trim positive); err < 0 = too soft -> less
    double step = err;
    if (step >  1.0) step =  1.0;
    if (step < -1.0) step = -1.0;
    step = floor(fabs(step)*4.0 + 0.5)/4.0 * (step < 0 ? -1.0 : 1.0);
    if (step == 0.0) step = (err > 0) ? 0.25 : -0.25;
    double nt = trim + step;
    if (nt >  ALC_MAX_TRIM) nt =  ALC_MAX_TRIM;
    if (nt < -ALC_MAX_TRIM) nt = -ALC_MAX_TRIM;
    if (nom_a + nt < ATTEN_FLOOR) nt = ATTEN_FLOOR - nom_a;      // below the floor the trim has no effect: no wind-up
    if (nt > ALC_MAX_TRIM) nt = ALC_MAX_TRIM;
    if (nt != trim){
        trim = nt;
        if (!tx_off && !cal_hold) apply_atten(nom_a + trim);
        fprintf(stderr,"   -> ALC: fout %+.2f dB, trim nu %+.2f dB (atten %.2f)\n", err, trim, applied_a); fflush(stderr);
        alc_state = (fabs(trim) >= ALC_MAX_TRIM-0.01) ? "limit" : "adj";
        m_next = mono() + ALC_SETTLE + 1.0;                 // follow quickly until within the deadband
    } else {
        alc_state = "limit"; m_next = mono() + ALC_PERIOD;
    }
}
// Continuous mask mode: the child delivers a MASK/SPEC/FLOOR block every second; process each complete block immediately
// tone measurement (audio linearity): the mask monitor delivers 'TONE f=<Hz> dev=<kHz>' before each MASK block as long as /tmp/skypluto.tone is fresh ('?A' touches that file)
static double tone_f = 0.0, tone_dev = 0.0, tone_t = -100.0; static unsigned long tone_seq = 0;
static void mask_drain(void){
    for (;;){
        m_buf[m_len] = 0;
        if (!strncmp(m_buf, "PWR ", 4)){
            char *e1 = strchr(m_buf, '\n');
            if (!e1) break;
            double pp, kk;
            if (sscanf(m_buf, "PWR p=%lf pk=%lf", &pp, &kk) == 2 && pw_en) pw_feed(pp, kk);
            int rest1 = m_len - (int)(e1 + 1 - m_buf); memmove(m_buf, e1 + 1, (size_t)rest1); m_len = rest1; continue;
        }
        if (!strncmp(m_buf, "TONE ", 5)){
            char *e0 = strchr(m_buf, '\n');
            if (!e0) break;                                           // line not complete yet
            double tf, td;
            if (sscanf(m_buf, "TONE f=%lf dev=%lf", &tf, &td) == 2){ tone_f = tf; tone_dev = td; tone_t = mono(); tone_seq++; }
            int rest0 = m_len - (int)(e0 + 1 - m_buf); memmove(m_buf, e0 + 1, (size_t)rest0); m_len = rest0; continue;
        }
        char *f = strstr(m_buf, "FLOOR ");
        char *e = f ? strchr(f, '\n') : NULL;
        if (e){
            char sv = e[1]; e[1] = 0;
            mask_done_blk(m_buf);
            e[1] = sv;
            int rest = m_len - (int)(e + 1 - m_buf);
            memmove(m_buf, e + 1, (size_t)rest); m_len = rest;
            continue;
        }
        if (!strncmp(m_buf, "MASK error", 10)){                       // error line without block: log and discard
            char *e2 = strchr(m_buf, '\n');
            if (e2){ *e2 = 0; lp_err(m_buf); fprintf(stderr, "   .. mask: %.90s\n", m_buf); fflush(stderr);
                     int rest = m_len - (int)(e2 + 1 - m_buf); memmove(m_buf, e2 + 1, (size_t)rest); m_len = rest; continue; }
        }
        break;
    }
}
// Green LED (led0:green): off = transmitter closed, blinking (2 Hz) = tuning / calibrating (output muted), on = really on the air.
// The kernel's 'tx-active' trigger is switched off at the first call so that the daemon owns the LED.
static void led_poll(double now){
    static int fd = -2, last = -1;
    if (fd == -2){
        int tf = open("/sys/class/leds/led0:green/trigger", O_WRONLY);
        if (tf >= 0){ if (write(tf, "none", 4) < 0){} close(tf); }
        fd = open("/sys/class/leds/led0:green/brightness", O_WRONLY);
    }
    if (fd < 0) return;
    int busy = cal_hold || cal_pending || cal_pid > 0;
    int on;
    if (tx_off) on = 0;
    else if (busy) on = ((long)(now * 4.0)) & 1;
    else if (cal_failed || applied_a >= ATTEN_MUTE - 0.1) on = 0;       // the output stays muted: not on the air
    else on = 1;
    if (on != last){ if (write(fd, on ? "1" : "0", 1) < 0){} last = on; }
}

static void measure_poll(void){
    if (m_pid <= 0) return;
    for (;;){
        ssize_t n = read(m_fd, m_buf + m_len, sizeof m_buf - 1 - m_len);
        if (n > 0){
            m_len += (int)n;
            if (m_kind == 1){ mask_drain(); if (m_len >= (int)sizeof m_buf - 1) m_len = 0; }     // buffer full without a block: discard
            else if (m_len >= (int)sizeof m_buf - 1) break;
            continue;
        }
        if (n == 0){                                                    // EOF: done
            int kind = m_kind;
            if (kind == 1) mask_drain();
            measure_finish();
            // after a mask process (25 s continuous) the ALC quickly gets its turn; the next mask process then starts immediately
            if (kind == 1){ mask_next = mono() + MASK_PERIOD; if (m_next < mono() + 1.0) m_next = mono() + 1.0; }
            else measure_done();
            return;
        }
        if (errno == EAGAIN) break;
        measure_finish(); m_next = mono() + ALC_PERIOD; return;
    }
    if (mono() - m_started > (m_kind == 1 ? 70.0 : 15.0)){          // hung: give up
        int kind = m_kind; measure_finish();
        if (kind == 1) mask_next = mono() + MASK_PERIOD; else m_next = mono() + ALC_PERIOD;
    }
}

// ---- child process: TX_MONITOR measurement ----------------------------------------------
static double cap_power(void){
    FILE *p = popen("iio_readdev -s 8192 cf-ad9361-lpc voltage0 voltage1 2>/dev/null", "r");
    if (!p) return -1.0;
    static int16_t b[16384];
    size_t n = fread(b, sizeof(int16_t), 16384, p);
    pclose(p);
    if (n < 16384) return -1.0;
    double s = 0.0;
    for (int i = 0; i < 8192; i++) s += (double)b[2*i]*b[2*i] + (double)b[2*i+1]*b[2*i+1];
    return s / 8192.0;
}
static void set_rxlo(long long hz){
    char c[128];
    snprintf(c,sizeof c,"iio_attr -o -c ad9361-phy altvoltage0 frequency %lld >/dev/null 2>&1", hz);
    system(c); usleep(40000);
}
static void set_rxgain(int g){
    char c[128];
    snprintf(c,sizeof c,"iio_attr -i -c ad9361-phy voltage0 hardwaregain %d >/dev/null 2>&1", g);
    system(c);
}
static int measure_main(long long txlo, int gain){
    struct sched_param sp; sp.sched_priority = 0;
    sched_setscheduler(0, SCHED_OTHER, &sp);
    system("iio_attr -i -c ad9361-phy voltage0 rf_port_select TX_MONITOR1 >/dev/null 2>&1");
    system("iio_attr -i -c ad9361-phy voltage0 gain_control_mode manual >/dev/null 2>&1");
    int g = gain ? gain : 65;
    double on = -1.0;
    for (int it = 0; it < 3; it++){
        set_rxgain(g);
        set_rxlo((txlo - 300000LL >= 70500000LL) ? txlo - 300000LL : txlo + 300000LL);     // the RX LO has a 70 MHz lower limit
        on = cap_power();
        if (on < 0){ break; }
        if (!gain){
            if (on > 1.0e6 && g > 25){ g -= 20; continue; }     // close to ADC clipping
            if (on < 1500.0 && g < 70){ g += 8;  continue; }    // too weak relative to the ADC resolution
        }
        break;
    }
    double f1 = -1.0, f2 = -1.0;
    if (on >= 0){
        set_rxlo((txlo + 30000000LL <= 5950000000LL) ? txlo + 30000000LL : txlo - 30000000LL);   // carrier outside the RX filter (below it when +30 MHz would pass 6 GHz)
        f1 = cap_power();
        set_rxlo((txlo - 30000000LL >= 75000000LL) ? txlo - 30000000LL : (txlo + 45000000LL <= 5950000000LL ? txlo + 45000000LL : txlo + 30000000LL));
        f2 = cap_power();
    }
    system("iio_attr -i -c ad9361-phy voltage0 rf_port_select A_BALANCED >/dev/null 2>&1");
    if (on < 0 || f1 < 0 || f2 < 0){ printf("ERR capture\n"); return 1; }
    double fl = 0.5*(f1 + f2), sig = on - fl;
    if (sig < 1.5*fl || sig <= 1.0){ printf("LOW g=%d on=%.1f off=%.1f\n", g, on, fl); return 0; }
    printf("OK g=%d on=%.1f off=%.1f lvl=%.2f\n", g, on, fl, 10.0*log10(sig));
    return 0;
}

// ---- command processing -----------------------------------------------------
static void set_level(double dbm){
    if (dbm > P_MAX_DBM) dbm = P_MAX_DBM;
    want_dbm = dbm;
    nom_a = quant(DBM_AT_0DB - dbm);
    trim = 0.0;
    if (!tx_off && !cal_hold) apply_atten(nom_a);
    alc_reset(ALC_SETTLE);
    fprintf(stderr,"   -> niveau %.2f dBm  (atten %.2f dB)\n", want_dbm, nom_a); fflush(stderr);
}

static void handle(char *line){
    while (*line==' '||*line=='\t') line++;
    if (*line==0) return;
    // queries and F repeat every poll cycle: do not log (ramdisk log)
    if (line[0] != '?' && line[0] != 'F'){ fprintf(stderr,"rx: %s\n", line); fflush(stderr); }

    char *cmd = line, *arg = line;
    while (*arg && *arg!=' ' && *arg!='\t') arg++;
    if (*arg){ *arg=0; arg++; while (*arg==' '||*arg=='\t') arg++; }
    char buf[192];

    // ---------- queries ----------
    if (cmd[0]=='?'){
        if (!strcmp(cmd,"?IQ")){
            snprintf(buf, sizeof buf, "dci=%d dcq=%d gain=%d skew=%d ofs=%.3f lo=%lld hw=%d\n", iq_dc_i, iq_dc_q, iq_gain_ppm, iq_skew_ppm, lowif_khz, lo_freq(), has_iq);
            tx_str(buf); return;
        }
        if (!strcmp(cmd,"?V")){
            char bid[16] = "-"; if (has_dbg) snprintf(bid, sizeof bid, "%08X", dbg_rd(12));
            snprintf(buf,sizeof buf,"magic=%08X fw=PlutoSky_7020_Broadcast-" SW_VERSION " proto=2 ver=" SW_VERSION " bit=%s\n", rd(R_MAGIC), bid);
            tx_str(buf); return;
        }
        if (!strcmp(cmd,"?S")){
            // kdev = the value that is in the FPGA NOW (deviation at full scale = 0.75 kHz x kdev), for the deviation meter on the Pico
            snprintf(buf,sizeof buf,"en=%d f=%lld p=%.2f att=%.2f tx=%s up=%d kdev=%d\n",
                     tx_off?0:1, last_f>0?last_f:0, want_dbm, applied_a>=0?applied_a:0.0, tx_off?"off":"on",
                     (int)(mono() - t_start), (int)(rd(R_KDEV) & 0x3FFFF));
            tx_str(buf); return;
        }
        if (!strcmp(cmd,"?T")){
            snprintf(buf,sizeof buf,"temp=%.1f\n", temp_mC/1000.0);
            tx_str(buf); return;
        }
        if (!strcmp(cmd,"?P")){
            double out = ref_valid ? want_dbm + (last_lvl - ref_lvl) : want_dbm;
            snprintf(buf,sizeof buf,"set=%.2f out=%.2f att=%.2f trim=%.2f alc=%s\n",
                     want_dbm, tx_off ? -99.0 : out, applied_a>=0?applied_a:0.0, trim, tx_off?"off":alc_state);
            tx_str(buf); return;
        }
        if (!strcmp(cmd,"?E")){
            uint32_t st = rd(R_STATUS);
            // lim = max gain reduction (dB) of the limiter in the last ~10 s, limn = number of samples with limiting since start,
            // uf = number of times the I2S dropped out (soft fade), pk = input peak in % of full scale (last ~10 s)
            char cb[16]; if (lim_present && lim_user) snprintf(cb, sizeof cb, "%.1f", ceil_khz); else snprintf(cb, sizeof cb, "off");
            uint32_t fm = rd(R_FMT) & 0x3FFFF; static const char *const fm_nm[4] = {"i2s","lj","rj","?"}; const char *fm_al = fm_nm[(fm >> 16) & 3]; int fm_slot = (int)((fm >> 8) & 0xFF), fm_bits = (int)(fm & 0xFF);
            snprintf(buf,sizeof buf,"unf=%d ovf=%d lim=%.1f limn=%u uf=%u pk=%d bg=%s ceil=%s cmax=%.1f gd=%s i2s=%d/%d al=%s cal=%s\n", (st>>1)&1, st&1, lim_gr_db, lim_evt, lim_uf, lim_in_pct,
                     lim_present ? (lim_user ? "on" : "off") : "na", cb, ceil_user, (guard_on && lim_present && lim_user) ? ((guard_init && !tx_off) ? "init" : guard_act ? "act" : "on") : "off", fm_bits, fm_slot, fm_al, cal_failed ? "fail" : (cal_pid > 0 || cal_pending) ? "run" : "ok");   // bg = FPGA limiter; ceil = limit now (kHz, or off); cmax = chosen maximum (H); gd = mask protection off (also shown while the limiter is off: the guard then has no effect) / init (waiting for enough mask data) / on / act (lowering)
            tx_str(buf); return;
        }
        if (!strcmp(cmd,"?C")){ tx_str("cal=auto\n"); return; }
        // ?D = actually transmitted peak deviation (kHz) over the last 0.25 s window, measured in the FPGA at the modulator input
        // (after limiter + interpolator, so incl. FIR overshoot); ceil = current limiter ceiling (kHz) or 'off'; q = window sequence number.
        // ?G = swing meter: peak deviation (kHz) per 20 ms window, the last 25 windows (0.5 s), OLDEST FIRST, same measurement as ?D;
        // q = window sequence number (equal to the previous reply = repeat). 'g=na' without a bitstream with 20 ms windows (from B1D0000E).
        // ?A = tone measurement of the own transmission (TX2 twin -> RX2, FM demodulation, 16384-point FFT): frequency (Hz) and deviation amplitude (kHz) of the
        // strongest audio tone (100 Hz-25 kHz), updated every ~0.25 s (q = sequence number, ams = age). The first '?A' enables the measurement (valid for 10 s; ask
        // at least every 10 s). 'a=na' as long as there is no (fresh) measurement or the mask monitor is not running.
        if (!strcmp(cmd,"?A")){
            { FILE *tf = fopen("/tmp/skypluto.tone", "w"); if (tf){ fputs("1\n", tf); fclose(tf); } }
            double age = mono() - tone_t;
            if (age > 2.0){ tx_str("a=na\n"); return; }
            snprintf(buf, sizeof buf, "a=%.1f %.3f q=%lu ams=%d\n", tone_f, tone_dev, tone_seq, (int)(age * 1000.0));
            tx_str(buf); return;
        }
        // ?J = status of the impulse measurement: j=idle | j=run <s> | j=ok / j=err <rc>, n = number of detected impulses (FPGA counter)
        if (!strcmp(cmd,"?J")){
            if (ir_kind == 2 && ir_pid > 0){ tx_str("j=busy\n"); return; }
            uint32_t n1 = rd(R_IMPN), n2 = rd(R_IMPN); if (n1 != n2) n1 = rd(R_IMPN);
            if (ir_pid > 0) snprintf(buf, sizeof buf, "j=run %d n=%u\n", (int)(mono() - ir_t0), n1);
            else if (ir_rc < 0) snprintf(buf, sizeof buf, "j=idle n=%u\n", n1);
            else snprintf(buf, sizeof buf, "j=%s %d n=%u\n", ir_rc == 0 ? "ok" : "err", ir_rc, n1);
            tx_str(buf); return;
        }
        // ?R = loop cable TX2 -> RX2 (needed for the mask monitor): r=ok / r=missing / r=na (transmitter closed, no twin, or no measurement yet); g = RX2 gain (dB), fail = number of consecutive failed measurements
        // ?O = power meter (RX1): o=<dBm at the transmitter output> w=<watt> in=<dBm at RX1> att=<attenuator dB> cal=<dB> g=<RX1 gain dB> st=ok|low|high|nosig|off
        if (!strcmp(cmd,"?O")){
            const char *st = "off"; double pin = -200.0, pout = -200.0, w = 0.0;
            if (pw_en){
                pin = pw_in_dbm(); pout = pin + pw_att; w = pow(10.0, (pout - 30.0) / 10.0);
                if (tx_off || last_f <= 0 || mono() - pw_t > 5.0 || pw_p <= 0.0) st = "nosig";
                else if (pin > -10.0) st = "high";
                else if (pw_p < 1.0e4 || pin < -70.5) st = "low";            // -70.5 dBm at RX1: below this the board's own crosstalk/noise floor (about -71.5 dBm) dominates
                else if (pw_p > 1.6e6) st = "high";
                else st = "ok";
            }
            if (pout < -150.0) snprintf(buf, sizeof buf, "o=na w=0 in=na att=%.2f cal=%.2f g=%d st=%s\n", pw_att, pw_cal, pw_gain, st);
            else snprintf(buf, sizeof buf, "o=%.2f w=%.6g in=%.2f att=%.2f cal=%.2f g=%d st=%s\n", pout, w, pin, pw_att, pw_cal, pw_gain, st);
            tx_str(buf); return;
        }
        if (!strcmp(cmd,"?R")){
            const char *st = "na";
            if (!tx_off && twin_enable && mask_rx_cfg == 2){
                if (lp_fail >= 2 || (lp_g >= 55 && mono() - lp_t < 30.0)) st = "missing";
                else if (lp_fail == 0 && mono() - lp_t < 30.0) st = "ok";
            }
            snprintf(buf, sizeof buf, "r=%s g=%d fail=%d\n", st, lp_g, lp_fail);
            tx_str(buf); return;
        }
        if (!strcmp(cmd,"?Z")){
            if (ir_kind == 3 && ir_pid > 0) snprintf(buf, sizeof buf, "z=run %d\n", (int)(mono() - ir_t0));
            else if (ir_kind != 3 || ir_rc < 0) snprintf(buf, sizeof buf, "z=idle\n");
            else snprintf(buf, sizeof buf, "z=%s %d\n", ir_rc == 0 ? "ok" : "err", ir_rc);
            tx_str(buf); return;
        }
        if (!strcmp(cmd,"?Y")){
            if (ir_kind == 2 && ir_pid > 0) snprintf(buf, sizeof buf, "y=run %d\n", (int)(mono() - ir_t0));
            else if (ir_kind != 2 || ir_rc < 0) snprintf(buf, sizeof buf, "y=idle\n");
            else snprintf(buf, sizeof buf, "y=%s %d\n", ir_rc == 0 ? "ok" : "err", ir_rc);
            tx_str(buf); return;
        }
        if (!strcmp(cmd,"?B")){
            snprintf(buf, sizeof buf, "b=%d %s\n", bs_pct, bs_txt[0] ? bs_txt : "-");
            tx_str(buf); return;
        }
        if (!strcmp(cmd,"?G")){
            if (!dev_ok || !gr_mode || gr_cnt <= 0){ tx_str("g=na\n"); return; }
            int nn = gr_cnt < 25 ? gr_cnt : 25; int o = 0;
            o += snprintf(buf + o, sizeof buf - o, "g=");
            for (int i = nn; i >= 1; i--) o += snprintf(buf + o, sizeof buf - o, i == nn ? "%.1f" : " %.1f", gr[(gr_head + GR_N - i) % GR_N]);
            o += snprintf(buf + o, sizeof buf - o, " q=%lu\n", dev_seq);
            tx_str(buf); return;
        }
        if (!strcmp(cmd,"?D")){
            if (!dev_ok){ tx_str("d=na\n"); return; }
            char cb[16]; if (lim_present && lim_user) snprintf(cb, sizeof cb, "%.1f", ceil_khz); else snprintf(cb, sizeof cb, "off");
            snprintf(buf, sizeof buf, "d=%.1f ceil=%s q=%lu ams=%d\n", dev_pk_khz, cb, dev_seq, (int)((mono() - dev_last_t) * 1000.0));
            tx_str(buf); return;
        }
        // mask monitor (SM.1268-5), live for the Pico web interface
        if (!strcmp(cmd,"?M")){
            if (mk_n <= 0){ tx_str("m=na n=0\n"); return; }
            snprintf(buf,sizeof buf,"m=%+.1f w=%+.1f sh=%.1f fl=%.1f dev=%.0f n=%d age=%d tw=%d c=%d seq=%lu ams=%d\n",
                     mk_last_margin, cmb_margin, cmb_sho, cmb_flo, cmb_dev, cmb_n, (int)(mono() - mk_last_t),
                     (mask_rx_cfg == 2 && twin_enable) ? 1 : 0, cmb_conc, mk_seq, (int)((mono() - mk_last_t) * 1000.0));
            tx_str(buf); return;
        }
        if (!strcmp(cmd,"?W") || !strcmp(cmd,"?N")){
            if (mk_n <= 0){ tx_str(cmd[1]=='W' ? "s=na\n" : "f=na\n"); return; }
            const double *a = (cmd[1] == 'W') ? cmb_sp : cmb_fl;
            char out[GRID + 4]; int o = 0;
            out[o++] = (cmd[1] == 'W') ? 's' : 'f'; out[o++] = '=';
            for (int k = 0; k < GRID; k++) out[o++] = mk_enc(a[k]);
            out[o++] = '\n'; out[o] = 0;
            tx_str(out); return;
        }
        // ?L = spectrum of the LAST single measurement (same encoding and grid as ?W, prefixed with 'l='), fresh every ~6-8 s
        if (!strcmp(cmd,"?L")){
            if (mk_n <= 0){ tx_str("l=na\n"); return; }
            const double *a = mk[(mk_i + MASK_KEEP - 1) % MASK_KEEP].sp;
            char out[GRID + 24]; int o = 0;
            out[o++] = 'l'; out[o++] = '=';
            for (int k = 0; k < GRID; k++) out[o++] = mk_enc(a[k]);
            o += snprintf(out + o, sizeof out - o, " q=%lu", mk_seq);       // sequence number of this spectrum: equal to the previous reply = repeat
            out[o++] = '\n'; out[o] = 0;
            tx_str(out); return;
        }
        tx_str("ERR query\n"); return;
    }

    // ---------- the three control commands ----------
    if (!strcmp(cmd,"E")){
        want_on = atoi(arg) ? 1 : 0;
        wr(R_CTRL,(rd(R_CTRL)&~1u)|1u);                 // fabric stays on; off = via atten
        tx_str("OK\n");                                 // reply FIRST (the iio_attr call takes ~140 ms under high CPU load)
        tx_set_state(want_on && tuned ? 0 : 1);          // without a tune (F) the transmitter stays closed, even after E 1
        return;
    }
    if (!strcmp(cmd,"F")){
        long long hz = atoll(arg);
        if (hz < 70000000LL || hz > 6000000000LL){ tx_str("ERR range\n"); return; }
        long long d = hz - last_f; if (d < 0) d = -d;
        if (tuned && last_f > 0 && d <= 10){ tx_str("OK\n"); return; }   // unchanged (PLL rounds to ~2 Hz)
        char c[160];
        snprintf(c,sizeof c,"iio_attr -q -o -c ad9361-phy altvoltage1 frequency %lld >/dev/null 2>&1", hz - lowif_hz());
        if (tx_off){                                    // transmitter closed (TX-LO off): remember; the tune opens the transmitter if the Pico wants E 1
            last_f = hz; tuned = 1; tx_str("OK\n");
            fprintf(stderr,"   -> F onthouden %lld (tune ontvangen)\n", hz); fflush(stderr);
            tx_set_state(want_on ? 0 : 1);               // tx_lo_power(1) programs last_f into the LO
            return;
        }
        tx_str("OK\n");                                 // reply first; the retune is done by the calibration child (below), so this command stays fast
        if (ir_pid > 0){                                // a J/Y measurement owns the RX path: retune directly, without muting and calibrating
            if (system(c)==0){ last_f = hz; alc_reset(3.0); fprintf(stderr,"   -> F toegepast (LO-hertune) %lld\n", hz); fflush(stderr); }
            else { fprintf(stderr,"   !! F %lld MISLUKT (iio_attr)\n", hz); fflush(stderr); }
            return;
        }
        last_f = hz; alc_reset(3.0);
        if (!cal_hold) cal_hold_t = mono();
        cal_hold = 1; cal_pending = 1; if (cal_retune < 1) cal_retune = 1; cal_due = mono(); cal_failed = 0; cal_fail = 0;
        { FILE *tf = fopen("/tmp/skypluto-tuned", "w"); if (tf){ fprintf(tf, "%lld\n", last_f); fclose(tf); } }     // a restart of the daemon adopts the CURRENT frequency
        { char mv[16]; snprintf(mv, sizeof mv, "-%.2f", ATTEN_MUTE);
          if (phy_write("out_voltage0_hardwaregain", mv) == 0){ applied_a = ATTEN_MUTE; phy_write("out_voltage1_hardwaregain", mv); applied_a2 = ATTEN_MUTE; } }   // mute at once
        fprintf(stderr,"   -> F %lld ontvangen: zender dicht, hertune + kalibratie, daarna weer open\n", hz); fflush(stderr);
        return;
    }
    // OFS <kHz>: low-IF offset (-250..250, 0 = zero-IF): the TX LO sits <kHz> below the carrier and the modulator's NCO is offset by the same amount, so the carrier stays where it is while the LO leakage and the I/Q
    // image move away from it. While the transmitter is open the output is muted, the NCO and the LO are shifted and the TX is calibrated at the new LO (like a tune). Stored in the iq conf.
    if (!strcmp(cmd,"OFS")){
        char *e; double v = strtod(arg, &e);
        if (e == arg || !isfinite(v) || v < -250.0 || v > 250.0){ tx_str("ERR range\n"); return; }
        if (fabs(v - lowif_khz) < 0.0005){ tx_str("OK\n"); return; }
        if (!tx_off && (ir_pid > 0 || cal_pid > 0)){ tx_str("ERR busy\n"); return; }
        tx_str("OK\n");
        if (!tx_off){ char mv[16]; snprintf(mv, sizeof mv, "-%.2f", ATTEN_MUTE);
          if (phy_write("out_voltage0_hardwaregain", mv) == 0){ applied_a = ATTEN_MUTE; phy_write("out_voltage1_hardwaregain", mv); applied_a2 = ATTEN_MUTE; } }   // mute at once
        lowif_khz = v; nco_offset_apply(); iq_conf_write();
        if (!tx_off){
            alc_reset(3.0);
            if (!cal_hold) cal_hold_t = mono();
            cal_hold = 1; cal_pending = 1; if (cal_retune < 1) cal_retune = 1; cal_due = mono(); cal_failed = 0; cal_fail = 0;
        }
        fprintf(stderr, "   -> low-IF offset %.3f kHz (TX-LO %lld Hz, draaggolf %lld Hz)\n", lowif_khz, lo_freq(), last_f); fflush(stderr);
        return;
    }
    // DC <i> <q>: digital DC offset on I and Q in 16-bit sample LSBs (-2047..2047) to cancel the analog LO leakage. Kept in the iq conf.
    if (!strcmp(cmd,"DC")){
        int a, b;
        if (sscanf(arg, "%d %d", &a, &b) != 2 || a < -2047 || a > 2047 || b < -2047 || b > 2047){ tx_str("ERR range\n"); return; }
        iq_dc_i = a; iq_dc_q = b; iq_apply(); iq_conf_write(); tx_str("OK\n"); return;
    }
    // IQ <gain_ppm> <skew_ppm>: cancel the analog I/Q imbalance digitally: Q gets <gain> (parts per million, a gain error) of itself and <skew> (parts per million of a radian: the phase error) of I added.
    // -400000..400000; needs a bitstream with the correction (B1D00019 and later, else ERR nobit). Kept in the iq conf.
    if (!strcmp(cmd,"IQ")){
        int g, k;
        if (sscanf(arg, "%d %d", &g, &k) != 2 || abs(g) > 400000 || abs(k) > 400000){ tx_str("ERR range\n"); return; }
        if (!has_iq){ tx_str("ERR nobit\n"); return; }
        iq_gain_ppm = g; iq_skew_ppm = k; iq_apply(); iq_conf_write(); tx_str("OK\n"); return;
    }
    // X: restart the mask monitor (clear the 5-min max-hold and statistics: ?W/?N empty, ?M counts from zero).
    // J <secs> [pct]: start an impulse measurement (the mask monitor is paused). Meanwhile the Pico puts one impulse per second in the I2S stream
    // (silence or pilot only around it, impulse above <pct> % of full scale, default 12 %). Duration 5..600 s. Result: /tmp/pluto_ir.csv.
    if (!strcmp(cmd,"J")){
        double secs = atof(arg);
        double pct = 12.0; { const char *sp = strchr(arg, ' '); if (sp) pct = atof(sp + 1); }
        if (secs < 5.0 || secs > 600.0 || pct < 1.0 || pct > 90.0 || !isfinite(secs) || !isfinite(pct)){ tx_str("ERR range\n"); return; }
        if (tx_off || last_f <= 0){ tx_str("ERR off\n"); return; }
        if (ir_pid > 0 || cal_hold || cal_pending || cal_pid > 0){ tx_str("ERR busy\n"); return; }
        if (access(MASKBIN, X_OK) != 0){ tx_str("ERR nomask\n"); return; }
        tx_str("OK\n"); ir_kind = 1;
        { FILE *hf = fopen("/tmp/skypluto.hold", "w"); if (hf){ fputs("ir\n", hf); fclose(hf); } }
        if (m_pid > 0) measure_finish();                      // stop a running mask measurement: the RX chain is for the impulse measurement
        wr(R_IMPT, (uint32_t)(pct / 100.0 * 8388608.0)); wr(R_IMPC, 1);
        char a1[32], a2[16]; snprintf(a1, sizeof a1, "%lld", last_f); snprintf(a2, sizeof a2, "%.1f", secs + 3.0);
        unlink("/tmp/pluto_ir.csv");
        pid_t p = fork();
        if (p == 0){ execl(MASKBIN, MASKBIN, "--ir", a1, "2", "A_BALANCED", a2, "/tmp/pluto_ir.csv", (char*)NULL); _exit(127); }
        if (p < 0){ ir_finish(); ir_rc = 99; return; }
        ir_pid = p; ir_t0 = mono(); ir_secs = secs; ir_rc = -1;
        fprintf(stderr, "   -> impulsmeting gestart (%.0f s, drempel %.0f %%)\n", secs, pct); fflush(stderr);
        return;
    }
    // Y <secs>: LO leakage / fine spectrum of the own transmission (TX2 twin -> RX2): per 0.5 s window the strongest tone, the level of the
    // carrier bin (dBc) and absolute levels; fine spectrum (+-150 kHz, RBW 94 Hz) of the window with the deepest carrier and of the first window.
    // Duration 5..600 s. Files: /tmp/pluto_lo_sweep.csv, _null.csv, _ref.csv. Status: ?Y.
    if (!strcmp(cmd,"Y")){
        double secs = atof(arg);
        if (secs < 5.0 || secs > 600.0 || !isfinite(secs)){ tx_str("ERR range\n"); return; }
        if (tx_off || last_f <= 0){ tx_str("ERR off\n"); return; }
        if (ir_pid > 0 || cal_hold || cal_pending || cal_pid > 0){ tx_str("ERR busy\n"); return; }
        if (access(MASKBIN, X_OK) != 0){ tx_str("ERR nomask\n"); return; }
        tx_str("OK\n"); ir_kind = 2;
        { FILE *hf = fopen("/tmp/skypluto.hold", "w"); if (hf){ fputs("lo\n", hf); fclose(hf); } }
        if (m_pid > 0) measure_finish();
        char a1[32], a2[16]; snprintf(a1, sizeof a1, "%lld", last_f); snprintf(a2, sizeof a2, "%.1f", secs);
        unlink("/tmp/pluto_lo_sweep.csv");
        pid_t p = fork();
        if (p == 0){ execl(MASKBIN, MASKBIN, "--lo", a1, "2", "A_BALANCED", a2, "/tmp/pluto_lo", (char*)NULL); _exit(127); }
        if (p < 0){ ir_finish(); ir_rc = 99; return; }
        ir_pid = p; ir_t0 = mono(); ir_secs = secs; ir_rc = -1;
        fprintf(stderr, "   -> LO/spectrum-meting gestart (%.0f s)\n", secs); fflush(stderr);
        return;
    }
    // Z <secs>: write out the demodulated MPX (192 kHz, float32 kHz deviation) of the own transmission (TX2 twin -> RX2), windows of 0.5 s with a
    // tone label per window (for stereo linearity: the Pico decodes the 38 kHz subcarrier itself). 5..300 s. Files /tmp/pluto_mpx.f32 + _index.csv. Status: ?Z.
    if (!strcmp(cmd,"Z")){
        double secs = atof(arg);
        if (secs < 5.0 || secs > 300.0 || !isfinite(secs)){ tx_str("ERR range\n"); return; }
        if (tx_off || last_f <= 0){ tx_str("ERR off\n"); return; }
        if (ir_pid > 0 || cal_hold || cal_pending || cal_pid > 0){ tx_str("ERR busy\n"); return; }
        if (access(MASKBIN, X_OK) != 0){ tx_str("ERR nomask\n"); return; }
        tx_str("OK\n"); ir_kind = 3;
        { FILE *hf = fopen("/tmp/skypluto.hold", "w"); if (hf){ fputs("mpx\n", hf); fclose(hf); } }
        if (m_pid > 0) measure_finish();
        char a1[32], a2[16]; snprintf(a1, sizeof a1, "%lld", last_f); snprintf(a2, sizeof a2, "%.1f", secs);
        unlink("/tmp/pluto_mpx.f32"); unlink("/tmp/pluto_mpx_index.csv");
        pid_t p = fork();
        if (p == 0){ execl(MASKBIN, MASKBIN, "--mpx", a1, "2", "A_BALANCED", a2, "/tmp/pluto_mpx", (char*)NULL); _exit(127); }
        if (p < 0){ ir_finish(); ir_rc = 99; return; }
        ir_pid = p; ir_t0 = mono(); ir_secs = secs; ir_rc = -1;
        fprintf(stderr, "   -> MPX-opname gestart (%.0f s)\n", secs); fflush(stderr);
        return;
    }
    // Q <dB>: attenuation of the external attenuator between the transmitter output and RX1 (0..120 dB, stored). QK <dBm>: calibrate: the power at the transmitter output is NOW
    // <dBm> (-60..+60) -> cal is set so that the meter shows that (requires a valid measurement). QM <0|1>: power meter off / on (stored).
    // ?O returns the measurement. Measurement only: nothing is controlled with it.
    if (!strcmp(cmd,"Q")){
        char *e; double v = strtod(arg, &e);
        if (e == arg || !isfinite(v) || v < 0.0 || v > 120.0){ tx_str("ERR range\n"); return; }
        pw_att = v; pw_conf_write(); tx_str("OK\n"); return;
    }
    // QG <gain|A>: hold the RX1 gain at <gain> dB (0..70), or A = automatic range switching again (not stored).
    if (!strcmp(cmd,"QG")){
        if (arg[0] == 'A' || arg[0] == 'a'){ pw_man = -1; tx_str("OK\n"); return; }
        char *e; double v = strtod(arg, &e);
        if (e == arg || !isfinite(v) || v < 0.0 || v > 70.0){ tx_str("ERR range\n"); return; }
        pw_man = (int)(v + 0.5); pw_set_gain(pw_man); tx_str("OK\n"); return;
    }
    if (!strcmp(cmd,"QM")){
        int on = atoi(arg) ? 1 : 0;
        pw_en = on; pw_conf_write(); pw_inited = 0; pw_ring_n = 0; tx_str("OK\n"); return;
    }
    if (!strcmp(cmd,"QK")){
        char *e; double known = strtod(arg, &e);
        if (e == arg || !isfinite(known) || known < -60.0 || known > 60.0){ tx_str("ERR range\n"); return; }
        if (!pw_en || pw_p <= 0.0 || mono() - pw_t > 3.0 || pw_p < 1.0e4 || pw_p > 1.6e6){ tx_str("ERR nosig\n"); return; }
        if (pw_ring_n < 5 || pw_spread > 0.5){ tx_str("ERR unstable\n"); return; }          // calibrate only on a settled reading (5 captures at this gain, spread <= 0.5 dB)
        double base = pw_in_dbm() - pw_cal + pw_att;                   // measurement without cal, at the transmitter output
        pw_cal = known - base; pw_conf_write(); tx_str("OK\n");
        fprintf(stderr, "   -> vermogensmeter gekalibreerd: cal %.2f dB (bekend %.2f dBm)\n", pw_cal, known); fflush(stderr);
        return;
    }
    // CAL: run the TX calibration (LO leakage, image) now, with the output muted (~2 s). Needs an open transmitter (ERR off otherwise) and no J/Y/Z measurement (ERR busy).
    if (!strcmp(cmd,"CAL")){
        if (tx_off || last_f <= 0){ tx_str("ERR off\n"); return; }
        if (ir_pid > 0){ tx_str("ERR busy\n"); return; }
        tx_str("OK\n");
        if (cal_pending || cal_pid > 0) return;                  // a calibration is already pending or running: it does exactly this
        if (!cal_hold) cal_hold_t = mono();
        { int was_failed = cal_failed; if (was_failed && cal_retune < 1) cal_retune = 1; }   // after a failure the LO may be wrong: retune it as well
        cal_hold = 1; cal_pending = 1; cal_due = mono(); cal_failed = 0; cal_fail = 0;
        fprintf(stderr, "   -> CAL: TX-kalibratie op verzoek\n"); fflush(stderr);
        return;
    }
    if (!strcmp(cmd,"X")){
        mk_n = 0; mk_i = 0; cmb_conc = 0; cmb_n = 0; cmb_margin = 0; cmb_sho = 0; cmb_flo = 0; cmb_dev = 0; guard_init = 1;
        tx_str("OK\n");
        fprintf(stderr, "   -> maskerbewaking gereset (op verzoek van de Pico)\n"); fflush(stderr);
        return;
    }
    if (!strcmp(cmd,"P") || !strcmp(cmd,"A")){
        double v = atof(arg);
        double dbm = (cmd[0]=='A') ? DBM_AT_0DB - v : v;       // A = deprecated alias (atten in dB)
        if (!isfinite(dbm) || dbm < DBM_AT_0DB - ATTEN_MAX || dbm > P_MAX_DBM){ tx_str("ERR range\n"); return; }
        if (fabs(dbm - want_dbm) < 0.01 && applied_a >= 0){ tx_str("OK\n"); return; }   // unchanged
        tx_str("OK\n");                                 // reply first, then perform the (slow) atten change
        set_level(dbm);
        return;
    }

    // K <n>: deviation range (kdev): 0 dBFS on the I2S = 0.75 kHz x n (100 = 75 kHz). The Pico is in charge (sends K at every link resync);
    // the conf file only applies as fallback until the first K. Range 10..250. Above 100 only with an active limiter in the
    // bitstream (otherwise ERR nolim: without a limiter an excessive drive could pass unchecked). The limiter ceiling in kHz
    // (ceil_khz) is recomputed at every K, so the FPGA limiter follows kdev automatically.
    if (!strcmp(cmd,"K")){
        int k = atoi(arg);
        if (k < 10 || k > 250){ tx_str("ERR range\n"); return; }
        if (k > KDEV_FIXED && !lim_present){ tx_str("ERR nolim\n"); return; }
        tx_str("OK\n");
        if (!kdev_from_pico || k != kdev_cfg){
            kdev_from_pico = 1; kdev_cfg = k; lim_apply();
            fprintf(stderr, "   -> kdev %d (op verzoek van de Pico; 0 dBFS = %.1f kHz)\n", k, 0.75 * k); fflush(stderr);
        }
        return;
    }
    // H <kHz>: ceiling of the FPGA limiter (peak deviation in kHz, dot or comma as decimal), 20..100 (chosen by the user: the
    // menu may also go above the 70 kHz limit of the conf file). The Pico is in charge (sends H at every link resync); ceil_khz from the
    // conf file is only the fallback until the first H. Not stored in the conf file. An H switches the adaptive ceiling control off
    // (otherwise it would overwrite the chosen ceiling again). ?E ceil= returns the value.
    if (!strcmp(cmd,"H")){
        char hb[32]; snprintf(hb, sizeof hb, "%s", arg);
        for (char *c = hb; *c; c++) if (*c == ',') *c = '.';
        char *e; double h = strtod(hb, &e);
        if (e == hb || !isfinite(h) || h < 20.0 || h > 100.0){ tx_str("ERR range\n"); return; }
        if (!lim_present){ tx_str("ERR nolim\n"); return; }
        tx_str("OK\n");
        if (fabs(h - ceil_user) > 0.005 || !ceil_from_pico){        // the same maximum again (a resync) changes nothing: the guard keeps its ceiling
            int first = !ceil_from_pico;
            ceil_from_pico = 1; ceil_user = h; ceil_tgt = h; guard_act = 0; guard_t = mono(); if (ceil_max_khz < h) ceil_max_khz = h;
            if (first || h < ceil_khz){ ceil_khz = h; lim_apply(); }   // lower: at once; higher: ceil_ramp() follows in steps of 0.5 kHz (no gain step)
            fprintf(stderr, "   -> begrenzerplafond %.2f kHz (op verzoek van de Pico)\n", h); fflush(stderr);
        }
        return;
    }
    // B <0|1>: the FPGA limiter (peak limiting at the mask-derived limit) off / on. The soft fade-in/out on I2S loss stays
    // always on. Without an active limiter in the bitstream (bypassed) -> ERR nolim, so that the Pico can grey out the menu.
    // Off means the Pluto itself has NO mask protection: the Pico limiter (and your ceiling) are then the only ones.
    // G <0|1>: mask protection (guard) off / on. On (default): the daemon lowers the limiter ceiling (H = maximum) as soon as the 5-minute mask margin drops below
    // +3.5 dB and slowly restores it when the margin is ample. Off: the ceiling stays exactly H. ?E shows ceil= (current), cmax= (H) and gd= (on/off/act).
    if (!strcmp(cmd,"G")){
        int on = atoi(arg) ? 1 : 0;
        tx_str("OK\n");
        guard_from_pico = 1;
        if (on != guard_on){
            guard_on = on; guard_t = mono(); if (on) guard_init = 1;
            if (!guard_on && guard_act){ ceil_khz = ceil_user; ceil_tgt = ceil_user; guard_act = 0; lim_apply(); }
            fprintf(stderr, "   -> maskerbescherming %s (op verzoek)\n", guard_on ? "AAN" : "UIT"); fflush(stderr);
        }
        return;
    }
    if (!strcmp(cmd,"B")){
        int on = atoi(arg) ? 1 : 0;
        if (!lim_present){ tx_str("ERR nolim\n"); return; }
        tx_str("OK\n");
        if (on != lim_user){
            lim_user = on;
            wr(R_LCTRL, (lim_user ? 0x1u : 0x0u) | 0x2u);
            fprintf(stderr, "   -> FPGA-begrenzer %s (op verzoek van de Pico)\n", lim_user ? "AAN" : "UIT"); fflush(stderr);
        }
        return;
    }

    // ---------- obsolete commands: fixed or automatic, so OK and ignore ----------
    if (!strcmp(cmd,"O")||!strcmp(cmd,"L")||!strcmp(cmd,"PT")||!strcmp(cmd,"PB")||
        !strcmp(cmd,"PE")||!strcmp(cmd,"CT")||!strcmp(cmd,"CE")||!strcmp(cmd,"C")){
        tx_str("OK\n"); return;
    }
    tx_str("ERR parse\n");
}

// ---- main -------------------------------------------------------------------
// ---- web interface (HTTP on port WEB_PORT): page + JSON API; commands go through the same handle() as the UART ---------------------------------------
#include "web_index.h"
#define WEB_PORT 80
#define WEB_CONF "/mnt/jffs2/skypluto-web.conf"          // optional: password=<password>; without file/password the interface is open (like the Pluto itself)
struct wconn { int fd; double t; int len; char buf[3072]; };
static struct wconn wc[4];
static int    wfd = -1; static double w_retry = 0.0;
static char   wpass[64] = ""; static char wtok[4][33]; static int wtn = 0; static double wconf_t = -100.0;
static void web_conf_read(void){
    wconf_t = mono(); wpass[0] = 0;
    FILE *f = fopen(WEB_CONF, "r"); if (!f) return;
    char ln[128]; while (fgets(ln, sizeof ln, f)){ if (!strncmp(ln, "password=", 9)){ snprintf(wpass, sizeof wpass, "%s", ln + 9); size_t l = strlen(wpass); while (l && (wpass[l-1]=='\n'||wpass[l-1]=='\r')) wpass[--l] = 0; } }
    fclose(f);
}
static void web_send(int fd, int code, const char *ctype, const char *body, size_t n, const char *extra){
    char h[320]; const char *st = code == 200 ? "OK" : code == 400 ? "Bad Request" : code == 401 ? "Unauthorized" : code == 403 ? "Forbidden" : "Not Found";
    int hl = snprintf(h, sizeof h, "HTTP/1.1 %d %s\r\nContent-Type: %s\r\nContent-Length: %zu\r\nCache-Control: no-store\r\nConnection: close\r\n%s\r\n", code, st, ctype, n, extra ? extra : "");
    struct timeval tv; tv.tv_sec = 0; tv.tv_usec = 250000; setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof tv);     // a client that does not read must not stall the main loop (UART replies) for long
    int fl = fcntl(fd, F_GETFL); fcntl(fd, F_SETFL, fl & ~O_NONBLOCK);
    double t0 = mono();                                                      // SO_SNDTIMEO bounds one send(), not the response: stop after 0.6 s in total
    send(fd, h, (size_t)hl, MSG_NOSIGNAL);
    size_t o = 0; while (o < n && mono() - t0 < 0.6){ ssize_t w = send(fd, body + o, n - o, MSG_NOSIGNAL); if (w <= 0) break; o += (size_t)w; }
}
// execute a command as the UART does and fetch the reply
static const char *web_cmd(const char *c){
    char line[192]; snprintf(line, sizeof line, "%s", c);
    for (char *p = line; *p; p++) if (*p == '\r' || *p == '\n') *p = 0;
    cap_len = 0; cap_buf[0] = 0; out_fd = -2; handle(line); out_fd = -1;
    while (cap_len && (cap_buf[cap_len-1] == '\n' || cap_buf[cap_len-1] == '\r')) cap_buf[--cap_len] = 0;
    return cap_buf;
}
static int jadd(char *o, int n, int cap, const char *k, const char *v){          // "k":"v" with escaping
    n += snprintf(o + n, (size_t)(cap - n), "\"%s\":\"", k);
    for (; *v && n < cap - 8; v++){ if (*v == '"' || *v == '\\') o[n++] = '\\'; if ((unsigned char)*v >= 32) o[n++] = *v; }
    n += snprintf(o + n, (size_t)(cap - n), "\"");
    return n;
}
static int web_authed(const char *cookie){
    if (!wpass[0]) return 1;
    if (!cookie) return 0;
    const char *p = strstr(cookie, "plauth="); if (!p) return 0; p += 7;
    for (int i = 0; i < wtn; i++) if (!strncmp(p, wtok[i], 32)) return 1;
    return 0;
}
static void web_handle(struct wconn *c){
    c->buf[c->len] = 0;
    char *hend = strstr(c->buf, "\r\n\r\n"); if (!hend){ return; }
    char method[8] = "", path[96] = ""; sscanf(c->buf, "%7s %95s", method, path);
    int clen = 0; { const char *p = strcasestr(c->buf, "Content-Length:"); if (p) clen = atoi(p + 15); }
    if (clen < 0){ web_send(c->fd, 400, "text/plain", "bad request", 11, NULL); close(c->fd); c->fd = -1; return; }
    char *body = hend + 4; int have = c->len - (int)(body - c->buf);
    if (clen > have) return;                                            // body not complete yet
    if (clen > 400) clen = 400;
    body[clen] = 0;
    char cookie[512] = ""; { const char *p = strcasestr(c->buf, "Cookie:"); if (p){ p += 7; while (*p == ' ') p++; size_t i = 0; while (*p && *p != '\r' && i < sizeof cookie - 1) cookie[i++] = *p++; cookie[i] = 0; } }
    int xrw = strcasestr(c->buf, "X-Requested-With: skypluto") != NULL;
    int post = !strcmp(method, "POST");
    if (mono() - wconf_t > 30.0) web_conf_read();
    static char out[4096];
    if (!strcmp(method, "GET") && !strcmp(path, "/")){ web_send(c->fd, 200, "text/html; charset=utf-8", WEB_INDEX, WEB_INDEX_LEN, NULL); }
    else if (post && !strcmp(path, "/login")){
        if (!xrw){ web_send(c->fd, 403, "text/plain", "no", 2, NULL); }
        else if (!wpass[0] || !strcmp(body, wpass)){
            char tok[33] = ""; unsigned char r[16]; memset(r, 0, sizeof r); int rok = 0; FILE *u = fopen("/dev/urandom", "r"); if (u){ rok = fread(r, 1, 16, u) == 16; fclose(u); }
            if (!rok){ web_send(c->fd, 403, "text/plain", "no", 2, NULL); close(c->fd); c->fd = -1; return; }       // no entropy: no session
            for (int i = 0; i < 16; i++) snprintf(tok + 2*i, 3, "%02x", r[i]);
            if (wtn < 4) wtn++;
            for (int i = wtn - 1; i > 0; i--) memcpy(wtok[i], wtok[i-1], 33);
            memcpy(wtok[0], tok, 33);
            char ex[128]; snprintf(ex, sizeof ex, "Set-Cookie: plauth=%s; Path=/; Max-Age=2592000; HttpOnly; SameSite=Strict\r\n", tok);
            web_send(c->fd, 200, "text/plain", "ok", 2, ex);
        } else web_send(c->fd, 403, "text/plain", "no", 2, NULL);
    }
    else if (!strncmp(path, "/api/", 5)){
        if (!web_authed(cookie)){ web_send(c->fd, 401, "text/plain", "login", 5, NULL); }
        else if (post && !strcmp(path, "/api/cmd")){
            if (!xrw){ web_send(c->fd, 403, "text/plain", "no", 2, NULL); }
            else { const char *r = web_cmd(body); web_send(c->fd, 200, "text/plain; charset=utf-8", r, strlen(r), NULL); }
        }
        else if (!post && !strcmp(path, "/api/state")){
            static const char *k[] = {"S","T","P","E","V","M","D","B","J","R","O","IQ"}; static const char *q[] = {"?S","?T","?P","?E","?V","?M","?D","?B","?J","?R","?O","?IQ"};
            int n = 0; out[n++] = '{';
            for (int i = 0; i < 12; i++){ n = jadd(out, n, (int)sizeof out, k[i], web_cmd(q[i])); out[n++] = ','; }
            char idb[16] = "-"; if (has_dbg){ snprintf(idb, sizeof idb, "%08X", dbg_rd(12)); }
            n = jadd(out, n, (int)sizeof out, "id", idb); out[n++] = ',';
            n += snprintf(out + n, sizeof out - (size_t)n, "\"link\":{\"age\":%.1f,\"cmds\":%lu,\"boot\":", pico_last_rx < 0 ? -1.0 : mono() - pico_last_rx, pico_cmds);
            { char bb[48]; snprintf(bb, sizeof bb, "%d %s", bs_pct, bs_txt); out[n++] = '"'; for (const char *p = bb; *p && n < (int)sizeof out - 8; p++){ if (*p == '"' || *p == '\\') out[n++] = '\\'; out[n++] = *p; } out[n++] = '"'; }
            n += snprintf(out + n, sizeof out - (size_t)n, "}}");
            web_send(c->fd, 200, "application/json", out, (size_t)n, NULL);
        }
        else if (!post && !strcmp(path, "/api/mask")){
            int n = 0; out[n++] = '{'; n = jadd(out, n, (int)sizeof out, "L", web_cmd("?L")); out[n++] = ','; n = jadd(out, n, (int)sizeof out, "W", web_cmd("?W")); out[n++] = ',';
            n = jadd(out, n, (int)sizeof out, "N", web_cmd("?N")); out[n++] = '}';
            web_send(c->fd, 200, "application/json", out, (size_t)n, NULL);
        }
        else if (!post && !strcmp(path, "/api/g")){ int n = 0; out[n++] = '{'; n = jadd(out, n, (int)sizeof out, "G", web_cmd("?G")); out[n++] = '}'; web_send(c->fd, 200, "application/json", out, (size_t)n, NULL); }
        else if (!post && !strcmp(path, "/api/tone")){ int n = 0; out[n++] = '{'; n = jadd(out, n, (int)sizeof out, "A", web_cmd("?A")); out[n++] = '}'; web_send(c->fd, 200, "application/json", out, (size_t)n, NULL); }
        else if (!post && !strcmp(path, "/api/ir.csv")){
            FILE *fi = fopen("/tmp/pluto_ir.csv", "r");
            if (!fi) web_send(c->fd, 404, "text/plain", "no result", 9, NULL);
            else { static char fb[65536]; size_t m = fread(fb, 1, sizeof fb, fi); fclose(fi); web_send(c->fd, 200, "text/csv; charset=utf-8", fb, m, NULL); }
        }
        else web_send(c->fd, 404, "text/plain", "not found", 9, NULL);
    }
    else web_send(c->fd, 404, "text/plain", "not found", 9, NULL);
    close(c->fd); c->fd = -1;
}
// every main loop round: accept new connections and keep reading pending requests (non-blocking)
static void web_poll(void){
    double now = mono();
    if (wfd < 0 && now >= w_retry){
        w_retry = now + 5.0;
        int l = socket(AF_INET, SOCK_STREAM | SOCK_CLOEXEC, 0);
        if (l >= 0){
            int one = 1; setsockopt(l, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
            struct sockaddr_in sa; memset(&sa, 0, sizeof sa); sa.sin_family = AF_INET; sa.sin_port = htons(WEB_PORT); sa.sin_addr.s_addr = htonl(INADDR_ANY);
            if (bind(l, (struct sockaddr *)&sa, sizeof sa) == 0 && listen(l, 4) == 0){ fcntl(l, F_SETFL, O_NONBLOCK); wfd = l; web_conf_read(); }
            else close(l);
        }
    }
    if (wfd < 0) return;
    for (int i = 0; i < 4; i++) if (wc[i].fd <= 0 && wc[i].fd != -1) wc[i].fd = -1;
    int nc;
    while ((nc = accept4(wfd, NULL, NULL, SOCK_CLOEXEC)) >= 0){
        int slot = -1; for (int i = 0; i < 4; i++) if (wc[i].fd < 0 || now - wc[i].t > 5.0){ if (wc[i].fd >= 0) close(wc[i].fd); slot = i; break; }
        if (slot < 0){ close(nc); continue; }
        fcntl(nc, F_SETFL, O_NONBLOCK); wc[slot].fd = nc; wc[slot].t = now; wc[slot].len = 0;
    }
    for (int i = 0; i < 4; i++){
        if (wc[i].fd < 0) continue;
        ssize_t r = recv(wc[i].fd, wc[i].buf + wc[i].len, sizeof wc[i].buf - 1 - (size_t)wc[i].len, 0);
        if (r > 0){ wc[i].len += (int)r; web_handle(&wc[i]); }
        else if (r == 0 || (r < 0 && errno != EAGAIN && errno != EWOULDBLOCK) || now - wc[i].t > 5.0){ close(wc[i].fd); wc[i].fd = -1; }
        else if (wc[i].len >= (int)sizeof wc[i].buf - 1){ close(wc[i].fd); wc[i].fd = -1; }
    }
}

// Client mode: skypluto-ctl -c "<command>" sends a line to the TCP console of the running daemon and shows the reply (the Pluto has no nc).
static int cli_main(const char *cmd){
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    struct sockaddr_in sa; memset(&sa, 0, sizeof sa);
    sa.sin_family = AF_INET; sa.sin_port = htons(TCP_PORT); sa.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (fd < 0 || connect(fd, (struct sockaddr *)&sa, sizeof sa) != 0){ fprintf(stderr, "geen verbinding met de daemon (poort %d)\n", TCP_PORT); return 1; }
    char out[256]; int n = snprintf(out, sizeof out, "%s\n", cmd);
    if (send(fd, out, (size_t)n, MSG_NOSIGNAL) != n) return 1;
    struct timeval tv; tv.tv_sec = 3; tv.tv_usec = 0; setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
    char buf[512]; int o = 0; ssize_t r;
    while (o < (int)sizeof buf - 1 && (r = recv(fd, buf + o, sizeof buf - 1 - (size_t)o, 0)) > 0){ o += (int)r; if (buf[o-1] == '\n') break; }
    buf[o] = 0; fputs(buf, stdout); close(fd);
    return o > 0 ? 0 : 2;
}

// Send a one-off boot line without a daemon (from autorun.sh): skypluto-ctl -b "10 Linux gestart"
static int bootmsg_main(const char *arg){
    int pct = atoi(arg); const char *t = arg; while (*t && *t != ' ') t++; while (*t == ' ') t++;
    int fd = open("/dev/mem", O_RDWR|O_SYNC);
    if (fd < 0) return 1;
    g_map = mmap(NULL, MAP_LEN, PROT_READ|PROT_WRITE, MAP_SHARED, fd, MAP_BASE);
    if (g_map == MAP_FAILED) return 1;
    bs_set(pct < 0 ? 0 : pct > 100 ? 100 : pct, t);
    bs_send();
    return 0;
}

int main(int argc, char **argv){
    if (argc >= 3 && !strcmp(argv[1],"-c")) return cli_main(argv[2]);
    if (argc >= 3 && !strcmp(argv[1],"-b")) return bootmsg_main(argv[2]);
    if (argc >= 4 && !strcmp(argv[1],"--measure"))
        return measure_main(atoll(argv[2]), atoi(argv[3]));

    int fd = open("/dev/mem", O_RDWR|O_SYNC);
    if (fd < 0){ perror("open /dev/mem"); return 1; }
    g_map = mmap(NULL, MAP_LEN, PROT_READ|PROT_WRITE, MAP_SHARED, fd, MAP_BASE);
    if (g_map == MAP_FAILED){ perror("mmap"); return 1; }
    fprintf(stderr,"skypluto-ctl 1.02: WFM magic=%08X (verwacht 57464D32)\n", rd(R_MAGIC));

    // fixed modulation settings
    uint32_t off_hw = rd(R_OFFSET) & 0xFFFFFF;                         // the NCO offset a running transmitter has (the register survives a restart of the daemon)
    { uint32_t id = has_dbg ? dbg_rd(12) : 0; has_iq = (id >= 0xB1D00019u && id <= 0xB1D000FFu); }
    iq_conf_read(); iq_apply();
    pw_conf_read();
    lim_conf_read(); lim_probe(); lim_apply();          // kdev (from conf, without limiter never > 100) + limiter ceiling
    // The transmitter starts CLOSED (tx_off = 1): attenuation at maximum and TX-LO off, until the Pico sends a tune (F) (and E is not 0).
    fprintf(stderr, "skypluto-ctl: limiter %s, kdev %d, grens %.1f kHz (max %.1f), guard %d\n",
            lim_present ? "aanwezig" : (lim_bypass ? "ONTBREEKT (omzeild)" : "ONTBREEKT"), kdev_cfg, ceil_khz, ceil_max_khz, guard_on);
    lim_next = mono() + LIM_POLL;

    // adopt the current chip state (without -q, because -q suppresses output)
    { char t[64];
      if (read_cmd_line("iio_attr -o -c ad9361-phy voltage0 hardwaregain 2>/dev/null", t, sizeof t)==0 && t[0]) applied_a = quant(-atof(t));
    }
    if (applied_a >= 0){ nom_a = applied_a; want_dbm = DBM_AT_0DB - applied_a; }
    if (applied_a >= 0 && applied_a < ATTEN_FLOOR - 0.01){                 // chip is (still) above the maximum: bring back
        fprintf(stderr, "skypluto-ctl: atten %.2f < %.2f (P_MAX %.1f dBm) -> begrensd\n", applied_a, ATTEN_FLOOR, P_MAX_DBM);
        nom_a = ATTEN_FLOOR; want_dbm = P_MAX_DBM; apply_atten(ATTEN_FLOOR);
    }
    {   // restart of the daemon alone (the file still exists then): take over the open transmitter instead of closing it, otherwise the RF drops out for ~5 s on every update
        long long ff = 0; FILE *tf = fopen("/tmp/skypluto-tuned", "r");
        if (tf){ if (fscanf(tf, "%lld", &ff) != 1) ff = 0; fclose(tf); }
        if (ff >= 70000000LL && applied_a >= 0 && applied_a < ATTEN_MUTE - 1.0){
            last_f = ff; tuned = 1; want_on = 1; tx_off = 0; lo_pd = 0;
            { int32_t inc = (off_hw & 0x800000) ? (int32_t)(off_hw | 0xFF000000u) : (int32_t)off_hw;        // the LO is where the running NCO offset says it is
              lowif_khz = (double)inc * 12288000.0 / 16777216.0 / 1000.0; if (fabs(lowif_khz) < 0.0005) lowif_khz = 0.0; }
            fprintf(stderr, "skypluto-ctl: herstart: zender blijft OPEN (f=%lld, atten %.2f dB)\n", ff, applied_a); fflush(stderr);
            apply_current();
        } else { nco_offset_apply(); apply_current(); tx_lo_power(0); }       // cold start: transmitter closed: atten max + TX-LO off (no RF until the first F from the Pico); the stored low-IF offset applies
    }
    find_temp_path(); refresh_temp(); temp_last = mono();
    fprintf(stderr,"skypluto-ctl: zender %s; f=%lld atten=%.2f temp=%.1f (%s)\n", tx_off ? "DICHT tot tune (F) van de Pico" : "OPEN (overgenomen)", last_f, applied_a, temp_mC/1000.0,
            temp_path[0]?temp_path:"sysfs-pad niet gevonden");

    // real-time: a reply must never be scheduled away halfway (TX FIFO underrun)
    { struct sched_param sp; sp.sched_priority = 10;
      if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0) perror("sched_setscheduler (verder zonder RT)");
      mlockall(MCL_CURRENT|MCL_FUTURE); }

    signal(SIGPIPE, SIG_IGN);
    t_start = mono();
    wr(U_CLR, 1);
    while (rx_byte() >= 0) { /* drain */ }
    m_next = mono() + 10.0;              // first ALC measurement only once the Pico has set its setpoint
    mask_next = mono() + 10.0;           // first mask measurement once the ALC has a reference
    { int en, rx; char port[40]; read_conf(&en, &rx, port, sizeof port); }   // twin=, rx=, ...
    twin_apply();                        // twin: TX2 at the attenuation of TX1 (>= TX2_MIN_ATTEN) or closed

    // TCP console (localhost only, port TCP_PORT): same command protocol as the UART (docs/serial-protocol.md), for manual
    // use and tests without a Pico: echo "?M" | nc 127.0.0.1 5555  (or skypluto-cmd.sh). One client at a time; replies go only to that client.
    // SOCK_CLOEXEC: the children (iio_readdev, mask tool) must not inherit the listening port, otherwise the port stays busy after a daemon restart.
    // If bind fails (e.g. still busy), it is retried every 5 s.
    int lfd = -1, cfd = -1; char cline[192]; int cidx = 0; double lfd_retry = 0.0;

    bs_t0 = mono(); bs_next = bs_t0 + 0.4; bs_phase = 0; bs_set(55, "Besturing v" SW_VERSION);
    for (int i = 0; i < 4; i++) wc[i].fd = -1;
    char line[192]; int idx = 0;
    for (;;){
        int got = 0, b;
        if (lfd < 0 && mono() >= lfd_retry){
            lfd_retry = mono() + 5.0;
            int l = socket(AF_INET, SOCK_STREAM | SOCK_CLOEXEC, 0);
            if (l >= 0){
                int one = 1; setsockopt(l, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
                struct sockaddr_in sa; memset(&sa, 0, sizeof sa);
                sa.sin_family = AF_INET; sa.sin_port = htons(TCP_PORT); sa.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
                if (bind(l, (struct sockaddr *)&sa, sizeof sa) == 0 && listen(l, 1) == 0){ fcntl(l, F_SETFL, O_NONBLOCK); lfd = l; }
                else close(l);
            }
        }
        if (lfd >= 0){
            if (cfd < 0){
                cfd = accept4(lfd, NULL, NULL, SOCK_CLOEXEC);
                if (cfd >= 0){ fcntl(cfd, F_SETFL, O_NONBLOCK); cidx = 0; }
            } else {
                char ch; ssize_t r;
                while ((r = recv(cfd, &ch, 1, 0)) > 0){
                    got = 1;
                    if (ch == '\r') continue;
                    if (ch == '\n'){
                        cline[cidx] = 0; out_fd = cfd; handle(cline); out_fd = -1; cidx = 0;
                    } else if (cidx < (int)sizeof cline - 1) cline[cidx++] = ch;
                    else cidx = 0;
                }
                if (r == 0 || (r < 0 && errno != EAGAIN && errno != EWOULDBLOCK)){ close(cfd); cfd = -1; }
            }
        }
        while ((b = rx_byte()) >= 0){
            got = 1; bs_active = 0; pico_last_rx = mono();                                // the Pico is speaking: no more unsolicited boot lines
            if (b == '\r') continue;
            if (b == '\n'){
                line[idx]=0;
                char head[12]; snprintf(head, sizeof head, "%s", line);
                double t0 = mono();
                pico_cmds++;
                handle(line);
                double dt = mono() - t0;                      // line in -> reply sent
                if (dt > h_max) h_max = dt;
                if (dt > 0.1){ fprintf(stderr,"   !! traag: '%s' %.0f ms\n", head, dt*1000.0); fflush(stderr); }
                idx=0;
            }
            else if (idx < (int)sizeof(line)-1) line[idx++]=(char)b;
            else idx = 0;
        }
        if (rd(U_ST) & 0x8) wr(U_CLR, 1);
        double now = mono();
        if (bs_phase < 3 && now - bs_t0 >= (bs_phase == 0 ? 0.4 : bs_phase == 1 ? 1.4 : 2.4)){
            bs_phase++;
            if (bs_phase == 2) bs_set(60, "Wacht op tune");
            if (bs_phase == 3) bs_set(100, "Gereed v" SW_VERSION);
            bs_next = 0.0;                                         // step change: send immediately
        }
        if (bs_active && bs_phase >= 1 && now >= bs_next){ bs_next = now + 1.0; bs_send(); }
        if (now - temp_last >= 5.0){ refresh_temp(); temp_last = now; }
        { static double h_log = 0.0;                          // log the longest processing time every 60 s
          if (now - h_log >= 60.0){ h_log = now; fprintf(stderr,"   .. max verwerkingstijd tot nu: %.1f ms\n", h_max*1000.0); fflush(stderr); } }
        measure_poll();
        web_poll();
        led_poll(now);
        ceil_ramp();
        if (pw_en){                                                        // power meter: initialise RX1; without a mask stream a short capture of its own
            if (!pw_inited && now >= pw_next){ pw_next = now + 5.0; pw_rx1_init(); }
            if (pw_pid > 0){
                char tmpb[96]; ssize_t r = read(pw_fd, tmpb, sizeof tmpb - 1);
                if (r > 0 && pw_blen + r < (int)sizeof pw_buf - 1){ memcpy(pw_buf + pw_blen, tmpb, (size_t)r); pw_blen += (int)r; pw_buf[pw_blen] = 0; }
                int stw; pid_t w = waitpid(pw_pid, &stw, WNOHANG);
                if (w == pw_pid || now - pw_started > 8.0){
                    if (w != pw_pid){ kill(pw_pid, SIGKILL); waitpid(pw_pid, &stw, 0); }
                    double pp, kk; if (sscanf(pw_buf, "PWR p=%lf pk=%lf", &pp, &kk) == 2) pw_feed(pp, kk);
                    close(pw_fd); pw_fd = -1; pw_pid = -1; pw_blen = 0; pw_next = now + 0.5;
                }
            } else if (pw_inited && !tx_off && last_f > 0 && m_pid <= 0 && cal_pid <= 0 && !cal_pending && ir_pid <= 0 && now >= pw_next && now - lvl_change_t > 1.0){
                int pf[2];
                if (pipe(pf) == 0){
                    char a1[32]; snprintf(a1, sizeof a1, "%lld", last_f);
                    pid_t p = fork();
                    if (p == 0){ dup2(pf[1], 1); close(pf[0]); close(pf[1]); execl(MASKBIN, MASKBIN, "--pwr", a1, "0.1", (char*)NULL); _exit(127); }
                    close(pf[1]);
                    if (p < 0){ close(pf[0]); pw_next = now + 2.0; }
                    else { fcntl(pf[0], F_SETFL, O_NONBLOCK); pw_pid = p; pw_fd = pf[0]; pw_started = now; pw_blen = 0; pw_buf[0] = 0; }
                } else pw_next = now + 2.0;
            }
        }
        if (ir_pid > 0){ int st; if (waitpid(ir_pid, &st, WNOHANG) == ir_pid){ ir_rc = WIFEXITED(st) ? WEXITSTATUS(st) : 98; ir_pid = -1; ir_finish();
            fprintf(stderr, "   -> impulsmeting klaar (rc %d)\n", ir_rc); fflush(stderr); } else if (mono() - ir_t0 > ir_secs * 1.5 + 120.0){ kill(ir_pid, SIGKILL); } }
        if (now >= lim_next){ lim_next = now + LIM_POLL; lim_conf_read(); lim_apply(); lim_poll(); }
        { static double dev_next = 0.0; if (has_dbg && now >= dev_next){ dev_next = now + 0.08; dev_poll(); } }   // fetch the peak-deviation windows ~12x per second (8 x 20 ms history)
        if (cal_pid > 0){
            int stc; pid_t cw = waitpid(cal_pid, &stc, WNOHANG);
            if (cw == cal_pid || now - cal_t0 > 15.0){
                int cok = (cw == cal_pid && WIFEXITED(stc) && WEXITSTATUS(stc) == 0);
                if (cw != cal_pid){ kill(cal_pid, SIGKILL); waitpid(cal_pid, &stc, 0); }
                cal_pid = -1;
                if (cok && cal_expect_f > 0 && phy_dir[0]){                       // the LO must really be on the requested frequency
                    char lp[140], fb[32] = ""; snprintf(lp, sizeof lp, "%s/out_altvoltage1_TX_LO_frequency", phy_dir);
                    FILE *lf = fopen(lp, "r"); long long got = 0; if (lf){ if (fgets(fb, sizeof fb, lf)) got = atoll(fb); fclose(lf); }
                    long long dd = got - cal_expect_f; if (dd < 0) dd = -dd;
                    if (dd > 1000){ cok = 0; fprintf(stderr, "   !! TX-LO staat op %lld Hz, verwacht %lld Hz\n", got, cal_expect_f); fflush(stderr); }
                }
                fprintf(stderr, "   -> TX-kalibratie (LO-lek, beeld) %s na %.1f s\n", cok ? "klaar" : "MISLUKT", now - cal_t0); fflush(stderr);
                if (!cok && !tx_off){
                    if (++cal_fail < 3){ cal_pending = 1; cal_retune = 1; cal_due = now + 0.5; }                // try again, still muted
                    else { cal_failed = 1; cal_failed_t = now; cal_pending = 0; fprintf(stderr, "   !! TX-kalibratie mislukt: de zender blijft dicht, nieuwe poging over 30 s (of eerder met F, E of CAL)\n"); fflush(stderr); }
                }
                if (cok){ cal_fail = 0; cal_failed = 0; cal_last_end = now; cal_temp_ref = temp_mC; }
                { char lp[140] = ""; FILE *lf; if (phy_dir[0]) snprintf(lp, sizeof lp, "%s/out_altvoltage1_TX_LO_powerdown", phy_dir);
                  if (!tx_off && lp[0] && (lf = fopen(lp, "r")) != NULL){ int pdv = 0; if (fscanf(lf, "%d", &pdv) != 1) pdv = 0; fclose(lf); if (pdv) tx_lo_power(1); } }   // LO still powered down: power it up
                if (cok && !cal_pending){ cal_hold = 0; apply_current(); alc_reset(ALC_SETTLE);
                                   fprintf(stderr, "   -> zender weer open (atten TX1 %.2f dB, TX2 %.2f dB)\n", applied_a, applied_a2); fflush(stderr); }
            }
        } else if (cal_pending && !tx_off && last_f > 0 && now >= cal_due){
            if (ir_pid > 0){                                      // a J/Y measurement is running: skip the calibration, release the output
                if (cal_retune == 2) tx_lo_power(1);
                cal_pending = 0; cal_retune = 0; cal_hold = 0; apply_current();
            } else {
                if (m_pid > 0) measure_finish();                  // the running mask/ALC measurement uses the RX path and would see the calibration: stop it, it restarts afterwards
                if (pw_pid > 0) kill(pw_pid, SIGKILL);            // one-shot power capture on RX1: reaped by the power-meter code below
                char cs[900], mt[160] = "";
                if (phy_find()[0]){                               // direct sysfs writes: no gap between the retune and the calibration; && chain: any failure shows in the exit status
                    if (cal_retune == 2) snprintf(mt, sizeof mt, "echo 0 > $D/out_altvoltage1_TX_LO_powerdown && ");
                    char fq[96] = ""; if (cal_retune) snprintf(fq, sizeof fq, "echo %lld > $D/out_altvoltage1_TX_LO_frequency && ", lo_freq());
                    snprintf(cs, sizeof cs, "D=%s; echo -%.2f > $D/out_voltage0_hardwaregain && echo -%.2f > $D/out_voltage1_hardwaregain && %s%secho rf_dc_offs > $D/calib_mode && echo tx_quad > $D/calib_mode",
                             phy_dir, ATTEN_MUTE, ATTEN_MUTE, mt, fq);
                } else {
                    if (cal_retune) snprintf(mt, sizeof mt, "iio_attr -q -o -c ad9361-phy altvoltage1 frequency %lld >/dev/null 2>&1 && ", lo_freq());
                    snprintf(cs, sizeof cs,
                             "iio_attr -q -o -c ad9361-phy voltage0 hardwaregain -- -%.2f >/dev/null 2>&1 && iio_attr -q -o -c ad9361-phy voltage1 hardwaregain -- -%.2f >/dev/null 2>&1 && "
                             "%siio_attr -d -q ad9361-phy calib_mode rf_dc_offs >/dev/null 2>&1 && iio_attr -d -q ad9361-phy calib_mode tx_quad >/dev/null 2>&1",
                             ATTEN_MUTE, ATTEN_MUTE, mt);
                }
                pid_t cp = fork();
                if (cp == 0){ prctl(PR_SET_PDEATHSIG, SIGKILL); execl("/bin/sh", "sh", "-c", cs, (char *)NULL); _exit(127); }
                if (cp > 0){
                    cal_pid = cp; cal_t0 = now; cal_pending = 0; cal_try = 0; cal_expect_f = cal_retune ? lo_freq() : 0; if (cal_retune == 2) lo_pd = 0; cal_retune = 0;
                    applied_a = ATTEN_MUTE; applied_a2 = ATTEN_MUTE;      // the child mutes TX1 and TX2 first
                    fprintf(stderr, "   -> TX-kalibratie gestart (f=%lld, zender dicht)\n", last_f); fflush(stderr);
                } else if (++cal_try <= 3) cal_due = now + 2.0;
                else { cal_pending = 0; if (cal_retune == 2) tx_lo_power(1); cal_retune = 0; cal_hold = 0; apply_current(); cal_try = 0;
                       fprintf(stderr, "   !! TX-kalibratie kon niet starten (fork), zender vrijgegeven zonder kalibratie\n"); fflush(stderr); }
            }
        }
        if (cal_failed && !tx_off && cal_pid <= 0 && !cal_pending && now - cal_failed_t >= 30.0){        // retry: a muted transmitter must not stay muted for good
            cal_failed = 0; cal_fail = 0; cal_hold = 1; cal_hold_t = now; cal_pending = 1; cal_retune = 1; cal_due = now;
            fprintf(stderr, "   .. TX-kalibratie opnieuw geprobeerd\n"); fflush(stderr);
        }
        if (cal_hold && cal_pid <= 0 && !cal_pending && !cal_failed && now - cal_hold_t > 30.0){ cal_hold = 0; apply_current(); }   // safety net: do not stay muted for ever (not after a failed calibration)
        // temperature-driven recalibration (done here, muted, instead of by a separate script): the chip's LO-leak/image calibration drifts with the die temperature
        if (now - cal_temp_chk >= 15.0){
            cal_temp_chk = now;
            int dT = temp_mC - cal_temp_ref; if (dT < 0) dT = -dT;
            if (!tx_off && cal_pid <= 0 && !cal_pending && !cal_hold && cal_temp_ref != 0 && dT >= 3000 && now - cal_last_end > 300.0) cal_drift++; else cal_drift = 0;
            if (cal_drift >= 4 && ir_pid <= 0){ cal_drift = 0; cal_hold = 1; cal_hold_t = now; cal_pending = 1; cal_retune = 0; cal_due = now;
                fprintf(stderr, "   -> temperatuurdrift %.1f C: TX-kalibratie\n", dT / 1000.0); fflush(stderr); }
            if (cal_temp_ref == 0 && temp_mC != 0 && !tx_off) cal_temp_ref = temp_mC;
        }
        // while the transmitter is closed the output must really be off: verify the TX attenuation and the TX-LO about once a second and repair them (a failed write is otherwise never retried)
        { static double rc_t = 0.0;
          if (tx_off && phy_dir[0] && now - rc_t >= 1.0){
            rc_t = now; char pth[140], buf[40]; FILE *rf; double a0 = 0.0;
            snprintf(pth, sizeof pth, "%s/out_voltage0_hardwaregain", phy_dir);
            if ((rf = fopen(pth, "r")) != NULL){ if (fgets(buf, sizeof buf, rf)) a0 = atof(buf); fclose(rf); if (a0 > -89.5){ char mv[16]; snprintf(mv, sizeof mv, "-%.2f", ATTEN_MUTE); phy_write("out_voltage0_hardwaregain", mv); phy_write("out_voltage1_hardwaregain", mv); applied_a = ATTEN_MUTE; applied_a2 = ATTEN_MUTE; fprintf(stderr, "   !! zender dicht maar demping was %.2f dB: hersteld\n", a0); fflush(stderr); } }
            snprintf(pth, sizeof pth, "%s/out_altvoltage1_TX_LO_powerdown", phy_dir);
            if ((rf = fopen(pth, "r")) != NULL){ int pd = 1; if (fscanf(rf, "%d", &pd) != 1) pd = 1; fclose(rf); if (pd == 0 && cal_pid <= 0){ phy_write("out_altvoltage1_TX_LO_powerdown", "1"); lo_pd = 1; fprintf(stderr, "   !! zender dicht maar de TX-LO stond aan: uitgezet\n"); fflush(stderr); } }
          } }
        if (m_pid <= 0 && cal_pid <= 0 && !cal_pending && !tx_off && last_f > 0 && now >= m_next) measure_start();
        // mask monitor: only if no ALC measurement is running and the level/frequency has been stable for ~4 s. NOT dependent on an ALC reference:
        // at low TX levels (high attenuation) the TX_MONITOR is too weak for the ALC, and the mask monitor would otherwise never start.
        if (m_pid <= 0 && cal_pid <= 0 && !cal_pending && !tx_off && last_f > 0 && now - lvl_change_t >= 4.0 && now >= mask_next){
            if (!mask_start()) mask_next = now + MASK_PERIOD;
        }
        if (!got) usleep(1000);
    }
    return 0;
}
