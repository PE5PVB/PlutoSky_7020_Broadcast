// SPDX-License-Identifier: Apache-2.0
// DAB transmission (ETSI EN 300 401), mode I: FIC generation, MSC sub-channels with EEP-A protection, energy dispersal,
// convolutional coding with puncturing, time and frequency interleaving, DQPSK and OFDM at 2.048 MS/s.
#ifndef DAB_MOD_H
#define DAB_MOD_H

#include <stdint.h>

#define DAB_MAX_SVC        8
#define DAB_FRAME_SAMPLES  196608          // 96 ms at 2.048 MS/s (null symbol + 76 OFDM symbols)

typedef struct {
    uint16_t sid;                          // service identifier (programme: 16 bit, country code in the top nibble)
    char     label[17];                    // up to 16 characters
    int      bitrate;                      // kbit/s, multiple of 8
    int      prot_level;                   // 1..4 = EEP-1A..4A (3A = rate 1/2, the usual one), 5..8 = EEP-1B..4B (bitrate a multiple of 32)
    int      pty;                          // programme type (0 = none)
    int      sls;                          // 1: the PAD carries a MOT slideshow (signalled in FIG 0/13, else radios do not show it)
} dab_service;

typedef struct {
    uint16_t eid;                          // ensemble identifier
    char     label[17];
    uint8_t  ecc;                          // extended country code (Netherlands: 0xE3)
    int      lto_halfhours;                // local time offset in half hours (+2 = UTC+1)
    int      nsvc;
    dab_service svc[DAB_MAX_SVC];          // one DAB+ audio sub-channel per service, sub-channel id = index
} dab_ensemble;

typedef struct { float re, im; } dab_cf;

typedef struct dab_mod dab_mod;

dab_mod *dab_mod_open(const dab_ensemble *ens);
void     dab_mod_close(dab_mod *m);
// bytes of one 24 ms logical frame of service i (= 3 * bitrate)
int      dab_mod_lf_bytes(const dab_mod *m, int i);
// Push one CIF (24 ms): lf[i] = the next logical frame of service i. Returns 1 when a transmission frame (4 CIFs) is
// complete; then dab_mod_frame() gives its DAB_FRAME_SAMPLES complex samples (RMS amplitude about 1).
int      dab_mod_push_cif(dab_mod *m, const uint8_t *const lf[]);
const dab_cf *dab_mod_frame(const dab_mod *m);
// the same frame in the form the FPGA takes: 76 symbols x 96 words, each word 16 carriers x 2-bit phase index p
// (carrier a: phase 45 degrees x (l + 2p) in symbol l; carrier order +1..+768, -768..-1); the null symbol is not included
const uint32_t *dab_mod_phase_words(const dab_mod *m);
#define DAB_PHASE_WORDS (76 * 96)
// 1: build only the phase words (dab_mod_frame() is then not filled); saves the IFFTs when the FPGA does the OFDM
void     dab_mod_set_phases_only(dab_mod *m, int on);
// Multiplex reconfiguration (EN 300 401, 6.5): the next ensemble (services added, removed or changed: bitrates, protection,
// service identifiers; a service that keeps SId, bitrate and protection runs on without a gap) is announced in FIG 0/0 (change flags, occurrence change) and FIG 0/1 / 0/2 with C/N = next, and
// takes effect with the CIF whose count is at_cif (0..4999; at most 6 s = 250 CIFs ahead). From then on dab_mod_lf_bytes()
// gives the new sizes. -1: a change is still pending, or nothing changes, or it does not fit.
int      dab_mod_reconfigure(dab_mod *m, const dab_ensemble *next, int at_cif);
int      dab_mod_change_pending(const dab_mod *m);
int      dab_mod_cif_count(const dab_mod *m);                 // the count of the next CIF pushed (0..4999)
// the capacity units of one sub-channel (of the 864), -1 when the combination is not allowed
int      dab_cu_size(int bitrate, int prot);
// labels, slideshow flags and programme types of the same services (same order and SIds): at once, no reconfiguration
int      dab_mod_set_meta(dab_mod *m, const dab_ensemble *e);
// labels: take effect at once (FIG 1/0, 1/1), no reconfiguration; NULL leaves one unchanged
void     dab_mod_set_labels(dab_mod *m, const char *ensemble, const char *service);
// capacity units used by the sub-channels (of 864)
int      dab_mod_cu_used(const dab_mod *m);

#endif
