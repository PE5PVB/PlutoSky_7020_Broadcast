// SPDX-License-Identifier: Apache-2.0
// The multiplex layout: one file, used by skypluto-dab (what goes on the air) and skypluto-ctl (web interface, encoder commands).
//
//   eid=8FED                         ensemble identifier (hex)
//   label=PlutoSky DAB               ensemble label (1..16 characters)
//   ecc=E3                           extended country code (hex)
//   lto=2                            local time offset in half hours
//   svc=i2s;8F01;PicoAudio;192;3A;0              one line per service, in sub-channel order:
//   svc=zmq:2491;8F02;MPB RADIO 1;128;3A;1         source;SId;label;kbit/s;protection;slideshow
//
// source: i2s (the stereo I2S input, encoded here), tone:<Hz> (a test tone, encoded here), zmq:<port> (DAB+ super frames from
// ODR-AudioEnc over ZeroMQ, passed on unchanged). At most one service is encoded here (i2s or tone).
// protection: 1A..4A (EEP-A, any multiple of 8 kbit/s) or 1B..4B (EEP-B, multiples of 32 kbit/s). Capacity: 864 CU.
#ifndef MUX_CONF_H
#define MUX_CONF_H

#include <stddef.h>

#define MUX_MAX_SVC 8
enum { MUX_SRC_I2S = 1, MUX_SRC_TONE = 2, MUX_SRC_ZMQ = 3 };

typedef struct {
    int src, port, tone_hz;
    unsigned sid;
    char label[17];
    int kbps, prot;            // prot 1..4 = EEP-1A..4A, 5..8 = EEP-1B..4B
    int sls;
} mux_service;

typedef struct {
    unsigned eid, ecc;
    int lto;
    char label[17];
    int n;
    mux_service s[MUX_MAX_SVC];
} mux_conf;

int  mux_cu(int kbps, int prot);                                   // CUs of one sub-channel, -1 if not allowed
int  mux_cu_total(const mux_conf *c);
// parse and check a layout; 0 = ok, else -1 with the reason in err
int  mux_parse(const char *text, mux_conf *c, char *err, size_t errlen);
int  mux_read(const char *file, mux_conf *c, char *err, size_t errlen);
int  mux_format(const mux_conf *c, char *out, size_t outlen);      // the file text; its length, -1 if it does not fit
int  mux_write(const char *file, const mux_conf *c);               // written whole and renamed
int  mux_local(const mux_conf *c);                                 // index of the service encoded here, -1 if none
void mux_default(mux_conf *c);                                     // one service: the I2S input

#endif
