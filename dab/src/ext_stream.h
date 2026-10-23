// SPDX-License-Identifier: Apache-2.0
// An already encoded DAB+ service from outside, carried as a second sub-channel: complete audio super frames (with RS and PAD)
// as ODR-AudioEnc sends them over ZeroMQ ("-o tcp://<pluto>:<port>"; a 12-byte header per super frame: version 1, encoder,
// data size, audio levels). The super frames are buffered against network jitter; when none is there a silent one of the
// same bitrate goes out, and the difference between the remote encoder's clock and ours is taken up by dropping or repeating
// a super frame now and then (as ODR-DabMux does).
#ifndef EXT_STREAM_H
#define EXT_STREAM_H

#include <stdint.h>

typedef struct ext_stream ext_stream;

typedef struct {
    unsigned long received, used, silent, dropped, bad;   // super frames
    int fill;                                              // super frames waiting
    int level_l, level_r;                                  // the encoder's last audio levels (0..32767)
    const char *peer;                                      // the sender, or NULL
} ext_stats;

// config file: port=<tcp port> br=<kbit/s> sid=<hex> label=<up to 16 chars> [sls=0|1, default 1] [file=<recorded messages, for the simulation>]
ext_stream *ext_open(const char *conf_file);
void        ext_close(ext_stream *e);
int         ext_kbps(const ext_stream *e);
int         ext_sls(const ext_stream *e);                // 1: its PAD carries a slideshow (signalled in FIG 0/13); sls=0 in the file turns it off
unsigned    ext_sid(const ext_stream *e);
const char *ext_label(const ext_stream *e);
void        ext_poll(ext_stream *e);                       // I/O side: take what arrived (non-blocking)
const uint8_t *ext_next(ext_stream *e);                    // mux side: the next super frame (120 * kbps / 8 bytes), never NULL
void        ext_get_stats(const ext_stream *e, ext_stats *s);

#endif
