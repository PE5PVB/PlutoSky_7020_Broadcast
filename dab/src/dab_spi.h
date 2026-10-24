// SPDX-License-Identifier: Apache-2.0
// Service and Programme Information (SPI, the DAB EPG), binary encoding of the basic profile (ETSI TS 102 371 V3.3.1, clause 5
// and annexes D/E): a tree of tag-length-value elements and attributes. Two kinds of object are built here:
//   - the Service Information (SI) object: serviceInformation > ensemble (ECC + EId, short/medium name) > service (short and
//     medium name, bearer dab:<ecc>.<eid>.<sid>.0) per service of the ensemble;
//   - one Programme Information (PI) object per service and day: epg > schedule (version) > scope (start/stop time,
//     serviceScope) and programme (shortId, mediumName, longName, location > time (UTC + duration), mediaDescription >
//     shortDescription).
// Times are UTC with the local time offset (clause 5.4.5.2, the same date and time form as FIG 0/10). The objects are carried
// by MOT in directory mode (dab_mot.h, dab_motdir) with ContentType 7 / ContentSubType 0 (SI) or 1 (PI) and the parameters of
// clause 6.4 (ScopeStart, ScopeEnd, ScopeID: dab_spi_param_*).
#ifndef DAB_SPI_H
#define DAB_SPI_H

#include <stdint.h>

#define DAB_SPI_MAX_OBJECT 16384           // basic profile objects: at most 16 kbytes (TS 102 371 6.2)
// The SPI in X-PAD: X-PAD application types 16 (start of a data group) and 17 (continuation), the first pair of the user
// defined range 16..30 (EN 300 401 7.4.3: 12..15 are for the "primary" MOT application, here the slideshow; 4..11 stay free),
// signalled in FIG 0/13 with user application type 0x007 (SPI), DSCTy 60 (MOT) and the profile list {0x01 basic}
#define DAB_SPI_XPAD_APP   16
#define DAB_SPI_UA_TYPE    0x007
#define DAB_SPI_NO_LTO     (-1000)         // a time without the local time offset (LTO flag 0)

// ---- the TLV writer -----------------------------------------------------------------------------------------------------------
typedef struct {
    uint8_t *b;
    int cap, len, err;                     // err: something did not fit (the result is then unusable)
    int open[16], depth;                   // start of each element still open (its tag byte)
} dab_spi_w;

void dab_spi_init(dab_spi_w *w, uint8_t *buf, int cap);
void dab_spi_open(dab_spi_w *w, int tag);                            // an element; children and attributes follow
void dab_spi_close(dab_spi_w *w);                                    // its length (1 byte, or 0xFE + 16 bit, 0xFF + 24 bit)
void dab_spi_attr(dab_spi_w *w, int tag, const void *data, int n);   // an attribute (or CDATA with tag 0x01)
void dab_spi_attr_uint(dab_spi_w *w, int tag, unsigned v, int bytes);
void dab_spi_text(dab_spi_w *w, int tag, const char *utf8);          // element <tag> holding the text as CDATA
// the result: its length, -1 when it did not fit or an element is still open
int  dab_spi_end(dab_spi_w *w);

// ---- field encodings ----------------------------------------------------------------------------------------------------------
// timePoint (5.4.5.2): utc_s = seconds since 1970 (UTC); long form (with seconds) when long_form; lto in half hours (-28..28) or
// DAB_SPI_NO_LTO. Writes 4..7 bytes, returns the count.
int  dab_spi_timepoint(uint8_t *o, long long utc_s, int lto_halfhours, int long_form);
// bearerURI in the dab: domain (5.4.5.1.2) for a programme service (16-bit SId): 6 bytes
int  dab_spi_bearer(uint8_t *o, unsigned ecc, unsigned eid, unsigned sid, unsigned scids);

// ---- objects ------------------------------------------------------------------------------------------------------------------
typedef struct {
    unsigned sid;
    const char *name;                      // the service label (medium name, up to 16 characters; short = first 8)
} dab_spi_service;

typedef struct {
    long long start;                       // UTC, seconds since 1970
    int dur_s;                             // 1..65535 s
    unsigned short_id;                     // 24 bit, unique within the carousel
    const char *mname, *lname, *desc;      // medium name (<= 16 characters), long name (<= 128), description (<= 180, may be
                                           // empty or NULL); UTF-8
} dab_spi_prog;

// The SI object: returns its length, or -1 when it does not fit cap.
int dab_spi_si(uint8_t *out, int cap, unsigned version, unsigned ecc, unsigned eid, const char *ens_name,
               const dab_spi_service *svc, int nsvc);
// The PI object of one service: the programmes in order of start time. scope_start/scope_end: UTC (seconds) of the start of
// the first and the end of the last programme. Returns its length, or -1 when it does not fit cap.
int dab_spi_pi(uint8_t *out, int cap, unsigned version, unsigned ecc, unsigned eid, unsigned sid, int lto_halfhours,
               long long scope_start, long long scope_end, const dab_spi_prog *p, int np);

// ---- MOT header parameters of an SPI object (TS 102 371 6.4) ------------------------------------------------------------------
#define DAB_SPI_PARAM_SCOPE_START 0x25
#define DAB_SPI_PARAM_SCOPE_END   0x26
#define DAB_SPI_PARAM_SCOPE_ID    0x27
// one MOT header extension parameter (PLI 11, 7-bit length) with the given data; returns its size
int dab_spi_param(uint8_t *o, int id, const uint8_t *data, int n);
// ScopeStart / ScopeEnd: short form UTC with the LTO (rounded down to the minute); returns the parameter's size
int dab_spi_param_time(uint8_t *o, int id, long long utc_s, int lto_halfhours);

// UTF-8 helpers: copy at most max_chars characters (whole sequences only) into dst (dst_size bytes); returns the bytes copied
int dab_spi_utf8_cut(char *dst, int dst_size, const char *src, int max_chars);

#endif
