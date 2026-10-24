// SPDX-License-Identifier: Apache-2.0
// SPI binary encoding (ETSI TS 102 371 V3.3.1). Own implementation from the specification text.
#include "dab_spi.h"
#include <string.h>

// element tags (annex D) and attribute tags (annex E) used here
enum {
    T_CDATA = 0x01,
    T_EPG = 0x02,
    T_SERVICE_INFORMATION = 0x03,
    T_SHORT_NAME = 0x10,
    T_MEDIUM_NAME = 0x11,
    T_LONG_NAME = 0x12,
    T_MEDIA_DESCRIPTION = 0x13,
    T_LOCATION = 0x19,
    T_SHORT_DESCRIPTION = 0x1A,
    T_PROGRAMME = 0x1C,
    T_SCHEDULE = 0x21,
    T_SCOPE = 0x24,
    T_SERVICE_SCOPE = 0x25,
    T_ENSEMBLE = 0x26,
    T_SERVICE = 0x28,
    T_SERVICE_BEARER = 0x29, // bearer as a child of service
    T_TIME = 0x2C,
};
enum {
    A_SI_VERSION = 0x80,       // serviceInformation version (integer, 16 bit)
    A_ENSEMBLE_ID = 0x80,      // ensemble id: ECC + EId
    A_BEARER_ID = 0x80,        // bearer id (bearerURI)
    A_SCHEDULE_VERSION = 0x80, // schedule version (integer, 16 bit)
    A_SCOPE_START = 0x80,      // scope startTime / stopTime (timePoint)
    A_SCOPE_STOP = 0x81,
    A_SERVICE_SCOPE_ID = 0x80, // serviceScope id (bearerURI)
    A_PROG_SHORT_ID = 0x81,    // programme shortId (integer, 24 bit)
    A_TIME_TIME = 0x80,        // time time (timePoint) / duration (16 bit seconds)
    A_TIME_DURATION = 0x81,
};

void dab_spi_init(dab_spi_w *w, uint8_t *buf, int cap){
    memset(w, 0, sizeof *w);
    w->b = buf;
    w->cap = cap;
}

static void put(dab_spi_w *w, const void *d, int n){
    if (w->err || w->len + n > w->cap){
        w->err = 1;
        return;
    }
    memcpy(w->b + w->len, d, (size_t)n);
    w->len += n;
}

void dab_spi_open(dab_spi_w *w, int tag){
    if (w->depth >= 16){
        w->err = 1;
        return;
    }
    w->open[w->depth++] = w->len;
    uint8_t h[2] = { (uint8_t)tag, 0 }; // the length byte is filled in by dab_spi_close()
    put(w, h, 2);
}

// the length field of the element or attribute whose tag sits at 'at' and whose value runs to the end: 1 byte up to 0xFD,
// else 0xFE + 16 bit or 0xFF + 24 bit (the value is moved up to make room)
static void set_len(dab_spi_w *w, int at){
    if (w->err) return;
    const int n = w->len - (at + 2);
    int extra = n <= 0xFD ? 0 : n <= 0xFFFF ? 2 : 3;
    if (extra){
        if (w->len + extra > w->cap){
            w->err = 1;
            return;
        }
        memmove(w->b + at + 2 + extra, w->b + at + 2, (size_t)n);
        w->len += extra;
    }
    uint8_t *l = w->b + at + 1;
    if (!extra) l[0] = (uint8_t)n;
    else if (extra == 2){
        l[0] = 0xFE;
        l[1] = (uint8_t)(n >> 8);
        l[2] = (uint8_t)n;
    } else {
        l[0] = 0xFF;
        l[1] = (uint8_t)(n >> 16);
        l[2] = (uint8_t)(n >> 8);
        l[3] = (uint8_t)n;
    }
}

void dab_spi_close(dab_spi_w *w){
    if (w->depth <= 0){
        w->err = 1;
        return;
    }
    set_len(w, w->open[--w->depth]);
}

void dab_spi_attr(dab_spi_w *w, int tag, const void *data, int n){
    const int at = w->len;
    uint8_t h[2] = { (uint8_t)tag, 0 };
    put(w, h, 2);
    put(w, data, n);
    set_len(w, at);
}

void dab_spi_attr_uint(dab_spi_w *w, int tag, unsigned v, int bytes){
    uint8_t d[4];
    for (int i = 0; i < bytes; i++) d[i] = (uint8_t)(v >> (8 * (bytes - 1 - i)));
    dab_spi_attr(w, tag, d, bytes);
}

void dab_spi_text(dab_spi_w *w, int tag, const char *utf8){
    dab_spi_open(w, tag);
    dab_spi_attr(w, T_CDATA, utf8, (int)strlen(utf8));
    dab_spi_close(w);
}

int dab_spi_end(dab_spi_w *w) { return w->err || w->depth ? -1 : w->len; }

// ---- field encodings ----------------------------------------------------------------------------------------------------------
int dab_spi_timepoint(uint8_t *o, long long utc_s, int lto, int long_form){
    const long long day = utc_s >= 0 ? utc_s / 86400 : (utc_s - 86399) / 86400;
    const unsigned sod = (unsigned)(utc_s - day * 86400), mjd = (unsigned)(day + 40587);
    const unsigned h = sod / 3600, mi = sod / 60 % 60, s = sod % 60;
    const int has_lto = lto != DAB_SPI_NO_LTO;
    // Rfa(1) Date(17) Rfa(1) LTO flag(1) UTC flag(1) then hours(5) minutes(6) [seconds(6) Rfa(10)]
    uint32_t v = ((mjd & 0x1FFFFu) << 14) | ((unsigned)has_lto << 12) | ((unsigned)(long_form != 0) << 11) | (h << 6) | mi;
    int n = 0;
    o[n++] = (uint8_t)(v >> 24);
    o[n++] = (uint8_t)(v >> 16);
    o[n++] = (uint8_t)(v >> 8);
    o[n++] = (uint8_t)v;
    if (long_form){
        o[n++] = (uint8_t)(s << 2); // seconds, then 10 reserved bits
        o[n++] = 0;
    }
    if (has_lto){
        const int a = lto < 0 ? -lto : lto;
        o[n++] = (uint8_t)((lto < 0 ? 0x20 : 0) | (a & 0x1F));
    }
    return n;
}

int dab_spi_bearer(uint8_t *o, unsigned ecc, unsigned eid, unsigned sid, unsigned scids){
    o[0] = (uint8_t)(0x40 | (scids & 15)); // Rfa 0, Ens flag 1, X-PAD flag 0, SId flag 0 (16-bit SId), SCIdS
    o[1] = (uint8_t)ecc;
    o[2] = (uint8_t)(eid >> 8);
    o[3] = (uint8_t)eid;
    o[4] = (uint8_t)(sid >> 8);
    o[5] = (uint8_t)sid;
    return 6;
}

// ---- objects ------------------------------------------------------------------------------------------------------------------
int dab_spi_utf8_cut(char *dst, int dst_size, const char *src, int max_chars){
    int n = 0, chars = 0;
    const unsigned char *s = (const unsigned char *)(src ? src : "");
    while (s[n] && chars < max_chars){
        int k = s[n] < 0x80 ? 1 : (s[n] & 0xE0) == 0xC0 ? 2 : (s[n] & 0xF0) == 0xE0 ? 3 : (s[n] & 0xF8) == 0xF0 ? 4 : 1;
        int ok = 1;
        for (int i = 1; i < k; i++)
            if ((s[n + i] & 0xC0) != 0x80) ok = 0;
        if (!ok) k = 1;
        if (n + k >= dst_size) break;
        n += k;
        chars++;
    }
    memcpy(dst, s, (size_t)n);
    dst[n] = 0;
    return n;
}

// a name cut to max characters, trailing spaces removed
static void name_cut(char *dst, int size, const char *src, int max){
    int n = dab_spi_utf8_cut(dst, size, src, max);
    while (n > 0 && dst[n - 1] == ' ') dst[--n] = 0;
}

int dab_spi_si(uint8_t *out, int cap, unsigned version, unsigned ecc, unsigned eid, const char *ens_name,
               const dab_spi_service *svc, int nsvc){
    dab_spi_w w;
    uint8_t b[8];
    char sn[40], mn[80];
    dab_spi_init(&w, out, cap);
    dab_spi_open(&w, T_SERVICE_INFORMATION);
    dab_spi_attr_uint(&w, A_SI_VERSION, version & 0xFFFF, 2);
    dab_spi_open(&w, T_ENSEMBLE);
    b[0] = (uint8_t)ecc;
    b[1] = (uint8_t)(eid >> 8);
    b[2] = (uint8_t)eid;
    dab_spi_attr(&w, A_ENSEMBLE_ID, b, 3);
    name_cut(sn, sizeof sn, ens_name, 8);
    name_cut(mn, sizeof mn, ens_name, 16);
    dab_spi_text(&w, T_SHORT_NAME, sn);
    dab_spi_text(&w, T_MEDIUM_NAME, mn);
    for (int i = 0; i < nsvc; i++){
        dab_spi_open(&w, T_SERVICE);
        name_cut(sn, sizeof sn, svc[i].name, 8);
        name_cut(mn, sizeof mn, svc[i].name, 16);
        dab_spi_text(&w, T_SHORT_NAME, sn);
        dab_spi_text(&w, T_MEDIUM_NAME, mn);
        dab_spi_open(&w, T_SERVICE_BEARER);
        uint8_t br[6];
        dab_spi_attr(&w, A_BEARER_ID, br, dab_spi_bearer(br, ecc, eid, svc[i].sid, 0));
        dab_spi_close(&w);
        dab_spi_close(&w);
    }
    dab_spi_close(&w);
    dab_spi_close(&w);
    return dab_spi_end(&w);
}

int dab_spi_pi(uint8_t *out, int cap, unsigned version, unsigned ecc, unsigned eid, unsigned sid, int lto,
               long long scope_start, long long scope_end, const dab_spi_prog *p, int np){
    dab_spi_w w;
    uint8_t b[8];
    char mn[80], ln[600], ds[800];
    dab_spi_init(&w, out, cap);
    dab_spi_open(&w, T_EPG);
    dab_spi_open(&w, T_SCHEDULE);
    dab_spi_attr_uint(&w, A_SCHEDULE_VERSION, version & 0xFFFF, 2);
    dab_spi_open(&w, T_SCOPE);
    dab_spi_attr(&w, A_SCOPE_START, b, dab_spi_timepoint(b, scope_start / 60 * 60, lto, 0));
    dab_spi_attr(&w, A_SCOPE_STOP, b, dab_spi_timepoint(b, scope_end / 60 * 60, lto, 0));
    dab_spi_open(&w, T_SERVICE_SCOPE);
    dab_spi_attr(&w, A_SERVICE_SCOPE_ID, b, dab_spi_bearer(b, ecc, eid, sid, 0));
    dab_spi_close(&w);
    dab_spi_close(&w);
    for (int i = 0; i < np; i++){
        dab_spi_open(&w, T_PROGRAMME);
        dab_spi_attr_uint(&w, A_PROG_SHORT_ID, p[i].short_id & 0xFFFFFF, 3);
        name_cut(mn, sizeof mn, p[i].mname, 16);
        name_cut(ln, sizeof ln, p[i].lname, 128);
        dab_spi_text(&w, T_MEDIUM_NAME, mn);
        dab_spi_text(&w, T_LONG_NAME, ln[0] ? ln : mn);
        dab_spi_open(&w, T_LOCATION);
        dab_spi_open(&w, T_TIME);
        dab_spi_attr(&w, A_TIME_TIME, b, dab_spi_timepoint(b, p[i].start, lto, (p[i].start % 60) != 0));
        dab_spi_attr_uint(&w, A_TIME_DURATION, (unsigned)(p[i].dur_s < 0 ? 0 : p[i].dur_s > 65535 ? 65535 : p[i].dur_s), 2);
        dab_spi_close(&w);
        dab_spi_close(&w);
        dab_spi_utf8_cut(ds, sizeof ds, p[i].desc, 180);
        if (ds[0]){
            dab_spi_open(&w, T_MEDIA_DESCRIPTION);
            dab_spi_text(&w, T_SHORT_DESCRIPTION, ds);
            dab_spi_close(&w);
        }
        dab_spi_close(&w);
    }
    dab_spi_close(&w);
    dab_spi_close(&w);
    return dab_spi_end(&w);
}

int dab_spi_param(uint8_t *o, int id, const uint8_t *data, int n){
    if (n > 127) n = 127;
    o[0] = (uint8_t)(0xC0 | (id & 0x3F)); // PLI 11
    o[1] = (uint8_t)n;                    // Ext 0, 7-bit DataFieldLength
    memcpy(o + 2, data, (size_t)n);
    return n + 2;
}

int dab_spi_param_time(uint8_t *o, int id, long long utc_s, int lto){
    uint8_t t[8];
    return dab_spi_param(o, id, t, dab_spi_timepoint(t, utc_s / 60 * 60, lto, 0));
}
