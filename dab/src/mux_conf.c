// SPDX-License-Identifier: Apache-2.0
#include "mux_conf.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int mux_cu(int kbps, int prot){
    if (kbps < 8 || kbps > 384 || kbps % 8 || prot < 1 || prot > 8 || (prot > 4 && kbps % 32)) return -1;
    static const int a[4] = { 12, 8, 6, 4 }, b[4] = { 27, 21, 18, 15 };
    return prot > 4 ? b[prot - 5] * (kbps / 32) : a[prot - 1] * (kbps / 8);
}

int mux_cu_total(const mux_conf *c){ int t = 0; for (int i = 0; i < c->n; i++) t += mux_cu(c->s[i].kbps, c->s[i].prot); return t; }

int mux_local(const mux_conf *c){ for (int i = 0; i < c->n; i++) if (c->s[i].src == MUX_SRC_I2S || c->s[i].src == MUX_SRC_TONE) return i; return -1; }

void mux_default(mux_conf *c){
    memset(c, 0, sizeof *c);
    c->eid = 0x8FED; c->ecc = 0xE3; c->lto = 2; snprintf(c->label, sizeof c->label, "PlutoSky DAB");
    c->n = 1; c->s[0].src = MUX_SRC_I2S; c->s[0].sid = 0x8F01; snprintf(c->s[0].label, sizeof c->s[0].label, "PlutoSky");
    c->s[0].kbps = 96; c->s[0].prot = 3;
}

static int label_ok(const char *s){ size_t n = strlen(s); if (n < 1 || n > 16) return 0; for (; *s; s++) if (*s < 0x20 || *s > 0x7E || *s == ';') return 0; return 1; }

static int parse_prot(const char *p){
    if (strlen(p) != 2 || p[0] < '1' || p[0] > '4') return 0;
    if (p[1] == 'A' || p[1] == 'a') return p[0] - '0';
    if (p[1] == 'B' || p[1] == 'b') return p[0] - '0' + 4;
    return 0;
}

#define FAIL(...) do { if (err) snprintf(err, errlen, __VA_ARGS__); return -1; } while (0)

int mux_parse(const char *text, mux_conf *c, char *err, size_t errlen){
    memset(c, 0, sizeof *c);
    c->eid = 0x8FED; c->ecc = 0xE3; c->lto = 2; snprintf(c->label, sizeof c->label, "PlutoSky DAB");
    const char *p = text; int lineno = 0;
    while (*p){
        char ln[160]; size_t k = 0;
        while (*p && *p != '\n'){ if (k < sizeof ln - 1 && *p != '\r') ln[k++] = *p; p++; }
        if (*p == '\n') p++;
        ln[k] = 0; lineno++;
        if (!ln[0] || ln[0] == '#') continue;
        char *eq = strchr(ln, '='); if (!eq) FAIL("line %d: no '='", lineno);
        *eq = 0; const char *key = ln, *val = eq + 1;
        if (!strcmp(key, "eid")) c->eid = (unsigned)strtoul(val, NULL, 16);
        else if (!strcmp(key, "ecc")) c->ecc = (unsigned)strtoul(val, NULL, 16);
        else if (!strcmp(key, "lto")) c->lto = atoi(val);
        else if (!strcmp(key, "label")){ if (!label_ok(val)) FAIL("ensemble label: 1..16 printable characters"); snprintf(c->label, sizeof c->label, "%s", val); }
        else if (!strcmp(key, "svc")){
            if (c->n >= MUX_MAX_SVC) FAIL("at most %d services", MUX_MAX_SVC);
            char f[6][40]; int nf = 0; const char *q = val;
            while (nf < 6){ size_t m = strcspn(q, ";"); if (m >= sizeof f[0]) FAIL("line %d: field too long", lineno); memcpy(f[nf], q, m); f[nf][m] = 0; nf++; q += m; if (*q != ';') break; q++; }
            if (nf != 6) FAIL("line %d: svc=source;SId;label;kbit/s;protection;slideshow", lineno);
            mux_service *s = &c->s[c->n];
            if (!strcmp(f[0], "i2s")) s->src = MUX_SRC_I2S;
            else if (!strncmp(f[0], "tone:", 5)){ s->src = MUX_SRC_TONE; s->tone_hz = atoi(f[0] + 5); if (s->tone_hz < 20 || s->tone_hz > 10000) FAIL("service %d: tone 20..10000 Hz", c->n + 1); }
            else if (!strncmp(f[0], "zmq:", 4)){ s->src = MUX_SRC_ZMQ; s->port = atoi(f[0] + 4); if (s->port < 1024 || s->port > 65535) FAIL("service %d: port 1024..65535", c->n + 1); }
            else FAIL("service %d: source i2s, tone:<Hz> or zmq:<port>", c->n + 1);
            char *e; s->sid = (unsigned)strtoul(f[1], &e, 16);
            if (*e || strlen(f[1]) != 4 || !s->sid) FAIL("service %d: SId: 4 hex digits", c->n + 1);
            if (!label_ok(f[2])) FAIL("service %d: label 1..16 printable characters, no ';'", c->n + 1);
            snprintf(s->label, sizeof s->label, "%s", f[2]);
            s->kbps = atoi(f[3]); s->prot = parse_prot(f[4]); s->sls = atoi(f[5]) ? 1 : 0;
            if (!s->prot) FAIL("service %d: protection 1A..4A or 1B..4B", c->n + 1);
            if (s->kbps > 192 || mux_cu(s->kbps, s->prot) < 0) FAIL("service %d: %d kbit/s is not possible with %s (8..192, %s)", c->n + 1, s->kbps, f[4], s->prot > 4 ? "multiple of 32 for EEP-B" : "multiple of 8");
            c->n++;
        } else FAIL("line %d: unknown key '%s'", lineno, key);
    }
    if (!label_ok(c->label)) FAIL("ensemble label missing");
    if (!c->eid || c->eid > 0xFFFF) FAIL("EId: 4 hex digits");
    if (c->n < 1) FAIL("no service");
    int local = 0;
    for (int i = 0; i < c->n; i++){
        if (c->s[i].src != MUX_SRC_ZMQ) local++;
        for (int j = 0; j < i; j++){
            if (c->s[i].sid == c->s[j].sid) FAIL("services %d and %d have the same SId", j + 1, i + 1);
            if (c->s[i].src == MUX_SRC_ZMQ && c->s[j].src == MUX_SRC_ZMQ && c->s[i].port == c->s[j].port) FAIL("services %d and %d listen on the same port", j + 1, i + 1);
        }
    }
    if (local > 1) FAIL("only one service can be encoded here (i2s or tone)");
    int cu = mux_cu_total(c);
    if (cu > 864) FAIL("%d CU needed, 864 available", cu);
    return 0;
}

int mux_read(const char *file, mux_conf *c, char *err, size_t errlen){
    FILE *f = fopen(file, "r"); if (!f) FAIL("%s cannot be read", file);
    char buf[4096]; size_t n = fread(buf, 1, sizeof buf - 1, f); fclose(f); buf[n] = 0;
    return mux_parse(buf, c, err, errlen);
}

int mux_format(const mux_conf *c, char *out, size_t outlen){
    static const char *const pr[9] = { "", "1A", "2A", "3A", "4A", "1B", "2B", "3B", "4B" };
    size_t n = 0; int r = snprintf(out, outlen, "eid=%04X\nlabel=%s\necc=%02X\nlto=%d\n", c->eid, c->label, c->ecc, c->lto);
    if (r < 0 || (size_t)r >= outlen) return -1;
    n = (size_t)r;
    for (int i = 0; i < c->n; i++){
        const mux_service *s = &c->s[i]; char src[24];
        if (s->src == MUX_SRC_ZMQ) snprintf(src, sizeof src, "zmq:%d", s->port);
        else if (s->src == MUX_SRC_TONE) snprintf(src, sizeof src, "tone:%d", s->tone_hz);
        else snprintf(src, sizeof src, "i2s");
        r = snprintf(out + n, outlen - n, "svc=%s;%04X;%s;%d;%s;%d\n", src, s->sid, s->label, s->kbps, pr[(s->prot >= 1 && s->prot <= 8) ? s->prot : 3], s->sls);
        if (r < 0 || (size_t)r >= outlen - n) return -1;
        n += (size_t)r;
    }
    return (int)n;
}

int mux_write(const char *file, const mux_conf *c){
    char buf[4096], tmp[256]; int n = mux_format(c, buf, sizeof buf); if (n < 0) return -1;
    snprintf(tmp, sizeof tmp, "%s.new", file);
    FILE *f = fopen(tmp, "w"); if (!f) return -1;
    if (fwrite(buf, 1, (size_t)n, f) != (size_t)n){ fclose(f); return -1; }
    fclose(f);
    return rename(tmp, file);
}
