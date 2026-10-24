// SPDX-License-Identifier: Apache-2.0
#include "epg_plan.h"
#include "dab_spi.h"
#include <dirent.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

// ---- dates ------------------------------------------------------------------------------------------------------------------
// days since 1970-01-01 of a civil date (proleptic Gregorian), and back
long long epg_days(int y, int mo, int d){
    y -= mo <= 2;
    const long long era = (y >= 0 ? y : y - 399) / 400;
    const unsigned yoe = (unsigned)(y - era * 400);
    const unsigned doy = (unsigned)((153 * (mo + (mo > 2 ? -3 : 9)) + 2) / 5 + d - 1);
    const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return era * 146097 + (long long)doe - 719468;
}
void epg_ymd(long long z, int *y, int *mo, int *d){
    z += 719468;
    const long long era = (z >= 0 ? z : z - 146096) / 146097;
    const unsigned doe = (unsigned)(z - era * 146097);
    const unsigned yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    const unsigned doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    const unsigned mp = (5 * doy + 2) / 153;
    *d = (int)(doy - (153 * mp + 2) / 5 + 1);
    *mo = (int)(mp < 10 ? mp + 3 : mp - 9);
    *y = (int)(yoe + era * 400 + (*mo <= 2));
}
static int parse_date(const char *s, int *days){
    int y, mo, d, n = 0;
    if (sscanf(s, "%4d-%2d-%2d%n", &y, &mo, &d, &n) != 3 || n != 10 || s[n]) return -1;
    if (y < 2000 || y > 2199 || mo < 1 || mo > 12 || d < 1 || d > 31) return -1;
    const long long z = epg_days(y, mo, d);
    int y2, m2, d2;
    epg_ymd(z, &y2, &m2, &d2);
    if (y2 != y || m2 != mo || d2 != d) return -1; // 31 April and the like
    *days = (int)z;
    return 0;
}
static void fmt_date(char *o, size_t n, long long days){
    int y, mo, d;
    epg_ymd(days, &y, &mo, &d);
    snprintf(o, n, "%04u-%02u-%02u", (unsigned)y % 10000u, (unsigned)mo % 100u, (unsigned)d % 100u);
}
static long long floor_div(long long a, long long b) { return a >= 0 ? a / b : -((-a + b - 1) / b); }

// ---- text -------------------------------------------------------------------------------------------------------------------
// a text field: control characters become spaces, '|' a '/', invalid UTF-8 a '?', leading/trailing spaces go; at most max
// characters
static void clean_text(char *dst, int size, const char *src, int max){
    char t[4096];
    int n = 0;
    const unsigned char *s = (const unsigned char *)src;
    while (*s && n < (int)sizeof t - 5){
        int k = *s < 0x80 ? 1 : (*s & 0xE0) == 0xC0 ? 2 : (*s & 0xF0) == 0xE0 ? 3 : (*s & 0xF8) == 0xF0 ? 4 : 0;
        for (int i = 1; i < k; i++)
            if ((s[i] & 0xC0) != 0x80) k = 0;
        if (k == 0){
            t[n++] = '?';
            s++;
            continue;
        }
        if (k == 1){
            t[n++] = *s < 0x20 || *s == 0x7F ? ' ' : *s == '|' ? '/' : (char)*s;
        } else {
            memcpy(t + n, s, (size_t)k);
            n += k;
        }
        s += k;
    }
    t[n] = 0;
    char *b = t;
    while (*b == ' ') b++;
    n = (int)strlen(b);
    while (n > 0 && b[n - 1] == ' ') b[--n] = 0;
    dab_spi_utf8_cut(dst, size, b, max);
}
static int utf8_chars(const char *s){
    int c = 0;
    for (; *s; s++)
        if (((unsigned char)*s & 0xC0) != 0x80) c++;
    return c;
}
// the long name cut to max characters: at the last space when that keeps at least half, else hard
static void cut_name(char *out, int size, const char *name, int max){
    if (utf8_chars(name) <= max){
        dab_spi_utf8_cut(out, size, name, max);
        return;
    }
    int n = dab_spi_utf8_cut(out, size, name, max);
    if (name[n] != ' '){
        int sp = n;
        while (sp > 0 && out[sp - 1] != ' ') sp--;
        if (sp > 0 && utf8_chars(out) - utf8_chars(out + sp) >= max / 2) n = sp;
    }
    out[n] = 0;
    while (n > 0 && out[n - 1] == ' ') out[--n] = 0;
}
void epg_medium(const epg_entry *e, char *out, int size){
    if (e->mname[0]) dab_spi_utf8_cut(out, size, e->mname, EPG_MED_CHARS);
    else cut_name(out, size, e->lname, EPG_MED_CHARS);
}
void epg_short(const epg_entry *e, char *out, int size){
    if (e->sname[0]) snprintf(out, (size_t)size, "%s", e->sname);
    else {
        char m[4 * EPG_MED_CHARS + 1];
        epg_medium(e, m, sizeof m);
        cut_name(out, size, m, EPG_SHORT_CHARS);
    }
}

// ---- the planner file -------------------------------------------------------------------------------------------------------
static const char *const REP[] = { "once", "daily", "weekdays", "weekend", "weekly" };

static int parse_entry(char *s, epg_entry *e, char *err, size_t errlen){
    char *f[10];
    int nf = 0;
    f[nf++] = s;
    for (char *p = s; *p; p++)
        if (*p == '|'){
            *p = 0;
            if (nf == 10){
                snprintf(err, errlen, "more than 10 fields");
                return -1;
            }
            f[nf++] = p + 1;
        }
    if (nf < 7){
        snprintf(err, errlen, "expected SId|start|minutes|repeat|from|until|long name[|medium|short|description]");
        return -1;
    }
    memset(e, 0, sizeof *e);
    char *end;
    unsigned long sid = strtoul(f[0], &end, 16);
    if (!f[0][0] || *end || sid < 1 || sid > 0xFFFF){
        snprintf(err, errlen, "service id '%s': 1..FFFF (hex)", f[0]);
        return -1;
    }
    e->sid = (unsigned)sid;
    int h, mi, n = 0;
    if (sscanf(f[1], "%2d:%2d%n", &h, &mi, &n) != 2 || f[1][n] || h < 0 || h > 23 || mi < 0 || mi > 59){
        snprintf(err, errlen, "start '%s': HH:MM", f[1]);
        return -1;
    }
    e->start = h * 60 + mi;
    long d = strtol(f[2], &end, 10);
    if (!f[2][0] || *end || d < 1 || d > EPG_MAX_MIN){
        snprintf(err, errlen, "duration '%s': 1..%d minutes", f[2], EPG_MAX_MIN);
        return -1;
    }
    e->dur = (int)d;
    e->rep = -1;
    if (!strncmp(f[3], "once:", 5)){
        e->rep = EPG_ONCE;
        if (parse_date(f[3] + 5, &e->date)){
            snprintf(err, errlen, "date '%s': YYYY-MM-DD", f[3] + 5);
            return -1;
        }
    } else if (!strncmp(f[3], "weekly:", 7)){
        e->rep = EPG_WEEKLY;
        for (const char *p = f[3] + 7; *p; p++){
            if (*p >= '1' && *p <= '7') e->days |= 1 << (*p - '1');
            else if (*p != ',' && *p != ' '){
                e->days = 0;
                break;
            }
        }
        if (!e->days){
            snprintf(err, errlen, "weekly: days 1..7 (Monday = 1), e.g. weekly:1,3,5");
            return -1;
        }
    } else
        for (int i = EPG_DAILY; i <= EPG_WEEKEND; i++)
            if (!strcmp(f[3], REP[i])) e->rep = i;
    if (e->rep < 0){
        snprintf(err, errlen, "repeat '%s': daily, weekdays, weekend, weekly:<days> or once:YYYY-MM-DD", f[3]);
        return -1;
    }
    e->from = e->until = -1;
    if (f[4][0] && parse_date(f[4], &e->from)){
        snprintf(err, errlen, "from '%s': YYYY-MM-DD or empty", f[4]);
        return -1;
    }
    if (f[5][0] && parse_date(f[5], &e->until)){
        snprintf(err, errlen, "until '%s': YYYY-MM-DD or empty", f[5]);
        return -1;
    }
    if (e->from >= 0 && e->until >= 0 && e->until < e->from){
        snprintf(err, errlen, "until before from");
        return -1;
    }
    clean_text(e->lname, sizeof e->lname, f[6], EPG_LONG_CHARS);
    if (!e->lname[0]){
        snprintf(err, errlen, "the long name is empty");
        return -1;
    }
    if (nf > 7) clean_text(e->mname, sizeof e->mname, f[7], EPG_MED_CHARS);
    if (nf > 8) clean_text(e->sname, sizeof e->sname, f[8], EPG_SHORT_CHARS);
    if (nf > 9) clean_text(e->desc, sizeof e->desc, f[9], EPG_DESC_CHARS);
    return 0;
}

int epg_parse(const char *text, epg_plan *p, char *err, size_t errlen){
    memset(p, 0, sizeof *p);
    p->on = 1;
    int ln = 0;
    const char *s = text;
    while (*s){
        const char *e = strchr(s, '\n');
        size_t len = e ? (size_t)(e - s) : strlen(s);
        char l[4096];
        ln++;
        if (len >= sizeof l){
            snprintf(err, errlen, "line %d: too long", ln);
            return -1;
        }
        memcpy(l, s, len);
        l[len] = 0;
        s += len + (e ? 1 : 0);
        while (len && (l[len - 1] == '\r' || l[len - 1] == ' ' || l[len - 1] == '\t')) l[--len] = 0;
        char *b = l;
        while (*b == ' ' || *b == '\t') b++;
        if (!*b || *b == '#') continue;
        if (!strncmp(b, "on=", 3)){
            p->on = atoi(b + 3) ? 1 : 0;
        } else if (!strncmp(b, "p=", 2)){
            if (p->n >= EPG_MAX_ENTRIES){
                snprintf(err, errlen, "line %d: more than %d programmes", ln, EPG_MAX_ENTRIES);
                return -1;
            }
            char e2[200];
            if (parse_entry(b + 2, &p->e[p->n], e2, sizeof e2)){
                snprintf(err, errlen, "line %d: %s", ln, e2);
                return -1;
            }
            p->n++;
        } else {
            snprintf(err, errlen, "line %d: unknown (on=, p=)", ln);
            return -1;
        }
    }
    return 0;
}

static void fmt_rep(const epg_entry *e, char *o, size_t n){
    if (e->rep == EPG_ONCE){
        char d[16];
        fmt_date(d, sizeof d, e->date);
        snprintf(o, n, "once:%s", d);
    } else if (e->rep == EPG_WEEKLY){
        int k = snprintf(o, n, "weekly:");
        for (int i = 0; i < 7; i++)
            if (e->days >> i & 1) k += snprintf(o + k, n - (size_t)k, "%s%d", k > 7 ? "," : "", i + 1);
    } else
        snprintf(o, n, "%s", REP[e->rep]);
}

int epg_format(const epg_plan *p, char *out, size_t cap){
    size_t n = 0;
    int k = snprintf(out, cap,
                     "# skypluto EPG planner: p=SId|HH:MM|minutes|repeat|from|until|long name|medium|short|description\n"
                     "on=%d\n",
                     p->on);
    if (k < 0 || (size_t)k >= cap) return -1;
    n = (size_t)k;
    for (int i = 0; i < p->n; i++){
        const epg_entry *e = &p->e[i];
        char rep[40], fr[16] = "", un[16] = "";
        fmt_rep(e, rep, sizeof rep);
        if (e->from >= 0) fmt_date(fr, sizeof fr, e->from);
        if (e->until >= 0) fmt_date(un, sizeof un, e->until);
        k = snprintf(out + n, cap - n, "p=%04X|%02d:%02d|%d|%s|%s|%s|%s|%s|%s|%s\n", e->sid, e->start / 60, e->start % 60,
                     e->dur, rep, fr, un, e->lname, e->mname, e->sname, e->desc);
        if (k < 0 || n + (size_t)k >= cap) return -1;
        n += (size_t)k;
    }
    return (int)n;
}

int epg_read(const char *file, epg_plan *p, char *err, size_t errlen){
    FILE *f = fopen(file, "rb");
    if (!f){
        snprintf(err, errlen, "cannot be opened");
        memset(p, 0, sizeof *p);
        return -1;
    }
    static char t[EPG_MAX_ENTRIES * 1500 + 256];
    size_t n = fread(t, 1, sizeof t - 1, f);
    fclose(f);
    t[n] = 0;
    return epg_parse(t, p, err, errlen);
}

int epg_write(const char *file, const epg_plan *p){
    static char t[EPG_MAX_ENTRIES * 1500 + 256];
    int n = epg_format(p, t, sizeof t);
    if (n < 0) return -1;
    char tmp[300];
    snprintf(tmp, sizeof tmp, "%s.new", file);
    FILE *f = fopen(tmp, "wb");
    if (!f) return -1;
    int bad = fwrite(t, 1, (size_t)n, f) != (size_t)n;
    bad |= fflush(f) != 0;
    bad |= fsync(fileno(f)) != 0;
    bad |= fclose(f) != 0;
    if (bad || rename(tmp, file)){
        unlink(tmp);
        return -1;
    }
    return 0;
}

// ---- expansion --------------------------------------------------------------------------------------------------------------
static int applies(const epg_entry *e, long long day){
    if (e->from >= 0 && day < e->from) return 0;
    if (e->until >= 0 && day > e->until) return 0;
    const int dow = (int)(((day + 3) % 7 + 7) % 7); // 0 = Monday (1970-01-01 was a Thursday)
    switch (e->rep){
    case EPG_ONCE: return day == e->date;
    case EPG_DAILY: return 1;
    case EPG_WEEKDAYS: return dow < 5;
    case EPG_WEEKEND: return dow >= 5;
    case EPG_WEEKLY: return e->days >> dow & 1;
    }
    return 0;
}
static int specificity(int rep) { return rep == EPG_ONCE ? 3 : rep == EPG_DAILY ? 1 : 2; }

typedef struct {
    long long min; // local minutes since 1970
    int dur, entry, spec;
} cand;
static int cmp_cand(const void *a, const void *b){
    const cand *x = a, *y = b;
    if (x->min != y->min) return x->min < y->min ? -1 : 1;
    if (x->spec != y->spec) return y->spec - x->spec;
    return x->entry - y->entry;
}

int epg_day(const epg_plan *p, unsigned sid, long long day, int lto, int carry, epg_prog *out, int max){
    static cand c[3 * EPG_MAX_ENTRIES];
    int n = 0, k = 0;
    for (long long d = day - 1; d <= day + 1; d++)
        for (int i = 0; i < p->n; i++)
            if (p->e[i].sid == sid && applies(&p->e[i], d))
                c[n++] = (cand){ d * 1440 + p->e[i].start, p->e[i].dur, i, specificity(p->e[i].rep) };
    qsort(c, (size_t)n, sizeof c[0], cmp_cand);
    int m = 0; // the same start: the first (most specific) stays
    for (int i = 0; i < n; i++)
        if (!m || c[i].min != c[m - 1].min) c[m++] = c[i];
    for (int i = 0; i + 1 < m; i++)
        if (c[i].min + c[i].dur > c[i + 1].min) c[i].dur = (int)(c[i + 1].min - c[i].min);
    const long long d0 = day * 1440, d1 = d0 + 1440;
    for (int i = 0; i < m && k < max; i++){
        const int in_day = c[i].min >= d0 && c[i].min < d1;
        const int carried = carry && c[i].min < d0 && c[i].min + c[i].dur > d0;
        if (!in_day && !carried) continue;
        out[k++] = (epg_prog){ c[i].min * 60 - (long long)lto * 1800, c[i].dur, c[i].entry };
    }
    return k;
}

// ---- SPI objects --------------------------------------------------------------------------------------------------------------
static int write_file(const char *path, const void *d, int n){
    char tmp[400];
    snprintf(tmp, sizeof tmp, "%s.new", path);
    FILE *f = fopen(tmp, "wb");
    if (!f) return -1;
    int bad = fwrite(d, 1, (size_t)n, f) != (size_t)n;
    bad |= fclose(f) != 0;
    if (bad || rename(tmp, path)){
        unlink(tmp);
        return -1;
    }
    return 0;
}
// remove the object files of earlier generations (all *.bin not in keep)
static void prune(const char *dir, char keep[][48], int nkeep){
    DIR *d = opendir(dir);
    if (!d) return;
    struct dirent *de;
    while ((de = readdir(d))){
        const size_t l = strlen(de->d_name);
        if (l < 5 || strcmp(de->d_name + l - 4, ".bin")) continue;
        int k = 0;
        for (int i = 0; i < nkeep && !k; i++) k = !strcmp(keep[i], de->d_name);
        if (!k){
            char fn[400];
            snprintf(fn, sizeof fn, "%s/%s", dir, de->d_name);
            unlink(fn);
        }
    }
    closedir(d);
}
static void no_epg(const char *dir){
    char fn[300];
    snprintf(fn, sizeof fn, "%s/index", dir);
    unlink(fn);
    prune(dir, NULL, 0);
}

int epg_generate(const epg_plan *p, const mux_conf *m, long long now, unsigned version, const char *dir, char *msg,
                 size_t msglen){
    mkdir(dir, 0755);
    if (!p->on){
        no_epg(dir);
        snprintf(msg, msglen, "off");
        return 0;
    }
    if (now < EPG_CLOCK_VALID){
        no_epg(dir);
        snprintf(msg, msglen, "off: the clock is not set (TM)");
        return 0;
    }
    const int lto = m->lto;
    const long long today = floor_div(now + (long long)lto * 1800, 86400);
    static char index[8192];
    static char keep[2 + 2 * MUX_MAX_SVC][48];
    static uint8_t obj[DAB_SPI_MAX_OBJECT];
    static epg_prog pr[EPG_MAX_ENTRIES * 2];
    static dab_spi_prog sp[EPG_MAX_ENTRIES * 2];
    static char mn[EPG_MAX_ENTRIES * 2][4 * EPG_MED_CHARS + 1];
    int il = 0, nobj = 0, nprog = 0, trimmed = 0;
    for (int s = 0; s < m->n; s++){
        const unsigned sid = m->s[s].sid;
        for (int dd = 0; dd < 2; dd++){
            const long long day = today + dd;
            int np = epg_day(p, sid, day, lto, dd == 0, pr, EPG_MAX_ENTRIES * 2);
            if (!np) continue;
            for (int i = 0; i < np; i++){
                const epg_entry *e = &p->e[pr[i].entry];
                epg_medium(e, mn[i], sizeof mn[i]);
                const long long lday = floor_div(pr[i].start / 60 + lto * 30, 1440);
                const int lmin = (int)(pr[i].start / 60 + lto * 30 - lday * 1440);
                sp[i] = (dab_spi_prog){ pr[i].start, pr[i].dur * 60, ((unsigned)(s & 7) << 21) | ((unsigned)(lday & 1023) << 11) | (unsigned)lmin,
                                        mn[i], e->lname, e->desc };
            }
            // the object must stay within 16 kbytes: without descriptions, then fewer programmes
            int len = -1, nd = np;
            while (nd > 0){
                const long long ss = sp[0].start, se = sp[nd - 1].start + sp[nd - 1].dur_s;
                len = dab_spi_pi(obj, sizeof obj, version, m->ecc, m->eid, sid, lto, ss, se, sp, nd);
                if (len > 0) break;
                if (sp[0].desc){
                    for (int i = 0; i < np; i++) sp[i].desc = NULL;
                } else
                    nd--;
                trimmed = 1;
            }
            if (len <= 0) continue;
            int y, mo, d;
            epg_ymd(day, &y, &mo, &d);
            char file[48], path[400];
            snprintf(file, sizeof file, "pi_%04X_%04d%02d%02d.bin", sid, y, mo, d);
            snprintf(path, sizeof path, "%s/%s", dir, file);
            if (write_file(path, obj, len)) continue;
            snprintf(keep[nobj++], sizeof keep[0], "%s", file);
            nprog += nd;
            uint8_t b[6];
            dab_spi_bearer(b, m->ecc, m->eid, sid, 0);
            il += snprintf(index + il, sizeof index - (size_t)il,
                           "o %s 7 1 PI_%04X_%04d%02d%02d ss=%lld se=%lld lto=%d scope=%02X%02X%02X%02X%02X%02X\n", file, sid, y,
                           mo, d, sp[0].start, sp[nd - 1].start + sp[nd - 1].dur_s, lto, b[0], b[1], b[2], b[3], b[4], b[5]);
        }
    }
    if (!nobj){
        no_epg(dir);
        snprintf(msg, msglen, "off: no programmes today or tomorrow for the services of the multiplex");
        return 0;
    }
    // the SI object: the ensemble and all its services
    dab_spi_service sv[MUX_MAX_SVC];
    for (int s = 0; s < m->n; s++) sv[s] = (dab_spi_service){ m->s[s].sid, m->s[s].label };
    int len = dab_spi_si(obj, sizeof obj, version, m->ecc, m->eid, m->label, sv, m->n);
    char path[400];
    snprintf(path, sizeof path, "%s/si.bin", dir);
    if (len <= 0 || write_file(path, obj, len)){
        no_epg(dir);
        snprintf(msg, msglen, "off: the SI object could not be written");
        return 0;
    }
    snprintf(keep[nobj++], sizeof keep[0], "si.bin");
    il += snprintf(index + il, sizeof index - (size_t)il, "o si.bin 7 0 SI scope=%02X%02X%02X\n", m->ecc & 0xFF,
                   (m->eid >> 8) & 0xFF, m->eid & 0xFF);
    char tmp[400];
    snprintf(path, sizeof path, "%s/index", dir);
    snprintf(tmp, sizeof tmp, "%s/index.new", dir);
    FILE *f = fopen(tmp, "w");
    if (!f || fprintf(f, "# skypluto EPG, version %u, generated at %lld\n%s", version & 0xFFFF, now, index) < 0 ||
        fclose(f) != 0 || rename(tmp, path)){
        no_epg(dir);
        snprintf(msg, msglen, "off: the index could not be written");
        return 0;
    }
    prune(dir, keep, nobj);
    char td[16];
    fmt_date(td, sizeof td, today);
    snprintf(msg, msglen, "on: %d objects, %d programmes (today %s, local time UTC%+d:%02d)%s", nobj, nprog, td, lto / 2,
             (lto < 0 ? -lto : lto) % 2 * 30, trimmed ? "; some descriptions or programmes left out (16 kB per day)" : "");
    return nobj;
}

// ---- JSON -------------------------------------------------------------------------------------------------------------------
static int jstr(char *o, int n, int cap, const char *s){
    if (n < cap) o[n++] = '"';
    for (; *s && n < cap - 8; s++){
        const unsigned char c = (unsigned char)*s;
        if (c == '"' || c == '\\'){
            o[n++] = '\\';
            o[n++] = (char)c;
        } else if (c < 0x20)
            n += snprintf(o + n, (size_t)(cap - n), "\\u%04x", c);
        else
            o[n++] = (char)c;
    }
    if (n < cap) o[n++] = '"';
    return n;
}

int epg_json(const epg_plan *p, const mux_conf *m, long long now, char *o, int cap){
    const int lto = m->lto, clock = now >= EPG_CLOCK_VALID;
    const long long today = floor_div(now + (long long)lto * 1800, 86400);
    char td[16];
    fmt_date(td, sizeof td, today);
    int n = snprintf(o, (size_t)cap, "{\"on\":%d,\"lto\":%d,\"clock\":%d,\"today\":\"%s\",\"max\":%d,\"e\":[", p->on, lto * 30, clock,
                     clock ? td : "", EPG_MAX_ENTRIES);
    for (int i = 0; i < p->n && n < cap - 2000; i++){
        const epg_entry *e = &p->e[i];
        char rep[40], fr[16] = "", un[16] = "", dm[80], dsh[40];
        fmt_rep(e, rep, sizeof rep);
        if (e->from >= 0) fmt_date(fr, sizeof fr, e->from);
        if (e->until >= 0) fmt_date(un, sizeof un, e->until);
        epg_medium(e, dm, sizeof dm);
        epg_short(e, dsh, sizeof dsh);
        n += snprintf(o + n, (size_t)(cap - n), "%s{\"sid\":\"%04X\",\"t\":\"%02d:%02d\",\"d\":%d,\"r\":\"%s\",\"from\":\"%s\",\"until\":\"%s\",\"ln\":",
                      i ? "," : "", e->sid, e->start / 60, e->start % 60, e->dur, rep, fr, un);
        n = jstr(o, n, cap, e->lname);
        n += snprintf(o + n, (size_t)(cap - n), ",\"mn\":");
        n = jstr(o, n, cap, e->mname);
        n += snprintf(o + n, (size_t)(cap - n), ",\"sn\":");
        n = jstr(o, n, cap, e->sname);
        n += snprintf(o + n, (size_t)(cap - n), ",\"ds\":");
        n = jstr(o, n, cap, e->desc);
        n += snprintf(o + n, (size_t)(cap - n), ",\"dm\":");
        n = jstr(o, n, cap, dm);
        n += snprintf(o + n, (size_t)(cap - n), ",\"dsn\":");
        n = jstr(o, n, cap, dsh);
        n += snprintf(o + n, (size_t)(cap - n), "}");
    }
    n += snprintf(o + n, (size_t)(cap - n), "],\"pv\":[");
    // today's and tomorrow's programmes per service of the layout (local times)
    int first = 1;
    static epg_prog pr[EPG_MAX_ENTRIES * 2];
    for (int s = 0; clock && s < m->n; s++)
        for (int dd = 0; dd < 2 && n < cap - 1000; dd++){
            const int np = epg_day(p, m->s[s].sid, today + dd, lto, dd == 0, pr, EPG_MAX_ENTRIES * 2);
            char dt[16];
            fmt_date(dt, sizeof dt, today + dd);
            n += snprintf(o + n, (size_t)(cap - n), "%s{\"sid\":\"%04X\",\"day\":\"%s\",\"p\":[", first ? "" : ",", m->s[s].sid, dt);
            first = 0;
            for (int i = 0; i < np && n < cap - 200; i++){
                const long long lm = pr[i].start / 60 + lto * 30, ld = floor_div(lm, 1440);
                const int mm = (int)(lm - ld * 1440);
                n += snprintf(o + n, (size_t)(cap - n), "%s{\"t\":\"%02d:%02d\",\"d\":%d,\"e\":%d,\"c\":%d}", i ? "," : "", mm / 60,
                              mm % 60, pr[i].dur, pr[i].entry, ld < today + dd ? 1 : 0);
            }
            n += snprintf(o + n, (size_t)(cap - n), "]}");
        }
    n += snprintf(o + n, (size_t)(cap - n), "]}");
    return n < cap ? n : -1;
}
