// SPDX-License-Identifier: Apache-2.0
// SPI encoder and EPG planner tests:
//   1. the binary encoding examples of ETSI TS 102 371 V3.3.1 annex C (SI table C.1, PI table C.2), built with the TLV writer
//      and compared byte by byte with the bytes printed in the standard (with its own text: the table spells the logo urls
//      "479S"/"479R"/"479S"/"479L" and the MIME type "imjge/png", and those bytes are what is compared);
//   2. timePoint and bearerURI fields against the same examples;
//   3. the planner: parsing, writing back, the rules of a day (repeat, from/until, the more specific rule, cutting);
//   4. test_spi <dir> [<UTC seconds> [<planner file or -> [<layout file>]]]: generates the EPG objects and index for that time into dir
//      (for tests/check_spi.py and the skypluto-dab simulation in tests/check_spi.sh).
#include "../src/dab_spi.h"
#include "../src/epg_plan.h"
#include "../src/mux_conf.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int fails = 0;
static void check(int ok, const char *what){
    printf("%s %s\n", ok ? "ok  " : "FAIL", what);
    if (!ok) fails++;
}
static int same(const uint8_t *a, int na, const uint8_t *b, int nb, const char *what){
    int ok = na == nb && !memcmp(a, b, (size_t)na);
    if (!ok){
        printf("     %s: %d bytes, expected %d\n     got:     ", what, na, nb);
        for (int i = 0; i < na; i++) printf("%02X%s", a[i], i % 32 == 31 ? "\n              " : " ");
        printf("\n     expected:");
        for (int i = 0; i < nb; i++) printf("%02X%s", b[i], i % 32 == 31 ? "\n              " : " ");
        printf("\n");
    }
    check(ok, what);
    return ok;
}
static int hexbytes(const char *h, uint8_t *o){
    int n = 0;
    unsigned v;
    while (*h){
        if (*h == ' '){
            h++;
            continue;
        }
        if (sscanf(h, "%2x", &v) != 1) break;
        o[n++] = (uint8_t)v;
        h += 2;
    }
    return n;
}

// TS 102 371 table C.1
static void annex_c1(void){
    static const char *exp_hex =
        "03 9E 26 9C 80 03 E1 C1 85 10 0A 01 08 4C 6F 6E 64 6F 6E 20 31 11 0A 01 08 4C 6F 6E 64 6F 6E 20 31"
        " 28 7D 10 09 01 07 43 61 70 69 74 61 6C 11 0C 01 0A 43 61 70 69 74 61 6C 20 46 4D"
        " 13 0B 2B 09 82 04 34 37 39 53 83 01 06"
        " 13 0B 2B 09 82 04 34 37 39 52 83 01 04"
        " 13 1E 2B 1C 82 04 34 37 39 53 83 01 02 80 09 69 6D 6A 67 65 2F 70 6E 67 85 02 00 80 84 02 00 80"
        " 13 1E 2B 1C 82 04 34 37 39 4C 83 01 02 80 09 69 6D 6A 67 65 2F 70 6E 67 85 02 00 F0 84 02 01 40"
        " 29 08 80 06 40 E1 C1 85 C4 79";
    uint8_t exp[256], buf[256];
    const int ne = hexbytes(exp_hex, exp);
    dab_spi_w w;
    dab_spi_init(&w, buf, sizeof buf);
    dab_spi_open(&w, 0x03);
    dab_spi_open(&w, 0x26);
    const uint8_t id[3] = { 0xE1, 0xC1, 0x85 };
    dab_spi_attr(&w, 0x80, id, 3);
    dab_spi_text(&w, 0x10, "London 1");
    dab_spi_text(&w, 0x11, "London 1");
    dab_spi_open(&w, 0x28);
    dab_spi_text(&w, 0x10, "Capital");
    dab_spi_text(&w, 0x11, "Capital FM");
    static const struct { const char *url; int type, h, w; } md[4] = {
        { "479S", 6, 0, 0 }, { "479R", 4, 0, 0 }, { "479S", 2, 128, 128 }, { "479L", 2, 240, 320 }
    };
    for (int i = 0; i < 4; i++){
        dab_spi_open(&w, 0x13);
        dab_spi_open(&w, 0x2B);
        dab_spi_attr(&w, 0x82, md[i].url, 4);
        dab_spi_attr_uint(&w, 0x83, (unsigned)md[i].type, 1);
        if (md[i].h){
            dab_spi_attr(&w, 0x80, "imjge/png", 9);
            dab_spi_attr_uint(&w, 0x85, (unsigned)md[i].h, 2);
            dab_spi_attr_uint(&w, 0x84, (unsigned)md[i].w, 2);
        }
        dab_spi_close(&w);
        dab_spi_close(&w);
    }
    dab_spi_open(&w, 0x29);
    uint8_t b[6];
    dab_spi_attr(&w, 0x80, b, dab_spi_bearer(b, 0xE1, 0xC185, 0xC479, 0));
    dab_spi_close(&w);
    dab_spi_close(&w);
    dab_spi_close(&w);
    dab_spi_close(&w);
    same(buf, dab_spi_end(&w), exp, ne, "TS 102 371 C.1: SI example (158 + 2 bytes)");
}

// TS 102 371 table C.2
static void annex_c2(void){
    static const char *exp_hex = "02 35 21 33 24 16 80 04 33 BF C4 40 81 04 33 BF C4 80 25 08 80 06 40 E1 CE 15 C2 24"
                                 " 1C 19 81 03 FA E4 51 11 04 01 02 50 4D 19 0C 2C 0A 80 04 33 BF C4 40 81 02 0E 10";
    uint8_t exp[128], buf[128], t[8];
    const int ne = hexbytes(exp_hex, exp);
    const long long t17 = (epg_days(2003, 12, 18)) * 86400 + 17 * 3600, t18 = t17 + 3600;
    dab_spi_w w;
    dab_spi_init(&w, buf, sizeof buf);
    dab_spi_open(&w, 0x02);
    dab_spi_open(&w, 0x21);
    dab_spi_open(&w, 0x24);
    dab_spi_attr(&w, 0x80, t, dab_spi_timepoint(t, t17, DAB_SPI_NO_LTO, 0));
    dab_spi_attr(&w, 0x81, t, dab_spi_timepoint(t, t18, DAB_SPI_NO_LTO, 0));
    dab_spi_open(&w, 0x25);
    dab_spi_attr(&w, 0x80, t, dab_spi_bearer(t, 0xE1, 0xCE15, 0xC224, 0));
    dab_spi_close(&w);
    dab_spi_close(&w);
    dab_spi_open(&w, 0x1C);
    dab_spi_attr_uint(&w, 0x81, 16442449, 3);
    dab_spi_text(&w, 0x11, "PM");
    dab_spi_open(&w, 0x19);
    dab_spi_open(&w, 0x2C);
    dab_spi_attr(&w, 0x80, t, dab_spi_timepoint(t, t17, DAB_SPI_NO_LTO, 0));
    dab_spi_attr_uint(&w, 0x81, 3600, 2);
    dab_spi_close(&w);
    dab_spi_close(&w);
    dab_spi_close(&w);
    dab_spi_close(&w);
    dab_spi_close(&w);
    same(buf, dab_spi_end(&w), exp, ne, "TS 102 371 C.2: PI example (53 + 2 bytes)");
    const uint8_t t_exp[4] = { 0x33, 0xBF, 0xC4, 0x40 };
    same(t, dab_spi_timepoint(t, t17, DAB_SPI_NO_LTO, 0), t_exp, 4, "timePoint 2003-12-18T17:00Z short form");
    // long form with LTO +1 h: same date/hours/minutes, LTO flag and UTC flag set, seconds 30, LTO byte 0x02
    const uint8_t t_exp2[7] = { 0x33, 0xBF, 0xDC, 0x40, 30 << 2, 0x00, 0x02 };
    same(t, dab_spi_timepoint(t, t17 + 30, 2, 1), t_exp2, 7, "timePoint long form with LTO +2 half hours");
    const uint8_t t_exp3[5] = { 0x33, 0xBF, 0xD4, 0x40, 0x23 };
    same(t, dab_spi_timepoint(t, t17, -3, 0), t_exp3, 5, "timePoint short form with LTO -3 half hours");
}

// an element with a value longer than 253 bytes: 0xFE and a 16-bit length
static void ext_len(void){
    static uint8_t buf[2000];
    char s[400];
    memset(s, 'x', 300);
    s[300] = 0;
    dab_spi_w w;
    dab_spi_init(&w, buf, sizeof buf);
    dab_spi_open(&w, 0x12);
    dab_spi_attr(&w, 0x01, s, 300);
    dab_spi_close(&w);
    const int n = dab_spi_end(&w);
    check(n == 2 + 2 + 2 + 2 + 300 && buf[1] == 0xFE && buf[2] == 0x01 && buf[3] == 0x30 && buf[4] == 0x01 && buf[5] == 0xFE &&
              buf[6] == 0x01 && buf[7] == 0x2C,
          "extended lengths (0xFE + 16 bit) for element and attribute");
}

static void planner(void){
    epg_plan *p = calloc(1, sizeof *p), *q = calloc(1, sizeof *q);
    char err[200];
    const char *txt = "# test\non=1\n"
                      "p=8F01|07:00|120|weekdays|||Morning Show with a rather long name|||Wake up\n"
                      "p=8F01|07:00|60|once:2026-10-26|||Special Monday||Spec|One-off\n"
                      "p=8F01|08:30|60|daily||2026-10-25|Ends Sunday\n"
                      "p=8F01|23:00|120|weekly:6,7|||Night Owl\n"
                      "p=8F02|12:00|30|weekend|2026-10-31||Lunch\n";
    check(epg_parse(txt, p, err, sizeof err) == 0 && p->n == 5, "planner: parse");
    static char out[20000];
    int n = epg_format(p, out, sizeof out);
    check(n > 0 && epg_parse(out, q, err, sizeof err) == 0 && q->n == 5 && !memcmp(p, q, sizeof *p), "planner: format and parse back");
    char m[80], s[40];
    epg_medium(&p->e[0], m, sizeof m);
    epg_short(&p->e[0], s, sizeof s);
    check(!strcmp(m, "Morning Show") && !strcmp(s, "Morning"), "planner: medium/short names cut from the long name at a space");
    epg_short(&p->e[1], s, sizeof s);
    check(!strcmp(s, "Spec"), "planner: short name as entered");
    const char *bad[] = { "p=8F01|25:00|10|daily|||x\n", "p=8F01|07:00|0|daily|||x\n", "p=8F01|07:00|10|sometimes|||x\n",
                          "p=8F01|07:00|10|once:2026-02-30|||x\n", "p=8F01|07:00|10|daily|||\n", "p=XYZ|07:00|10|daily|||x\n",
                          "p=8F01|07:00|10|weekly:9|||x\n", "q=1\n" };
    int allbad = 1;
    for (unsigned i = 0; i < sizeof bad / sizeof bad[0]; i++) allbad &= epg_parse(bad[i], q, err, sizeof err) != 0;
    check(allbad, "planner: bad lines refused");
    // Monday 2026-10-26, LTO +1 h: the one-off replaces the weekday show at 07:00 and is cut by... nothing (60 min);
    // 08:30 daily ended on Sunday; Sunday's 23:00 Night Owl runs into Monday (carried)
    epg_prog pr[16];
    const long long mon = epg_days(2026, 10, 26);
    int np = epg_day(p, 0x8F01, mon, 2, 1, pr, 16);
    check(np == 2 && pr[0].entry == 3 && pr[0].dur == 120 && pr[1].entry == 1 &&
              pr[1].start == (mon * 1440 + 7 * 60) * 60 - 3600,
          "planner: Monday = Sunday's night show carried + the one-off at 07:00 (UTC = local - 1 h)");
    // Tuesday: weekday show (120 min), no 08:30 (until Sunday)
    np = epg_day(p, 0x8F01, mon + 1, 2, 0, pr, 16);
    check(np == 1 && pr[0].entry == 0 && pr[0].dur == 120, "planner: Tuesday = the weekday show");
    // Friday 2026-10-23: weekday show 07:00-09:00 is cut at 08:30 by the daily show
    np = epg_day(p, 0x8F01, epg_days(2026, 10, 23), 2, 0, pr, 16);
    check(np == 2 && pr[0].dur == 90 && pr[1].entry == 2, "planner: a programme is cut at the start of the next");
    np = epg_day(p, 0x8F02, epg_days(2026, 10, 25), 2, 0, pr, 16);
    int np2 = epg_day(p, 0x8F02, epg_days(2026, 11, 1), 2, 0, pr, 16);
    check(np == 0 && np2 == 1, "planner: from date");
    free(p);
    free(q);
}

int main(int argc, char **argv){
    annex_c1();
    annex_c2();
    ext_len();
    planner();
    if (argc > 1){ // generate the EPG of a planner for a time
        const long long now = argc > 2 ? atoll(argv[2]) : 1792836000LL;
        static epg_plan p;
        mux_conf m;
        char err[200], msg[300];
        if (argc > 3 && strcmp(argv[3], "-")){
            if (epg_read(argv[3], &p, err, sizeof err)){
                fprintf(stderr, "%s: %s\n", argv[3], err);
                return 2;
            }
        } else {
            const char *txt = "on=1\n"
                              "p=8F01|00:00|360|daily|||Night Programme|Nachtprogramma|Nacht|Music through the night\n"
                              "p=8F01|06:00|180|weekdays|||Good Morning Holland with the news every half hour|||News, traffic, weather and music, with the headlines every half hour. Ünïcödé: café, 東京\n"
                              "p=8F01|09:00|180|daily|||Late Morning||Morning|\n"
                              "p=8F01|12:00|60|daily|||Lunch Hour|||\n"
                              "p=8F01|13:00|300|daily|||Afternoon Show|||Pop and requests\n"
                              "p=8F01|18:00|360|daily|||Evening and Night|||\n"
                              "p=8F01|20:00|60|once:2026-10-24|||Special Concert|||Live from the studio\n"
                              "p=8F02|07:00|120|daily|||Second Service Morning|||\n";
            if (epg_parse(txt, &p, err, sizeof err)){
                fprintf(stderr, "planner: %s\n", err);
                return 2;
            }
        }
        if (argc > 4){
            if (mux_read(argv[4], &m, err, sizeof err)){
                fprintf(stderr, "%s: %s\n", argv[4], err);
                return 2;
            }
        } else {
            mux_default(&m);
            m.eid = 0x8FED;
            snprintf(m.label, sizeof m.label, "PlutoSky DAB");
            m.s[0].sid = 0x8F01;
            snprintf(m.s[0].label, sizeof m.s[0].label, "PlutoSky Test");
            m.n = 2;
            m.s[1] = m.s[0];
            m.s[1].src = MUX_SRC_ZMQ;
            m.s[1].port = 2491;
            m.s[1].sid = 0x8F02;
            snprintf(m.s[1].label, sizeof m.s[1].label, "Second");
        }
        int n = epg_generate(&p, &m, now, 7, argv[1], msg, sizeof msg);
        printf("generate: %d objects: %s\n", n, msg);
        static char js[65536];
        int jl = epg_json(&p, &m, now, js, sizeof js);
        char fn[400];
        snprintf(fn, sizeof fn, "%s/plan.json", argv[1]);
        FILE *f = fopen(fn, "w");
        if (f && jl > 0) fwrite(js, 1, (size_t)jl, f);
        if (f) fclose(f);
        check(n > 0 && jl > 0, "planner: EPG generated");
    }
    printf("%s (%d failed)\n", fails ? "FAIL" : "PASS", fails);
    return fails ? 1 : 0;
}
