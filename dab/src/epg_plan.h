// SPDX-License-Identifier: Apache-2.0
// The EPG planner: programme rules per service (stored in one file, edited in skypluto-ctl's web interface), expanded into the
// programmes of a local day and encoded as SPI objects (dab_spi.h) for skypluto-dab's carousel (-E dir).
//
// The planner file (/mnt/jffs2/epg.conf on the Pluto), one item per line, '#' = comment:
//   on=1                                              1: the EPG is broadcast, 0: not
//   p=8F01|07:00|120|weekdays|||Morning Show|||Wake up with the news
//      SId|start|minutes|repeat|from|until|long name|medium name|short name|description
// SId (hex) of a service in the multiplex layout; start = local time HH:MM; minutes 1..1080; repeat: daily, weekdays
// (Mon-Fri), weekend (Sat, Sun), weekly:<days> (1 = Monday .. 7 = Sunday, e.g. weekly:1,3,5) or once:YYYY-MM-DD; from/until:
// optional first/last date (YYYY-MM-DD, empty = open). Long name 1..128 characters, medium name <= 16 and short name <= 8
// (empty = cut from the long name), description <= 180; UTF-8, no '|' and no control characters.
//
// Local time = UTC + the layout's local time offset (lto, in half hours; FIG 0/9). There is no time zone database: on a
// change of daylight saving time the encoder sends the new offset (TM <utc> <offset>) and the next generation uses it.
//
// On a day, a rule applies when its repeat matches the day and the day lies within from/until. At the same start time the
// more specific rule wins (once > weekly/weekdays/weekend > daily; then the earlier line); a programme that runs into the next
// one is cut at its start.
#ifndef EPG_PLAN_H
#define EPG_PLAN_H

#include "mux_conf.h"
#include <stddef.h>

#define EPG_MAX_ENTRIES 128
#define EPG_LONG_CHARS  128
#define EPG_MED_CHARS   16
#define EPG_SHORT_CHARS 8
#define EPG_DESC_CHARS  180
#define EPG_MAX_MIN     1080             // 18 h (the SPI duration is 16-bit seconds)
#define EPG_CLOCK_VALID 1735689600LL     // 2025-01-01: an earlier clock is not set (as for FIG 0/10)

enum { EPG_ONCE = 0, EPG_DAILY, EPG_WEEKDAYS, EPG_WEEKEND, EPG_WEEKLY };

typedef struct {
    unsigned sid;
    int start, dur;                      // minutes after local midnight, minutes
    int rep, days;                       // repeat, weekly days (bit 0 = Monday .. bit 6 = Sunday)
    int date, from, until;               // days since 1970-01-01 (once: the date); from/until: -1 = open
    char lname[4 * EPG_LONG_CHARS + 1], mname[4 * EPG_MED_CHARS + 1], sname[4 * EPG_SHORT_CHARS + 1],
        desc[4 * EPG_DESC_CHARS + 1];    // as entered (medium/short may be empty)
} epg_entry;

typedef struct {
    int on, n;
    epg_entry e[EPG_MAX_ENTRIES];
} epg_plan;

// one programme of a day
typedef struct {
    long long start;                     // UTC seconds
    int dur;                             // minutes (after cutting at the next programme)
    int entry;                           // index in the plan
} epg_prog;

int  epg_parse(const char *text, epg_plan *p, char *err, size_t errlen);   // 0 = ok, -1 with the reason (line number)
int  epg_format(const epg_plan *p, char *out, size_t cap);                 // the file text; its length, -1 if it does not fit
int  epg_read(const char *file, epg_plan *p, char *err, size_t errlen);
int  epg_write(const char *file, const epg_plan *p);                       // written whole and renamed
// medium (<= 16) / short (<= 8 characters) name of an entry: as entered, or cut from the long name (at a space if possible)
void epg_medium(const epg_entry *e, char *out, int size);
void epg_short(const epg_entry *e, char *out, int size);
// the programmes of service sid that start on local day 'day' (days since 1970), in order; with carry = 1 also the one from
// the day before that is still running at midnight. lto in half hours. Returns the count.
int  epg_day(const epg_plan *p, unsigned sid, long long day, int lto, int carry, epg_prog *out, int max);
// Builds the SPI objects for the layout m at time now (UTC): the SI object and one PI object per service and local day (today,
// tomorrow) that has programmes, into dir (files + 'index', the index last). Nothing (the index removed) when the plan is
// off, the clock is not valid (before 2025) or no service has programmes. version: the objects' version (16 bit). Returns
// the number of objects (0 = no EPG); msg tells why/what.
int  epg_generate(const epg_plan *p, const mux_conf *m, long long now, unsigned version, const char *dir, char *msg,
                  size_t msglen);
// the plan and the programmes of today and tomorrow as JSON (for GET /api/epg); returns the length, -1 if it does not fit
int  epg_json(const epg_plan *p, const mux_conf *m, long long now, char *out, int cap);
// days since 1970 <-> YYYY-MM-DD
long long epg_days(int y, int mo, int d);
void epg_ymd(long long days, int *y, int *mo, int *d);

#endif
