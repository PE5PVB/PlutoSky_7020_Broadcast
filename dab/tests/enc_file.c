// SPDX-License-Identifier: Apache-2.0
// Encodes a raw PCM file (float32, interleaved) to DAB+ super frames.
//   enc_file <in.f32> <channels> <dac_rate 0|1> <kbit/s> <out prefix>
// writes <prefix>.sf (super frames), <prefix>.aus ([u16 len][AU]...), <prefix>.asc (AudioSpecificConfig).
#include "../src/dabplus_enc.h"
#include "../src/aac_enc.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main(int argc, char **argv){
    if (argc < 6){
        fprintf(stderr, "usage: enc_file in.f32 channels dac_rate kbps outprefix\n");
        return 2;
    }
    const int ch = atoi(argv[2]);
    dabplus_cfg cfg = { atoi(argv[4]), atoi(argv[3]), 0, 0, ch == 2 };
    dabplus_enc *e = dabplus_enc_open(&cfg, 0);
    if (!e){
        fprintf(stderr, "bad configuration\n");
        return 2;
    }
    FILE *fi = fopen(argv[1], "rb");
    if (!fi){
        perror(argv[1]);
        return 2;
    }
    char fn[1024];
    snprintf(fn, sizeof fn, "%s.sf", argv[5]);
    FILE *fsf = fopen(fn, "wb");
    snprintf(fn, sizeof fn, "%s.aus", argv[5]);
    FILE *fau = fopen(fn, "wb");
    snprintf(fn, sizeof fn, "%s.asc", argv[5]);
    FILE *fasc = fopen(fn, "wb");
    if (!fsf || !fau || !fasc){
        perror("output");
        return 2;
    }
    uint8_t asc[2];
    dabplus_enc_asc(e, asc);
    fwrite(asc, 1, 2, fasc);
    fclose(fasc);
    const int nf = dabplus_enc_frames(e), ns = nf * 960 * ch;
    float *pcm = calloc((size_t)ns, sizeof(float));
    uint8_t *sf = malloc((size_t)dabplus_total_bytes(&cfg));
    int nsf = 0, fails = 0;
    for (;;){
        size_t got = fread(pcm, sizeof(float), (size_t)ns, fi);
        if (got == 0) break;
        if ((int)got < ns) memset(pcm + got, 0, sizeof(float) * (size_t)(ns - (int)got));
        if (dabplus_enc_superframe(e, pcm, sf) != 0){
            fails++;
            fprintf(stderr, "super frame %d failed\n", nsf);
            break;
        }
        fwrite(sf, 1, (size_t)dabplus_total_bytes(&cfg), fsf);
        const uint8_t *au[6];
        int len[6];
        int n = dabplus_enc_last_aus(e, au, len);
        for (int i = 0; i < n; i++){
            uint8_t l[2] = { (uint8_t)(len[i] >> 8), (uint8_t)len[i] };
            fwrite(l, 1, 2, fau);
            fwrite(au[i], 1, (size_t)len[i], fau);
        }
        nsf++;
    }
    fclose(fsf);
    fclose(fau);
    fclose(fi);
    printf("%d super frames (%d kbit/s, %s, %d kHz)%s\n", nsf, cfg.bitrate_kbps, ch == 2 ? "stereo" : "mono",
           cfg.dac_rate ? 48 : 32, fails ? " FAILED" : "");
    dabplus_enc_close(e);
    free(pcm);
    free(sf);
    return fails ? 1 : 0;
}
