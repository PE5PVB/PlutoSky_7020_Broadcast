// SPDX-License-Identifier: Apache-2.0
// Complete DAB+ transmitter on a file: PCM (float32 stereo, interleaved) -> DAB+ audio super frames -> MSC + FIC ->
// OFDM mode I, written as complex float32 IQ at 2.048 MS/s (name it *.cf32.iq for welle-cli).
//   dab_tx_file <in.f32> <48000|32000> <kbit/s> <out.cf32.iq> [service label] [ensemble label] [DLS text]
#include "../src/dabplus_enc.h"
#include "../src/dab_mod.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main(int argc, char **argv){
    if (argc < 5){ fprintf(stderr, "usage: dab_tx_file in.f32 rate kbps out.cf32.iq [service] [ensemble]\n"); return 2; }
    const int rate = atoi(argv[2]), kbps = atoi(argv[3]);
    dabplus_cfg cfg = { kbps, rate == 48000, 0, 0, 1 };
    dabplus_enc *enc = dabplus_enc_open(&cfg, 0);
    dab_ensemble ens; memset(&ens, 0, sizeof ens);
    ens.eid = 0x8FED; snprintf(ens.label, sizeof ens.label, "%s", argc > 6 ? argv[6] : "PlutoSky DAB");
    ens.ecc = 0xE3; ens.lto_halfhours = 2; ens.nsvc = 1;
    ens.svc[0].sid = 0x8F01; snprintf(ens.svc[0].label, sizeof ens.svc[0].label, "%s", argc > 5 ? argv[5] : "PlutoSky Test");
    ens.svc[0].bitrate = kbps; ens.svc[0].prot_level = 3;
    dab_mod *mod = dab_mod_open(&ens);
    if (!enc || !mod){ fprintf(stderr, "bad configuration\n"); return 2; }
    if (argc > 7) dabplus_enc_set_dls(enc, argv[7], 15);
    FILE *fi = fopen(argv[1], "rb"), *fo = fopen(argv[4], "wb");
    if (!fi || !fo){ perror("file"); return 2; }
    const int nf = dabplus_enc_frames(enc), ns = nf * 960 * 2, sfb = dabplus_total_bytes(&cfg), lfb = dab_mod_lf_bytes(mod, 0);
    float *pcm = calloc((size_t)ns, sizeof(float));
    uint8_t *sf = malloc((size_t)sfb);
    int nsf = 0, frames = 0, eof = 0;
    while (!eof){
        size_t got = fread(pcm, sizeof(float), (size_t)ns, fi);
        if (got < (size_t)ns){ memset(pcm + got, 0, sizeof(float) * (ns - got)); eof = 1; if (got == 0 && nsf > 0) break; }
        if (dabplus_enc_superframe(enc, pcm, sf)){ fprintf(stderr, "encoder failed\n"); return 1; }
        nsf++;
        for (int k = 0; k < 5; k++){                                   // a super frame is 5 logical frames (CIFs)
            const uint8_t *lf[1] = { sf + k * lfb };
            if (dab_mod_push_cif(mod, lf)){ fwrite(dab_mod_frame(mod), sizeof(dab_cf), DAB_FRAME_SAMPLES, fo); frames++; }
        }
    }
    fclose(fi); fclose(fo);
    printf("%d super frames, %d transmission frames (%.1f s), %d of 864 CU used\n", nsf, frames, frames * 0.096, dab_mod_cu_used(mod));
    dab_mod_close(mod); dabplus_enc_close(enc); free(pcm); free(sf);
    return 0;
}
