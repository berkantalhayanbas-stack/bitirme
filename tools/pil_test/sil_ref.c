/* sil_ref.c -- PC SIL referansı: bms/common'ı PC'de derleyip data/<senaryo>_in.csv'yi app_pil.c ile
 * aynı şekilde (bms_init(0.5, 1, 1, 0, 0), can_ok = 1) koşturur, data/<senaryo>_c.csv'yi yazar.
 * pil_test.py / can_replay.py kartın çıktısını bununla karşılaştırır.
 *
 * Derleme + çalıştırma: python tools/pil_test/sil_ref.py   (MATLAB'ın MinGW-w64 gcc'si ya da PATH'teki gcc)
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "bms_algo.h"

int main(int argc, char **argv)
{
    if (argc < 3) { fprintf(stderr, "kullanim: sil_ref <in.csv> <out.csv>\n"); return 2; }
    FILE *fi = fopen(argv[1], "r"), *fo = fopen(argv[2], "w");
    if (!fi || !fo) { fprintf(stderr, "dosya acilamadi\n"); return 1; }
    static bms_t bms;
    char line[256];
    bms_init(&bms, 0.5f, 1.0f, 1.0f, 0.0f, 0.0f);
    if (!fgets(line, sizeof line, fi)) return 1;                       /* başlık */
    fprintf(fo, "soc,bias,innov,rej,tc,ts,ru,tv,iv,rr,state,i_dis,i_chg,psi,cause\n");
    while (fgets(line, sizeof line, fi)) {
        float v[6];
        if (sscanf(line, "%f,%f,%f,%f,%f,%f", &v[0], &v[1], &v[2], &v[3], &v[4], &v[5]) != 6) continue;
        bms_in_t in = { v[0], v[1], v[2], v[3], v[4], v[5], 1 };
        bms_out_t o;
        bms_step(&bms, &in, &o);
        fprintf(fo, "%.7g,%.7g,%.7g,%d,%.7g,%.7g,%.7g,%d,%d,%.7g,%d,%.7g,%.7g,%.7g,%d\n",
                o.soc, o.bias, o.innov, o.rejected, o.tc, o.ts, o.ru, o.t_valid, o.i_valid, o.rr,
                o.dec.state, o.dec.i_allow_dis, o.dec.i_allow_chg, o.dec.psi, o.dec.cause);
    }
    fclose(fi);  fclose(fo);
    return 0;
}
