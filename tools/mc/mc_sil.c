/* mc_sil.c -- Monte Carlo'nun PC (SIL) sürümü: bms/sim/mc_core.c + bms/common, kartla aynı kod.
 * Kullanım: mc_sil <n_runs> <base_seed> [first_id]   -> stdout: RS/RE satırları (kartla aynı biçim,
 * son alan koşu süresi [ms]; PC'de duvar saati yerine 0). tools/mc_run.py derler ve karşılaştırır. */
#include <stdio.h>
#include <stdlib.h>
#include "mc_core.h"

int main(int argc, char **argv)
{
    const unsigned n = argc > 1 ? (unsigned)strtoul(argv[1], 0, 10) : 100u;
    const unsigned seed = argc > 2 ? (unsigned)strtoul(argv[2], 0, 10) : 1u;
    const unsigned first = argc > 3 ? (unsigned)strtoul(argv[3], 0, 10) : 1u;
    char buf[160];
    for (unsigned id = first; id < first + n; id++) {
        mc_cfg_t c;
        mc_res_t r;
        mc_config(id, seed, &c);
        mc_line_rs(&c, buf);
        puts(buf);
        mc_run(&c, &r);
        mc_line_re(&c, &r, buf);
        printf("%s,0\n", buf);
    }
    return 0;
}
