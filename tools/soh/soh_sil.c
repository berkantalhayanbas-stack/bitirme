/* soh_sil.c -- kapasite SOH'unun uzun süreli SIL testi (docs/soh_design.md, doğrulama 1).
 * İkiz (bms/sim/twin_model.c) + ECU (bms_step + soh_step, bms/common) kapalı çevrimde, günlerce:
 * her gün = açılış (kalıcı kayıttan) + park + sürüş (mc_cycle, SOC ~%45 düşene kadar) + park + şarj
 * (%90'a) + park. Gerçek SOH günler boyunca doğrusal düşer; ECU yeniden başlatmada yalnız kaydı bilir.
 * Kullanım: soh_sil <gün> <son SOH> <geri besleme 0|1> <olay.csv> <gün.csv> [iz.csv iz_gün]
 *   geri besleme 1: öğrenilen kapasite EKF'ye verilir (bms_set_capacity), 0: EKF nominalde kalır.
 *   iz.csv: ilk iz_gün günün her adımı (Python eşdeğerlik testi, twin/soh_capacity.py).
 * tools/soh/soh_run.py derler, koşturur, özet ve figür üretir. */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "bms_config.h"
#include "bms_algo.h"
#include "twin_model.h"
#include "mc_cycle.h"

static uint32_t rng = 0x12345678u;
static float urand(void) { rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5; return (float)(rng >> 8) * (1.0f / 16777216.0f); }
static float grand(void) { return (urand() + urand() + urand() + urand() - 2.0f) * 1.7320508f; }
static float quant(float x, float lsb) { const float y = x / lsb; return (float)(int32_t)(y + (y < 0.0f ? -0.5f : 0.5f)) * lsb; }

static twin_t tw;
static bms_t bms;
static soh_t soh;
static soh_rec_t rec;               /* "flash": yalnız persist istenince yazılır */
static FILE *fev, *ftr;
static int day, fb, trace, boot;
static float soh_true, soc_err_max, i_dis, i_chg;
static uint32_t k_glob;

static float soc_ocv_cell(float soc)    /* ECU polinomu (bms_config.h OCV_POLY) */
{
    float y = OCV_POLY[0];
    for (int k = 1; k < OCV_POLY_N; k++) y = y * soc + OCV_POLY[k];
    return y;
}

static void step(float I_req)
{
    float I = I_req;
    if (I > i_dis) I = i_dis;
    if (I < -i_chg) I = -i_chg;
    twin_step(&tw, I, 0u, 0.0f, 1.0f);
    bms_in_t in;
    in.V = quant(tw.v_pack + 0.25f * grand(), 0.1f);
    in.I = quant(I + 0.8f + 0.5f * grand(), 0.1f);          /* sabit 0,8 A sensör ofseti (A9 telafi) */
    in.T = quant(tw.ts + 0.3f * grand(), 0.1f);
    in.V_min = quant(tw.v_min + 0.005f * grand(), 0.001f);
    in.V_max = quant(tw.v_max + 0.005f * grand(), 0.001f);
    in.T_amb = tw.t_amb;
    in.can_ok = 1u;
    bms_out_t o;
    bms_step(&bms, &in, &o);
    soh_step(&soh, &in, &o);
    i_dis = o.dec.i_allow_dis;  i_chg = o.dec.i_allow_chg;
    if (soh.persist) { rec = soh.r;  soh.persist = 0u;  soh.ah_saved = rec.ah_total;  soh.kr_saved = rec.k_r; }
    if (soh.event == SOH_EV_ACCEPT && fb) bms_set_capacity(&bms, soh.r.soh_c);
    if (soh.event != SOH_EV_NONE)
        fprintf(fev, "%d,%.1f,%d,%.5f,%.5f,%.5f,%.5f,%.4f,%u,%.4f,%.4f\n", day, k_glob * 0.1, soh.event, soh_true,
                soh.r.soh_c, soh.r.soh_disp, soh.last_ratio, soh.last_soc, (unsigned)soh.r.n_meas,
                tw.soc_avg, twin_ocv(tw.soc_avg) - soc_ocv_cell(tw.soc_avg));
    if (ftr && day < trace)
        fprintf(ftr, "%.9g,%.9g,%.9g,%.9g,%u,%u,%u,%d,%.9g,%.9g,%u\n", in.I, in.V, in.T, o.kr,
                o.t_valid, o.i_valid, in.can_ok, boot, soh.r.soh_c, soh.r.soh_disp, soh.event);
    boot = 0;
    const float e = fabsf(o.soc - tw.soc_avg);
    if (bms.ekf.n_kr >= EKF_N_KR_FREEZE && e > soc_err_max) soc_err_max = e;
    k_glob++;
}

static void park(int seconds) { for (int k = 0; k < seconds * 10; k++) step(0.0f); }

int main(int argc, char **argv)
{
    if (argc < 6) { fprintf(stderr, "usage: soh_sil days soh_end feedback events.csv days.csv [trace.csv trace_days]\n"); return 1; }
    const int days = atoi(argv[1]);
    const float soh_end = (float)atof(argv[2]);
    fb = atoi(argv[3]);
    fev = fopen(argv[4], "w");
    FILE *fd = fopen(argv[5], "w");
    if (argc > 7) { ftr = fopen(argv[6], "w"); trace = atoi(argv[7]); }
    fprintf(fev, "day,t_s,event,soh_true,soh_c,soh_disp,ratio,soc_ocv,n_meas,soc_true,ocv_poly_err\n");
    fprintf(fd, "day,soh_true,soh_c,soh_disp,n_meas,efc,soc_err_max\n");
    if (ftr) fprintf(ftr, "I,V,T,k_r,t_valid,i_valid,can_ok,boot,soh_c,soh_disp,event\n");

    twin_init(&tw, 0.90f, 25.0f, BMS_TS);
    soh_init(&soh, 0);
    rec = soh.r;
    uint32_t off = 0u;
    for (day = 0; day < days; day++) {
        soh_true = 1.0f - (1.0f - soh_end) * (float)day / (float)(days > 1 ? days - 1 : 1);
        tw.soh = soh_true;
        /* açılış: app_ecu ile aynı (EKF %50'den), öğrenilmiş değerler kayıttan */
        bms_init(&bms, 0.5f, 1.0f, 1.0f, 0.0f, 0.0f);
        soh_init(&soh, &rec);
        boot = 1;
        if (fb) bms_set_capacity(&bms, rec.soh_c);
        bms_set_kr(&bms, rec.k_r);
        i_dis = i_chg = 0.0f;
        soc_err_max = 0.0f;
        park(1500);
        const float soc0 = tw.soc_avg;
        for (int k = 0; tw.soc_avg > soc0 - 0.45f && tw.soc_avg > 0.15f && k < 4 * 36000; k++) {
            const uint32_t s = (uint32_t)(k / 10) + off;
            step((float)MC_CYCLE[s % MC_CYCLE_N]);
        }
        off += 997u;
        park(1500);
        for (int k = 0; tw.soc_avg < 0.90f && k < 4 * 36000; k++) step(-0.5f * SOH_Q_NOM);
        park(1500);
        fprintf(fd, "%d,%.5f,%.5f,%.5f,%u,%.2f,%.4f\n", day, soh_true, soh.r.soh_c, soh.r.soh_disp,
                (unsigned)soh.r.n_meas, soh.r.ah_total / (2.0f * SOH_Q_NOM), soc_err_max);
    }
    fclose(fev);  fclose(fd);
    if (ftr) fclose(ftr);
    printf("days %d  soh_true_end %.4f  soh_c %.4f  soh_disp %.4f  n_meas %u  efc %.1f\n", days, soh_true,
           soh.r.soh_c, soh.r.soh_disp, (unsigned)soh.r.n_meas, soh.r.ah_total / (2.0f * SOH_Q_NOM));
    return 0;
}
