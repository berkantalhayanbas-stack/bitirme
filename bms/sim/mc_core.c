/* mc_core.c -- kart üstü Monte Carlo (açıklama mc_core.h'de). STM32 (APP_MC) ve PC (tools/mc/mc_sil.c) ortak. */
#include "mc_core.h"
#include "mc_cycle.h"
#include "twin_model.h"
#include "bms_algo.h"
#include "bms_config.h"
#include <math.h>

/* ---------------------------------------------------------------- rastgele sayı (xorshift32)
 * Tamsayı işlemleri ve [0,1) dönüşümü iki platformda bit bit aynı: aynı tohum = aynı senaryo. */
static uint32_t rng;
static void  rng_seed(uint32_t s) { rng = s ? s : 0x9E3779B9u; for (int k = 0; k < 8; k++) { rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5; } }
static float urand(void) { rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5; return (float)(rng >> 8) * (1.0f / 16777216.0f); }
static float uni(float a, float b) { return a + (b - a) * urand(); }
static float grand(void) { return (urand() + urand() + urand() + urand() - 2.0f) * 1.7320508f; }   /* ~N(0,1), twin_emu ile aynı */

/* CAN kuantalaması (0x101: 0,1 V / 0,1 A / 0,1 °C, 0x102: 1 mV); libm roundf'e bağlı kalmamak için elle */
static float quant(float x, float lsb)
{
    const float y = x / lsb;
    return (float)(int32_t)(y + (y < 0.0f ? -0.5f : 0.5f)) * lsb;
}

/* ---------------------------------------------------------------- senaryo üretimi */
static const char *const FNAME[MC_F_N] = { "none", "cooling", "ext_heat", "ntc_open", "i_offset" };
const char *mc_fault_name(uint8_t f) { return f < MC_F_N ? FNAME[f] : "?"; }

void mc_config(uint32_t id, uint32_t base_seed, mc_cfg_t *c)
{
    c->id = id;
    c->seed = base_seed + id;
    rng_seed(c->seed);
    const float u = urand();
    c->fault = u < 0.30f ? MC_F_NONE : u < 0.50f ? MC_F_COOL : u < 0.65f ? MC_F_HEAT
             : u < 0.80f ? MC_F_NTC : MC_F_OFFSET;
    c->t_amb   = uni(0.0f, 40.0f);
    c->soc0    = uni(0.35f, 0.95f);
    c->k_cur   = uni(0.6f, 1.6f);                 /* sürüş sertliği (S2: 1,6) */
    c->kq      = uni(0.98f, 1.02f);               /* paket kapasitesi ±%2 */
    c->kr      = uni(0.95f, 1.05f);               /* direnç ±%5 */
    c->cyc_off = (uint32_t)(urand() * (float)MC_CYCLE_N) % MC_CYCLE_N;
    c->t_fault = uni(60.0f, 480.0f);
    /* akım sensörü ofseti: normalde |b| <= 2,5 A (A9 telafi eder); arızada 4-12 A, rastgele işaret */
    c->bias    = uni(-2.5f, 2.5f);
    if (c->fault == MC_F_OFFSET) {
        const float m = uni(4.0f, 12.0f);
        c->bias = urand() < 0.5f ? -m : m;
    }
    rng_seed(c->seed ^ 0xA5A5A5A5u);              /* ölçüm gürültüsü ayrı akış */
}

/* ---------------------------------------------------------------- tek koşu */
static twin_t tw;
static bms_t  bms;

static float cycle_i(float t_s, uint32_t off)
{
    const uint32_t s = (uint32_t)t_s;
    const float f = t_s - (float)s;
    const float a = MC_CYCLE[(s + off) % MC_CYCLE_N], b = MC_CYCLE[(s + off + 1u) % MC_CYCLE_N];
    return a + (b - a) * f;
}

void mc_run(const mc_cfg_t *c, mc_res_t *r)
{
    const float dt = BMS_TS;
    twin_init(&tw, c->soc0, c->t_amb, dt);
    tw.kq = c->kq;  tw.kr = c->kr;
    bms_init(&bms, 0.5f, 1.0f, 1.0f, 0.0f, 0.0f);   /* app_ecu ile aynı: EKF %50'den başlar */
    float i_dis = 0.0f, i_chg = 0.0f;
    r->min_soc = 2.0f;  r->max_tc = -99.0f;  r->soc_err = 0.0f;
    r->tc_limp = r->tc_safe = -99.0f;
    r->t_limp = r->t_safe = r->t_cool = r->t_sens = -1;
    bms_out_t o = { 0 };
    int32_t tf = -1;                                  /* ilk arızalı örneğin zamanı [ms] */

    for (uint32_t k = 0; k < MC_STEPS; k++) {
        const float t = (float)k * dt;
        const int32_t t_ms = (int32_t)(k * 100u);
        const uint8_t fault_on = c->fault != MC_F_NONE && c->fault != MC_F_OFFSET && t >= c->t_fault;
        if (fault_on && tf < 0) tf = t_ms;
        /* ---- ikiz: akım talebi ECU izniyle kırpılır (closed_loop.py / twin_emu ile aynı) */
        float I = c->k_cur * cycle_i(t, c->cyc_off);
        if (I > i_dis) I = i_dis;
        if (I < -i_chg) I = -i_chg;
        if (c->fault == MC_F_COOL && fault_on) tw.ru_k = 4.0f;
        twin_step(&tw, I, 0u, 0.0f, 1.0f);
        if (c->fault == MC_F_HEAT && fault_on && tw.ts < 62.0f) {   /* twin_emu A2: 0,5 °C/s, 62 °C'ye */
            const float e = 62.0f - tw.ts, d = e > 0.05f ? 0.05f : e;
            tw.ts += d;  tw.tc += d;
        }
        /* ---- ölçüm (0x101/0x102): gürültü + ofset + kuantalama */
        bms_in_t in;
        in.V     = quant(tw.v_pack + 0.25f * grand(), 0.1f);
        in.I     = quant(I + c->bias + 0.5f * grand(), 0.1f);
        in.T     = quant(tw.ts + 0.3f * grand(), 0.1f);
        in.V_min = quant(tw.v_min + 0.005f * grand(), 0.001f);
        in.V_max = quant(tw.v_max + 0.005f * grand(), 0.001f);
        in.T_amb = c->t_amb;
        in.can_ok = 1u;
        if (c->fault == MC_F_NTC && fault_on) in.T = -40.0f;
        /* ---- ECU */
        bms_step(&bms, &in, &o);
        i_dis = o.dec.i_allow_dis;  i_chg = o.dec.i_allow_chg;
        /* ---- kayıt */
        if (tw.soc_min < r->min_soc) r->min_soc = tw.soc_min;
        if (tw.tc > r->max_tc) r->max_tc = tw.tc;
        if (t >= 120.0f) {
            const float e = fabsf(o.soc - tw.soc_avg);
            if (e > r->soc_err) r->soc_err = e;
        }
        if (o.dec.state == ST_LIMP && r->t_limp < 0) { r->t_limp = t_ms;  r->tc_limp = tw.tc; }
        if (o.dec.state == ST_SAFE && r->t_safe < 0) { r->t_safe = t_ms;  r->tc_safe = tw.tc; }
        if ((o.dec.cause & 2u) && r->t_cool < 0) r->t_cool = t_ms;
        if ((o.dec.cause & 32u) && r->t_sens < 0) r->t_sens = t_ms;
    }
    r->final_state = o.dec.state;

    /* ---------------------------------------------------------------- hüküm
     * Her arızanın beklenen tepkisi ve süresi; arızasız koşuda yanlış alarm yok. Tümünde SOC hatası
     * (120 s sonra) < %4 ve beklenmeyen SAFE yok. */
    r->react_ms = -1;
    r->why = "-";
    uint8_t ok = 1;
    if (r->soc_err > 0.04f) { ok = 0; r->why = "soc_err"; }
    switch (c->fault) {
    case MC_F_NONE:
        if (r->t_limp >= 0 || r->t_safe >= 0 || r->t_cool >= 0 || r->t_sens >= 0) { ok = 0; r->why = "false_alarm"; }
        break;
    case MC_F_COOL:                       /* güvenlik: çekirdek 55 °C'yi aşmaz, SAFE yok; tepki = alarm */
        if (r->t_cool >= 0) r->react_ms = r->t_cool - tf;
        if (r->max_tc > 56.0f) { ok = 0; r->why = "overtemp"; }
        if (r->t_safe >= 0) { ok = 0; r->why = "unexpected_safe"; }
        if (r->t_sens >= 0) { ok = 0; r->why = "false_sensor"; }
        break;
    case MC_F_HEAT:                       /* LIMP gerçek çekirdek <= 57 °C, SAFE <= 62 °C */
        if (r->t_safe >= 0) r->react_ms = r->t_safe - tf;
        if (r->t_safe < 0) { ok = 0; r->why = "no_safe"; }
        else if (r->tc_safe > 62.0f) { ok = 0; r->why = "late_safe"; }
        if (r->t_limp < 0 && r->t_safe >= 0 && r->tc_safe > 57.0f) { ok = 0; r->why = "no_limp"; }
        if (r->t_limp >= 0 && r->tc_limp > 57.0f) { ok = 0; r->why = "late_limp"; }
        break;
    case MC_F_NTC:                        /* 0,5 s içinde sensör nedeniyle LIMP, SAFE yok */
        if (r->t_sens >= 0) r->react_ms = r->t_sens - tf;
        if (r->t_sens < 0 || r->react_ms > 500) { ok = 0; r->why = "late_detect"; }
        if (r->t_safe >= 0) { ok = 0; r->why = "unexpected_safe"; }
        break;
    case MC_F_OFFSET:                     /* A9: açılış kontrolü 1 s içinde sensör arızası */
        r->react_ms = r->t_sens;
        if (r->t_sens < 0 || r->t_sens > 1000) { ok = 0; r->why = "late_detect"; }
        if (r->t_safe >= 0) { ok = 0; r->why = "unexpected_safe"; }
        break;
    default: break;
    }
    if (c->fault != MC_F_OFFSET && c->fault != MC_F_NTC && c->fault != MC_F_NONE && r->t_sens >= 0) {
        ok = 0; r->why = "false_sensor";
    }
    r->pass = ok;
}

/* ---------------------------------------------------------------- satır biçimleme (printf yok) */
static char *p_str(char *p, const char *s) { while (*s) *p++ = *s++; return p; }
static char *p_int(char *p, int32_t v)
{
    char b[12];
    int i = 0;
    uint32_t u = v < 0 ? (uint32_t)(-v) : (uint32_t)v;
    if (v < 0) *p++ = '-';
    do { b[i++] = (char)('0' + u % 10u); u /= 10u; } while (u);
    while (i) *p++ = b[--i];
    return p;
}
static char *p_fix(char *p, float v, int dec)
{
    static const float sc[] = { 1.0f, 10.0f, 100.0f, 1000.0f };
    const float s = sc[dec];
    int32_t x = (int32_t)(v * s + (v < 0.0f ? -0.5f : 0.5f));
    if (x < 0) { *p++ = '-'; x = -x; }
    p = p_int(p, x / (int32_t)s);
    if (dec) {
        *p++ = '.';
        int32_t f = x % (int32_t)s;
        for (int32_t d = (int32_t)s / 10; d > 0; d /= 10) { *p++ = (char)('0' + f / d); f %= d; }
    }
    return p;
}

uint32_t mc_line_rs(const mc_cfg_t *c, char *buf)
{
    char *p = p_str(buf, "RS,");
    p = p_int(p, (int32_t)c->id);
    p = p_str(p, ",");  p = p_str(p, mc_fault_name(c->fault));
    p = p_str(p, " Ta=");  p = p_fix(p, c->t_amb, 1);
    p = p_str(p, " SOC0=");  p = p_fix(p, 100.0f * c->soc0, 0);
    p = p_str(p, " k=");  p = p_fix(p, c->k_cur, 2);
    p = p_str(p, " b=");  p = p_fix(p, c->bias, 1);
    if (c->fault != MC_F_NONE && c->fault != MC_F_OFFSET) { p = p_str(p, " tf=");  p = p_fix(p, c->t_fault, 1); }
    *p = 0;
    return (uint32_t)(p - buf);
}

uint32_t mc_line_re(const mc_cfg_t *c, const mc_res_t *r, char *buf)
{
    char *p = p_str(buf, "RE,");
    p = p_int(p, (int32_t)c->id);
    p = p_str(p, r->pass ? ",PASS," : ",FAIL,");
    p = p_str(p, mc_fault_name(c->fault));  p = p_str(p, ",");
    p = p_int(p, r->react_ms);              p = p_str(p, ",");
    p = p_fix(p, 100.0f * r->min_soc, 1);   p = p_str(p, ",");
    p = p_fix(p, r->max_tc, 1);             p = p_str(p, ",");
    p = p_fix(p, 100.0f * r->soc_err, 2);   p = p_str(p, ",");
    p = p_int(p, r->final_state);           p = p_str(p, ",");
    p = p_fix(p, r->tc_limp, 1);            p = p_str(p, ",");
    p = p_fix(p, r->tc_safe, 1);            p = p_str(p, ",");
    p = p_str(p, r->why);
    *p = 0;
    return (uint32_t)(p - buf);
}
