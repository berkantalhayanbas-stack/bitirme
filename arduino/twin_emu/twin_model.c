/* twin_model.c -- paket ikizi (açıklama twin_model.h'de). AVR'de tablolar flash'ta (PROGMEM). */
#include "twin_model.h"
#include <math.h>

#ifdef __AVR__
#include <avr/pgmspace.h>
#define RD(x) pgm_read_float(&(x))
#else
#define PROGMEM
#define RD(x) (x)
#define pgm_read_byte(p) (*(const uint8_t *)(p))
#endif

/* ---------------- hücre parametreleri (bms_config.h ile aynı) */
#define CELL_Q_AH   4.85f
#define CELL_R0     0.0264f
#define CELL_R1     0.0093f
#define CELL_C1     1075.0f
#define CELL_R2     0.012f
#define CELL_C2     15000.0f
#define CELL_EA_R0  20000.0f
#define RGAS        8.314f
#define CELL_CC     64.0f
#define CELL_CS     5.0f
#define CELL_RC     1.8f
#define CELL_RU     9.0f

/* ---------------- OCV(SOC): Chen vd. (2020), LG M50 */
#define OCV_N 23
static const float SOC_TAB[OCV_N] PROGMEM = {
    0.0f, 0.02f, 0.05f, 0.1f, 0.15f, 0.2f, 0.25f, 0.3f, 0.35f, 0.4f, 0.45f, 0.5f,
    0.55f, 0.6f, 0.65f, 0.7f, 0.75f, 0.8f, 0.85f, 0.9f, 0.95f, 0.98f, 1.0f };
static const float OCV_TAB[OCV_N] PROGMEM = {
    2.4977f, 2.8373f, 3.0726f, 3.2574f, 3.3956f, 3.4487f, 3.4934f, 3.5473f, 3.5959f, 3.6329f,
    3.6664f, 3.7051f, 3.7538f, 3.8055f, 3.859f, 3.9203f, 3.9671f, 4.0165f, 4.0658f, 4.0927f,
    4.111f, 4.1458f, 4.1809f };

/* ---------------- grup sapmaları: kapasite [0,1 %], direnç [%], başlangıç SOC [0,1 % SOC]
 * Grup 7 (indeks 6) zayıf: -%5 kapasite, +%15 direnç. Diğerleri üretim saçılması (+-%0,5 / +-%3);
 * başlangıç SOC farkı +-%1,2 (paket ~22 mV gerilim farkıyla başlar, dengeleme gösterilebilsin). */
static const int8_t GRP[TW_NG][3] PROGMEM = {
    {   3,  1,   2 }, {  -4, -2,  -3 }, {   2,  0,   5 }, {   0,  3,   0 }, {  -2, -1,  -6 },
    {   5,  2,   8 }, { -50, 15, -10 }, {   1,  0,   3 }, {  -3, -3,  -2 }, {   4,  1,  12 },
    {   0,  2,  -4 }, {  -1, -1,   1 }, {   2,  0,   6 }, {  -5,  1,  -8 }, {   3, -2,   0 },
    {   1,  3,   4 }, {  -2,  0,  -5 }, {   0, -1,   9 }, {   4,  2,  -1 } };
#define GRP_KQ(j)  (1.0f + 0.001f * (int8_t)pgm_read_byte(&GRP[j][0]))
#define GRP_KR(j)  (1.0f + 0.01f  * (int8_t)pgm_read_byte(&GRP[j][1]))
#define GRP_DS(j)  (0.001f * (int8_t)pgm_read_byte(&GRP[j][2]))

float twin_ocv(float soc)
{
    if (soc <= 0.0f) return RD(OCV_TAB[0]);
    if (soc >= 1.0f) return RD(OCV_TAB[OCV_N - 1]);
    uint8_t k = 1;
    while (k < OCV_N - 1 && soc > RD(SOC_TAB[k])) k++;
    const float s0 = RD(SOC_TAB[k - 1]), s1 = RD(SOC_TAB[k]);
    const float o0 = RD(OCV_TAB[k - 1]), o1 = RD(OCV_TAB[k]);
    return o0 + (o1 - o0) * (soc - s0) / (s1 - s0);
}

/* ---------------- başlatma */
void twin_init(twin_t *tw, float soc0, float t_amb, float dt)
{
    for (uint8_t j = 0; j < TW_NG; j++) tw->g[j].v1 = tw->g[j].v2 = 0.0f;
    twin_set_soc(tw, soc0);
    tw->tc = tw->ts = tw->t_amb = t_amb;
    tw->ru_k = 1.0f;
    tw->dt = dt;
    tw->a1 = expf(-dt / (CELL_R1 * CELL_C1));
    tw->a2 = expf(-dt / (CELL_R2 * CELL_C2));
    twin_step(tw, 0.0f, 0, 0.0f, 1.0f);     /* çıktıları doldur (dt kadar dinlenme) */
}

void twin_set_soc(twin_t *tw, float soc)
{
    for (uint8_t j = 0; j < TW_NG; j++) tw->g[j].soc = soc + GRP_DS(j);
    tw->soc_avg = soc;
}

/* ---------------- bir adım (dt) */
void twin_step(twin_t *tw, float i_pack, uint32_t bal_mask, float i_bleed, float k_soc)
{
    const float i  = i_pack / TW_NP;        /* hücre akımı */
    const float ib = i_bleed / TW_NP;       /* dengeleme direncinin hücre başına payı */
    const float ibs = ib * (k_soc - 1.0f);  /* hızlandırmanın SOC'ye eklenen kısmı (gerilime değil) */
    const float dt = tw->dt;
    /* R0 sıcaklık çarpanı (Arrhenius) tüm gruplar için aynı: bir kez hesapla */
    const float r0t = CELL_R0 * expf(CELL_EA_R0 / RGAS * (1.0f / (tw->tc + 273.15f) - 1.0f / 298.15f));
    float q = 0.0f, vs = 0.0f, ss = 0.0f, vmin = 10.0f, vmax = 0.0f, smin = 2.0f, smax = -1.0f;

    for (uint8_t j = 0; j < TW_NG; j++) {
        tw_group_t *g = &tw->g[j];
        const float kr  = GRP_KR(j);
        const uint8_t on = bal_mask >> j & 1u;
        const float ig  = on ? i + ib : i;
        const float ocv = twin_ocv(g->soc);
        const float r0  = r0t * (1.0f + 0.6f * expf(-g->soc / 0.08f)) * kr;   /* + düşük SOC artışı */
        g->v1 = tw->a1 * g->v1 + CELL_R1 * kr * (1.0f - tw->a1) * ig;
        g->v2 = tw->a2 * g->v2 + CELL_R2 * kr * (1.0f - tw->a2) * ig;
        g->v  = ocv - r0 * ig - g->v1 - g->v2;
        q += i * (ocv - g->v);              /* Bernardi tersinmez ısı (entropik terim ihmal) */
        g->soc -= (on ? ig + ibs : ig) * dt / (3600.0f * CELL_Q_AH * GRP_KQ(j));
        vs += g->v;  ss += g->soc;
        if (g->v < vmin) vmin = g->v;
        if (g->v > vmax) vmax = g->v;
        if (g->soc < smin) smin = g->soc;
        if (g->soc > smax) smax = g->soc;
    }
    q *= 1.0f / TW_NG;
    tw->soc_min = smin;  tw->soc_max = smax;

    /* 2 düğümlü termal model (ortalama hücre) */
    const float dtc = (q + (tw->ts - tw->tc) / CELL_RC) / CELL_CC;
    const float dts = ((tw->t_amb - tw->ts) / (CELL_RU * tw->ru_k) -(tw->ts - tw->tc) / CELL_RC) / CELL_CS;
    tw->tc += dtc * dt;
    tw->ts += dts * dt;

    tw->v_pack  = vs * ((float)TW_NS / TW_NG);
    tw->v_min   = vmin;
    tw->v_max   = vmax;
    tw->soc_avg = ss * (1.0f / TW_NG);
}
