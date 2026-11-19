/* test_common.c -- bms/common için PC birim testleri (donanım yok, yalnız gcc).
 * Çalıştırma: python tools/host_tests/run.py   (ASan + UBSan ile derler)
 * Kapsam: CAN kodlama/çözme + CRC, alım izleyicisi, dengeleme kararı, karar mekanizması sert
 * sınırları, bms_step uçtan uca sağlamlık. Algoritma çıktısı değiştirilmez; yalnız denetler.
 */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "bms_config.h"
#include "bms_algo.h"
#include "bms_bal.h"
#include "bms_can.h"

static int n_test, n_fail;
#define CHECK(c) do { n_test++; if (!(c)) { n_fail++; printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); } } while (0)
#define NEAR(a, b, tol) CHECK(fabsf((float)(a) - (float)(b)) <= (tol))

/* ------------------------------------------------------------------ CAN */
static void t_crc(void)
{
    puts("crc / kodlama");
    CHECK(crc8_j1850((const uint8_t *)"123456789", 9) == 0x4B);

    uint8_t d[8], c;
    const can_meas1_t a = { 61.7f, -23.4f, 31.2f };
    can_meas1_t ra;
    can_encode_meas1(&a, 7, d);
    CHECK(can_decode_meas1(d, &ra, &c) && c == 7);
    NEAR(ra.V, a.V, 0.051f); NEAR(ra.I, a.I, 0.051f); NEAR(ra.T, a.T, 0.051f);
    /* bozuk CRC, her bayt için tek bit: reddedilmeli */
    int rej = 0;
    for (int i = 0; i < 8; i++) { uint8_t e[8]; memcpy(e, d, 8); e[i] ^= 0x10; rej += !can_decode_meas1(e, &ra, &c); }
    CHECK(rej == 8);
    /* veri kimliği: 0x101 verisi 0x102 olarak çözülemez */
    can_meas2_t b2;
    CHECK(!can_decode_meas2(d, &b2, &c));

    const can_meas2_t b = { 3.123f, 4.087f, -12.3f };
    can_encode_meas2(&b, 15, d);
    CHECK(can_decode_meas2(d, &b2, &c) && c == 15);
    NEAR(b2.V_min, b.V_min, 0.0006f); NEAR(b2.V_max, b.V_max, 0.0006f); NEAR(b2.T_amb, b.T_amb, 0.051f);

    /* doyma: aralık dışı değerler sarmaz */
    const can_meas1_t big = { 1e9f, 1e9f, -1e9f };
    can_encode_meas1(&big, 0, d);
    CHECK(can_decode_meas1(d, &ra, &c));
    NEAR(ra.V, 6553.5f, 0.1f); NEAR(ra.I, 3276.7f, 0.1f); NEAR(ra.T, -3276.8f, 0.1f);

    const can_ctrl_t k = { 87.3f, 40.0f, ST_DERATE, 17 };
    can_ctrl_t rk;
    can_encode_ctrl(&k, 3, d);
    CHECK(can_decode_ctrl(d, &rk, &c) && rk.state == ST_DERATE && rk.cause == 17);
    NEAR(rk.I_allow_dis, 87.3f, 0.06f); NEAR(rk.I_allow_chg, 40.0f, 0.06f);

    const can_status_t s = { 55.55f, 31.4f, 92.0f, 0.5f };
    can_status_t rs;
    can_encode_status(&s, 1, d);
    CHECK(can_decode_status(d, &rs, &c));
    NEAR(rs.soc_pct, 55.55f, 0.006f); NEAR(rs.tc_hat, 31.4f, 0.06f); NEAR(rs.soh_pct, 92.0f, 0.3f); NEAR(rs.psi, 0.5f, 0.003f);

    const can_cmd_t q = { CAN_CMD_ECU_RESTART, CAN_CMD_UNLOCK };
    can_cmd_t rq;
    can_encode_cmd(&q, 9, d);
    CHECK(can_decode_cmd(d, &rq, &c) && rq.cmd == q.cmd && rq.value == q.value && c == 9);
}

static void t_cells(void)
{
    puts("hücre çerçeveleri 0x110-0x114");
    float v[CAN_N_CELLS];
    for (uint32_t i = 0; i < CAN_N_CELLS; i++) v[i] = 3.300f + 0.0137f * (float)i;
    uint32_t seen = 0;
    for (uint8_t k = 0; k < CAN_N_CELL_FRAMES; k++) {
        can_cell_t m = { { 0 }, 0, (uint8_t)(k + 1) & 0x0F }, r;
        for (uint32_t j = 0; j < CAN_CELLS_PER_FRAME; j++)
            if (4u * k + j < CAN_N_CELLS) { m.v[j] = v[4u * k + j]; m.valid |= (uint8_t)(1u << j); }
        uint8_t d[8], c;
        can_encode_cell(k, &m, 5, d);
        CHECK(can_decode_cell(k, d, &r, &c) && c == 5 && r.valid == m.valid && r.bal == m.bal);
        for (uint32_t j = 0; j < CAN_CELLS_PER_FRAME; j++)
            if (m.valid >> j & 1u) { NEAR(r.v[j], m.v[j], 0.0006f); seen++; }
        /* başka çerçevenin kimliğiyle çözülemez */
        CHECK(!can_decode_cell((uint8_t)((k + 1) % CAN_N_CELL_FRAMES), d, &r, &c));
    }
    CHECK(seen == CAN_N_CELLS);
    /* geçersiz yuva (0xFFF) */
    can_cell_t m = { { 3.5f, 3.6f, 3.7f, 3.8f }, 0x5, 0 }, r;
    uint8_t d[8], c;
    can_encode_cell(0, &m, 0, d);
    CHECK(can_decode_cell(0, d, &r, &c) && r.valid == 0x5);
    /* alt/üst doyma: 1,000 V altı ve 5,094 V üstü */
    can_cell_t lo = { { 0.2f, 9.0f, 1.0f, 5.094f }, 0xF, 0 };
    can_encode_cell(2, &lo, 0, d);
    CHECK(can_decode_cell(2, d, &r, &c) && r.valid == 0xF);
    NEAR(r.v[0], 1.0f, 1e-4f); NEAR(r.v[1], 5.094f, 1e-3f); NEAR(r.v[3], 5.094f, 1e-3f);
}

static void t_bal_msg(void)
{
    puts("dengeleme mesajı 0x210");
    can_bal_t b = { 0x5A5A5u, CAN_BAL_F_ACTIVE | CAN_BAL_F_DRIVE, 0.0123f }, r;
    uint8_t d[8], c;
    can_encode_bal(&b, 2, d);
    CHECK(can_decode_bal(d, &r, &c) && r.mask == b.mask && r.flags == b.flags);
    NEAR(r.dv, b.dv, 0.0001f);
    b.mask = 0xFFFFFFFFu;                       /* 19 bit dışı bitler taşınmaz */
    can_encode_bal(&b, 2, d);
    CHECK(can_decode_bal(d, &r, &c) && r.mask == 0x7FFFFu);
}

static void t_monitor(void)
{
    puts("alım izleyicisi");
    can_monitor_t m;
    can_monitor_init(&m, 0);
    CHECK(can_monitor_rx(&m, 10, 1, 4));                 /* ilk mesaj */
    CHECK(!can_monitor_rx(&m, 20, 1, 4));                /* tekrar/donma */
    CHECK(can_monitor_rx(&m, 30, 1, 5));
    CHECK(can_monitor_rx(&m, 40, 1, 7) && m.n_lost == 1);  /* 1 kayıp kabul */
    CHECK(!can_monitor_rx(&m, 50, 1, 12));               /* fark 5: dizi hatası */
    CHECK(can_monitor_rx(&m, 60, 1, 13));                /* yeniden eşitlendi */
    CHECK(!can_monitor_rx(&m, 70, 0, 14) && m.n_crc_err == 1);
    CHECK(can_monitor_rx(&m, 80, 1, 14));                /* CRC hatası sayacı bozmaz */
    CHECK(can_monitor_rx(&m, 90, 1, 1));                 /* 15 -> 0 -> 1 taşma, fark 3 */
    CHECK(m.n_cnt_err == 2);

    /* zaman aşımı eşikleri */
    can_monitor_init(&m, 0);
    can_monitor_tick(&m, CAN_TIMEOUT_FIRST_MS - 1);
    CHECK(!m.fault && !m.safe);
    can_monitor_tick(&m, CAN_TIMEOUT_FIRST_MS);
    CHECK(m.fault && m.safe);                            /* açılışta hiç mesaj yok */
    can_monitor_init(&m, 0);
    can_monitor_rx(&m, 1000, 1, 0);
    can_monitor_tick(&m, 1000 + CAN_TIMEOUT_FAULT_MS - 1);
    CHECK(!m.fault);
    can_monitor_tick(&m, 1000 + CAN_TIMEOUT_FAULT_MS);
    CHECK(m.fault && !m.safe);
    can_monitor_rx(&m, 1300, 1, 1);                      /* toparlanır: fault kalkar */
    can_monitor_tick(&m, 1310);
    CHECK(!m.fault && !m.safe);
    can_monitor_tick(&m, 1300 + CAN_TIMEOUT_SAFE_MS);
    CHECK(m.safe);
    can_monitor_rx(&m, 1900, 1, 2);                      /* SAFE kalıcı */
    can_monitor_tick(&m, 1910);
    CHECK(m.safe);
    /* ms sayacı taşması (uint32) */
    can_monitor_init(&m, 0xFFFFFF00u);
    can_monitor_rx(&m, 0xFFFFFF10u, 1, 0);
    can_monitor_tick(&m, 0x00000008u);                   /* 248 ms sonra (taşma) */
    CHECK(!m.fault);
}

/* ------------------------------------------------------------------ dengeleme */
static uint32_t bal_run(bal_t *b, const float *v, float I, uint8_t st, float tc, int steps)
{
    uint32_t m = 0;
    for (int i = 0; i < steps; i++) m = bal_step(b, v, 1, I, st, tc);
    return m;
}
static void t_bal(void)
{
    puts("dengeleme kararı");
    bal_t b;
    float v[BMS_NS];
    for (int i = 0; i < BMS_NS; i++) v[i] = 3.80f;
    v[3] = 3.83f; v[10] = 3.812f; v[7] = 3.804f;
    bal_init(&b);
    /* bekleme süresi dolmadan hiçbir direnç açılmaz */
    CHECK(bal_run(&b, v, 0.0f, ST_NORMAL, 25.0f, (int)(BAL_T_ON / BMS_TS) - 5) == 0);
    CHECK(!(b.flags & CAN_BAL_F_ACTIVE));
    uint32_t m = bal_run(&b, v, 0.0f, ST_NORMAL, 25.0f, 10);
    CHECK(b.flags & CAN_BAL_F_ACTIVE);
    CHECK(m == ((1u << 3) | (1u << 10)));                /* 30 mV ve 12 mV; 4 mV açılmaz */
    /* sürüşte hemen kapanır, bekleme sıfırlanır */
    CHECK(bal_step(&b, v, 1, 5.0f, ST_NORMAL, 25.0f) == 0 && (b.flags & CAN_BAL_F_DRIVE));
    CHECK(bal_run(&b, v, 0.0f, ST_NORMAL, 25.0f, 10) == 0);
    /* şarjda (negatif akım) serbest */
    bal_init(&b);
    CHECK(bal_run(&b, v, -10.0f, ST_NORMAL, 25.0f, (int)(BAL_T_ON / BMS_TS) + 2) != 0);
    /* LIMP / sıcak / düşük gerilim engeller */
    bal_init(&b);
    CHECK(bal_run(&b, v, 0.0f, ST_LIMP, 25.0f, 700) == 0 && (b.flags & CAN_BAL_F_BLOCKED));
    bal_init(&b);
    CHECK(bal_run(&b, v, 0.0f, ST_NORMAL, BAL_T_MAX + 1.0f, 700) == 0);
    float lowv[BMS_NS];
    for (int i = 0; i < BMS_NS; i++) lowv[i] = BAL_V_MIN - 0.1f + (i == 5 ? 0.05f : 0.0f);
    bal_init(&b);
    CHECK(bal_run(&b, lowv, 0.0f, ST_NORMAL, 25.0f, 700) == 0);
    /* histerezis: açıkken 5-10 mV arasında açık kalır, < 5 mV'de kapanır */
    bal_init(&b);
    bal_run(&b, v, 0.0f, ST_NORMAL, 25.0f, 700);
    float w[BMS_NS];
    for (int i = 0; i < BMS_NS; i++) w[i] = 3.80f;
    w[3] = 3.80f + 0.0075f;
    m = bal_run(&b, w, 0.0f, ST_NORMAL, 25.0f, 200);     /* süzgeç oturur */
    CHECK(m & (1u << 3));
    w[3] = 3.80f + 0.003f;
    m = bal_run(&b, w, 0.0f, ST_NORMAL, 25.0f, 200);
    CHECK(!(m & (1u << 3)));
    /* eski veri: maske 0, süzgeç sıfırlanır */
    CHECK(bal_step(&b, v, 0, 0.0f, ST_NORMAL, 25.0f) == 0 && (b.flags & CAN_BAL_F_STALE));
}

/* ------------------------------------------------------------------ karar */
static void dec_run(decision_t *d, decision_out_t *o, float I, float vmin, float vmax, float tc, float ts,
                    float ru, float soc, float bias, uint8_t tv, uint8_t iv, uint8_t can_ok, int n)
{
    for (int i = 0; i < n; i++) decision_step(d, I, vmin, vmax, tc, ts, ru, soc, bias, tv, iv, can_ok, o);
}
static void t_decision(void)
{
    puts("karar mekanizması");
    decision_t d;
    decision_out_t o;
    /* nominal: rampa ile tam akıma */
    decision_init(&d);
    dec_run(&d, &o, 0.0f, 3.9f, 3.95f, 25.0f, 25.0f, 9.0f, 0.6f, 0.0f, 1, 1, 1, 1);
    CHECK(o.state == ST_NORMAL && o.i_allow_dis <= 1.0f + 1e-3f);   /* rampa: 10 A/s x 0,1 s */
    dec_run(&d, &o, 0.0f, 3.9f, 3.95f, 25.0f, 25.0f, 9.0f, 0.6f, 0.0f, 1, 1, 1, 200);
    CHECK(o.state == ST_NORMAL && o.i_allow_dis > 80.0f && o.i_allow_chg > 30.0f);
    /* sert sınırlar kilitlenir */
    decision_init(&d);
    dec_run(&d, &o, 0.0f, 3.9f, 3.95f, DEC_T_SAFE + 1.0f, 25.0f, 9.0f, 0.6f, 0.0f, 1, 1, 1, 1);
    CHECK(o.state == ST_SAFE && o.i_allow_dis == 0.0f && o.cause == 128);
    dec_run(&d, &o, 0.0f, 3.9f, 3.95f, 25.0f, 25.0f, 9.0f, 0.6f, 0.0f, 1, 1, 1, 500);
    CHECK(o.state == ST_SAFE);                           /* durum düzelse de SAFE kalıcı */
    decision_init(&d);
    dec_run(&d, &o, 0.0f, 3.9f, 3.95f, 25.0f, 25.0f, 9.0f, 0.6f, 0.0f, 1, 1, 0, 1);   /* CAN yok */
    CHECK(o.state == ST_SAFE);
    decision_init(&d);                                   /* düşük gerilim: 1 s kesintisiz */
    dec_run(&d, &o, 0.0f, DEC_V_SAFE_LO - 0.1f, 3.9f, 25.0f, 25.0f, 9.0f, 0.6f, 0.0f, 1, 1, 1, DEC_N_HARD - 1);
    CHECK(o.state != ST_SAFE);
    dec_run(&d, &o, 0.0f, DEC_V_SAFE_LO - 0.1f, 3.9f, 25.0f, 25.0f, 9.0f, 0.6f, 0.0f, 1, 1, 1, 1);
    CHECK(o.state == ST_SAFE);
    decision_init(&d);                                   /* tek örneklik düşüş sayacı sıfırlar */
    dec_run(&d, &o, 0.0f, DEC_V_SAFE_LO - 0.1f, 3.9f, 25.0f, 25.0f, 9.0f, 0.6f, 0.0f, 1, 1, 1, DEC_N_HARD - 1);
    dec_run(&d, &o, 0.0f, 3.9f, 3.95f, 25.0f, 25.0f, 9.0f, 0.6f, 0.0f, 1, 1, 1, 1);
    dec_run(&d, &o, 0.0f, DEC_V_SAFE_LO - 0.1f, 3.9f, 25.0f, 25.0f, 9.0f, 0.6f, 0.0f, 1, 1, 1, DEC_N_HARD - 1);
    CHECK(o.state != ST_SAFE);
    /* LIMP: geçersiz sensör, yüksek sıcaklık, boş paket; çıkış 10 s histerezisle */
    decision_init(&d);
    dec_run(&d, &o, 0.0f, 3.9f, 3.95f, 25.0f, 25.0f, 9.0f, 0.6f, 0.0f, 0, 1, 1, 1);
    CHECK(o.state == ST_LIMP);
    dec_run(&d, &o, 0.0f, 3.9f, 3.95f, 25.0f, 25.0f, 9.0f, 0.6f, 0.0f, 1, 1, 1, DEC_N_EXIT - 1);
    CHECK(o.state == ST_LIMP);
    dec_run(&d, &o, 0.0f, 3.9f, 3.95f, 25.0f, 25.0f, 9.0f, 0.6f, 0.0f, 1, 1, 1, 1);
    CHECK(o.state != ST_LIMP);
    decision_init(&d);
    dec_run(&d, &o, 0.0f, 3.9f, 3.95f, DEC_T_LIMP + 1.0f, 25.0f, 9.0f, 0.6f, 0.0f, 1, 1, 1, 300);
    CHECK(o.state == ST_LIMP && o.i_allow_dis <= DEC_FLOOR * 7.275f * BMS_NP + 1e-3f);
    decision_init(&d);
    dec_run(&d, &o, 0.0f, 3.9f, 3.95f, 25.0f, 25.0f, 9.0f, DEC_SOC_EMPTY * 0.5f, 0.0f, 1, 1, 1, 1);
    CHECK(o.state == ST_LIMP);
    /* DERATE: sıcaklık stres indeksi, akım tek yönlü azalır */
    decision_init(&d);
    dec_run(&d, &o, 0.0f, 3.9f, 3.95f, 25.0f, 25.0f, 9.0f, 0.6f, 0.0f, 1, 1, 1, 300);
    const float full = o.i_allow_dis;
    dec_run(&d, &o, 0.0f, 3.9f, 3.95f, 50.0f, 40.0f, 9.0f, 0.6f, 0.0f, 1, 1, 1, 2);
    CHECK(o.state == ST_DERATE && o.i_allow_dis < full && (o.cause & 1));
    /* soğutma alarmı: açılış beklemesi + histerezis */
    decision_init(&d);
    dec_run(&d, &o, 0.0f, 3.9f, 3.95f, 25.0f, 25.0f, 30.0f, 0.6f, 0.0f, 1, 1, 1, DEC_N_COOL_BLANK - 2);
    CHECK(!(o.cause & 2));                               /* beklemede alarm yok */
    dec_run(&d, &o, 0.0f, 3.9f, 3.95f, 25.0f, 25.0f, 30.0f, 0.6f, 0.0f, 1, 1, 1, 3);
    CHECK(o.cause & 2);
    dec_run(&d, &o, 0.0f, 3.9f, 3.95f, 25.0f, 25.0f, 16.5f, 0.6f, 0.0f, 1, 1, 1, 1);
    CHECK(o.cause & 2);                                  /* 15-18 arası: açık kalır */
    dec_run(&d, &o, 0.0f, 3.9f, 3.95f, 25.0f, 25.0f, 14.0f, 0.6f, 0.0f, 1, 1, 1, 1);
    CHECK(!(o.cause & 2));
}

static void t_nonfinite(void)
{
    puts("sonlu olmayan girdi -> SAFE (savunmacı)");
    const float nanv = nanf(""), inf = (float)INFINITY;
    for (int c = 0; c < 6; c++) {
        decision_t d;
        decision_out_t o;
        decision_init(&d);
        float tc = 25.0f, vmin = 3.9f, vmax = 3.95f, soc = 0.6f, ts = 25.0f;
        if (c == 0) tc = nanv;
        if (c == 1) vmin = nanv;
        if (c == 2) vmax = inf;
        if (c == 3) soc = nanv;
        if (c == 4) ts = nanv;
        if (c == 5) tc = -inf;
        dec_run(&d, &o, 0.0f, vmin, vmax, tc, ts, 9.0f, soc, 0.0f, 1, 1, 1, 1);
        CHECK(o.state == ST_SAFE && o.i_allow_dis == 0.0f && o.i_allow_chg == 0.0f);
    }
}

/* ------------------------------------------------------------------ uçtan uca */
static float ocv_cell(float s) { float y = OCV_POLY[0]; for (int k = 1; k < OCV_POLY_N; k++) y = y * s + OCV_POLY[k]; return y; }
static void t_bms(void)
{
    puts("bms_step sağlamlığı");
    bms_t b;
    bms_out_t o;
    bms_init(&b, 0.5f, 1.0f, 1.0f, 0.0f, 0.0f);
    int bad = 0;
    float soc_true = 0.8f, v1 = 0.0f, v2 = 0.0f;
    const float k = (float)BMS_NS / (float)BMS_NP;       /* hücre -> paket direnç ölçeği */
    const float a1 = expf(-BMS_TS / (CELL_R1 * CELL_C1)), a2 = expf(-BMS_TS / (CELL_R2 * CELL_C2));
    for (int i = 0; i < 6000; i++) {             /* 10 dk: 1 dk dinlenme + 20 A deşarj, 2-RC "gerçek" hücre */
        const float I = (i > 600) ? 20.0f : 0.0f;
        soc_true -= I * BMS_TS / (3600.0f * CELL_Q_AH * BMS_NP);
        v1 = a1 * v1 + CELL_R1 * k * (1.0f - a1) * I;
        v2 = a2 * v2 + CELL_R2 * k * (1.0f - a2) * I;
        const float V = BMS_NS * ocv_cell(soc_true) - I * CELL_R0 * k - v1 - v2;
        /* tam sabit I/T "donmuş sensör" sayılır (LIMP); gerçek sensör gibi küçük titreşim ekle */
        const float dith = (i % 2) ? 0.1f : -0.1f;
        const bms_in_t in = { I + dith, V, 25.0f + 0.1f * (float)(i % 3), V / BMS_NS, V / BMS_NS, 25.0f, 1 };
        bms_step(&b, &in, &o);
        if (!isfinite(o.soc) || !isfinite(o.tc) || !isfinite(o.ru) || !isfinite(o.dec.i_allow_dis)) bad++;
    }
    CHECK(bad == 0);
    CHECK(o.dec.state == ST_NORMAL || o.dec.state == ST_DERATE);
    NEAR(o.soc, soc_true, 0.05f);                /* EKF yakınsar (tolerans geniş: model sade) */
    /* ölçüm sıfır/aşırı (CAN kodlama uçları) gelse de sayısal patlama olmaz */
    bms_init(&b, 0.5f, 1.0f, 1.0f, 0.0f, 0.0f);
    bad = 0;
    for (int i = 0; i < 2000; i++) {
        const float V = (i % 2) ? 0.0f : 6553.5f, I = (i % 3) ? 3276.7f : -3276.8f;
        const bms_in_t in = { I, V, (i % 5) ? -40.0f : 150.0f, 0.0f, 6.0f, 25.0f, 1 };
        bms_step(&b, &in, &o);
        if (!isfinite(o.soc) || !isfinite(o.dec.i_allow_dis) || !isfinite(o.dec.i_allow_chg)) bad++;
    }
    CHECK(bad == 0);
}

int main(void)
{
    t_crc();  t_cells();  t_bal_msg();  t_monitor();  t_bal();  t_decision();  t_nonfinite();  t_bms();
    printf("\n%d kontrol, %d hata\n", n_test, n_fail);
    return n_fail ? 1 : 0;
}
