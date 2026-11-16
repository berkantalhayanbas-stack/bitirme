/* app_can_lb.c -- FDCAN1 iç geri döngü (internal loopback) testi, NUCLEO-G431RB, transceiver yok.
 *
 * Zincir:  bms_can kodlayıcı -> FDCAN1 Tx FIFO -> (kart içinde) -> Rx FIFO 0 -> bms_can çözücü
 *          -> alım izleyicisi.  Sonuç LPUART1'e (ST-LINK sanal COM, 115200 8N1) metin olarak yazılır.
 *
 * Testler
 *   T1  7 mesaj tipi x N_ROUNDS tur: kimlik, uzunluk, ham baytlar birebir; çözülen değerler
 *       ölçek çözünürlüğünün yarısı içinde; izleyici ilk 5 tipin her çerçevesini kabul eder.
 *       Hücre gerilimi (0x110-0x114, 12 bit paketleme, boş yuva) ve dengeleme (0x210) de çözülür.
 *   T2  E2E hata enjeksiyonu (CAN üzerinden): bozuk CRC, yanlış kimlik (data ID), donmuş sayaç,
 *       atlanmış sayaç -> izleyici sayaçları beklenen değerde.
 *   T3  Zaman aşımı (sanal zaman): 250 ms -> fault, 500 ms -> safe (kalıcı). Açılış: ilk
 *       mesajdan önce CAN_TIMEOUT_FIRST_MS (2 s) boyunca SAFE yok; hiç mesaj gelmezse 2 s'de SAFE.
 *   T4  FDCAN hata sayaçları sıfır, error passive / bus-off yok.
 *   T5  Normal moddaki kabul filtresi: yalnızca 0x101, 0x102, 0x301 ve 0x110-0x114 alınır.
 * LED (PA5): PASS -> yavaş (500 ms), FAIL -> hızlı (100 ms) yanıp söner.
 */
#include <math.h>
#include <string.h>
#include "app_can_lb.h"
#include "bsp.h"
#include "bsp_can.h"
#include "bms_can.h"

#define N_ROUNDS     1000u
#define RX_TIMEOUT   (BSP_CPU_HZ / 1000u)        /* 1 ms (bir çerçeve 500 kbit/s'de ~0,25 ms) */

/* ---------------------------------------------------------------- metin çıktısı (printf yok) */
static void put_s(const char *s) { bsp_uart_write((const uint8_t *)s, (uint32_t)strlen(s)); }
static void put_u(uint32_t v)
{
    char b[11];
    int i = 10;
    b[i] = 0;
    do { b[--i] = (char)('0' + v % 10u); v /= 10u; } while (v);
    put_s(&b[i]);
}
static void put_hex(uint32_t v, int digits)
{
    char b[9];
    for (int i = 0; i < digits; i++) b[i] = "0123456789ABCDEF"[(v >> (4 * (digits - 1 - i))) & 0xF];
    b[digits] = 0;
    put_s(b);
}

/* ---------------------------------------------------------------- yardımcılar */
static uint32_t n_fail;
static uint32_t lat_max, lat_sum, lat_n;          /* gönderim isteği -> Rx FIFO, DWT döngüsü */

static void fail(const char *what, uint16_t id, uint32_t round)
{
    if (n_fail++ < 10) {                          /* ilk 10 hatayı ayrıntılı yaz */
        put_s("  FAIL "); put_s(what); put_s(" id=0x"); put_hex(id, 3);
        put_s(" round="); put_u(round); put_s("\r\n");
    }
}

/* Gönder, aynı çerçeveyi geri al. Kimlik/uzunluk/veri birebir değilse 0. */
static int xfer(uint16_t id, const uint8_t tx[8], uint8_t rx[8], uint32_t round)
{
    uint16_t rid;
    uint8_t len;
    while (!bsp_can_send(id, tx, 8)) { }
    const uint32_t t0 = bsp_cycles();
    while (!bsp_can_recv(&rid, rx, &len)) {
        if (bsp_cycles() - t0 > RX_TIMEOUT) { fail("rx timeout", id, round); return 0; }
    }
    const uint32_t dt = bsp_cycles() - t0;
    if (dt > lat_max) lat_max = dt;
    lat_sum += dt;  lat_n++;
    if (rid != id || len != 8 || memcmp(tx, rx, 8) != 0) { fail("raw mismatch", id, round); return 0; }
    return 1;
}

static int near(float a, float b, float scale) { return fabsf(a - b) <= 0.5f * scale + 1e-4f * fabsf(b); }

/* ---------------------------------------------------------------- T1 */
enum { M1, M2, CTRL, STAT, CMD, N_MSG };
static can_monitor_t mon[N_MSG];

static void check_rx(int ok_decode, int ok_values, int m, uint16_t id, uint32_t k, uint8_t cnt)
{
    if (!ok_decode) { fail("decode/crc", id, k); return; }
    if (!ok_values) fail("value", id, k);
    if (!can_monitor_rx(&mon[m], k * 100u, 1, cnt)) fail("monitor reject", id, k);
}

static void t1_roundtrip(void)
{
    uint8_t tx[8], rx[8], cnt;
    for (int m = 0; m < N_MSG; m++) can_monitor_init(&mon[m], 0);

    for (uint32_t k = 0; k < N_ROUNDS; k++) {
        const uint8_t c = (uint8_t)(k & 0x0F);
        const float s = (float)k / (float)N_ROUNDS;           /* 0..1, değerleri tarar */

        can_meas1_t a = { 55.0f + 25.0f * s, -300.0f + 600.0f * s, -25.0f + 90.0f * s }, a2;
        can_encode_meas1(&a, c, tx);
        if (xfer(CAN_ID_EMU_MEAS1, tx, rx, k))
            check_rx(can_decode_meas1(rx, &a2, &cnt) && cnt == c,
                     near(a2.V, a.V, 0.1f) && near(a2.I, a.I, 0.1f) && near(a2.T, a.T, 0.1f),
                     M1, CAN_ID_EMU_MEAS1, k, cnt);

        can_meas2_t b = { 2.5f + 1.0f * s, 3.0f + 1.25f * s, -20.0f + 60.0f * s }, b2;
        can_encode_meas2(&b, c, tx);
        if (xfer(CAN_ID_EMU_MEAS2, tx, rx, k))
            check_rx(can_decode_meas2(rx, &b2, &cnt) && cnt == c,
                     near(b2.V_min, b.V_min, 0.001f) && near(b2.V_max, b.V_max, 0.001f)
                         && near(b2.T_amb, b.T_amb, 0.1f),
                     M2, CAN_ID_EMU_MEAS2, k, cnt);

        can_ctrl_t d = { 87.3f * s, 40.7f * s, (uint8_t)(k % 4u), (uint8_t)(k * 7u) }, d2;
        can_encode_ctrl(&d, c, tx);
        if (xfer(CAN_ID_ECU_CTRL, tx, rx, k))
            check_rx(can_decode_ctrl(rx, &d2, &cnt) && cnt == c,
                     near(d2.I_allow_dis, d.I_allow_dis, 0.1f) && near(d2.I_allow_chg, d.I_allow_chg, 0.1f)
                         && d2.state == d.state && d2.cause == d.cause,
                     CTRL, CAN_ID_ECU_CTRL, k, cnt);

        can_status_t e = { 100.0f * s, -10.0f + 70.0f * s, 60.0f + 40.0f * s, s }, e2;
        can_encode_status(&e, c, tx);
        if (xfer(CAN_ID_ECU_STATUS, tx, rx, k))
            check_rx(can_decode_status(rx, &e2, &cnt) && cnt == c,
                     near(e2.soc_pct, e.soc_pct, 0.01f) && near(e2.tc_hat, e.tc_hat, 0.1f)
                         && near(e2.soh_pct, e.soh_pct, 0.5f) && near(e2.psi, e.psi, 0.005f),
                     STAT, CAN_ID_ECU_STATUS, k, cnt);

        can_cmd_t f = { (uint8_t)(k % 5u), (int16_t)(k * 61u - 30000) }, f2;
        can_encode_cmd(&f, c, tx);
        if (xfer(CAN_ID_PC_CMD, tx, rx, k))
            check_rx(can_decode_cmd(rx, &f2, &cnt) && cnt == c,
                     f2.cmd == f.cmd && f2.value == f.value,
                     CMD, CAN_ID_PC_CMD, k, cnt);

        /* hücre çerçevesi (sırası k % 5) ve dengeleme komutu: 4 x 12 bit paketleme, boş yuva */
        const uint8_t fk = (uint8_t)(k % CAN_N_CELL_FRAMES);
        const uint16_t cid = (uint16_t)(CAN_ID_EMU_CELL0 + fk);
        can_cell_t g = { { 1.0f + 4.094f * s, 4.2f - 1.7f * s, 2.5f + 0.001f * (float)(k % 7u), 3.6f + 0.5f * s },
                         (uint8_t)(fk == 4u ? 0x7u : 0xFu), (uint8_t)(k & 0xFu) }, g2;
        can_encode_cell(fk, &g, c, tx);
        if (xfer(cid, tx, rx, k)) {
            int okv = can_decode_cell(fk, rx, &g2, &cnt) && cnt == c && g2.valid == g.valid && g2.bal == g.bal;
            for (int j = 0; j < (int)CAN_CELLS_PER_FRAME; j++)
                if (g.valid >> j & 1u) okv = okv && near(g2.v[j], g.v[j], 0.001f);
            if (!okv) fail("cell value", cid, k);
        }
        can_bal_t h = { (k * 2654435761u) & 0x7FFFFu, (uint8_t)(k & 0x0Fu), 0.05f * s }, h2;
        can_encode_bal(&h, c, tx);
        if (xfer(CAN_ID_ECU_BAL, tx, rx, k))
            if (!(can_decode_bal(rx, &h2, &cnt) && cnt == c && h2.mask == h.mask && h2.flags == h.flags
                  && near(h2.dv, h.dv, 0.0001f)))
                fail("bal value", CAN_ID_ECU_BAL, k);
    }
    for (int m = 0; m < N_MSG; m++)
        if (mon[m].n_ok != N_ROUNDS || mon[m].n_lost || mon[m].n_cnt_err || mon[m].n_crc_err)
            fail("monitor counts", (uint16_t)m, N_ROUNDS);
}

/* ---------------------------------------------------------------- T2 */
static void t2_fault_injection(void)
{
    can_monitor_t mn;
    can_meas1_t a = { 70.0f, 12.3f, 25.0f }, a2;
    can_meas2_t b2;
    uint8_t tx[8], rx[8], cnt = 0;
    can_monitor_init(&mn, 0);

    /* a) geçerli çerçeve, sayaç 3 */
    can_encode_meas1(&a, 3, tx);
    if (xfer(CAN_ID_EMU_MEAS1, tx, rx, 0)) {
        int ok = can_decode_meas1(rx, &a2, &cnt);
        can_monitor_rx(&mn, 0, ok, cnt);
    }
    /* b) bozuk CRC: tek bit çevrildi -> çözücü reddetmeli */
    can_encode_meas1(&a, 4, tx);
    tx[7] ^= 0x01;
    if (xfer(CAN_ID_EMU_MEAS1, tx, rx, 1)) {
        int ok = can_decode_meas1(rx, &a2, &cnt);
        if (ok) fail("bad crc accepted", CAN_ID_EMU_MEAS1, 1);
        can_monitor_rx(&mn, 100, ok, cnt);
    }
    /* c) yanlış kimlik: MEAS1 verisi 0x102 ile -> CRC'deki data ID tutmamalı */
    can_encode_meas1(&a, 4, tx);
    if (xfer(CAN_ID_EMU_MEAS2, tx, rx, 2))
        if (can_decode_meas2(rx, &b2, &cnt)) fail("wrong id accepted", CAN_ID_EMU_MEAS2, 2);
    /* d) donmuş sayaç: 3 tekrar -> izleyici reddetmeli */
    can_encode_meas1(&a, 3, tx);
    if (xfer(CAN_ID_EMU_MEAS1, tx, rx, 3)) {
        int ok = can_decode_meas1(rx, &a2, &cnt);
        if (can_monitor_rx(&mn, 200, ok, cnt)) fail("frozen counter accepted", CAN_ID_EMU_MEAS1, 3);
    }
    /* e) atlama: 3 -> 5 (bir çerçeve kayıp) -> kabul, n_lost = 1 */
    can_encode_meas1(&a, 5, tx);
    if (xfer(CAN_ID_EMU_MEAS1, tx, rx, 4)) {
        int ok = can_decode_meas1(rx, &a2, &cnt);
        if (!can_monitor_rx(&mn, 300, ok, cnt)) fail("skip rejected", CAN_ID_EMU_MEAS1, 4);
    }
    if (mn.n_ok != 2 || mn.n_crc_err != 1 || mn.n_cnt_err != 1 || mn.n_lost != 1)
        fail("monitor counts T2", CAN_ID_EMU_MEAS1, 5);
}

/* ---------------------------------------------------------------- T3 */
static void t3_timeout(void)
{
    can_monitor_t mn;
    can_monitor_init(&mn, 0);
    can_monitor_rx(&mn, 1000, 1, 0);                 /* son geçerli mesaj t = 1000 ms */
    can_monitor_tick(&mn, 1240);
    if (mn.fault || mn.safe) fail("fault too early", 0, 240);
    can_monitor_tick(&mn, 1250);
    if (!mn.fault || mn.safe) fail("fault at 250 ms", 0, 250);
    can_monitor_tick(&mn, 1500);
    if (!mn.safe) fail("safe at 500 ms", 0, 500);
    can_monitor_rx(&mn, 1510, 1, 1);
    can_monitor_tick(&mn, 1520);
    if (!mn.safe) fail("safe not latched", 0, 520);

    /* açılış: ilk mesaj CAN_TIMEOUT_FIRST_MS içinde gelirse SAFE yok */
    can_monitor_init(&mn, 0);
    can_monitor_tick(&mn, 600);
    if (mn.fault || mn.safe) fail("startup: safe before first timeout", 0, 600);
    can_monitor_tick(&mn, CAN_TIMEOUT_FIRST_MS - 10);
    if (mn.fault || mn.safe) fail("startup: safe before first timeout", 0, CAN_TIMEOUT_FIRST_MS - 10);
    can_monitor_rx(&mn, 1500, 1, 7);                 /* ilk mesaj geldi -> normal eşikler */
    can_monitor_tick(&mn, 1500 + CAN_TIMEOUT_SAFE_MS - 10);
    if (mn.safe) fail("startup: safe after first msg", 0, 1);
    can_monitor_tick(&mn, 1500 + CAN_TIMEOUT_SAFE_MS);
    if (!mn.safe) fail("startup: normal timeout after first msg", 0, 2);

    /* açılış: hiç mesaj gelmezse CAN_TIMEOUT_FIRST_MS'de SAFE */
    can_monitor_init(&mn, 100);
    can_monitor_tick(&mn, 100 + CAN_TIMEOUT_FIRST_MS - 1);
    if (mn.safe) fail("startup: early safe", 0, 3);
    can_monitor_tick(&mn, 100 + CAN_TIMEOUT_FIRST_MS);
    if (!mn.fault || !mn.safe) fail("startup: no safe at first timeout", 0, 4);
    /* açılışta yalnız bozuk (CRC hatalı) mesaj gelmesi açılış süresini uzatmaz */
    can_monitor_init(&mn, 0);
    can_monitor_rx(&mn, 500, 0, 0);
    can_monitor_tick(&mn, CAN_TIMEOUT_FIRST_MS);
    if (!mn.safe) fail("startup: crc-bad frame extended grace", 0, 5);
}

/* ---------------------------------------------------------------- T5 */
/* Normal moddaki kabul filtresi (0x101, 0x102, 0x301) iç geri döngüde denenir: geri dönen
 * çerçeveler de alım filtresinden geçer. Her kimlik tek tek gönderilir, 1 ms beklenir. */
static void t5_filter(void)
{
    static const uint16_t ids[] = { 0x100, 0x101, 0x102, 0x103, 0x10F, 0x110, 0x112, 0x114, 0x115,
                                    0x201, 0x202, 0x210, 0x300, 0x301, 0x7FF };
    static const uint8_t  acc[] = { 0,     1,     1,     0,     0,     1,     1,     1,     0,
                                    0,     0,     0,     0,     1,     0     };
    uint8_t tx[8] = { 1, 2, 3, 4, 5, 6, 7, 8 }, rx[8], len;
    uint16_t rid;

    bsp_can_init_loopback_filtered();
    for (uint32_t k = 0; k < sizeof(ids) / sizeof(ids[0]); k++) {
        while (!bsp_can_send(ids[k], tx, 8)) { }
        const uint32_t t0 = bsp_cycles();
        int got = 0;
        while (bsp_cycles() - t0 < RX_TIMEOUT)
            if (bsp_can_recv(&rid, rx, &len)) {
                got = 1;
                if (rid != ids[k]) fail("filter wrong id", rid, k);
            }
        put_s("   0x"); put_hex(ids[k], 3);
        put_s(got ? " accepted" : " rejected");
        if (got != acc[k]) { put_s("  <-- WRONG"); fail("filter", ids[k], k); }
        put_s("\r\n");
    }
}

/* ---------------------------------------------------------------- çalıştır */
static void report(const char *name, uint32_t fails_before)
{
    put_s(name); put_s(n_fail == fails_before ? ": PASS\r\n" : ": FAIL\r\n");
}

void can_lb_run(void)
{
    for (int k = 0; k < 6; k++) { bsp_led_toggle(); bsp_delay_ms(150); }
    put_s("\r\n=== FDCAN1 internal loopback test, 500 kbit/s ===\r\n");
    bsp_can_init_loopback();

    uint32_t f0 = n_fail;
    t1_roundtrip();
    report("T1 roundtrip 7 msg x 1000", f0);
    put_s("   frames "); put_u(lat_n);
    put_s(", latency mean "); put_u(lat_n ? (lat_sum / lat_n) / (BSP_CPU_HZ / 1000000u) : 0);
    put_s(" us, max "); put_u(lat_max / (BSP_CPU_HZ / 1000000u)); put_s(" us\r\n");

    f0 = n_fail;  t2_fault_injection();  report("T2 E2E fault injection", f0);
    f0 = n_fail;  t3_timeout();          report("T3 timeout monitor", f0);

    bsp_can_err_t e;
    bsp_can_errors(&e);
    f0 = n_fail;
    if (e.tec || e.rec || e.ep || e.bo || e.lec || e.cel || e.n_rx_lost) fail("fdcan error state", 0, 0);
    report("T4 FDCAN error counters", f0);
    put_s("   TEC "); put_u(e.tec); put_s(" REC "); put_u(e.rec); put_s(" LEC "); put_u(e.lec);
    put_s(" EP "); put_u(e.ep); put_s(" BO "); put_u(e.bo); put_s(" CEL "); put_u(e.cel);
    put_s(" rx_lost "); put_u(e.n_rx_lost); put_s("\r\n");

    f0 = n_fail;  t5_filter();           report("T5 acceptance filter 0x101/0x102/0x301/0x110-0x114", f0);

    put_s(n_fail ? "CAN LOOPBACK: FAIL (" : "CAN LOOPBACK: PASS (");
    put_u(n_fail); put_s(" errors)\r\n");

    const uint32_t blink = n_fail ? 100u : 500u;
    for (;;) { bsp_led_toggle(); bsp_delay_ms(blink); }
}
