/* app_can_node.c -- FDCAN1 normal mod düğümü, gerçek CAN hattı (PA11 RX / PA12 TX + transceiver).
 *
 *   her 10 ms : bsp_can_service (bus-off kurtarma), alınan çerçeveleri çöz, izleyiciler
 *   her 100 ms: ECU_STATUS (0x202) gönder, sayaç ve CRC ile
 *   her 1 s   : LPUART1'e (115200 8N1) tek satır durum raporu
 *
 * Kabul filtresi yalnızca 0x101, 0x102, 0x301'i geçirir. Hatta başka düğüm yoksa gönderilen
 * çerçeveye ACK gelmez: TEC 128'e çıkar ve düğüm error passive'de kalır (LEC = 3, ack hatası).
 * Bu, kablo/transceiver yokken beklenen davranıştır, bus-off'a götürmez.
 */
#include <string.h>
#include "app_can_node.h"
#include "bsp.h"
#include "bsp_can.h"
#include "bms_can.h"

enum { R_M1, R_M2, R_CMD, N_RX };          /* 0x101, 0x102, 0x301 */

static void put_s(const char *s) { bsp_uart_write((const uint8_t *)s, (uint32_t)strlen(s)); }
static void put_u(uint32_t v)
{
    char b[11];
    int i = 10;
    b[i] = 0;
    do { b[--i] = (char)('0' + v % 10u); v /= 10u; } while (v);
    put_s(&b[i]);
}
static void kv(const char *k, uint32_t v) { put_s(k); put_u(v); }

void can_node_run(void)
{
    static can_monitor_t mon[N_RX];
    uint32_t now_ms = 0, t_last = bsp_cycles(), tx_ok = 0, tx_full = 0, other = 0;
    uint8_t cnt_tx = 0;

    for (int k = 0; k < 6; k++) { bsp_led_toggle(); bsp_delay_ms(150); }
    put_s("\r\n=== FDCAN1 normal mode node, 500 kbit/s, PA11 RX / PA12 TX ===\r\n");
    bsp_can_init_normal();
    for (int m = 0; m < N_RX; m++) can_monitor_init(&mon[m], 0);

    for (;;) {
        /* 1 ms'lik adımlarla zaman (DWT taşması 25 s'de bir; fark hesabı taşmaya dayanıklı) */
        while (bsp_cycles() - t_last < BSP_CPU_HZ / 1000u) { }
        t_last += BSP_CPU_HZ / 1000u;
        now_ms++;

        if (now_ms % 10u == 0) {
            (void)bsp_can_service(now_ms);       /* olaylar saniyelik raporda: bo#, rec# */

            uint16_t id;  uint8_t d[8], len, c;
            while (bsp_can_recv(&id, d, &len)) {
                can_meas1_t a;  can_meas2_t b;  can_cmd_t q;
                int m = -1, ok = 0;
                if (len != 8)                    { other++; continue; }
                if (id == CAN_ID_EMU_MEAS1)      { m = R_M1;  ok = can_decode_meas1(d, &a, &c); }
                else if (id == CAN_ID_EMU_MEAS2) { m = R_M2;  ok = can_decode_meas2(d, &b, &c); }
                else if (id == CAN_ID_PC_CMD)    { m = R_CMD; ok = can_decode_cmd(d, &q, &c); }
                if (m < 0) { other++; continue; }               /* filtre doğruysa olmamalı */
                can_monitor_rx(&mon[m], now_ms, ok, c);
            }
            for (int m = 0; m < N_RX; m++) can_monitor_tick(&mon[m], now_ms);
        }

        if (now_ms % 100u == 0) {
            const can_status_t st = { 50.0f, 25.0f, 100.0f, 0.0f };   /* örnek içerik */
            uint8_t d[8];
            can_encode_status(&st, cnt_tx, d);
            if (bsp_can_send(CAN_ID_ECU_STATUS, d, 8)) { tx_ok++; cnt_tx = (cnt_tx + 1u) & 0x0Fu; }
            else tx_full++;
        }

        if (now_ms % 1000u == 0) {
            bsp_can_err_t e;
            bsp_can_errors(&e);
            kv("t=", now_ms / 1000u);
            for (int m = 0; m < N_RX; m++) {
                put_s(m == R_M1 ? "s 101:" : m == R_M2 ? " 102:" : " 301:");
                put_u(mon[m].n_ok);
                if (mon[m].n_crc_err || mon[m].n_cnt_err || mon[m].n_lost) {
                    kv("(crc", mon[m].n_crc_err); kv(" cnt", mon[m].n_cnt_err); kv(" lost", mon[m].n_lost);
                    put_s(")");
                }
                if (mon[m].safe) put_s("!SAFE"); else if (mon[m].fault) put_s("!TO");
            }
            kv(" other:", other);
            kv(" | tx ok:", tx_ok); kv(" busy:", tx_full);
            kv(" | TEC ", e.tec); kv(" REC ", e.rec); kv(" LEC ", e.lec); kv(" CEL ", e.cel);
            put_s(e.bo ? " BUS-OFF" : e.ep ? " PASSIVE" : e.ew ? " WARN" : " ACTIVE");
            kv(" bo#", e.n_busoff); kv(" rec#", e.n_recover); kv(" lost#", e.n_rx_lost);
            put_s("\r\n");
            bsp_led_toggle();
        }
    }
}
