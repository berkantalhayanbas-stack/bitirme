/* app_can_soak.c -- 10 dakikalık CAN dayanıklılık testi (gerçek hat: PA11 RX / PA12 TX + TJA1050).
 *
 * Karşı düğüm: arduino/can_soak (Arduino Uno + MCP2515), 10 Hz, ID 0x101, DLC 8,
 *              bayt 0-1 = artan 16 bit sayaç (little-endian), diğer baytlar 0.
 * Bu düğüm hiç göndermez, yalnız alır (ACK verir). Kabul filtresi normal moddakiyle aynı
 * (0x101/0x102/0x301); E2E (CRC/alive counter) burada YOK, çerçeve ham sayaçtır.
 *
 *   her 1 ms : Rx FIFO'yu boşalt; 0x101'in sayacını bir öncekiyle karşılaştır:
 *              fark 1 normal, fark > 1 -> (fark - 1) çerçeve kayıp, fark 0 -> tekrar,
 *              (geriye gidiş fark > 32768 sayılır: sıra dışı / gönderen yeniden başladı)
 *              İlk 10 s içinde sayaç < 16'ya geri dönerse gönderen yeniden başlamıştır (Uno seri port
 *              açılınca sıfırlanır; o arada ACK alamayıp tamponda bekleyen çerçeveler de sırasız
 *              gelir): istatistikler sıfırlanır ve test o çerçeveden yeniden başlar.
 *   her 10 ms: bsp_can_service (bus-off kurtarma)
 *   her 10 s : UART'a tek satır: geçen süre, alınan, kayıp, tekrar/sıra dışı, TEC, REC, bus-off
 *   test     : süre ilk çerçeveden itibaren sayılır. 600 s dolunca, alınan çerçeve henüz 6000'e
 *              ulaşmadıysa (Arduino'nun seramik rezonatörü ±%0,5 sapabilir) en çok 10 s daha
 *              beklenir. Sonra "RESULT" satırları (tablo) ve PASS/FAIL; test durur, LED
 *              PASS'te 1 s, FAIL'de 0,1 s aralıkla yanıp söner.
 *   Ölçüt    : alınan >= 6000, kayıp = 0, tekrar/sıra dışı = 0, TEC = REC = 0, bus-off = 0,
 *              Rx FIFO taşması = 0.
 */
#include <string.h>
#include "app_can_soak.h"
#include "bsp.h"
#include "bsp_can.h"

#define SOAK_ID          0x101u
#define SOAK_DURATION_MS 600000u
#define SOAK_EXTRA_MS     10000u
#define SOAK_MIN_FRAMES    6000u
#define SOAK_REPORT_MS    10000u
#define SOAK_RESTART_SEQ     16u   /* ilk 10 s'de sayaç bunun altına geri dönerse: gönderen yeniden başladı */
#define SOAK_RESTART_WIN_MS 10000u

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
static void kv(const char *k, uint32_t v) { put_s(k); put_u(v); }

static void report(uint32_t t_ms, uint32_t rx, uint32_t lost, uint32_t dup, uint32_t other,
                   const bsp_can_err_t *e)
{
    kv("t=", t_ms / 1000u);  put_s("s");
    kv(" rx ", rx);  kv(" lost ", lost);  kv(" dup/ooo ", dup);  kv(" other ", other);
    kv(" | TEC ", e->tec);  kv(" REC ", e->rec);  kv(" LEC ", e->lec);
    put_s(e->bo ? " BUS-OFF" : e->ep ? " PASSIVE" : e->ew ? " WARN" : " ACTIVE");
    kv(" bus-off ", e->n_busoff);  kv(" fifo-lost ", e->n_rx_lost);
    put_s("\r\n");
}

void can_soak_run(void)
{
    uint32_t now_ms = 0, t_last, t_first = 0, rx = 0, lost = 0, dup = 0, other = 0;
    uint16_t seq_prev = 0;
    uint8_t started = 0;
    bsp_can_err_t e;

    for (int k = 0; k < 6; k++) { bsp_led_toggle(); bsp_delay_ms(150); }
    put_s("\r\n=== CAN soak test: RX 0x101 (16-bit counter, 10 Hz), 600 s, 500 kbit/s ===\r\n");
    put_s("criteria: rx >= 6000, lost = 0, TEC = REC = 0, no bus-off; waiting for the first frame...\r\n");
    bsp_can_init_normal();
    t_last = bsp_cycles();

    for (;;) {
        while (bsp_cycles() - t_last < BSP_CPU_HZ / 1000u) { }
        t_last += BSP_CPU_HZ / 1000u;
        now_ms++;

        /* ---- her 1 ms: alım ---- */
        {
            uint16_t id;  uint8_t d[8], len;
            while (bsp_can_recv(&id, d, &len)) {
                if (id != SOAK_ID || len < 2) { other++; continue; }
                const uint16_t seq = (uint16_t)(d[0] | (d[1] << 8));
                if (!started) {
                    started = 1;  t_first = now_ms;
                    put_s("first frame, counter ");  put_u(seq);  put_s(": test started\r\n");
                } else {
                    const uint16_t diff = (uint16_t)(seq - seq_prev);
                    if (diff > 32768u && seq < SOAK_RESTART_SEQ && now_ms - t_first < SOAK_RESTART_WIN_MS) {
                        /* gönderen yeni başladı (Uno, seri port açılınca sıfırlanır): ölçüm
                         * bu çerçeveden itibaren yeniden başlar; öncesi ısınma sayılır */
                        put_s("sender restarted (counter ");  put_u(seq_prev);  put_s(" -> ");  put_u(seq);
                        put_s("): test restarted\r\n");
                        t_first = now_ms;  rx = 0;  lost = 0;  dup = 0;  other = 0;
                        bsp_can_clear_stats();
                    } else if (diff == 0u || diff > 32768u) {
                        dup++;                                    /* tekrar ya da geriye gidiş */
                    } else {
                        lost += diff - 1u;
                    }
                }
                seq_prev = seq;
                rx++;
            }
        }

        /* ---- her 10 ms: bus-off kurtarma ---- */
        if (now_ms % 10u == 0) (void)bsp_can_service(now_ms);

        if (!started) {
            if (now_ms % SOAK_REPORT_MS == 0) { bsp_can_errors(&e); report(0, 0, 0, 0, other, &e); }
            continue;
        }
        const uint32_t t = now_ms - t_first;

        /* ---- her 10 s: rapor ---- */
        if (t % SOAK_REPORT_MS == 0 && t > 0) {
            bsp_can_errors(&e);
            report(t, rx, lost, dup, other, &e);
            bsp_led_toggle();
        }

        /* ---- bitiş: 600 s (ve 6000 çerçeve ya da +10 s) ---- */
        if (t >= SOAK_DURATION_MS && (rx >= SOAK_MIN_FRAMES || t >= SOAK_DURATION_MS + SOAK_EXTRA_MS)) {
            bsp_can_errors(&e);
            const int pass = rx >= SOAK_MIN_FRAMES && lost == 0 && dup == 0 && e.tec == 0 && e.rec == 0
                             && e.n_busoff == 0 && e.n_rx_lost == 0;
            report(t, rx, lost, dup, other, &e);
            put_s("RESULT | duration_s | rx | lost | dup/ooo | TEC | REC | bus-off | fifo-lost | verdict\r\n");
            put_s("RESULT | ");  put_u(t / 1000u);  put_s(".");  put_u((t % 1000u) / 100u);
            kv(" | ", rx);  kv(" | ", lost);  kv(" | ", dup);  kv(" | ", e.tec);  kv(" | ", e.rec);
            kv(" | ", e.n_busoff);  kv(" | ", e.n_rx_lost);
            put_s(pass ? " | PASS\r\n" : " | FAIL\r\n");
            for (;;) {                       /* test bitti: LED ile sonuç (PASS yavaş, FAIL hızlı) */
                bsp_led_toggle();
                bsp_delay_ms(pass ? 1000u : 100u);
            }
        }
    }
}
