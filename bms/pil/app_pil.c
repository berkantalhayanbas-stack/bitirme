/* app_pil.c -- Processor-in-the-loop (PIL) uygulaması, NUCLEO-G431RB. Donanıma bsp.h üzerinden erişir.
 *
 * Protokol (ikili, little-endian, CRC-8 SAE J1850):
 *   PC -> kart, 28 bayt:  A5 5A | cmd (1) | 6 x float32: I V T V_min V_max T_amb | crc(cmd..payload)
 *       cmd 0 = bir BMS adımı, cmd 1 = sıfırla (payload[0] = EKF başlangıç SOC'si)
 *   kart -> PC, 48 bayt:  5A A5 | status (1) | 9 x float32: soc bias tc ts ru rr i_dis i_chg psi
 *                         | state (1) | cause (2) | flags (1) | cycles (4) | crc(status..cycles)
 *       status 0 = tamam, 1 = gelen çerçevede CRC hatası, 2 = sıfırlandı
 *       flags bit0 T_valid, bit1 I_valid, bit2 EKF ölçümü reddetti
 *       cycles = bms_step süresi, DWT döngü sayısı (170 MHz'de 170 döngü = 1 µs)
 * LED (PA5): açılışta 3 kez yanıp söner, sonra her 100 adımda bir durum değiştirir.
 */
#include <string.h>
#include "app_pil.h"
#include "bsp.h"
#include "bms_algo.h"
#include "bms_can.h"           /* crc8_j1850 */

#define RX_LEN 28
#define TX_LEN 48

static bms_t bms;

static void put_f(uint8_t *p, float v) { memcpy(p, &v, 4); }

static void send(uint8_t status, const bms_out_t *o, uint32_t cycles)
{
    uint8_t tx[TX_LEN] = { 0x5A, 0xA5, status };
    if (o) {
        const float f[9] = { o->soc, o->bias, o->tc, o->ts, o->ru, o->rr,
                             o->dec.i_allow_dis, o->dec.i_allow_chg, o->dec.psi };
        for (int k = 0; k < 9; k++) put_f(&tx[3 + 4 * k], f[k]);
        tx[39] = o->dec.state;
        tx[40] = (uint8_t)(o->dec.cause & 0xFF);
        tx[41] = (uint8_t)(o->dec.cause >> 8);
        tx[42] = (uint8_t)(o->t_valid | (o->i_valid << 1) | (o->rejected << 2));
    }
    memcpy(&tx[43], &cycles, 4);
    tx[47] = crc8_j1850(&tx[2], 45);
    bsp_uart_write(tx, TX_LEN);
}

void pil_run(void)
{
    static const char hello[] = "BMS PIL ready\r\n";
    uint8_t rx[RX_LEN];
    uint32_t n = 0;

    for (int k = 0; k < 6; k++) { bsp_led_toggle(); bsp_delay_ms(150); }
    bsp_uart_write((const uint8_t *)hello, sizeof(hello) - 1);
    bms_init(&bms, 0.5f, 1.0f, 1.0f, 0.0f, 0.0f);

    for (;;) {
        /* ---- senkronizasyon: A5 5A ara ---- */
        bsp_uart_read(&rx[0], 1);
        if (rx[0] != 0xA5) continue;
        bsp_uart_read(&rx[1], 1);
        if (rx[1] != 0x5A) continue;
        bsp_uart_read(&rx[2], RX_LEN - 2);
        if (crc8_j1850(&rx[2], 25) != rx[27]) { send(1, NULL, 0); continue; }

        float in_f[6];
        memcpy(in_f, &rx[3], sizeof(in_f));
        if (rx[2] == 1) {                                 /* sıfırla */
            bms_init(&bms, in_f[0], 1.0f, 1.0f, 0.0f, 0.0f);
            n = 0;
            send(2, NULL, 0);
            continue;
        }
        bms_in_t in = { in_f[0], in_f[1], in_f[2], in_f[3], in_f[4], in_f[5], 1 };
        bms_out_t out;
        const uint32_t t0 = bsp_cycles();
        bms_step(&bms, &in, &out);
        const uint32_t cycles = bsp_cycles() - t0;
        send(0, &out, cycles);
        if (++n % 100 == 0) bsp_led_toggle();
    }
}
