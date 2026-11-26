/* sw_spi_cfg.h -- MCP2515 için yazılımla SPI (bit-bang) seçimi ve pinleri.
 *
 * Neden (2026-11-26): tezgâhtaki Uno'nun donanım MOSI pini (D11 = PB3) hasarlı; D11 ve ICSP MOSI'den çıkış
 * yok (arduino/uno_spi_selftest ile ölçüldü), D12 / D13 sağlam. MCP2515'in SI kablosu D9'a taşındı.
 *   TW_SW_SPI 1: MOSI D9 (PB1), SCK D13 (PB5), MISO D12 (PB4), CS D10; SPI modu 0, MSB önce, ~1,3 MHz
 *               (doğrudan port erişimi; donanım SPI'ı açılmaz, D11 giriş kalır, hiç sürülmez)
 *   TW_SW_SPI 0: sağlam bir Uno'da kütüphanenin donanım SPI'ı (D11 MOSI), kablolama docs/bench_wiring.md
 * Zamanlama: bayt ~6 µs (donanımda ~1 µs). 100 ms'lik çevrimde ~10 çerçeve + yoklama ~1-2 ms ekler;
 * sketch başındaki "Zamanlama" payı içinde. */
#ifndef SW_SPI_CFG_H
#define SW_SPI_CFG_H

#define TW_SW_SPI 1

#if TW_SW_SPI
#include <Arduino.h>

#define SW_MOSI_BIT  1      /* PB1 = D9 */
#define SW_MISO_BIT  4      /* PB4 = D12 */
#define SW_SCK_BIT   5      /* PB5 = D13 */

static inline void sw_spi_init(void)
{
    pinMode(11, INPUT);                 /* hasarlı donanım MOSI: sürme */
    pinMode(9, OUTPUT);
    pinMode(12, INPUT);
    pinMode(13, OUTPUT);
    PORTB &= (uint8_t)~_BV(SW_SCK_BIT); /* mod 0: saat boşta düşük */
}

/* SPI modu 0: veri düşen kenarda değişir, yükselen kenarda örneklenir. PORTB bit işlemleri sbi/cbi (atomik) */
static inline uint8_t sw_spi_xfer(uint8_t out)
{
    uint8_t in = 0;
    for (uint8_t i = 0; i < 8; i++) {
        if (out & 0x80) PORTB |= _BV(SW_MOSI_BIT); else PORTB &= (uint8_t)~_BV(SW_MOSI_BIT);
        out <<= 1;
        PORTB |= _BV(SW_SCK_BIT);
        in = (uint8_t)((in << 1) | ((PINB >> SW_MISO_BIT) & 1u));
        PORTB &= (uint8_t)~_BV(SW_SCK_BIT);
    }
    return in;
}
#endif

#endif
