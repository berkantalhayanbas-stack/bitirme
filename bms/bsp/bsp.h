/* bsp.h -- NUCLEO-G431RB kart destek katmanı (board support package), kayıt seviyesinde.
 * HAL yok: saat, GPIO ve LPUART1 doğrudan CMSIS kayıt tanımlarıyla (stm32g431xx.h) kurulur.
 *
 *   Saat  : HSI16 -> PLL (/4 x85 /2) = 170 MHz, "boost" modu, flash 4 bekleme durumu
 *   LED   : LD2 yeşil, PA5
 *   UART  : LPUART1, PA2 (TX) / PA3 (RX), AF12 -> ST-LINK sanal COM portu, 115200 8N1
 *   Zaman : DWT döngü sayacı (170 döngü = 1 µs)
 */
#ifndef BSP_H
#define BSP_H
#include <stdint.h>

#define BSP_CPU_HZ   170000000u
#define BSP_BAUD     115200u

void     bsp_init(void);                          /* FPU, saat, LED, UART, DWT */
void     bsp_led_toggle(void);
void     bsp_delay_ms(uint32_t ms);
uint32_t bsp_cycles(void);                        /* DWT->CYCCNT */
void     bsp_uart_write(const uint8_t *buf, uint32_t n);
/* Bloklamayan gönderim: 2 KB halka tampon + TXE kesmesi; döner = kuyruğa giren bayt (dolu ise
 * kalan atılır, bsp_uart_tx_drops sayar). bsp_uart_write önce bu kuyruğun boşalmasını bekler. */
uint32_t bsp_uart_write_async(const uint8_t *buf, uint32_t n);
uint32_t bsp_uart_tx_drops(void);
void     bsp_uart_read(uint8_t *buf, uint32_t n);  /* n bayt gelene kadar bekler */
/* Kesmeli alım (komut satırları, APP_MC): açıldıktan sonra RXNE kesmesi baytları 256 B halka
 * tampona yazar; bsp_uart_read ile birlikte kullanma. getc: bayt varsa 1 döner. */
void     bsp_uart_rx_enable(void);
int      bsp_uart_getc(uint8_t *c);
void     bsp_reset(void);                         /* yazılımla sistem sıfırlaması (geri dönmez) */

#endif
