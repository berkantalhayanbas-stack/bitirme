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
void     bsp_uart_read(uint8_t *buf, uint32_t n);  /* n bayt gelene kadar bekler */
void     bsp_reset(void);                         /* yazılımla sistem sıfırlaması (geri dönmez) */

#endif
