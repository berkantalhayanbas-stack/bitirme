/* main.c -- BMS ECU (NUCLEO-G431RB, HAL'siz).
 * Boş CubeIDE projesindeki Src/main.c'nin yerine bu içerik konur.
 *
 * Hangi uygulamanın çalışacağı APP_MODE ile seçilir:
 *   APP_PIL          PC'den UART ile PIL testi (tools/pil_test/pil_test.py)
 *   APP_CAN_LOOPBACK FDCAN1 iç geri döngü testi, sonuç UART'a metin olarak yazılır
 *   APP_CAN_NODE     FDCAN1 normal mod, gerçek hat (PA11/PA12 + transceiver), UART'a durum raporu
 *   APP_ECU          BMS ECU: CAN 0x101/0x102 -> bms_step -> 0x201/0x202 (gerçek hat)
 *   APP_CAN_SOAK     10 dakikalık CAN dayanıklılık testi (karşı düğüm: arduino/can_soak)
 *   APP_CORDIC       CORDIC ile yazılım expf/logf karşılaştırması (döngü + hata), UART tablosu
 * APP_MODE derleme bayrağıyla da verilebilir: -DAPP_MODE=5
 */
#include "bsp.h"
#include "app_pil.h"
#include "app_can_lb.h"
#include "app_can_node.h"
#include "app_ecu.h"
#include "app_can_soak.h"
#include "app_cordic.h"

#define APP_PIL           1
#define APP_CAN_LOOPBACK  2
#define APP_CAN_NODE      3
#define APP_ECU           4
#define APP_CAN_SOAK      5
#define APP_CORDIC        6
#ifndef APP_MODE
#define APP_MODE  APP_ECU
#endif

int main(void)
{
    bsp_init();     /* FPU, 170 MHz saat, LED, LPUART1, DWT */
#if APP_MODE == APP_CAN_LOOPBACK
    can_lb_run();   /* geri dönmez */
#elif APP_MODE == APP_CAN_NODE
    can_node_run(); /* geri dönmez */
#elif APP_MODE == APP_ECU
    ecu_run();      /* geri dönmez */
#elif APP_MODE == APP_CAN_SOAK
    can_soak_run(); /* geri dönmez */
#elif APP_MODE == APP_CORDIC
    cordic_run();   /* geri dönmez */
#else
    pil_run();      /* geri dönmez */
#endif
    for (;;) { }
}
