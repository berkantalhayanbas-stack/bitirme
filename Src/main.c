/* main.c -- BMS ECU (NUCLEO-G431RB, HAL'siz).
 * Boş CubeIDE projesindeki Src/main.c'nin yerine bu içerik konur.
 *
 * Hangi uygulamanın çalışacağı APP_MODE ile seçilir:
 *   APP_PIL          PC'den UART ile PIL testi (pil_test.py)
 *   APP_CAN_LOOPBACK FDCAN1 iç geri döngü testi, sonuç UART'a metin olarak yazılır
 *   APP_CAN_NODE     FDCAN1 normal mod, gerçek hat (PA11/PA12 + transceiver), UART'a durum raporu
 */
#include "bsp.h"
#include "app_pil.h"
#include "app_can_lb.h"
#include "app_can_node.h"

#define APP_PIL           1
#define APP_CAN_LOOPBACK  2
#define APP_CAN_NODE      3
#ifndef APP_MODE
#define APP_MODE  APP_CAN_LOOPBACK
#endif

int main(void)
{
    bsp_init();     /* FPU, 170 MHz saat, LED, LPUART1, DWT */
#if APP_MODE == APP_CAN_LOOPBACK
    can_lb_run();   /* geri dönmez */
#elif APP_MODE == APP_CAN_NODE
    can_node_run(); /* geri dönmez */
#else
    pil_run();      /* geri dönmez */
#endif
    for (;;) { }
}
