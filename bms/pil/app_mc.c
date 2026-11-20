/* app_mc.c -- kart üstü Monte Carlo (APP_MODE = APP_MC = 8).
 *
 * bms/sim/mc_core.c'nin koşularını gerçek zaman beklemeden art arda çalıştırır: her koşu 600 s'lik
 * kapalı çevrim (dijital ikiz + bms_step, Ts = 0,1 s). CAN yok; ikiz aynı çekirdekte. Aynı tohumla
 * PC'de tools/mc/mc_sil.c birebir aynı satırları üretmeli (tools/mc_run.py karşılaştırır).
 *
 * UART (LPUART1 115200, İngilizce; gateway.py protokolü):
 *   MC,START,<n>,<seed>,<steps>
 *   RS,<id>,<name>
 *   RE,<id>,<PASS|FAIL>,<fault>,<react_ms>,<min_soc>,<max_tc>,<soc_err>,<final_state>,<tc_limp>,<tc_safe>,<why>,<run_ms>
 *   MC,DONE,<n>,<pass>,<fail>,<total_ms>,<max_run_ms>
 * Derleme: python tools/fw_build.py --app 8 [--define MC_N=500 --define MC_SEED=1] --flash
 */
#include <string.h>
#include "app_mc.h"
#include "bsp.h"
#include "mc_core.h"

#ifndef MC_N
#define MC_N     200u           /* koşu sayısı */
#endif
#ifndef MC_SEED
#define MC_SEED  1u             /* temel tohum (tohum = MC_SEED + koşu no) */
#endif

static void put_s(const char *s) { bsp_uart_write((const uint8_t *)s, (uint32_t)strlen(s)); }
static void put_u(uint32_t v)
{
    char b[11];
    int i = 10;
    b[i] = 0;
    do { b[--i] = (char)('0' + v % 10u); v /= 10u; } while (v);
    put_s(&b[i]);
}

void mc_app_run(void)
{
    static char buf[160];
    bsp_delay_ms(1500);         /* PC portu açınca baştan okuyabilsin */
    put_s("MC,START,");  put_u(MC_N);  put_s(",");  put_u(MC_SEED);  put_s(",");  put_u(MC_STEPS);  put_s("\r\n");
    uint32_t n_pass = 0, total_ms = 0, max_ms = 0;
    for (uint32_t id = 1; id <= MC_N; id++) {
        mc_cfg_t c;
        mc_res_t r;
        mc_config(id, MC_SEED, &c);
        mc_line_rs(&c, buf);  put_s(buf);  put_s("\r\n");
        const uint32_t c0 = bsp_cycles();
        mc_run(&c, &r);
        const uint32_t ms = (bsp_cycles() - c0) / (BSP_CPU_HZ / 1000u);
        mc_line_re(&c, &r, buf);  put_s(buf);  put_s(",");  put_u(ms);  put_s("\r\n");
        n_pass += r.pass;
        total_ms += ms;
        if (ms > max_ms) max_ms = ms;
        bsp_led_toggle();
    }
    put_s("MC,DONE,");  put_u(MC_N);  put_s(",");  put_u(n_pass);  put_s(",");  put_u(MC_N - n_pass);
    put_s(",");  put_u(total_ms);  put_s(",");  put_u(max_ms);  put_s("\r\n");
    for (;;) { }
}
