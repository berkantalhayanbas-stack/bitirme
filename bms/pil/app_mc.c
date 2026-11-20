/* app_mc.c -- kart üstü Monte Carlo (APP_MODE = APP_MC = 8).
 *
 * bms/sim/mc_core.c'nin koşularını gerçek zaman beklemeden art arda çalıştırır: her koşu 600 s'lik
 * kapalı çevrim (dijital ikiz + bms_step, Ts = 0,1 s). CAN yok; ikiz aynı çekirdekte. Aynı tohumla
 * PC'de tools/mc/mc_sil.c birebir aynı satırları üretmeli (tools/mc_run.py karşılaştırır).
 *
 * UART (LPUART1 115200, İngilizce; tools/web/gateway.py protokolü):
 *   MC,READY,<n>,<seed>,<steps>                      açılış
 *   RS,<id>,<name>                                   koşu başladı
 *   D,<t_ms>,<id>,<soc>,<soh>,<Tc>,<I_allow>,<state>,0,0   iz: benzetim zamanında 1 s'de bir
 *   RE,<id>,<PASS|FAIL>,<fault>,<react_ms>,<min_soc>,<max_tc>,<soc_err>,<final_state>,<tc_limp>,<tc_safe>,<why>,<run_ms>
 *   MC,DONE,<n>,<pass>,<fail>,<total_ms>,<max_run_ms>   ALL bitti (STOP ile yarıda: MC,STOPPED,...)
 * Komutlar (PC/ESP -> kart, satır sonu \n): RUN <id> (tek koşu, izli) | ALL [n] | STOP
 * ALL'da her MC_TRACE_EVERY'inci koşu izlidir (0 = hiç). MC_AUTO = 1: açılışta kendiliğinden ALL.
 * Derleme: python tools/fw_build.py --app 8 [--define MC_N=500 --define MC_SEED=1] --flash
 */
#include <string.h>
#include "app_mc.h"
#include "bsp.h"
#include "mc_core.h"

#ifndef MC_N
#define MC_N     200u           /* ALL koşu sayısı */
#endif
#ifndef MC_SEED
#define MC_SEED  1u             /* temel tohum (tohum = MC_SEED + koşu no) */
#endif
#ifndef MC_AUTO
#define MC_AUTO  1              /* 1: açılışta ALL (tools/mc_run.py bunu bekler) */
#endif
#ifndef MC_TRACE_EVERY
#define MC_TRACE_EVERY 100u     /* ALL'da her N. koşu 1 Hz izli (gateway/dashboard) */
#endif

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
static void put_f1(float v)     /* bir ondalık, virgülle başlar */
{
    int32_t x = (int32_t)(v * 10.0f + (v < 0.0f ? -0.5f : 0.5f));
    put_s(",");
    if (x < 0) { put_s("-"); x = -x; }
    put_u((uint32_t)x / 10u);  put_s(".");  put_u((uint32_t)x % 10u);
}

/* ---------------------------------------------------------------- iz (D satırı) */
static void trace(const mc_cfg_t *c, uint32_t t_ms, const bms_out_t *o)
{
    const float soh = soh_tree(0.0f, 0.0f, o->tc, o->soc, 0.0f, o->rr);   /* app_ecu ile aynı */
    put_s("D,");  put_u(t_ms);  put_s(",");  put_u(c->id);
    put_f1(100.0f * o->soc);  put_f1(100.0f * soh);  put_f1(o->tc);  put_f1(o->dec.i_allow_dis);
    put_s(",");  put_u(o->dec.state);  put_s(",0,0\r\n");
}

/* ---------------------------------------------------------------- komut satırı */
static char line[48];
static uint32_t line_n;
enum { CMD_NONE, CMD_RUN, CMD_ALL, CMD_STOP };
static uint32_t parse_u(const char *s, uint32_t def)
{
    while (*s == ' ') s++;
    if (*s < '0' || *s > '9') return def;
    uint32_t v = 0;
    while (*s >= '0' && *s <= '9') v = v * 10u + (uint32_t)(*s++ - '0');
    return v;
}
static int poll_cmd(uint32_t *arg)
{
    uint8_t b;
    while (bsp_uart_getc(&b)) {
        if (b == '\r') continue;
        if (b != '\n') { if (line_n < sizeof(line) - 1u) line[line_n++] = (char)b;  continue; }
        line[line_n] = 0;  line_n = 0;
        if (!strncmp(line, "RUN", 3)) { *arg = parse_u(line + 3, 1u);  return CMD_RUN; }
        if (!strncmp(line, "ALL", 3)) { *arg = parse_u(line + 3, MC_N);  return CMD_ALL; }
        if (!strncmp(line, "STOP", 4)) return CMD_STOP;
        put_s("MC,ERR,unknown command\r\n");
    }
    return CMD_NONE;
}

/* ---------------------------------------------------------------- koşular */
static char buf[160];
static uint32_t one(uint32_t id, int traced, uint8_t *pass)
{
    mc_cfg_t c;
    mc_res_t r;
    mc_config(id, MC_SEED, &c);
    mc_line_rs(&c, buf);  put_s(buf);  put_s("\r\n");
    mc_trace = traced ? trace : 0;
    const uint32_t c0 = bsp_cycles();
    mc_run(&c, &r);
    const uint32_t ms = (bsp_cycles() - c0) / (BSP_CPU_HZ / 1000u);
    mc_line_re(&c, &r, buf);  put_s(buf);  put_s(",");  put_u(ms);  put_s("\r\n");
    *pass = r.pass;
    bsp_led_toggle();
    return ms;
}

static void all(uint32_t n)
{
    uint32_t n_pass = 0, total_ms = 0, max_ms = 0, id = 1, arg;
    for (; id <= n; id++) {
        const int cmd = poll_cmd(&arg);
        if (cmd == CMD_STOP) break;
        if (cmd != CMD_NONE) put_s("MC,BUSY,send STOP first\r\n");
        uint8_t p;
        const uint32_t ms = one(id, MC_TRACE_EVERY && id % MC_TRACE_EVERY == 0u, &p);
        n_pass += p;  total_ms += ms;
        if (ms > max_ms) max_ms = ms;
    }
    const uint32_t done = id - 1u;
    put_s(done == n ? "MC,DONE," : "MC,STOPPED,");  put_u(done);  put_s(",");  put_u(n_pass);
    put_s(",");  put_u(done - n_pass);  put_s(",");  put_u(total_ms);  put_s(",");  put_u(max_ms);  put_s("\r\n");
}

void mc_app_run(void)
{
    bsp_uart_rx_enable();
    bsp_delay_ms(1500);         /* PC portu açınca baştan okuyabilsin */
    put_s("MC,READY,");  put_u(MC_N);  put_s(",");  put_u(MC_SEED);  put_s(",");  put_u(MC_STEPS);  put_s("\r\n");
    if (MC_AUTO) all(MC_N);
    for (;;) {
        uint32_t arg = 0;
        uint8_t p;
        switch (poll_cmd(&arg)) {
        case CMD_RUN:  (void)one(arg ? arg : 1u, 1, &p);  break;
        case CMD_ALL:  all(arg);  break;
        default: break;
        }
    }
}
