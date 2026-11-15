/* app_cordic.c -- Adım 7: expf / logf, yazılım (newlib libm) ile CORDIC karşılaştırması (APP_CORDIC).
 *
 * Girdiler (sabit, 1000'er nokta):
 *   exp: x ∈ [-12, 3] doğrusal  (BMS'deki kullanım: Arrhenius e^(Ea/R·(1/T-1/Tref)) ≈ [-0,6, 0,9],
 *        RC ayrıklaştırma e^(-Ts/τ) ≈ [-0,01, 0], SOC terimi e^(-SOC/0,08) ∈ [-12,5, 0])
 *   ln : x ∈ [1e-3, 1e3] logaritmik (yaşlanma modelindeki log terimleri)
 *   Arrhenius: T ∈ [-20, 60] °C, bms_algo.c'deki arrhenius() argümanıyla
 * Ölçüm: her çağrı DWT ile tek tek zamanlanır (ölçüm yükü boş ölçümle çıkarılır): ortalama ve en
 *   kötü döngü. Hata: libm sonucuna göre (libm float, ~1 ulp): exp için en büyük bağıl hata, ln için
 *   en büyük mutlak hata. CORDIC hassasiyeti 3, 6, 10, 15 döngüyle (12..60 yineleme) ayrı ayrı.
 * Çıktı: UART'a tablo ("RESULT" satırları), sonra LED yanıp söner.
 */
#include <math.h>
#include <string.h>
#include "app_cordic.h"
#include "bsp.h"
#include "bsp_cordic.h"
#include "bms_config.h"

#define N 1000

static float xe[N], xl[N], ref_e[N], ref_l[N];
static volatile float sink;

static void put_s(const char *s) { bsp_uart_write((const uint8_t *)s, (uint32_t)strlen(s)); }
static void put_u(uint32_t v)
{
    char b[11];
    int i = 10;
    b[i] = 0;
    do { b[--i] = (char)('0' + v % 10u); v /= 10u; } while (v);
    put_s(&b[i]);
}
static void put_e(float v)                      /* bilimsel gösterim, 2 ondalık: 1.23e-06 */
{
    if (v == 0.0f) { put_s("0"); return; }
    int ex = 0;
    while (v >= 10.0f) { v /= 10.0f; ex++; }
    while (v < 1.0f)   { v *= 10.0f; ex--; }
    const uint32_t m = (uint32_t)(v * 100.0f + 0.5f);
    put_u(m / 100u); put_s("."); put_u((m / 10u) % 10u); put_u(m % 10u);
    put_s("e"); put_s(ex < 0 ? "-" : "+");
    if (ex < 0) ex = -ex;
    if (ex < 10) put_s("0");
    put_u((uint32_t)ex);
}

typedef float (*fn_t)(float);
typedef struct { uint32_t avg, max; float err; } res_t;

static uint32_t overhead;

/* fn'yi x[] üzerinde çağırır; ref verilirse hata (rel = bağıl, değilse mutlak) */
static res_t bench(fn_t fn, const float *x, const float *ref, int rel)
{
    res_t r = { 0, 0, 0.0f };
    uint64_t sum = 0;
    for (int i = 0; i < N; i++) {
        const uint32_t t0 = bsp_cycles();
        const float y = fn(x[i]);
        uint32_t c = bsp_cycles() - t0;
        c = c > overhead ? c - overhead : 0u;
        sum += c;
        if (c > r.max) r.max = c;
        sink = y;
        if (ref) {
            float d = fabsf(y - ref[i]);
            if (rel) d /= fabsf(ref[i]);
            if (d > r.err) r.err = d;
        }
    }
    r.avg = (uint32_t)((sum + N / 2) / N);
    return r;
}

static float lib_exp(float x) { return expf(x); }
static float lib_log(float x) { return logf(x); }
static float arr_lib(float T) { return expf(CELL_EA_R0 / RGAS * (1.0f / (T + 273.15f) - 1.0f / 298.15f)); }
static float arr_cor(float T) { return bsp_cordic_expf(CELL_EA_R0 / RGAS * (1.0f / (T + 273.15f) - 1.0f / 298.15f)); }
static float nop(float x) { return x; }

static void row(const char *name, const char *prec, res_t e, res_t l, res_t a)
{
    put_s("RESULT | "); put_s(name); put_s(" | "); put_s(prec);
    put_s(" | "); put_u(e.avg); put_s(" | "); put_u(e.max); put_s(" | "); put_e(e.err);
    put_s(" | "); put_u(l.avg); put_s(" | "); put_u(l.max); put_s(" | "); put_e(l.err);
    put_s(" | "); put_e(a.err); put_s("\r\n");
}

void cordic_run(void)
{
    static float xa[N], ref_a[N];
    for (int k = 0; k < 6; k++) { bsp_led_toggle(); bsp_delay_ms(150); }
    put_s("\r\n=== CORDIC vs software expf/logf (newlib, FPU), 170 MHz, N = 1000 per function ===\r\n");
    bsp_cordic_init();

    for (int i = 0; i < N; i++) {
        xe[i] = -12.0f + 15.0f * (float)i / (float)(N - 1);
        xl[i] = expf(logf(1e-3f) + (logf(1e3f) - logf(1e-3f)) * (float)i / (float)(N - 1));
        xa[i] = -20.0f + 80.0f * (float)i / (float)(N - 1);
        ref_e[i] = expf(xe[i]);  ref_l[i] = logf(xl[i]);  ref_a[i] = arr_lib(xa[i]);
    }
    overhead = 0;
    overhead = bench(nop, xe, 0, 0).avg;          /* zamanlama + çağrı yükü */

    put_s("RESULT | impl | precision | exp avg cyc | exp max cyc | exp max rel err | ln avg cyc | "
          "ln max cyc | ln max abs err | Arrhenius max rel err\r\n");
    row("libm", "-", bench(lib_exp, xe, 0, 1), bench(lib_log, xl, 0, 0), bench(arr_lib, xa, 0, 1));
    static const uint32_t prec[] = { 3, 6, 10, 15 };
    for (unsigned p = 0; p < sizeof(prec) / sizeof(prec[0]); p++) {
        bsp_cordic_precision(prec[p]);
        char pb[16] = "    cycles";
        pb[0] = (char)('0' + prec[p] / 10u);  pb[1] = (char)('0' + prec[p] % 10u);
        if (pb[0] == '0') pb[0] = ' ';
        row("CORDIC", pb, bench(bsp_cordic_expf, xe, ref_e, 1), bench(bsp_cordic_logf, xl, ref_l, 0),
            bench(arr_cor, xa, ref_a, 1));
    }
    put_s("(cycles: per call incl. call overhead, timing overhead subtracted; 170 cycles = 1 us; "
          "errors vs libm float)\r\nDONE\r\n");
    for (;;) { bsp_led_toggle(); bsp_delay_ms(500); }
}
