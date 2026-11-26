/* app_ecu.c -- BMS ECU uygulaması: gerçek CAN hattında tam zincir (PA11 RX / PA12 TX + TJA1050).
 *
 *   0x101 (V, I, T) + 0x102 (V_min, V_max, T_amb)  ->  bms_step  ->  0x201 (I_allow, durum, neden)
 *                                                                    0x202 (SOC, Tc, SOH, psi)
 *   0x301 (PC komutu, olay mesajı)                   ->  ECU komutları 0x01-0x03 (bms_can.h)
 *   0x110-0x114 (19 grup gerilimi)                   ->  bal_step (bms_bal.c) -> 0x210 (dengeleme)
 *
 *   Adım 8: hücre çerçeveleri ayrı izleyicilerle izlenir; eski/eksikse yalnız dengeleme kapanır
 *   (CAN_ok'a girmez, mcp2515_node / köprü gibi hücre göndermeyen EMU'larla da çalışır).
 *   0x210 her BMS adımında 0x201 ve 0x202'den sonra gönderilir.
 *
 *   her 1 ms  : alınan çerçeveleri çöz (CRC + sayaç); yeni ölçüm çifti geldiyse bir bms_step,
 *               0x201 ve 0x202 gönder; 0x301 komutunu hemen işle
 *   her 10 ms : bsp_can_service (bus-off kurtarma), zaman aşımı izleyicileri (yalnız 0x101 / 0x102)
 *   her 1 s   : LPUART1'e (115200 8N1) tek satır durum raporu
 *
 * Kurallar
 *   - Adım ölçümle tetiklenir: son adımdan beri hem yeni 0x101 hem yeni 0x102 geldiyse bir
 *     bms_step koşar (her ölçüm örneği tam bir kez kullanılır; EMU periyodu 100 ms = BMS_TS).
 *     Çift ECU_STEP_LATE_MS içinde tamamlanmazsa (kayıp çerçeve) son geçerli ölçümlerle adım
 *     atılır (sıfır dereceli tutucu), sonra 100 ms'de bir; 500 ms'de SAFE.
 *     Bu sayede kayıtlı veri CAN'dan oynatılırken (arduino/mcp2515_bridge + can_replay.py)
 *     sonuçlar PC'deki SIL çıktısıyla örnek örnek karşılaştırılabilir.
 *   - CAN_ok = 0x101 ve 0x102 izleyicilerinin hiçbiri SAFE değil. SAFE kalıcıdır (izleyici ve
 *     karar mekanizması kilitler); çıkış ECU'nun yeniden başlatılmasıyla: reset düğmesi, güç
 *     ya da korumalı 0x301 komutu (CAN_CMD_ECU_RESTART).
 *   - Açılış: iki mesajdan da ilk geçerli ölçüm gelene kadar bms_step koşmaz ve ECU hiçbir şey
 *     göndermez. CAN_TIMEOUT_FIRST_MS içinde ölçüm gelmezse yalnız 0x201 gönderilir:
 *     SAFE, I_allow = 0, neden 128 (0x202 gönderilmez: geçerli bir kestirim yok).
 *   - 0x301: CRC doğru ve alive counter bir öncekinden farklıysa kabul edilir (tekrar reddedilir).
 *     Açılıştan sonraki ilk CAN_CMD_HOLDOFF_MS (1 s) içinde gelen komutlar reddedilir (ECU kapalıyken
 *     gönderilip EMU'da bekletilmiş bayat çerçeveler; bkz. bms_can.h).
 *       0x01 yeniden başlat  value = 0xA5A5 ve taze 0x101'de (< 250 ms) |I| < 1 A
 *       0x02 SOC ayarla      value = SOC [0.01 %] 0..10000 -> EKF SOC durumu (kovaryans aynı)
 *       0x03 sayaçları sil   CAN/uygulama hata sayaçları (TEC/REC donanımın, dokunulmaz)
 *       0x10-0x1F            EMU komutları: yok sayılır (sayılır)
 *     Her komut UART'a tek satır sonuç yazar: "cmd 0x02 value 5000: OK ..." / "REJECTED ...".
 *       0x05 yeni paket      value = 0xA5A5 ve |I| < 1 A: öğrenilmiş SOH kaydı silinir, %100'den başlar
 *   - SOH (docs/soh_design.md): kapasite SOH'u park-park arası öğrenilir (soh_step, bms_step'ten sonra;
 *     PIL/SIL zinciri değişmez). Kayıt flash'ta (bsp_nvm): açılışta okunur, EKF kapasitesi ve k_R oradan
 *     başlar; soh_step isteyince ve akım ~0 iken yazılır (sayfa silme ~22 ms). 0x202 / D satırı gösterilen
 *     SOH'u taşır (eskiden ML ağacı; ağaç stres endeksiyle birlikte yeniden ele alınacak).
 */
#include <math.h>
#include <string.h>
#include "app_ecu.h"
#include "bsp.h"
#include "bsp_can.h"
#include "bms_config.h"
#include "bms_algo.h"
#include "bms_bal.h"
#include "bsp_nvm.h"
#include "bms_can.h"
#ifndef BMS_SAFETY
#define BMS_SAFETY 0                         /* 1: IWDG + flash CRC + reset nedeni (adım 6) */
#endif
#if BMS_SAFETY
#include "bsp_safety.h"
#endif
#ifndef BMS_OUTPUTS
#define BMS_OUTPUTS 0                        /* 1: far PWM (PA8/PA9) + kontaktör (PB5) (adım 4) */
#endif
#if BMS_OUTPUTS
#include "bsp_out.h"
#define OUT_I_NOM_DIS   (7.275f * BMS_NP)    /* 87,3 A: veri sayfası 1,5C x Np (decision.py I_nom_dis) */
#define OUT_LOW_MIN     0.25f                /* kısa far alt sınırı */
#define OUT_HIGH_MIN    0.70f                /* I_allow/I_nom bunun altında: uzun far kapalı */
#endif
#ifndef BMS_TELEMETRY
#define BMS_TELEMETRY 0                      /* 1: her adımda gateway "D,..." satırı + "E,..." olaylar
                                                (tools/web/gateway.py); 2: ayrıca geniş "DX,..." satırı
                                                (tools/live_panel.py) */
#endif
#ifndef BMS_CANLOG
#define BMS_CANLOG 0                         /* 1: alınan/gönderilen her CAN çerçevesi "C,<t_ms>,<R|T>,<id>,<veri>"
                                                satırı (tools/can/can_log.py -> candump, SavvyCAN); ~3,2 kB/s */
#endif

#define SOC0_EKF  0.5f                     /* EKF başlangıç SOC'si (PIL ile aynı) */
#define ECU_STEP_LATE_MS  150u               /* ölçüm çifti bu kadar gecikirse tutulan değerle adım */
#define ECU_STEP_HOLD_MS  100u               /* tutulan değerle adım periyodu (BMS_TS) */
/* Açılışta ilk 50 ms'de gelen çerçeveler atılır: ECU sıfırlanırken hatta ACK veren olmadığından EMU
 * (MCP2515) son çerçevelerini tekrar gönderip bekletir; ECU açılınca bu BAYAT çerçeveler (sıfırlama
 * anındaki yük akımı) teslim edilir ve 1 s sıfır akım kontrolünü (A9) bozar: sürüşte sıfırlamada
 * 2/10 örnek 44 A -> ortalama 9,5 A -> sahte sensör LIMP'i + EKF'ye yanlış ofset (2026-11-20).
 * Taze çerçeve 100 ms'de bir gelir; 50 ms en çok bir taze çifti kaybettirir (ilk zaman aşımı 2 s). */
#define ECU_RX_FLUSH_MS   50u

enum { R_M1, R_M2, N_RX };                   /* zaman aşımı izlenen periyodik mesajlar */

static bms_t bms;
static can_monitor_t mon[N_RX];
static can_meas1_t m1;
static uint8_t have_m1;
static uint32_t n_hold, other, tx_busy, cyc_max, n_flush;
static uint32_t n_cmd, n_cmd_rej, n_cmd_emu, n_cmd_crc, n_cmd_dup;
static uint8_t boot_safe;                    /* BMS_SAFETY: watchdog reseti ya da flash CRC hatası */
/* adım 8: grup gerilimleri (0x110-0x114) ve pasif dengeleme (0x210) */
static can_monitor_t mon_cell[CAN_N_CELL_FRAMES];
static float   vcell[BMS_NS];
static uint8_t cell_valid[CAN_N_CELL_FRAMES], cell_fb[CAN_N_CELL_FRAMES];
static bal_t   bal;

/* Tüm hücre çerçeveleri taze (< 250 ms) ve 19 grubun hepsi geçerli mi */
static uint8_t cells_ok(void)
{
    for (uint32_t k = 0; k < CAN_N_CELL_FRAMES; k++) {
        const uint8_t need = (uint8_t)(k == CAN_N_CELL_FRAMES - 1u ? 0x7u : 0xFu);   /* 19 = 4 x 4 + 3 */
        if (!mon_cell[k].have_counter || mon_cell[k].fault || (cell_valid[k] & need) != need) return 0;
    }
    return 1;
}
/* EMU'nun geri bildirdiği açık anahtarlar (19 bit) */
static uint32_t cells_fb(void)
{
    uint32_t m = 0;
    for (uint32_t k = 0; k < CAN_N_CELL_FRAMES; k++) m |= (uint32_t)cell_fb[k] << (4u * k);
    return m & 0x7FFFFu;
}
static const char *boot_reason = "";
static soh_t sh;                             /* kapasite SOH'u (kalıcı kısmı sh.r, flash'ta) */
static uint32_t n_nvm_ok, n_nvm_fail;
_Static_assert(sizeof(soh_rec_t) == BSP_NVM_DATA, "soh_rec_t kayıt boyutu");

static void soh_save(void)
{
    if (bsp_nvm_save(&sh.r)) { n_nvm_ok++;  sh.persist = 0u;  sh.ah_saved = sh.r.ah_total;  sh.kr_saved = sh.r.k_r; }
    else n_nvm_fail++;                       /* persist kalır: bir sonraki dinlenmede yeniden dener */
}

/* CAN alım ara deposu: uzun bir iş (çıkış ölçümü, 20 ms) sürerken donanım FIFO'su (3 çerçeve)
 * buraya boşaltılır; ana döngü önce buradan, sonra donanımdan okur (sıra korunur). */
#define RXS_N 16u
static struct { uint16_t id; uint8_t len, d[8]; } rxs[RXS_N];
static uint32_t rxs_h, rxs_t, rxs_drop;
#if BMS_OUTPUTS
static void can_stash(void)
{
    uint16_t id;  uint8_t d[8], len;
    while (((rxs_h + 1u) % RXS_N) != rxs_t && bsp_can_recv(&id, d, &len)) {
        rxs[rxs_h].id = id;  rxs[rxs_h].len = len;  memcpy(rxs[rxs_h].d, d, 8);
        rxs_h = (rxs_h + 1u) % RXS_N;
    }
    if (((rxs_h + 1u) % RXS_N) == rxs_t) rxs_drop++;          /* dolu (olmamalı) */
}
#endif
static int can_get(uint16_t *id, uint8_t *d, uint8_t *len)
{
    if (rxs_t == rxs_h) return bsp_can_recv(id, d, len);
    *id = rxs[rxs_t].id;  *len = rxs[rxs_t].len;  memcpy(d, rxs[rxs_t].d, 8);
    rxs_t = (rxs_t + 1u) % RXS_N;
    return 1;
}

/* ---------------------------------------------------------------- metin çıktısı (printf yok) */
/* kesmeli (bloklamayan): rapor yazılırken CAN alımı durmasın */
static void put_s(const char *s) { (void)bsp_uart_write_async((const uint8_t *)s, (uint32_t)strlen(s)); }
static void put_u(uint32_t v)
{
    char b[11];
    int i = 10;
    b[i] = 0;
    do { b[--i] = (char)('0' + v % 10u); v /= 10u; } while (v);
    put_s(&b[i]);
}
static void put_i(int32_t v) { if (v < 0) { put_s("-"); v = -v; } put_u((uint32_t)v); }
static void put_x8(uint8_t v)
{
    const char h[] = "0123456789ABCDEF";
    char b[3] = { h[v >> 4], h[v & 15u], 0 };
    put_s(b);
}
static void kv(const char *k, uint32_t v) { put_s(k); put_u(v); }
#if BMS_CANLOG
/* Ham çerçeve kaydı: C,<t_ms>,<R|T>,<id 3 hex>,<len x 2 hex> */
static void canlog(uint32_t t_ms, char dir, uint16_t id, const uint8_t *d, uint8_t len)
{
    const char h[] = "0123456789ABCDEF";
    char b[48];
    uint32_t n = 0;
    b[n++] = ',';  b[n++] = dir;  b[n++] = ',';
    b[n++] = h[(id >> 8) & 15u];  b[n++] = h[(id >> 4) & 15u];  b[n++] = h[id & 15u];  b[n++] = ',';
    for (uint8_t i = 0; i < len && i < 8u; i++) { b[n++] = h[d[i] >> 4];  b[n++] = h[d[i] & 15u]; }
    b[n++] = '\r';  b[n++] = '\n';  b[n] = 0;
    put_s("C,");  put_u(t_ms);  put_s(b);
}
#define CANLOG(t, dir, id, d, len) canlog((t), (dir), (id), (d), (len))
#else
#define CANLOG(t, dir, id, d, len) ((void)0)
#endif
static void kf(const char *k, float v)       /* bir ondalık basamak */
{
    int32_t x = (int32_t)(v * 10.0f + (v < 0.0f ? -0.5f : 0.5f));
    put_s(k);
    if (x < 0) { put_s("-"); x = -x; }
    put_u((uint32_t)x / 10u); put_s("."); put_u((uint32_t)x % 10u);
}
#if BMS_TELEMETRY
static void put_fix(float v, uint32_t dec)   /* sabit ondalık (dec <= 4), virgülle başlar */
{
    static const float scale[] = { 1.0f, 10.0f, 100.0f, 1000.0f, 10000.0f };
    const float s = scale[dec];
    int32_t x = (int32_t)(v * s + (v < 0.0f ? -0.5f : 0.5f));
    put_s(",");
    if (x < 0) { put_s("-"); x = -x; }
    put_u((uint32_t)x / (uint32_t)s);
    if (dec) {
        uint32_t f = (uint32_t)x % (uint32_t)s, d = (uint32_t)s / 10u;
        put_s(".");
        while (d) { put_u(f / d); f %= d; d /= 10u; }
    }
}
#endif

static const char *state_name(uint8_t s)
{
    return s == ST_NORMAL ? "NORMAL" : s == ST_DERATE ? "DERATE" : s == ST_LIMP ? "LIMP" : "SAFE";
}

#if BMS_OUTPUTS
/* ---------------------------------------------------------------- araç çıkışları (adım 4)
 * Kısa far duty = I_allow_dis / I_nom (alt sınır %25); oran < %70 -> uzun far kapalı, değilse aynı duty.
 * Kontaktör: NORMAL / DERATE / LIMP-HOME'da kapalı (röle çekili), SAFE-STOP'ta açık; SAFE'te tüm
 * farlar sönük. Ölçüm gelmeden (açılış) her şey kapalı. CAN kesilirse izleyici SAFE'e geçirir.
 * Açılış sıfır akım kontrolü (A9, ilk 1 s): kontaktör açık (akım kesin 0), farlar sönük.
 * Gama: göz parlaklığı doğrusal algılamaz (%25 duty ile %100 arası az fark edilir); yukarıdaki oran
 * "algılanan parlaklık" seviyesidir, PWM duty = seviye^2,5 (%25 -> %3,1, %50 -> %17,7, %100 -> %100). */
static float out_low, out_high;              /* PWM duty (gama sonrası) */
static uint8_t out_k;
static float gamma_duty(float l) { return l * l * sqrtf(l); }      /* l^2,5 */
static void apply_outputs(uint8_t state, float i_allow_dis)
{
    if (state == ST_SAFE || state == 0u || bms.dec.n_zero < DEC_N_ZERO) {
        out_low = 0.0f;  out_high = 0.0f;  out_k = 0u;
    } else {
        float r = i_allow_dis / OUT_I_NOM_DIS;
        if (r > 1.0f) r = 1.0f;
        out_low  = gamma_duty(r < OUT_LOW_MIN ? OUT_LOW_MIN : r);
        out_high = r < OUT_HIGH_MIN ? 0.0f : gamma_duty(r);
        out_k = 1u;
    }
    bsp_out_set(out_low, out_high, out_k);
}
#endif

#if BMS_TELEMETRY
/* ---------------------------------------------------------------- olaylar (E satırı, gateway)
 * E,<t_ms>,0,<code>,<detail>: açılış, durum geçişi, neden bitinin yükselmesi, CAN kaybı, kontaktör.
 * Yalnız değişimde yazılır (10 Hz satırın yanında birkaç bayt). */
static const char *const ST_CODE[5] = { "-", "NORMAL", "DERATE", "LIMP_HOME", "SAFE_STOP" };
#define N_CAUSE 12
static const char *const CAUSE_CODE[N_CAUSE] = { "THERMAL_DERATE", "COOLING_ALARM", "SOP_DISCHARGE",
                                           "SOP_CHARGE", "DATASHEET_T_LIMIT", "SENSOR_FAULT", "SOC_EMPTY",
                                           "HARD_LIMIT", "THERMAL_ANOMALY", "TEMP_RISE_RATE",
                                           "RESISTANCE_GROWTH", "THERMAL_EVENT" };
static void ev(uint32_t t_ms, const char *code)
{
    put_s("E,");  put_u(t_ms);  put_s(",0,");  put_s(code);  put_s(",");
}
static void events(uint32_t t_ms, uint8_t state, uint16_t cause, float tc, uint8_t can_ok)
{
    static uint8_t first = 1, st_prev, can_prev = 1, k_prev;
    static uint16_t cause_prev;
    if (first) {
        ev(t_ms, "BOOT");  put_s(boot_reason[0] ? boot_reason + 1 : "power-on");  put_s("\r\n");
        st_prev = 0;  cause_prev = 0;  first = 0;
    }
    if (state != st_prev) {
        ev(t_ms, state <= 4u ? ST_CODE[state] : "?");
        put_s("from=");  put_s(st_prev <= 4u ? ST_CODE[st_prev] : "?");
        kv(" cause=", cause);  kf(" Tc=", tc);  put_s("\r\n");
    }
    const uint16_t up = (uint16_t)(cause & ~cause_prev);
    for (int b = 0; b < N_CAUSE; b++)
        if (b != 7 && (up & (1u << b))) {                   /* 7 sert sınır: SAFE_STOP olayı yeter */
            ev(t_ms, CAUSE_CODE[b]);  kf("Tc=", tc);  put_s("\r\n");
        }
    if (can_prev && !can_ok) { ev(t_ms, "CAN_TIMEOUT");  put_s("measurement frames lost\r\n"); }
#if BMS_OUTPUTS
    if (out_k != k_prev) { ev(t_ms, out_k ? "CONTACTOR_CLOSE" : "CONTACTOR_OPEN");  put_s("-\r\n"); }
    k_prev = out_k;
#else
    (void)k_prev;
#endif
    st_prev = state;  cause_prev = cause;  can_prev = can_ok;
}
#endif

/* ---------------------------------------------------------------- 0x301 komutları */
static void cmd_result(int ok, const char *msg)
{
    if (ok) put_s(": OK "); else { put_s(": REJECTED "); n_cmd_rej++; }
    put_s(msg);
    put_s("\r\n");
}

static void command(const can_cmd_t *q, uint32_t now_ms)
{
    put_s("cmd 0x"); put_x8(q->cmd); put_s(" value "); put_i(q->value);
    if (now_ms < CAN_CMD_HOLDOFF_MS) { cmd_result(0, "boot hold-off (stale frame?)"); return; }
    switch (q->cmd) {
    case CAN_CMD_ECU_RESTART: {
        /* Taze ölçüm şart: yeni açılmış ECU'ya, kapalıyken gönderilip EMU'da bekletilmiş eski bir
         * yeniden başlatma komutu gelirse (ölçüm yok) reddedilir; aksi hâlde art arda yeniden başlar. */
        const uint8_t fresh = have_m1 && (now_ms - mon[R_M1].last_ok_ms) < CAN_TIMEOUT_FAULT_MS;
        if (q->value != CAN_CMD_UNLOCK)                    cmd_result(0, "restart: wrong unlock value");
        else if (!fresh)                                   cmd_result(0, "restart: current unknown (no fresh 0x101)");
        else if (fabsf(m1.I) >= CAN_CMD_RESTART_I_A)       cmd_result(0, "restart: current flowing");
        else { cmd_result(1, "restarting"); bsp_uart_write((const uint8_t *)"", 0); bsp_reset(); }   /* kuyruğu boşalt */
        break;
    }
    case CAN_CMD_ECU_SET_SOC:
        if (q->value < 0 || q->value > 10000) { cmd_result(0, "set SOC: out of range 0..10000"); break; }
        bms.ekf.x[0] = (float)q->value * 1e-4f;
        soh_reset_anchor(&sh);                        /* dış müdahale: kapasite ölçüm zinciri kırılır */
        cmd_result(1, "EKF SOC set");
        break;
    case CAN_CMD_ECU_NEW_PACK: {
        const uint8_t fresh = have_m1 && (now_ms - mon[R_M1].last_ok_ms) < CAN_TIMEOUT_FAULT_MS;
        if (q->value != CAN_CMD_UNLOCK)                    cmd_result(0, "new pack: wrong unlock value");
        else if (!fresh || fabsf(m1.I) >= CAN_CMD_RESTART_I_A) cmd_result(0, "new pack: current flowing or unknown");
        else {
            const float b0 = sh.b0;
            const int ok = bsp_nvm_erase_all();
            soh_init(&sh, 0);
            sh.n_zero = DEC_N_ZERO;  sh.b0 = b0;      /* ofset bu açılışta zaten ölçüldü */
            bms_set_capacity(&bms, 1.0f);
            soh_save();
            cmd_result(ok, ok ? "SOH record erased, capacity 100 %" : "SOH record erase failed");
        }
        break;
    }
    case CAN_CMD_ECU_CLR_STATS:
        for (int m = 0; m < N_RX; m++) { mon[m].n_crc_err = 0; mon[m].n_cnt_err = 0; mon[m].n_lost = 0; }
        for (uint32_t k = 0; k < CAN_N_CELL_FRAMES; k++) {
            mon_cell[k].n_crc_err = 0; mon_cell[k].n_cnt_err = 0; mon_cell[k].n_lost = 0;
        }
        n_hold = 0; other = 0; tx_busy = 0; cyc_max = 0;
        n_cmd_rej = 0; n_cmd_emu = 0; n_cmd_crc = 0; n_cmd_dup = 0;
        bsp_can_clear_stats();
        cmd_result(1, "statistics cleared");
        break;
#if BMS_SAFETY
    case CAN_CMD_ECU_HANG:
        if (q->value != CAN_CMD_UNLOCK) { cmd_result(0, "hang: wrong unlock value"); break; }
        cmd_result(1, "hanging (watchdog test)");
        for (;;) { }                                  /* IWDG ~100 ms sonra sıfırlar */
#endif
    default:
        if (q->cmd >= 0x10u && q->cmd <= 0x1Fu) { n_cmd_emu++; put_s(": EMU command, ignored\r\n"); }
        else cmd_result(0, "unknown command");
        break;
    }
}

/* ---------------------------------------------------------------- uygulama */
void ecu_run(void)
{
    can_meas2_t m2 = { 0 };
    bms_out_t out;
    uint8_t have_m2 = 0, fresh_m1 = 0, fresh_m2 = 0, stepped = 0;
    uint8_t cnt_ctrl = 0, cnt_stat = 0, cnt_bal = 0, cmd_have = 0, cmd_cnt = 0;
    uint32_t now_ms = 0, t_last, deadline_ms = 0, tx_ok = 0;
    float soh = 1.0f;
#if BMS_SAFETY
    const bsp_reset_t rst = bsp_reset_cause();        /* bayraklar okunup temizlenir */
#endif

    for (int k = 0; k < 6; k++) { bsp_led_toggle(); bsp_delay_ms(150); }
    put_s("\r\n=== BMS ECU: CAN 0x101/0x102 -> bms_step -> 0x201/0x202, 0x301 commands, 500 kbit/s ===\r\n");
#if BMS_SAFETY
    {   /* adım 6: reset nedeni ve flash imajı CRC'si; watchdog reseti ya da CRC hatası -> SAFE-STOP */
        uint32_t crc, len;
        const uint32_t t0 = bsp_cycles();
        const bsp_crc_t c = bsp_flash_crc_check(&crc, &len);
        const uint32_t us = (bsp_cycles() - t0) / (BSP_CPU_HZ / 1000000u);
        put_s("safety: reset cause: ");  put_s(bsp_reset_name(rst));
        kv(" | flash image ", len);  put_s(" B, CRC 0x");
        for (int b = 3; b >= 0; b--) put_x8((uint8_t)(crc >> (8 * b)));
        put_s(": ");  put_s(bsp_crc_name(c));  kv(" (", us);  put_s(" us)");
        kv(" | IWDG ", BSP_IWDG_TIMEOUT_MS);  put_s(" ms\r\n");
        if (rst == RST_IWDG)  { boot_safe = 1;  boot_reason = " WDG-RESET";  put_s("WDG reset -> SAFE-STOP\r\n"); }
        if (c == CRC_MISMATCH || c == CRC_LEN_MISMATCH) {
            boot_safe = 1;  boot_reason = " FLASH-CRC";  put_s("flash CRC error -> SAFE-STOP\r\n");
        }
    }
#endif
    bms_init(&bms, SOC0_EKF, 1.0f, 1.0f, 0.0f, 0.0f);
    {   /* öğrenilmiş SOH kaydı (flash): EKF kapasitesi ve k_R oradan başlar */
        soh_rec_t rec;
        bsp_nvm_info_t ni;
        const int found = bsp_nvm_load(&rec, &ni);
        soh_init(&sh, found ? &rec : 0);
        bms_set_capacity(&bms, sh.r.soh_c);
        bms_set_kr(&bms, sh.r.k_r);
        put_s("soh: ");
        if (found) {
            kv("record #", ni.seq);  kf(" capacity ", 100.0f * sh.r.soh_c);  kf("% shown ", 100.0f * sh.r.soh_disp);
            kv("% meas ", sh.r.n_meas);  kf(" kR ", sh.r.k_r);  kf(" Ah ", sh.r.ah_total);
            put_s(sh.r.anchor ? " anchor yes" : " anchor no");
        } else {
            put_s("no record (new pack, 100 %)");
        }
        kv(" | slots ok ", ni.n_valid);  kv(" bad ", ni.n_bad);
        if (ni.ecc) put_s(" ECC!");
        if (!bsp_nvm_layout_ok()) put_s(" | IMAGE OVERLAPS NVM PAGES!");
        put_s("\r\n");
        soh = sh.r.soh_disp;
    }
    memset(&out, 0, sizeof(out));
    bsp_can_init_normal();
#if BMS_OUTPUTS
    bsp_out_init();                                   /* farlar sönük, kontaktör açık */
#endif
    for (int m = 0; m < N_RX; m++) can_monitor_init(&mon[m], 0);
    for (uint32_t k = 0; k < CAN_N_CELL_FRAMES; k++) can_monitor_init(&mon_cell[k], 0);
    bal_init(&bal);
    /* Zaman tabanı CAN açıldıktan SONRA alınır: önceden alınırsa döngü açılıştaki LED süresini
     * (0.9 s) birkaç ms'de "yetişir"; bu sırada gelen ilk çerçeve (EMU'nun ACK alamayıp beklettiği)
     * hemen ~900 ms yaşlanır ve izleyici açılışta SAFE'e kilitlenir. */
    t_last = bsp_cycles();
#if BMS_SAFETY
    bsp_iwdg_start();                                 /* ~100 ms; döngü her 1 ms besler */
#endif

    for (;;) {
        /* 1 ms'lik adımlarla zaman (DWT taşması 25 s'de bir; fark hesabı taşmaya dayanıklı) */
        while (bsp_cycles() - t_last < BSP_CPU_HZ / 1000u) { }
        t_last += BSP_CPU_HZ / 1000u;
        now_ms++;
#if BMS_SAFETY
        bsp_iwdg_kick();
#endif

        /* ---- her 1 ms: alım ---- */
        {
            uint16_t id;  uint8_t d[8], len, c;
            while (can_get(&id, d, &len)) {
                if (now_ms <= ECU_RX_FLUSH_MS) { n_flush++; continue; }   /* bayat açılış çerçeveleri */
                CANLOG(now_ms, 'R', id, d, len);
                if (len != 8) { other++; continue; }
                if (id == CAN_ID_EMU_MEAS1) {
                    can_meas1_t a;
                    const int ok = can_decode_meas1(d, &a, &c);
                    if (can_monitor_rx(&mon[R_M1], now_ms, ok, c)) { m1 = a;  have_m1 = 1;  fresh_m1 = 1; }
                } else if (id == CAN_ID_EMU_MEAS2) {
                    can_meas2_t b;
                    const int ok = can_decode_meas2(d, &b, &c);
                    if (can_monitor_rx(&mon[R_M2], now_ms, ok, c)) { m2 = b;  have_m2 = 1;  fresh_m2 = 1; }
                } else if (id >= CAN_ID_EMU_CELL0 && id < CAN_ID_EMU_CELL0 + CAN_N_CELL_FRAMES) {
                    const uint8_t k = (uint8_t)(id - CAN_ID_EMU_CELL0);
                    can_cell_t g;
                    const int ok = can_decode_cell(k, d, &g, &c);
                    if (can_monitor_rx(&mon_cell[k], now_ms, ok, c)) {
                        for (uint32_t j = 0; j < CAN_CELLS_PER_FRAME && 4u * k + j < BMS_NS; j++)
                            vcell[4u * k + j] = g.v[j];
                        cell_valid[k] = g.valid;  cell_fb[k] = g.bal;
                    }
                } else if (id == CAN_ID_PC_CMD) {           /* olay mesajı: CRC + tekrar kontrolü */
                    can_cmd_t q;
                    if (!can_decode_cmd(d, &q, &c))         n_cmd_crc++;
                    else if (cmd_have && c == cmd_cnt)      n_cmd_dup++;
                    else { cmd_have = 1;  cmd_cnt = c;  n_cmd++;  command(&q, now_ms); }
                } else {
                    other++;                                    /* filtre doğruysa olmamalı */
                }
            }
        }

        /* ---- her 10 ms: bus-off kurtarma ve izleyiciler ---- */
        if (now_ms % 10u == 0) {
            (void)bsp_can_service(now_ms);
            for (int m = 0; m < N_RX; m++) can_monitor_tick(&mon[m], now_ms);
            for (uint32_t k = 0; k < CAN_N_CELL_FRAMES; k++) can_monitor_tick(&mon_cell[k], now_ms);
        }

        /* ---- BMS adımı ve gönderim: yeni ölçüm çifti gelince (ya da gecikince tutulan değerle);
         *      ölçüm hiç gelmediyse 100 ms'de bir durum değerlendirmesi ---- */
        const uint8_t have = have_m1 && have_m2, fresh = fresh_m1 && fresh_m2;
        const uint8_t late = have && (int32_t)(now_ms - deadline_ms) >= 0;
        if (have ? (fresh || late) : (now_ms % 100u == 0)) {
            /* boot_safe (watchdog reseti / flash CRC) da CAN_ok gibi sert sınır: SAFE-STOP, kalıcı */
            const uint8_t can_ok = !(mon[R_M1].safe || mon[R_M2].safe) && !boot_safe;
            can_ctrl_t ctrl;
            uint8_t d[8], send_stat = 0, send_ctrl = 0;

            if (have) {
                if (!fresh) n_hold++;
                deadline_ms = now_ms + (fresh ? ECU_STEP_LATE_MS : ECU_STEP_HOLD_MS);
                fresh_m1 = 0;  fresh_m2 = 0;
                const bms_in_t in = { m1.I, m1.V, m1.T, m2.V_min, m2.V_max, m2.T_amb, can_ok };
                const uint32_t t0 = bsp_cycles();
                bms_step(&bms, &in, &out);
                const uint32_t cyc = bsp_cycles() - t0;
                if (cyc > cyc_max) cyc_max = cyc;
                soh_step(&sh, &in, &out);
                if (sh.event == SOH_EV_ACCEPT) bms_set_capacity(&bms, sh.r.soh_c);
                if (sh.persist && fabsf(m1.I - sh.b0) < CAN_CMD_RESTART_I_A) soh_save();   /* yalnız akım ~0 */
                soh = sh.r.soh_disp;
                if (sh.event != SOH_EV_NONE) {        /* dinlenme noktası: tek satır (UART, seyrek) */
                    static const char *const EVN[] = { "", "anchor", "ACCEPTED", "small dSOC", "out of range",
                                                       "invalid point (T / flat OCV)" };
                    kf("soh: rest point SOC ", 100.0f * sh.last_soc);  put_s(" % -> ");
                    put_s(EVN[sh.event]);
                    if (sh.event == SOH_EV_ACCEPT || sh.event == SOH_EV_RANGE) { kf(" ratio ", 100.0f * sh.last_ratio); put_s("%"); }
                    kf(" | capacity ", 100.0f * sh.r.soh_c);  kf("% shown ", 100.0f * sh.r.soh_disp);
                    kv("% meas ", sh.r.n_meas);  kv(" | flash ok ", n_nvm_ok);  kv(" fail ", n_nvm_fail);
                    put_s("\r\n");
                }
                stepped = 1;
                ctrl.I_allow_dis = out.dec.i_allow_dis;  ctrl.I_allow_chg = out.dec.i_allow_chg;
                ctrl.state = out.dec.state;              ctrl.cause = (uint8_t)out.dec.cause;
                send_ctrl = 1;  send_stat = 1;
            } else if (!can_ok) {                    /* açılışta ölçüm hiç gelmedi */
                ctrl.I_allow_dis = 0.0f;  ctrl.I_allow_chg = 0.0f;
                ctrl.state = ST_SAFE;     ctrl.cause = 128;
                out.dec.state = ST_SAFE;  out.dec.cause = 128;
                send_ctrl = 1;
            }

#if BMS_OUTPUTS
            if (send_ctrl) apply_outputs(ctrl.state, ctrl.I_allow_dis);
#endif
            if (send_ctrl) {
                can_encode_ctrl(&ctrl, cnt_ctrl, d);
                if (bsp_can_send(CAN_ID_ECU_CTRL, d, 8)) { tx_ok++; cnt_ctrl = (cnt_ctrl + 1u) & 0x0Fu;
                                                           CANLOG(now_ms, 'T', CAN_ID_ECU_CTRL, d, 8); }
                else tx_busy++;
            }
            if (send_stat) {
                const can_status_t st = { 100.0f * out.soc, out.tc, 100.0f * soh, out.dec.psi };
                can_encode_status(&st, cnt_stat, d);
                if (bsp_can_send(CAN_ID_ECU_STATUS, d, 8)) { tx_ok++; cnt_stat = (cnt_stat + 1u) & 0x0Fu;
                                                             CANLOG(now_ms, 'T', CAN_ID_ECU_STATUS, d, 8); }
                else tx_busy++;
                /* adım 8: dengeleme kararı (bms_step'ten bağımsız; PIL/SIL zincirini değiştirmez) */
                const uint8_t v_ok = cells_ok();
                const can_bal_t bm = { bal_step(&bal, vcell, v_ok, m1.I, out.dec.state, out.tc),
                                       bal.flags, v_ok ? bal.v_max - bal.v_min : 0.0f };
                can_encode_bal(&bm, cnt_bal, d);
                if (bsp_can_send(CAN_ID_ECU_BAL, d, 8)) { tx_ok++; cnt_bal = (cnt_bal + 1u) & 0x0Fu;
                                                          CANLOG(now_ms, 'T', CAN_ID_ECU_BAL, d, 8); }
                else tx_busy++;
            }
#if BMS_TELEMETRY
            /* gateway satırı (tools/web/gateway.py, sunucu şeması): canlı modda run = 0; T = çekirdek
             * kestirimi; durum 1 NORMAL, 2 DERATE, 3 LIMP-HOME, 4 SAFE-STOP */
            if (send_ctrl) {
                bsp_can_err_t ce;
                bsp_can_errors(&ce);
                put_s("D,");  put_u(now_ms);  put_s(",0");
                put_fix(100.0f * out.soc, 1);  put_fix(100.0f * soh, 1);  put_fix(out.tc, 1);
                put_fix(out.dec.i_allow_dis, 1);
                put_s(",");  put_u(out.dec.state);  put_s(",");  put_u(ce.tec);  put_s(",");  put_u(ce.rec);
                put_s("\r\n");
                events(now_ms, out.dec.state, out.dec.cause, out.tc, can_ok);
            }
#endif
#if BMS_TELEMETRY >= 2
            /* adım 9: her adımda tek satır (~90 karakter, ~8 ms; ölçüm çifti az önce işlendiği için
             * CAN Rx FIFO'su boş). DX,ms,durum,neden,SOC%,Tc,Ts,Idis,Ichg,psi,SOH%,I,V,T,Vmin,Vmax,
             * Psi_T,Psi_C,Psi_Tdot,Psi_H,Psi,baskın,dT/dt[°C/dk],k_R,<6 SOH alanı>,Psi_A,Psi_B,Psi_P,R_cyc,R_cal,
             * hasar (36 alan). psi = toplam kısıtlama
             * 1 - I_allow/I_nom (0x202), Psi = çok kriterli stres indeksi (CAN'da yok). Yeni alanlar
             * sonda: eski çözücüler ilk 16 alanı okumaya devam eder (~140 karakter, ~12 ms). */
            if (send_ctrl) {
                put_s("DX,");  put_u(now_ms);  put_s(",");  put_u(out.dec.state);  put_s(",");  put_u(out.dec.cause);
                put_fix(100.0f * out.soc, 2);  put_fix(out.tc, 2);  put_fix(out.ts, 2);
                put_fix(out.dec.i_allow_dis, 1);  put_fix(out.dec.i_allow_chg, 1);  put_fix(out.dec.psi, 3);
                put_fix(100.0f * soh, 1);
                put_fix(m1.I, 1);  put_fix(m1.V, 1);  put_fix(m1.T, 1);  put_fix(m2.V_min, 3);  put_fix(m2.V_max, 3);
                for (int i = 0; i < PSI_A; i++) put_fix(out.dec.psi_i[i], 3);   /* T, C, Tdot, H (eski sıra) */
                put_fix(out.dec.psi_s, 3);  put_s(",");  put_u(out.dec.dom);
                put_fix(out.dec.tdot, 2);  put_fix(out.kr, 3);
                /* kapasite SOH'u (2026-11-25): öğrenilen %, ölçüm sayısı, eşdeğer tam çevrim, çapa (0/1),
                 * dinlenme süresi [s], son ölçüm oranı % */
                put_fix(100.0f * sh.r.soh_c, 2);  put_s(",");  put_u(sh.r.n_meas);
                put_fix(sh.r.ah_total / (2.0f * SOH_Q_NOM), 2);  put_s(",");  put_u(sh.r.anchor);
                put_s(",");  put_u((uint32_t)sh.n_rest / 10u);  put_fix(100.0f * sh.last_ratio, 2);
                /* stres indeksi yeniden tasarımı (2026-11-26): Ψ_A yaşlanma hızı, Ψ_B dengesizlik, Ψ_P (deşarjı
                 * kısan koruma payı), çevrim / takvim hızı oranı, birikimli hasar (referans EFC). Psi (alan 20)
                 * artık altı kriterin olasılıksal VEYA'sı */
                put_fix(out.dec.psi_i[PSI_A], 3);  put_fix(out.dec.psi_i[PSI_B], 3);  put_fix(out.dec.psi_p, 3);
                put_fix(out.dec.r_cyc, 2);  put_fix(out.dec.r_cal, 2);  put_fix(out.dec.dmg, 4);
                put_s("\r\n");
            }
#endif
        }

        /* ---- her 1 s: rapor ---- */
        if (now_ms % 1000u == 0) {
            bsp_can_err_t e;
            bsp_can_errors(&e);
            kv("t=", now_ms / 1000u); put_s("s");  put_s(boot_reason);  put_s(" ");
            if (stepped) {
                put_s(state_name(out.dec.state));
                kf(" SOC ", 100.0f * out.soc);        kf("% Idis ", out.dec.i_allow_dis);
                kf(" Ichg ", out.dec.i_allow_chg);    kf(" Tc ", out.tc);
                kf(" SOH ", 100.0f * soh);            kv(" (n", sh.r.n_meas);
                kv(" rest ", (uint32_t)sh.n_rest / 10u);  put_s("s)");
                kv(" cause ", out.dec.cause);
                kv(" Tv", out.t_valid);               kv(" Iv", out.i_valid);
                kv(" | step ", cyc_max / (BSP_CPU_HZ / 1000000u)); put_s("us");
            } else {
                put_s(out.dec.state == ST_SAFE ? "SAFE (no measurements)" : "WAIT (no measurements yet)");
            }
            for (int m = 0; m < N_RX; m++) {
                put_s(m == R_M1 ? " | 101:" : " 102:");
                put_u(mon[m].n_ok);
                if (mon[m].n_crc_err || mon[m].n_cnt_err || mon[m].n_lost) {
                    kv("(crc", mon[m].n_crc_err); kv(" cnt", mon[m].n_cnt_err); kv(" lost", mon[m].n_lost);
                    put_s(")");
                }
                if (mon[m].safe) put_s("!SAFE"); else if (mon[m].fault) put_s("!TO");
            }
            kv(" hold:", n_hold); kv(" other:", other);  if (n_flush) kv(" flush:", n_flush);
            kv(" | cmd:", n_cmd);
            if (n_cmd_rej || n_cmd_emu || n_cmd_crc || n_cmd_dup) {
                kv("(rej", n_cmd_rej); kv(" emu", n_cmd_emu); kv(" crc", n_cmd_crc); kv(" dup", n_cmd_dup);
                put_s(")");
            }
            {   /* adım 8: hücre çerçeveleri ve dengeleme */
                uint32_t n = 0, e8 = 0;
                for (uint32_t k = 0; k < CAN_N_CELL_FRAMES; k++) {
                    n += mon_cell[k].n_ok;
                    e8 += mon_cell[k].n_crc_err + mon_cell[k].n_cnt_err + mon_cell[k].n_lost;
                }
                kv(" | cell:", n);
                if (e8) { kv("(err ", e8); put_s(")"); }
                const uint32_t fb = cells_fb();
                put_s(" bal 0x"); put_x8((uint8_t)(bal.mask >> 16)); put_x8((uint8_t)(bal.mask >> 8)); put_x8((uint8_t)bal.mask);
                put_s(" fb 0x");  put_x8((uint8_t)(fb >> 16));       put_x8((uint8_t)(fb >> 8));       put_x8((uint8_t)fb);
                kv(" f", bal.flags);
                kf(" dV ", 1000.0f * (bal.v_max - bal.v_min));  put_s("mV");
                kf(" Vmin ", 1000.0f * bal.v_min);
            }
            kv(" | tx ok:", tx_ok); kv(" busy:", tx_busy);
            kv(" | TEC ", e.tec); kv(" REC ", e.rec); kv(" LEC ", e.lec);
            put_s(e.bo ? " BUS-OFF" : e.ep ? " PASSIVE" : e.ew ? " WARN" : " ACTIVE");
            kv(" bo#", e.n_busoff); kv(" lost#", e.n_rx_lost);
            if (bsp_uart_tx_drops()) kv(" uart_drop ", bsp_uart_tx_drops());
            if (rxs_drop) kv(" rxs_drop ", rxs_drop);
#if BMS_OUTPUTS
            {   /* pin seviyesinde doğrulama (20 ms örnekleme): set edilen ve ölçülen */
                bsp_out_meas_t om;
                bsp_out_measure(20000u, &om, can_stash);
                kf(" | OUT low set ", 100.0f * out_low);  kf("% meas ", 100.0f * om.d_low);
                kv("% ", om.f_low_hz);  kf("Hz | high set ", 100.0f * out_high);
                kf("% meas ", 100.0f * om.d_high);  kv("% ", om.f_high_hz);
                kv("Hz | K ", om.contactor);  put_s(om.contactor ? " closed" : " open");
                kv(" pin ", om.k_pin);                    /* PB5 gerçek seviye (IDR) */
            }
#endif
            put_s("\r\n");
            bsp_led_toggle();
        }
    }
}
