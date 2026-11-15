/* app_ecu.c -- BMS ECU uygulaması: gerçek CAN hattında tam zincir (PA11 RX / PA12 TX + TJA1050).
 *
 *   0x101 (V, I, T) + 0x102 (V_min, V_max, T_amb)  ->  bms_step  ->  0x201 (I_allow, durum, neden)
 *                                                                    0x202 (SOC, Tc, SOH, psi)
 *
 *   her 1 ms  : alınan çerçeveleri çöz (CRC + sayaç); yeni ölçüm çifti geldiyse bir bms_step,
 *               0x201 ve 0x202 gönder
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
 *     karar mekanizması kilitler); çıkış için ECU yeniden başlatılır.
 *   - Açılış: iki mesajdan da ilk geçerli ölçüm gelene kadar bms_step koşmaz ve ECU hiçbir şey
 *     göndermez. CAN_TIMEOUT_FIRST_MS içinde ölçüm gelmezse yalnız 0x201 gönderilir:
 *     SAFE, I_allow = 0, neden 128 (0x202 gönderilmez: geçerli bir kestirim yok).
 *   - 0x301 (PC komutu) çözülür ve sayılır; komutların anlamı henüz uygulanmadı.
 *   - SOH: soh_tree'nin efc ve days girdileri ECU'da henüz tutulmuyor (kalıcı bellek yok),
 *     0 verilir; T = Tc_hat, m = anlık SOC, rr = direnç kestiricisi. Bu yüzden SOH yaklaşık.
 */
#include <string.h>
#include "app_ecu.h"
#include "bsp.h"
#include "bsp_can.h"
#include "bms_config.h"
#include "bms_algo.h"
#include "bms_can.h"

#define SOC0_EKF  0.5f                       /* EKF başlangıç SOC'si (PIL ile aynı) */
#define ECU_STEP_LATE_MS  150u               /* ölçüm çifti bu kadar gecikirse tutulan değerle adım */
#define ECU_STEP_HOLD_MS  100u               /* tutulan değerle adım periyodu (BMS_TS) */

enum { R_M1, R_M2, R_CMD, N_RX };
#define N_PERIODIC  2                        /* R_M1, R_M2: zaman aşımı izlenenler */

static bms_t bms;

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
static void kv(const char *k, uint32_t v) { put_s(k); put_u(v); }
static void kf(const char *k, float v)       /* bir ondalık basamak */
{
    int32_t x = (int32_t)(v * 10.0f + (v < 0.0f ? -0.5f : 0.5f));
    put_s(k);
    if (x < 0) { put_s("-"); x = -x; }
    put_u((uint32_t)x / 10u); put_s("."); put_u((uint32_t)x % 10u);
}
static const char *state_name(uint8_t s)
{
    return s == ST_NORMAL ? "NORMAL" : s == ST_DERATE ? "DERATE" : s == ST_LIMP ? "LIMP" : "SAFE";
}

/* ---------------------------------------------------------------- uygulama */
void ecu_run(void)
{
    static can_monitor_t mon[N_RX];
    can_meas1_t m1 = { 0 };
    can_meas2_t m2 = { 0 };
    bms_out_t out;
    uint8_t have_m1 = 0, have_m2 = 0, fresh_m1 = 0, fresh_m2 = 0, stepped = 0;
    uint8_t cnt_ctrl = 0, cnt_stat = 0;
    uint32_t now_ms = 0, t_last = bsp_cycles(), deadline_ms = 0, n_hold = 0;
    uint32_t tx_ok = 0, tx_busy = 0, other = 0, n_cmd = 0, cyc_max = 0;
    float soh = 1.0f;

    for (int k = 0; k < 6; k++) { bsp_led_toggle(); bsp_delay_ms(150); }
    put_s("\r\n=== BMS ECU: CAN 0x101/0x102 -> bms_step -> 0x201/0x202, 500 kbit/s ===\r\n");
    bms_init(&bms, SOC0_EKF, 1.0f, 1.0f, 0.0f, 0.0f);
    memset(&out, 0, sizeof(out));
    bsp_can_init_normal();
    for (int m = 0; m < N_RX; m++) can_monitor_init(&mon[m], 0);

    for (;;) {
        /* 1 ms'lik adımlarla zaman (DWT taşması 25 s'de bir; fark hesabı taşmaya dayanıklı) */
        while (bsp_cycles() - t_last < BSP_CPU_HZ / 1000u) { }
        t_last += BSP_CPU_HZ / 1000u;
        now_ms++;

        /* ---- her 1 ms: alım ---- */
        {
            uint16_t id;  uint8_t d[8], len, c;
            while (bsp_can_recv(&id, d, &len)) {
                if (len != 8) { other++; continue; }
                if (id == CAN_ID_EMU_MEAS1) {
                    can_meas1_t a;
                    const int ok = can_decode_meas1(d, &a, &c);
                    if (can_monitor_rx(&mon[R_M1], now_ms, ok, c)) { m1 = a;  have_m1 = 1;  fresh_m1 = 1; }
                } else if (id == CAN_ID_EMU_MEAS2) {
                    can_meas2_t b;
                    const int ok = can_decode_meas2(d, &b, &c);
                    if (can_monitor_rx(&mon[R_M2], now_ms, ok, c)) { m2 = b;  have_m2 = 1;  fresh_m2 = 1; }
                } else if (id == CAN_ID_PC_CMD) {
                    can_cmd_t q;
                    const int ok = can_decode_cmd(d, &q, &c);
                    if (can_monitor_rx(&mon[R_CMD], now_ms, ok, c)) n_cmd++;
                } else {
                    other++;                                    /* filtre doğruysa olmamalı */
                }
            }
        }

        /* ---- her 10 ms: bus-off kurtarma ve izleyiciler ---- */
        if (now_ms % 10u == 0) {
            (void)bsp_can_service(now_ms);
            for (int m = 0; m < N_PERIODIC; m++) can_monitor_tick(&mon[m], now_ms);
        }

        /* ---- BMS adımı ve gönderim: yeni ölçüm çifti gelince (ya da gecikince tutulan değerle);
         *      ölçüm hiç gelmediyse 100 ms'de bir durum değerlendirmesi ---- */
        const uint8_t have = have_m1 && have_m2, fresh = fresh_m1 && fresh_m2;
        const uint8_t late = have && (int32_t)(now_ms - deadline_ms) >= 0;
        if (have ? (fresh || late) : (now_ms % 100u == 0)) {
            const uint8_t can_ok = !(mon[R_M1].safe || mon[R_M2].safe);
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
                soh = soh_tree(0.0f, 0.0f, out.tc, out.soc, 0.0f, out.rr);
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

            if (send_ctrl) {
                can_encode_ctrl(&ctrl, cnt_ctrl, d);
                if (bsp_can_send(CAN_ID_ECU_CTRL, d, 8)) { tx_ok++; cnt_ctrl = (cnt_ctrl + 1u) & 0x0Fu; }
                else tx_busy++;
            }
            if (send_stat) {
                const can_status_t st = { 100.0f * out.soc, out.tc, 100.0f * soh, out.dec.psi };
                can_encode_status(&st, cnt_stat, d);
                if (bsp_can_send(CAN_ID_ECU_STATUS, d, 8)) { tx_ok++; cnt_stat = (cnt_stat + 1u) & 0x0Fu; }
                else tx_busy++;
            }
        }

        /* ---- her 1 s: rapor ---- */
        if (now_ms % 1000u == 0) {
            bsp_can_err_t e;
            bsp_can_errors(&e);
            kv("t=", now_ms / 1000u); put_s("s ");
            if (stepped) {
                put_s(state_name(out.dec.state));
                kf(" SOC ", 100.0f * out.soc);        kf("% Idis ", out.dec.i_allow_dis);
                kf(" Ichg ", out.dec.i_allow_chg);    kf(" Tc ", out.tc);
                kf(" SOH ", 100.0f * soh);            kv(" cause ", out.dec.cause);
                kv(" Tv", out.t_valid);               kv(" Iv", out.i_valid);
                kv(" | step ", cyc_max / (BSP_CPU_HZ / 1000000u)); put_s("us");
            } else {
                put_s(out.dec.state == ST_SAFE ? "SAFE (no measurements)" : "WAIT (no measurements yet)");
            }
            for (int m = 0; m < N_PERIODIC; m++) {
                put_s(m == R_M1 ? " | 101:" : " 102:");
                put_u(mon[m].n_ok);
                if (mon[m].n_crc_err || mon[m].n_cnt_err || mon[m].n_lost) {
                    kv("(crc", mon[m].n_crc_err); kv(" cnt", mon[m].n_cnt_err); kv(" lost", mon[m].n_lost);
                    put_s(")");
                }
                if (mon[m].safe) put_s("!SAFE"); else if (mon[m].fault) put_s("!TO");
            }
            kv(" hold:", n_hold); kv(" cmd:", n_cmd); kv(" other:", other);
            kv(" | tx ok:", tx_ok); kv(" busy:", tx_busy);
            kv(" | TEC ", e.tec); kv(" REC ", e.rec); kv(" LEC ", e.lec);
            put_s(e.bo ? " BUS-OFF" : e.ep ? " PASSIVE" : e.ew ? " WARN" : " ACTIVE");
            kv(" bo#", e.n_busoff); kv(" lost#", e.n_rx_lost);
            put_s("\r\n");
            bsp_led_toggle();
        }
    }
}
