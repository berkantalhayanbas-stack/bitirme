/* twin_emu.ino -- Arduino Uno + MCP2515: paket dijital ikizi emülatörü (HIL adım 3, "EMU" düğümü).
 *
 * mcp2515_node'un sabit değerli taklidi yerine C:\bitirme\twin ikizinin C portunu (twin_model.c)
 * 10 Hz çalıştırır: araç modeli -> akım talebi -> ECU izni (0x201 I_allow) ile sınırla -> ikiz ->
 * gürültü ekle -> 0x101 / 0x102. Gürültü twin/closed_loop.py ile aynı: V sigma 0,25 V, I +0,5 A
 * sapma + sigma 0,5 A, T sigma 0,3 °C, V_min/V_max sigma 5 mV. 0x101'deki T = yüzey sıcaklığı.
 *
 * Kütüphaneler : "mcp2515" (autowp), LiquidCrystal_I2C (Library Manager)
 * Bağlantı     : MCP2515 CS = D10, INT = D2, SPI D11/D12/D13; 500 kbit/s, kristal 8 MHz
 *                (TW_SW_SPI = 1, sw_spi_cfg.h: MOSI D9, yazılımla SPI; tezgâh Uno'sunun D11'i hasarlı)
 *                A0 arıza şiddeti potu (yoksa varsayılan şiddet; aşağıda "Şiddet")
 *                A1 LM35 çıkışı (ortam sıcaklığı, 10 mV/°C)       yoksa: 25 °C
 *                5 senaryo butonu D3, A2, A3, D4, D5 (GND'ye basar, dahili pull-up; aşağıda)
 *                16x2 I2C LCD (PCF8574, adres 0x27 ya da 0x3F) SDA A4, SCL A5   yoksa: yalnız seri
 * Algılama     : pot açılışta (A0'da pull-up açıkken < 1000 okunursa var), LM35 her saniye
 *                (kısa pull-up darbesinden sonra makul aralıkta okunursa var), LCD açılışta I2C ACK.
 *
 * Sürüş modları ("m <n>"; açılışta 0):
 *   0 PROF  twin/drive_cycle.py sentetik çevrimi (hız izleyen sürücü), sürekli tekrar
 *   1 PEDL  pedal = çekiş kuvveti (0..1200 N; seri "p"). Moda girince pedal bir kez
 *           bırakılmalı (< %5).
 *   2 SARJ  araç durur, 0,7C (40,7 A) şarj talebi; ECU şarj izni sınırlar (CV bölgesi ECU'da)
 *   3 BEKL  araç durur ve park eder: 0 A (yardımcı yük de kapalı; dengeleme bu modda ve şarjda çalışır)
 * Her durumda paket akımı = kırp(talep, -I_allow_chg, +I_allow_dis); 0x201 zaman aşımında 0 A.
 *
 * Hücre grupları (HIL adım 8): ikiz 19 grubu ayrı simüle eder (twin_model.c tablosu: saçılma, zayıf
 *   grup 7, başlangıç SOC farkı). Her 100 ms 0x110-0x114 ile 19 grup gerilimi (AFE gürültüsü sigma
 *   1 mV) ve dengeleme anahtarı geri bildirimi gider; 0x102'deki V_min/V_max gerçek en düşük / en
 *   yüksek gruptur. ECU'nun 0x210 maskesi taze ise (< 250 ms) grupların 10 ohm direnci açılır
 *   (I = V / 10 ohm ~ 0,41 A). Gerçek hızda %1 SOC farkı ~1,4 saatte kapanır; gösterim için
 *   "b <k>" direncin SOC etkisini k katına çıkarır (zaman hızlandırma; gerilim düşümü gerçek akımla
 *   kalır). HIZLANDIRILMIŞ olduğu raporda ve LCD'de görünür.
 * Mod butonları: her butonun üstünde ECU'nun varacağı mod yazar, altında bir SEBEP çalışır; modu
 * ECU kendi ölçüm ve teşhisinden seçer (ECU senaryoyu bilmez, CAN çerçeveleri aynı). Kısa basış
 * (< 1 s, bırakınca) hızlı gösterim; uzun basış (>= 1 s) aynı modun "gizli arıza" senaryosu (tezin
 * HIL bulguları). Tekrar basış o arızayı kaldırır (ECU'yu yeniden başlatmaz). LCD 3 s "sebep>mod".
 *   D3  NORMAL   servis: tüm arızaları kaldır, paketi 25-35 °C'ye getir (ortam + 2; ön koşullandırma), DERATE
 *                şarjından önceki SOC'ye dön, park (I = 0); rampa bitip 2 s oturunca (ECU ısınmayı
 *                yeniden başladıktan sonra görmesin: termal anomali / Psi_Tdot) ECU'ya 0x301 cmd 0x01
 *                (CAN_CMD_UNLOCK) 0,4 s arayla en çok 3 kez (ECU |I| < 1 A ister). Yeniden başlama
 *                0x201 boşluğundan (>= 150 ms) görülür; görülmezse tur en çok 3 kez yinelenir. Sonra
 *                1,5 s bekleyip sürüş çevrimi. ECU NORMAL'e ~11 s'de (1 s sıfır akım + rampa).
 *   A2  DERATE   kısa: dolu bataryaya hızlı şarj (SOC %88, 0,7C) -> hücre gerilimi üst sınıra yaklaşır
 *                -> SOP şarj payı (cause 8) -> DERATE (< 1 s). Tekrar: şarj biter, eski SOC, sürüş.
 *                uzun: soğutma arızası (R_u x4, twin cooling_fault). ECU'ya söylenmez; ECU R_u
 *                kestiriminden bulur (açılıştan >= 120 s sonra) -> soğutma alarmı, izin %70.
 *   A3  LIMP     sıcaklık sensörü kopuk: 0x101 T = -40 °C -> ECU sensör teşhisi -> LIMP (%25).
 *   D4  STOP     kısa: ölçüm hattı kopuk: 0x101/0x102/hücre çerçeveleri gitmez -> ECU 0,5 s'de SAFE.
 *                uzun: aşırı ısınma (dış ısı / termal olay): paket 0,5 °C/s ile 62 °C'ye -> termal
 *                anomali LIMP -> 60 °C SAFE (kilitli). Tekrar uzun basış: ortama soğuma (aynı hız).
 *   D5  WATCHDOG ECU'ya 0x301 cmd 0x04 (CAN_CMD_UNLOCK): ECU sonsuz döngüye girer, IWDG sıfırlar,
 *                ECU SAFE-STOP'ta açılır (yalnız BMS_SAFETY=1 ECU derlemesinde).
 *   SAFE ve LIMP kilitlidir: çıkış NORMAL butonuyla (servis yeniden başlatması).
 * Şiddet (A0 potu ya da seri "v"): uzun basış arızalarının büyüklüğü, arıza sürerken değiştirilebilir;
 *   soğutma arızası R_u x(1 + 7 s), aşırı ısınma hızı 1,17 s °C/s (s = 0..1). Pot yoksa s = 3/7:
 *   R_u x4 ve 0,5 °C/s (önceki sabit değerler). Arıza sürerken şiddet değişince LCD 3 s gösterir.
 * Yaşlanma (seri "y <SOH %>"): ikizin kapasitesi SOH katı, direnci 1 + 1,31 (1 - SOH) katı
 *   (twin_model.h TW_AGE_BETA, Kirkaldy 40 hücre). ECU'ya söylenmez; EKF k_R'den bulur (Ψ_H).
 *   D3 basılı açılış  sıcak gün: ortam = LM35 + 15 °C (LM35 yoksa 40 °C; twin hot_day).
 *   LM35 kopması açılışta LM35 vardıysa ve sonra kaybolursa sıcaklık sensörü kopuk sayılır:
 *                0x101 T = -40 °C (cmd 0x14 ile aynı) -> ECU sensör teşhisi -> LIMP. Takılınca kalkar.
 *                Ortam son değerde tutulur. Açılışta LM35 yoksa eskisi gibi 25 °C (hata yok).
 * Zamanlama (100 ms döngü, adım ~18 ms): adım + 0x101/0x102 -> ECU cevabı (~1 ms) -> 10 ms'den
 *   itibaren 5 ms arayla 5 hücre çerçevesi; sonra 35-55 ms penceresinde TEK iş: rapor (~17 ms) ya
 *   da bir LCD satırı (~30 ms). Böylece MCP2515'in 2 alım tamponu ECU'nun 3 çerçevesi gelirken
 *   boşaltılır, hücre çerçeveleri hiçbir turda atlanmaz ve adım zamanında başlar.
 *
 * Seri (115200, satır sonu Newline):
 *   c <cmd> <value>  0x301 komutu (mcp2515_node ile aynı; 0x10-0x1F EMU hataları burada da uygulanır,
 *                    hücre çerçevelerine de)
 *   m <0..3>         sürüş modu       s <yüzde>  ikiz SOC'yi ayarla (ör. s 80; grup farkları korunur)
 *   a <°C>           ortam sıcaklığını zorla (a 99 = kapat, LM35 / 25 °C'ye dön)
 *   b <k>            dengeleme akımı çarpanı (1 = gerçek 0,41 A; 1..2000)
 *   f <0|1>          soğutma arızası        i <0|1>  aşırı ısınma         h <0|1>  sıcak gün (+15 °C)
 *   k <1-5>          butona kısa basış (1 NORMAL, 2 DERATE, 3 LIMP, 4 STOP, 5 WATCHDOG; demo_run.py)
 *   l <1-5>          butona uzun basış (2 soğutma arızası, 4 aşırı ısınma; diğerleri = kısa)
 *   p <0..100>       pedal yüzdesi (PEDL moduna geçer)
 *   v <0..100>       arıza şiddeti yüzdesi (pot yerine; v -1 = pota / varsayılana dön)
 *   y <50..100>      ikizin SOH'si (yaşlanma; 100 = yeni)
 *   t <°C>           paket sıcaklığını hedefe getir (çekirdek = yüzey), örnek başına en çok 0,5 °C:
 *                    ani sıçrama ECU'nun sıcaklık hızı teşhisine (2 °C/örnek) takılıp LIMP verirdi
 * Her basışta (buton ya da k/l) bir satır: B,<olay>,<1 başladı | 0 kaldırıldı>; olay = EV_* (0 NORMAL
 *   servis, 1 dolu+şarj, 2 sensör kopuk, 3 hat kopuk, 4 watchdog, 5 soğutma arızası, 6 aşırı ısınma).
 *   tools/web/gateway.py --emu bunu panele "BUTTON" olayı olarak gönderir ve ECU'nun tepkisini ölçer.
 * Her 1 s bir rapor satırı (tools/twin_emu_run.py bunu özetler):
 *   R,t_s,mode,v_kmh,I,V,soc%,Tc,Ts,Ta,ta_src,ecu_state,Idis,Ichg,cause,soc_ecu%,tc_ecu,ok201,ok202,err,
 *     TEC,REC,bal_mask,bal_flags,dv_ecu_mV,dv_mV,dsoc%,bal_k,ok210,step_us,faults
 *
 * bms_can.c / bms_can.h = bms/common/bms_can.*'nin birebir kopyası; o değişirse bunlar da güncellenmeli.
 * twin_model.c parametreleri bms_config.h / bms_param_struct.m / battery_model.py ile aynı olmalı.
 */
#include <SPI.h>
#include <Wire.h>
#include "sw_spi_cfg.h"      /* TW_SW_SPI 1: MOSI D9, yazılımla SPI (hasarlı D11) */
#if TW_SW_SPI
#include "mcp2515_sw.h"
#else
#include <mcp2515.h>
#endif
#include <LiquidCrystal_I2C.h>
extern "C" {
#include "bms_can.h"
#include "twin_model.h"
}

const uint8_t  PIN_CS = 10, PIN_INT = 2;
const uint8_t  PIN_POT = A0, PIN_LM35 = A1;
/* mod butonları (dosya başı): D3 NORMAL, A2 DERATE, A3 LIMP, D4 STOP, D5 WATCHDOG */
enum { B_NORM, B_DER, B_LIMP, B_STOP, B_WDG, N_SC };
const uint8_t  PIN_SC[N_SC] = { 3, A2, A3, 4, 5 };
const uint8_t  LONG_TICKS = 10;            /* 100 ms örnek: >= 1 s basılı = uzun basış */
/* LCD bildirimi (16 karakter, "sebep>mod"); ev_sc bu tablonun indisidir */
enum { EV_NORM, EV_CHG, EV_LIMP, EV_STOP, EV_WDG, EV_COOL, EV_HEAT, N_EV };
const char EV_TXT[N_EV][2][17] PROGMEM = {
  { "SERVIS>NORMAL   ", "SERVIS>NORMAL   " },
  { "DOLU+SARJ>DERATE", "SARJ BITTI      " },
  { "SENSOR KOP>LIMP ", "SENSOR ONARILDI " },
  { "HAT KOPTU>STOP  ", "HAT ONARILDI    " },
  { "WATCHDOG>SAFE   ", "WATCHDOG>SAFE   " },
  { "SOGUTMA>DERATE  ", "SOGUTMA ONARILDI" },
  { "ISINMA>LIMP>STOP", "ISINMA: SOGUYOR " } };
const float    T_HEAT_MAX = 62.0f, HEAT_RATE = 0.05f;   /* °C, °C/örnek (0,5 °C/s; ECU hız teşhisi 2) */
const float    SEV_DEF  = 3.0f / 7.0f;    /* pot yoksa şiddet: R_u x4 (twin cooling_fault), 0,5 °C/s */
const float    T_HOT    = 15.0f;           /* sıcak gün ortam ofseti [°C] (25 + 15 = twin hot_day 40 °C) */
const uint32_t TX_PERIOD = 100;            /* ms, = BMS_TS */
const float    DT = 0.1f;

MCP2515 mcp2515(PIN_CS);
LiquidCrystal_I2C *lcd = 0;

/* ---------------------------------------------------------------- araç modeli (drive_cycle.py) */
#define VEH_M       450.0f      /* kg */
#define VEH_CRR     0.011f
#define VEH_CDA     0.45f
#define VEH_RHO     1.2f
#define ETA_D       0.88f
#define ETA_R       0.60f
#define P_AUX       250.0f      /* W */
#define I_DIS_MAX   87.3f       /* A, 1,5C x 12 */
#define I_REGEN_MAX 30.0f
#define I_CHG_REQ   40.7f       /* 0,7C x 12 */
#define F_PEDAL_MAX 1200.0f     /* N */
#define F_BRAKE     2500.0f
#define V_MOT_MIN   2.0f        /* m/s: güç hesabında alt sınır (duruşta da tork -> akım) */

enum { M_PROF, M_PEDAL, M_CHG, M_IDLE, N_MODE };
const char MODE_NAME[N_MODE][5] = { "PROF", "PEDL", "SARJ", "BEKL" };

twin_t   tw;
uint8_t  mode = M_PROF;
float    veh_v = 0.0f;                     /* araç hızı [m/s] */
float    I_dem = 0.0f, I_act = 0.0f;
float    pedal = 0.0f;
uint8_t  have_pot = 0, pedal_armed = 0, brake = 0;
int8_t   sev_force = -1;                   /* seri "v": 0..100 %, -1 = pot / varsayılan */
float    sev = SEV_DEF, sev_shown = -1.0f; /* arıza şiddeti 0..1; LCD'de son gösterilen */
float    t_pack_goal = 99.0f;              /* seri "t": paket sıcaklığı hedefi (99 = yok) */

/* sentetik çevrim: (hedef km/h, hızlanma s, sabit s, yavaşlama s, bekleme s) */
const uint8_t CYCLE[8][5] PROGMEM = {
  { 30, 10, 20, 7, 10 }, { 50, 14, 40, 9, 15 }, { 40, 11, 25, 7, 8 }, { 70, 22, 60, 12, 12 },
  { 50, 13, 30, 8, 10 }, { 90, 32, 80, 16, 20 }, { 35, 9, 15, 6, 12 }, { 60, 18, 45, 10, 15 } };
uint8_t  cy_seg = 0, cy_ph = 3;            /* faz: 0 hızlan, 1 sabit, 2 yavaşla, 3 bekle */
uint16_t cy_k = 0, cy_n = 100;             /* ilk 10 s bekleme */
float    cy_vt = 0.0f, v_ref = 0.0f;

/* ---------------------------------------------------------------- gürültü (xorshift32) */
uint32_t rng = 2463534242UL;
float urand() { rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5; return (rng >> 8) * (1.0f / 16777216.0f); }
float grand() { return (urand() + urand() + urand() + urand() - 2.0f) * 1.7320508f; }   /* ~N(0,1) */

/* ---------------------------------------------------------------- CAN durumu */
uint8_t  cnt_tx = 0, cnt_cmd = 0, cmd_have = 0, cmd_last = 0;
uint32_t t_tx = 0, t_rep = 0, t_lcd = 0;
uint32_t n_tx_err = 0;
can_monitor_t mon_status, mon_ctrl, mon_bal;
can_status_t  last_status;
can_ctrl_t    last_ctrl;
can_bal_t     last_bal;
uint8_t       have_status = 0, have_ctrl = 0, have_bal = 0;

/* ---------------------------------------------------------------- dengeleme / zamanlama */
const float   R_BLEED = 10.0f;             /* ohm, grup başına (arac_bms_analizi.md: 10 ohm 2 W) */
uint16_t bal_k = 1;                        /* gösterim çarpanı */
uint32_t bal_applied = 0;                  /* gerçekten açık olan dirençler */
uint32_t t_step = 0, step_us = 0;
uint8_t  cell_k = 5, cell_cnt = 0;         /* sıradaki hücre çerçevesi (5 = bitti), o turun sayacı */

/* ---------------------------------------------------------------- sensörler */
float    t_amb = 25.0f, t_force = 99.0f;   /* t_force < 90 -> zorla */
uint8_t  lm35_ok = 0, lm35_init = 0;
uint8_t  lm35_seen = 0, lm35_lost = 0;     /* açılışta vardı / sonradan koptu (-> NTC kopuk) */
uint8_t  hot_day = 0, cool_fault = 0;
uint8_t  heat_on = 0;                      /* A2: aşırı ısınma (0 yok, 1 ısın, 2 ortama soğu) */
uint8_t  page = 0;
uint8_t  btn_n[N_SC];                      /* basılı kalma süresi (100 ms adım, 255 = yok say) */
uint8_t  chg_on = 0;                       /* DERATE kısa basış: dolu bataryaya şarj sürüyor */
float    soc_before = -1.0f;               /* şarj senaryosundan önceki SOC (NORMAL geri yükler) */
uint8_t  svc = 0;                          /* NORMAL servis adımı: 0 yok, 1 soğut/bekle, 2 komut, 3 sürüşe dön */
uint32_t svc_t = 0, svc_t0 = 0;            /* adım zamanı / servis başlangıcı */
uint8_t  svc_n = 0, svc_round = 0, svc_seen = 0;
int8_t   ev_sc = -1;                       /* LCD'de gösterilen son bildirim (EV_*, -1 yok) */
uint8_t  ev_on = 0, ev_sev = 0;            /* ev_sev: bildirim yerine şiddet satırı */
uint32_t ev_until = 0, t_page = 0;

/* ---------------------------------------------------------------- hata enjeksiyonu (0x10-0x14) */
enum { F_CRC, F_FREEZE, F_SILENT, F_NTC, N_FAULT };
const char FAULT_NAME[N_FAULT][7] = { "CRC", "FREEZE", "SILENT", "NTC" };
uint8_t  f_on[N_FAULT], f_perm[N_FAULT];
uint32_t f_until[N_FAULT];
uint8_t  cnt_frozen = 0;

bool faultActive(uint8_t f, uint32_t now)
{
  if (f_on[f] && !f_perm[f] && (int32_t)(now - f_until[f]) >= 0) f_on[f] = 0;
  return f_on[f];
}

void applyEmuCommand(uint8_t cmd, int16_t value, uint32_t now)
{
  const uint16_t dur = (uint16_t)value;
  if (cmd == CAN_CMD_EMU_CLEAR) { memset(f_on, 0, sizeof(f_on)); Serial.println(F("EMU: all faults cleared")); return; }
  if (cmd < CAN_CMD_EMU_CRC || cmd > CAN_CMD_EMU_NTC_OPEN) { Serial.println(F("EMU: unknown EMU command")); return; }
  const uint8_t f = cmd - CAN_CMD_EMU_CRC;
  if (f == F_FREEZE && !f_on[F_FREEZE]) cnt_frozen = cnt_tx;
  f_on[f] = 1;  f_perm[f] = (dur == 0);  f_until[f] = now + dur;
  Serial.print(F("EMU: fault "));  Serial.println(FAULT_NAME[f]);
}

/* ---------------------------------------------------------------- CAN gönder / al */
bool sendRaw(uint16_t id, const uint8_t d[8])
{
  struct can_frame f;
  f.can_id = id;  f.can_dlc = 8;  memcpy(f.data, d, 8);
  return mcp2515.sendMessage(&f) == MCP2515::ERROR_OK;
}

void sendMeasurements(uint32_t now)
{
  can_meas1_t m1 = { tw.v_pack + 0.25f * grand(), I_act + 0.5f + 0.5f * grand(), tw.ts + 0.3f * grand() };
  if (faultActive(F_NTC, now) || lm35_lost) m1.T = -40.0f;
  const can_meas2_t m2 = { tw.v_min + 0.005f * grand(), tw.v_max + 0.005f * grand(), t_amb };
  const uint8_t c = faultActive(F_FREEZE, now) ? cnt_frozen : cnt_tx;
  const bool bad_crc = faultActive(F_CRC, now);
  uint8_t d[8];

  cell_cnt = c;                         /* bu turun hücre çerçeveleri aynı sayacı taşır */
  cnt_tx = (cnt_tx + 1) & 0x0F;
  if (faultActive(F_SILENT, now)) return;
  can_encode_meas1(&m1, c, d);
  if (bad_crc) d[7] ^= 0xFF;
  n_tx_err += !sendRaw(CAN_ID_EMU_MEAS1, d);
  can_encode_meas2(&m2, c, d);
  if (bad_crc) d[7] ^= 0xFF;
  n_tx_err += !sendRaw(CAN_ID_EMU_MEAS2, d);
}

/* hücre çerçevesi k (0x110 + k): gruplar 4k+1..4k+4, AFE gürültüsü sigma 1 mV */
void sendCell(uint8_t k, uint32_t now)
{
  can_cell_t m;
  m.valid = 0;  m.bal = 0;
  for (uint8_t j = 0; j < CAN_CELLS_PER_FRAME; j++) {
    const uint8_t g = 4 * k + j;
    m.v[j] = 0.0f;
    if (g >= TW_NG) continue;
    m.v[j] = tw.g[g].v + 0.001f * grand();
    m.valid |= 1 << j;
    if (bal_applied >> g & 1) m.bal |= 1 << j;
  }
  if (faultActive(F_SILENT, now)) return;
  uint8_t d[8];
  can_encode_cell(k, &m, cell_cnt, d);
  if (faultActive(F_CRC, now)) d[7] ^= 0xFF;
  n_tx_err += !sendRaw(CAN_ID_EMU_CELL0 + k, d);
}

void receiveAll(uint32_t now)
{
  struct can_frame f;
  while (mcp2515.readMessage(&f) == MCP2515::ERROR_OK) {
    const uint16_t id = f.can_id & CAN_SFF_MASK;
    uint8_t c = 0;
    if (f.can_dlc != 8 || (f.can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG))) continue;
    if (id == CAN_ID_ECU_STATUS) {
      can_status_t st;
      const int ok = can_decode_status(f.data, &st, &c);
      if (can_monitor_rx(&mon_status, now, ok, c)) { last_status = st; have_status = 1; }
    } else if (id == CAN_ID_ECU_CTRL) {
      can_ctrl_t ct;
      const int ok = can_decode_ctrl(f.data, &ct, &c);
      if (can_monitor_rx(&mon_ctrl, now, ok, c)) { last_ctrl = ct; have_ctrl = 1; }
    } else if (id == CAN_ID_ECU_BAL) {
      can_bal_t b;
      const int ok = can_decode_bal(f.data, &b, &c);
      if (can_monitor_rx(&mon_bal, now, ok, c)) { last_bal = b; have_bal = 1; }
    } else if (id == CAN_ID_PC_CMD) {
      can_cmd_t q;
      if (can_decode_cmd(f.data, &q, &c) && !(cmd_have && c == cmd_last)) {
        cmd_have = 1;  cmd_last = c;
        if (q.cmd >= 0x10 && q.cmd <= 0x1F) applyEmuCommand(q.cmd, q.value, now);
      }
    }
  }
}

void sendCommand(uint8_t cmd, int16_t value, uint32_t now)
{
  const can_cmd_t q = { cmd, value };
  uint8_t d[8];
  can_encode_cmd(&q, cnt_cmd, d);
  const bool ok = sendRaw(CAN_ID_PC_CMD, d);
  Serial.print(F("0x301 cmd 0x"));  Serial.print(cmd, HEX);
  Serial.print(F(" value "));       Serial.print(value);
  Serial.println(ok ? F(" sent") : F(" TX FAILED"));
  if (ok) cnt_cmd = (cnt_cmd + 1) & 0x0F;
  if (cmd >= 0x10 && cmd <= 0x1F) applyEmuCommand(cmd, value, now);
}

/* ---------------------------------------------------------------- sensörler ve butonlar */
void press(uint8_t k, uint8_t lng, uint32_t now);

void setCoolFault(uint8_t on)
{
  cool_fault = on;
  tw.ru_k = on ? 1.0f + 7.0f * sev : 1.0f;
  Serial.print(on ? F("cooling fault ON (R_u x") : F("cooling fault OFF"));
  if (on) { Serial.print(tw.ru_k, 1); Serial.print(')'); }
  Serial.println();
}

void readInputs(uint8_t tick, uint32_t now)
{
  /* LM35: her saniye bağlı mı diye bak (kopuk pin pull-up darbesinden sonra yüksek kalır) */
  if (tick == 0) {
    pinMode(PIN_LM35, INPUT_PULLUP);  delayMicroseconds(50);
    pinMode(PIN_LM35, INPUT);         delay(1);
    lm35_ok = analogRead(PIN_LM35) < 205;          /* < 100 °C */
  }
  uint16_t s = 0;
  for (uint8_t k = 0; k < 16; k++) s += analogRead(PIN_LM35);
  const float t = s * (500.0f / 1024.0f / 16.0f);  /* 5 V ref, 10 mV/°C */
  if (lm35_ok && (t < 2.0f || t > 60.0f)) lm35_ok = 0;
  /* bir kez görülen LM35 kaybolursa sensör kopuk: 0x101 T = -40 °C (sendMeasurements) */
  if (lm35_ok) lm35_seen = 1;
  const uint8_t lost = lm35_seen && !lm35_ok;
  if (lost != lm35_lost) {
    lm35_lost = lost;
    Serial.println(lost ? F("LM35 lost -> NTC open (T = -40)") : F("LM35 back"));
  }
  const float hot = hot_day ? T_HOT : 0.0f;
  if (t_force < 90.0f)  t_amb = t_force;
  else if (lm35_ok) {
    if (!lm35_init) { t_amb = t + hot; lm35_init = 1; }
    t_amb += 0.01f * (t + hot - t_amb);            /* tau ~10 s */
  } else if (!lm35_seen) { t_amb = 25.0f + hot; lm35_init = 0; }
  /* koptuysa ortam son değerde kalır */

  /* arıza şiddeti: seri "v" > pot > varsayılan; soğutma arızası sürerken R_u izler */
  if (sev_force >= 0) sev = sev_force * 0.01f;
  else if (have_pot) {
    const float p = (analogRead(PIN_POT) - 30) * (1.0f / 960.0f);
    sev = p < 0.0f ? 0.0f : (p > 1.0f ? 1.0f : p);
  } else sev = SEV_DEF;
  if (cool_fault) tw.ru_k = 1.0f + 7.0f * sev;
  if ((cool_fault || heat_on == 1) && fabsf(sev - sev_shown) > 0.03f) {   /* LCD 3 s */
    sev_shown = sev;  ev_sev = 1;  ev_until = now + 3000;
  }

  /* butonlar 100 ms'de bir örneklenir (sıçrama süzgeci yerine geçer). Kısa basış bırakınca,
   * uzun basış 1 s dolunca (basılıyken) tetiklenir; bırakış ondan sonra yok sayılır. */
  for (uint8_t k = 0; k < N_SC; k++) {
    if (digitalRead(PIN_SC[k]) == LOW) {
      if (btn_n[k] < 255 && ++btn_n[k] == LONG_TICKS) { press(k, 1, now); btn_n[k] = 255; }
    } else {
      if (btn_n[k] && btn_n[k] < LONG_TICKS) press(k, 0, now);
      btn_n[k] = 0;
    }
  }
}

/* LCD'de 3 s bildirim: i = EV_*, on = 1 arıza/senaryo başladı, 0 kaldırıldı */
void notify(uint8_t i, uint8_t on, uint32_t now)
{
  ev_sc = i;  ev_on = on;  ev_until = now + 3000;  t_lcd = 0;  ev_sev = 0;  sev_shown = sev;
  /* gateway satırı (tools/web/gateway.py --emu): B,<EV_*>,<1 başladı | 0 kaldırıldı> */
  Serial.print(F("B,"));  Serial.print(i);  Serial.print(',');  Serial.println(on);
}

/* DERATE kısa basış: dolu bataryaya hızlı şarj (aç/kapa) */
void chargeFull(uint8_t on)
{
  chg_on = on;
  if (on) { if (soc_before < 0.0f) soc_before = tw.soc_avg;  twin_set_soc(&tw, 0.88f);  mode = M_CHG; }
  else    { if (soc_before >= 0.0f) twin_set_soc(&tw, soc_before);  soc_before = -1.0f;  mode = M_PROF; }
  Serial.println(on ? F("full-battery charge ON (SOC 88 %)") : F("full-battery charge OFF"));
}

/* NORMAL: servis. Arızaları kaldır, paketi soğut, park; adımlar svcStep()'te */
void serviceStart(uint32_t now)
{
  memset(f_on, 0, sizeof(f_on));
  if (cool_fault) setCoolFault(0);
  heat_on = 0;
  if (chg_on) chargeFull(0);
  /* ön koşullandırma: paket 25-35 °C'ye (ortam + 2). 23 °C altı veri sayfası şarj sınırı (DERATE),
   * 43 °C üstü sıcaklık sınırları. 0,5 °C/örnek rampa (servis kısayolu). Bu ısıtmayı çalışan ECU
   * açıklanamayan ısı olarak görür (termal anomali LIMP, neden 257; Psi_Tdot): bu yüzden ECU yalnız
   * rampa bitip paket oturduktan sonra yeniden başlatılır, yeni açılan ECU ısınmayı hiç görmez.
   * Soğuk ortamda paket zamanla (tau ~10 dk) yine soğur: o zaman DERATE gerçektir. */
  float goal = t_amb + 2.0f;
  goal = goal < 25.0f ? 25.0f : (goal > 35.0f ? 35.0f : goal);
  if (fabsf(tw.ts - goal) > 0.5f) t_pack_goal = goal;
  mode = M_IDLE;                                      /* ECU yeniden başlatmayı |I| < 1 A'da kabul eder */
  svc = 1;  svc_t = svc_t0 = now;  svc_n = 0;  svc_round = 0;  svc_seen = 0;
  Serial.println(F("service: faults cleared, parking"));
}

void svcStep(uint32_t now)
{
  /* 1: rampa sürüyorsa ya da araç duruyor değilse bekleme saati baştan; 2 s oturunca komut.
   *    10 s'de olmazsa yine dene (en uzun rampa 4 s; ECU |I| >= 1 A'da reddeder, raporlanır) */
  if (svc == 1) {
    if (t_pack_goal < 90.0f || veh_v != 0.0f) svc_t = now;
    if (now - svc_t >= 2000 || now - svc_t0 >= 10000) { svc = 2;  svc_t = now - 400; }
  }
  /* yeniden başlama kanıtı: 0x201 adım başında normalde ~100 ms yaşında; ECU sıfırlanınca en az bir
   * cevap eksik (açılış + ölçüm çifti beklemesi) -> >= 150 ms. Komut adım başında gider, ECU hemen
   * sıfırlanır, sonraki adımda görülür. */
  if (svc >= 2 && have_ctrl && now - mon_ctrl.last_ok_ms >= 150) {
    if (!svc_seen) Serial.println(F("service: ECU restart seen"));
    svc_seen = 1;
    if (svc == 2) { svc = 3;  svc_t = now; }          /* kalan komutlar gereksiz (ECU 1 s reddeder) */
  }
  if (svc == 2 && now - svc_t >= 400) {               /* en çok 3 kez, 0,4 s arayla */
    sendCommand(CAN_CMD_ECU_RESTART, CAN_CMD_UNLOCK, now);
    svc_t = now;
    if (++svc_n >= 3) svc = 3;
  }
  if (svc == 3 && now - svc_t >= 1500) {
    if (!svc_seen && ++svc_round < 3) {               /* reddedildi ya da gitmedi: turu yinele */
      Serial.println(F("service: ECU restart not seen, retrying"));
      svc = 2;  svc_t = now - 400;  svc_n = 0;
    } else {
      svc = 0;  mode = M_PROF;
      Serial.println(svc_seen ? F("service: done, driving") : F("service: ECU did not restart, driving"));
    }
  }
}

/* buton k (seri "k <1-5>" kısa, "l <1-5>" uzun da buraya gelir); dosya başındaki tablo */
void press(uint8_t k, uint8_t lng, uint32_t now)
{
  if (k == B_DER && lng) {                            /* soğutma arızası (gizli; ECU kendisi bulur) */
    setCoolFault(!cool_fault);
    if (cool_fault && mode != M_PEDAL) mode = M_PROF; /* ısı akışı gerekir: sürüş */
    notify(EV_COOL, cool_fault, now);  return;
  }
  if (k == B_STOP && lng) {                           /* aşırı ısınma -> anomali LIMP -> 60 °C SAFE */
    heat_on = heat_on == 1 ? 2 : 1;  t_pack_goal = 99.0f;
    Serial.println(heat_on == 1 ? F("overheat ON") : F("overheat OFF (cooling down)"));
    notify(EV_HEAT, heat_on == 1, now);  return;
  }
  switch (k) {
    case B_NORM: serviceStart(now);  notify(EV_NORM, 1, now);  break;
    case B_DER:  chargeFull(!chg_on);  notify(EV_CHG, chg_on, now);  break;
    case B_LIMP:
    case B_STOP: {                 /* yalnız kendi hatasını aç/kapa (EMU_CLEAR hepsini silerdi) */
      const uint8_t f = k == B_LIMP ? F_NTC : F_SILENT;
      const uint8_t on = !f_on[f];
      if (on) applyEmuCommand(k == B_LIMP ? CAN_CMD_EMU_NTC_OPEN : CAN_CMD_EMU_SILENT, 0, now);
      else { f_on[f] = 0;  Serial.print(F("EMU: fault off "));  Serial.println(FAULT_NAME[f]); }
      notify(k == B_LIMP ? EV_LIMP : EV_STOP, on, now);
      break;
    }
    default:     sendCommand(CAN_CMD_ECU_HANG, CAN_CMD_UNLOCK, now);   /* 1 TX tamponu; 0x101/0x102 ile 3 */
                 notify(EV_WDG, 1, now);
  }
}

/* ---------------------------------------------------------------- sürüş çevrimi: v_ref, a_ref */
float cycleStep()
{
  const float v_prev = v_ref;
  if (++cy_k >= cy_n) {
    cy_k = 0;
    if (++cy_ph > 3) {
      cy_ph = 0;
      cy_seg = (cy_seg + 1) & 7;
      cy_vt = pgm_read_byte(&CYCLE[cy_seg][0]) * (1.0f + 0.1f * grand()) / 3.6f;
    }
    cy_n = pgm_read_byte(&CYCLE[cy_seg][cy_ph + 1]) * 10;
  }
  const float x = (float)cy_k / cy_n;
  switch (cy_ph) {
    case 0:  v_ref = cy_vt * x; break;
    case 1:  v_ref = cy_vt + 0.4f * sinf(cy_k * 0.05f); break;
    case 2:  v_ref = cy_vt * (1.0f - x); break;
    default: v_ref = 0.0f;
  }
  return (v_ref - v_prev) / DT;
}

/* ---------------------------------------------------------------- araç + akım sınırlama */
void vehicleStep()
{
  const float V   = tw.v_pack;
  const float res = (veh_v > 0.1f ? VEH_M * 9.81f * VEH_CRR : 0.0f) + 0.5f * VEH_RHO * VEH_CDA * veh_v * veh_v;
  const float ve  = veh_v > V_MOT_MIN ? veh_v : V_MOT_MIN;
  float F_dem = 0.0f, i_req;

  switch (mode) {
    case M_PROF: {
      const float a_ref = cycleStep();
      F_dem = VEH_M * a_ref + res + VEH_M * 1.0f * (v_ref - veh_v);
      if (v_ref < 0.01f && veh_v < 0.3f) F_dem = 0.0f;
      break;
    }
    case M_PEDAL:
      if (!pedal_armed && pedal < 0.05f) pedal_armed = 1;
      F_dem = pedal_armed ? pedal * F_PEDAL_MAX : 0.0f;
      if (brake) F_dem = -F_BRAKE;
      break;
    default:
      if (veh_v > 0.0f) F_dem = -F_BRAKE;
  }
  if (veh_v < 0.05f && F_dem < 0.0f) F_dem = 0.0f;

  /* talep edilen akım */
  if (mode == M_CHG && veh_v == 0.0f) i_req = -I_CHG_REQ + P_AUX / V;
  else if (mode == M_IDLE && veh_v == 0.0f) i_req = 0.0f;          /* park: yardımcı yük de kapalı */
  else {
    const float pw = F_dem * ve;
    const float pe = pw >= 0.0f ? pw / ETA_D : pw * ETA_R;
    i_req = (pe + P_AUX) / V;
    if (i_req < -I_REGEN_MAX) i_req = -I_REGEN_MAX;
  }
  if (i_req > I_DIS_MAX) i_req = I_DIS_MAX;
  I_dem = i_req;

  /* ECU izni: 0x201 taze değilse akım yok */
  float i_dis = 0.0f, i_chg = 0.0f;
  if (have_ctrl && !mon_ctrl.fault) { i_dis = last_ctrl.I_allow_dis; i_chg = last_ctrl.I_allow_chg; }
  float I = i_req > i_dis ? i_dis : i_req;
  if (I < -i_chg) I = -i_chg;
  I_act = I;

  /* gerçekleşen kuvvet: çekişte akım sınırı kuvveti keser; frende eksik kalanı mekanik fren tamamlar */
  float F = F_dem;
  if (F_dem > 0.0f) {
    const float f_el = (I * V - P_AUX) * ETA_D / ve;
    F = f_el < F_dem ? f_el : F_dem;
    if (F < 0.0f) F = 0.0f;
  }
  veh_v += (F - res) / VEH_M * DT;
  /* duruş: talep yokken (fren / bekleme) 0,1 m/s altında araç durur. Yoksa fren 0,05 m/s altında
   * bırakılıyor, araç ~0,2 km/h ile sürünüp kalıyordu (park modunda yardımcı yük 3 A, servis durmayı
   * hiç göremiyordu). */
  if (veh_v < 0.0f || (F_dem <= 0.0f && veh_v < 0.1f)) veh_v = 0.0f;
}

/* ---------------------------------------------------------------- 10 Hz adım */
void step(uint32_t now)
{
  static uint8_t tick = 0, first = 1;
  const uint32_t t0 = micros();
  readInputs(tick, now);
  svcStep(now);
  if (++tick >= 10) tick = 0;
  vehicleStep();
  tw.t_amb = t_amb;
  if (first) { tw.tc = tw.ts = t_amb; first = 0; }   /* paket ortam sıcaklığında başlar */
  if (t_pack_goal < 90.0f) {                          /* "t": hedefe rampa (0,5 °C / örnek) */
    const float e = t_pack_goal - tw.ts;
    const float d = e > 0.5f ? 0.5f : (e < -0.5f ? -0.5f : e);
    tw.ts += d;  tw.tc = tw.ts;
    if (d == e) t_pack_goal = 99.0f;
  }
  if (heat_on) {                                      /* A2: aşırı ısınma / soğuma rampası */
    const float g = heat_on == 1 ? T_HEAT_MAX : t_amb + 2.0f;
    const float e = g - tw.ts;
    const float r = heat_on == 1 ? HEAT_RATE * sev / SEV_DEF : HEAT_RATE;   /* soğuma sabit hızda */
    const float d = e > r ? r : (e < -r ? -r : e);
    tw.ts += d;  tw.tc += d;
    if (heat_on == 2 && d == e) heat_on = 0;          /* ortama yaklaştı: ikizin kendi termali sürer */
  }
  /* dengeleme: ECU'nun 0x210'u taze değilse dirençler kapalı */
  bal_applied = (have_bal && !mon_bal.fault) ? last_bal.mask & 0x7FFFFUL : 0;
  twin_step(&tw, I_act, bal_applied, tw.v_max / R_BLEED, bal_k);
  const uint32_t us = micros() - t0;
  if (us > step_us) step_us = us;
  sendMeasurements(now);
  can_monitor_tick(&mon_status, now);
  can_monitor_tick(&mon_ctrl, now);
  can_monitor_tick(&mon_bal, now);
}

/* ---------------------------------------------------------------- seri komutlar */
void serialCommands(uint32_t now)
{
  static char line[32];
  static uint8_t n = 0;
  while (Serial.available()) {
    const char ch = (char)Serial.read();
    if (ch != 10 && ch != 13) { if (n < sizeof(line) - 1) line[n++] = ch; continue; }
    if (n == 0) continue;
    line[n] = 0;  n = 0;
    char *p = line, *e;
    while (*p == ' ') p++;
    const char k = *p++ | 0x20;
    const long a = strtol(p, &e, 0);
    if (e == p) { Serial.println(F("usage: c <cmd> <val> | m <0-3> | s <soc%> | a <degC> | b <k> | f/i/h <0|1> | k/l <1-5> | p | t | v | y")); continue; }
    if (k == 'c') {
      p = e;
      const long val = strtol(p, &e, 0);
      if (e == p || a < 0 || a > 255 || val < -32768L || val > 65535L) { Serial.println(F("bad c")); continue; }
      sendCommand((uint8_t)a, (int16_t)(uint16_t)val, now);
    } else if (k == 'm' && a >= 0 && a < N_MODE) { mode = a; pedal_armed = 0; Serial.print(F("mode ")); Serial.println(MODE_NAME[mode]); }
    else if (k == 's' && a >= 0 && a <= 100) { twin_set_soc(&tw, a * 0.01f); Serial.println(F("twin SOC set")); }
    else if (k == 'a') { t_force = a; Serial.println(F("T_amb override")); }
    else if (k == 'b' && a >= 1 && a <= 2000) { bal_k = a; Serial.println(F("bleed factor set")); }
    else if (k == 'f') setCoolFault(a != 0);
    else if (k == 'i') { if ((heat_on == 1) != (a != 0)) press(B_STOP, 1, now); }
    else if ((k == 'k' || k == 'l') && a >= 1 && a <= N_SC) press(a - 1, k == 'l', now);
    else if (k == 'v' && a >= -1 && a <= 100) { sev_force = a; Serial.println(F("severity set")); }
    else if (k == 'y' && a >= 50 && a <= 100) { tw.soh = a * 0.01f; Serial.println(F("twin SOH set")); }
    else if (k == 'p' && a >= 0 && a <= 100) {
      pedal = a * 0.01f;  mode = M_PEDAL;  pedal_armed = 1;
      Serial.println(F("pedal set"));
    }
    else if (k == 't' && a > -20 && a < 70) { t_pack_goal = a; Serial.println(F("pack T ramp")); }
    else if (k == 'h') { hot_day = a != 0; Serial.println(hot_day ? F("hot day ON (+15 C)") : F("hot day OFF")); }
    else Serial.println(F("bad command"));
  }
}

/* ---------------------------------------------------------------- rapor (CSV, 1 s) */
void pf(float x, uint8_t d) { Serial.print(','); Serial.print(x, d); }
void pi(long x)             { Serial.print(','); Serial.print(x); }

void report(uint32_t now)
{
  Serial.print(F("R"));  pi(now / 1000);
  Serial.print(','); Serial.print(MODE_NAME[mode]);
  pf(veh_v * 3.6f, 1);  pf(I_act, 1);  pf(tw.v_pack, 2);  pf(tw.soc_avg * 100.0f, 2);
  pf(tw.tc, 2);  pf(tw.ts, 2);  pf(t_amb, 1);
  Serial.print(t_force < 90.0f ? F(",SET") : (lm35_ok ? F(",LM35") : F(",FIX")));
  pi(have_ctrl ? last_ctrl.state : 0);
  pf(last_ctrl.I_allow_dis, 1);  pf(last_ctrl.I_allow_chg, 1);  pi(last_ctrl.cause);
  pf(last_status.soc_pct, 2);  pf(last_status.tc_hat, 2);
  pi(mon_ctrl.n_ok);  pi(mon_status.n_ok);
  pi(mon_ctrl.n_crc_err + mon_ctrl.n_cnt_err + mon_status.n_crc_err + mon_status.n_cnt_err + n_tx_err
     + (mon_ctrl.fault ? 1000 : 0));
  pi(mcp2515.errorCountTX());  pi(mcp2515.errorCountRX());
  Serial.print(F(",0x"));  Serial.print(bal_applied, HEX);
  pi(have_bal ? last_bal.flags : 255);
  pf(last_bal.dv * 1000.0f, 1);  pf((tw.v_max - tw.v_min) * 1000.0f, 1);
  pf((tw.soc_max - tw.soc_min) * 100.0f, 3);
  pi(bal_k);  pi(mon_bal.n_ok);  pi(step_us);
  step_us = 0;
  Serial.print(',');
  for (uint8_t f = 0; f < N_FAULT; f++) if (faultActive(f, now)) { Serial.print(FAULT_NAME[f]); Serial.print(' '); }
  if (lm35_lost)  Serial.print(F("LM35LOST "));
  if (cool_fault) Serial.print(F("COOL "));
  if (hot_day)    Serial.print(F("HOT "));
  if (heat_on)    Serial.print(heat_on == 1 ? F("HEAT ") : F("COOLDOWN "));
  if (cool_fault || heat_on == 1) { Serial.print(F("SEV")); Serial.print((int)(sev * 100.0f + 0.5f)); Serial.print(' '); }
  if (tw.soh < 0.995f) { Serial.print(F("SOH")); Serial.print((int)(tw.soh * 100.0f + 0.5f)); Serial.print(' '); }
  Serial.println();
}

/* ---------------------------------------------------------------- LCD (2 Hz) */
/* 5 = CAN!: 0x201 zaman aşımında (> CAN_TIMEOUT_FAULT_MS; kablo kopuk ya da ECU yeniden başlıyor).
 * ECU'nun durumu bilinemez; ECU bu sırada kendi tarafında SAFE'e geçer (far söner, röle bırakır). */
const char STATE_NAME[6][5] = { "----", "NORM", "DERT", "LIMP", "SAFE", "CAN!" };

void lcdNum(float x, int8_t w, uint8_t d) { char b[10]; dtostrf(x, w, d, b); lcd->print(b); }

/* Tek satır yazar (~30 ms; I2C 100 kHz'de karakter başına ~6 aktarım): tam ekran 100 ms'lik
 * döngüyü taşırıp hücre çerçevelerini kaçırtıyordu. Satırlar sırayla, her biri 1 Hz. */
void lcdUpdate(uint8_t row)
{
  const uint8_t st = mon_ctrl.fault ? 5 : (have_ctrl ? (last_ctrl.state <= 4 ? last_ctrl.state : 0) : 0);
  lcd->setCursor(0, row);
  if ((int32_t)(millis() - ev_until) < 0 && ev_sc >= 0) {   /* senaryo bildirimi (3 s) */
    if (!row && ev_sev) {                                     /* şiddet: SOGUTMA Ru x4.0 / ISINMA 0.50C/s */
      if (cool_fault) { lcd->print(F("SOGUTMA Ru x")); lcdNum(tw.ru_k, 3, 1); lcd->print(' '); }
      else            { lcd->print(F("ISINMA ")); lcdNum(HEAT_RATE * sev / SEV_DEF * 10.0f, 4, 2); lcd->print(F("C/s ")); }
    } else if (!row) lcd->print((const __FlashStringHelper *)EV_TXT[ev_sc][ev_on ? 0 : 1]);
    else      { lcd->print(F("ECU ")); lcd->print(STATE_NAME[st]); lcd->print(F(" Id")); lcdNum(last_ctrl.I_allow_dis, 4, 0);
                lcd->print(F("A  ")); }
  } else if (page == 0) {        /* PROF NORM  42kmh / I+ 23.4 SOC 78.2 */
    if (!row) { lcd->print(MODE_NAME[mode]); lcd->print(' '); lcd->print(STATE_NAME[st]); lcd->print(' ');
                lcdNum(veh_v * 3.6f, 4, 0); lcd->print(F("kmh")); }
    else      { lcd->print('I'); lcdNum(I_act, 6, 1); lcd->print(F(" S")); lcdNum(tw.soc_avg * 100.0f, 5, 1); lcd->print(F("% ")); }
  } else if (page == 1) {        /* ECU: izin ve kestirim */
    if (!row) { lcd->print(F("Id")); lcdNum(last_ctrl.I_allow_dis, 4, 0); lcd->print(F(" Ic")); lcdNum(last_ctrl.I_allow_chg, 3, 0);
                lcd->print(' '); lcd->print(STATE_NAME[st]); lcd->print(' '); }
    else      { lcd->print(F("S^")); lcdNum(last_status.soc_pct, 5, 1); lcd->print(F(" Tc^")); lcdNum(last_status.tc_hat, 5, 1); lcd->print(' '); }
  } else if (page == 3) {        /* dengeleme: gerçek gerilim farkı, açık direnç sayısı / SOC farkı, çarpan */
    if (!row) { uint8_t n = 0;
                for (uint8_t j = 0; j < TW_NG; j++) n += bal_applied >> j & 1;
                lcd->print(F("dV")); lcdNum((tw.v_max - tw.v_min) * 1000.0f, 5, 1); lcd->print(F("mV B")); lcdNum(n, 2, 0);
                lcd->print(F("  ")); }
    else      { lcd->print(F("dS")); lcdNum((tw.soc_max - tw.soc_min) * 100.0f, 5, 2); lcd->print(F("% x")); lcdNum(bal_k, 4, 0);
                lcd->print(F("  ")); }
  } else {                       /* sıcaklıklar, kaynak ve demo arızaları (S sıcak gün, C soğutma) */
    if (!row) { lcd->print(F("Tc")); lcdNum(tw.tc, 5, 1); lcd->print(F(" Ts")); lcdNum(tw.ts, 5, 1); lcd->print(F("  ")); }
    else      { lcd->print(F("Ta")); lcdNum(t_amb, 5, 1); lcd->print(' ');
                lcd->print(t_force < 90.0f ? F("SET ") : (lm35_ok ? F("LM35") : (lm35_lost ? F("KOP!") : F("FIX "))));
                lcd->print(' '); lcd->print(hot_day ? 'S' : '-');
                lcd->print(cool_fault ? 'C' : (heat_on == 1 ? 'H' : '-'));
                lcd->print(mcp2515.errorCountTX() ? '!' : ' '); }   /* ! = CAN TEC > 0 (değer raporda) */
  }
}

/* ---------------------------------------------------------------- kurulum / döngü */
void setup()
{
  Serial.begin(115200);
  pinMode(PIN_INT, INPUT_PULLUP);
  for (uint8_t k = 0; k < N_SC; k++) pinMode(PIN_SC[k], INPUT_PULLUP);
  delay(2);
  /* D3 basılıyken açılış = sıcak gün; bırakılırken soğutma arızası tetiklenmesin (kenar yok) */
  hot_day = digitalRead(PIN_SC[B_NORM]) == LOW;
  if (hot_day) btn_n[B_NORM] = 255;                   /* bırakış ve basılı tutma tetiklemesin */

  /* şiddet potu var mı: pull-up açıkken kopuk pin 1023 okur; pot herhangi bir konumda daha düşük */
  pinMode(PIN_POT, INPUT_PULLUP);  delay(2);
  have_pot = analogRead(PIN_POT) < 1000;
  pinMode(PIN_POT, INPUT);
  mode = M_PROF;

  /* LCD: 0x27 ya da 0x3F'te ACK veren PCF8574 */
  Wire.begin();
  Wire.setWireTimeout(3000, true);
  const uint8_t addr[2] = { 0x27, 0x3F };
  for (uint8_t k = 0; k < 2 && !lcd; k++) {
    Wire.beginTransmission(addr[k]);
    if (Wire.endTransmission() == 0) {
      static LiquidCrystal_I2C l(addr[k], 16, 2);
      lcd = &l;
      lcd->init();  lcd->backlight();
    }
  }

  Serial.println();
  Serial.print(F("=== twin_emu: 10 Hz pack twin, TX 0x101/0x102, RX 0x201/0x202 | pot "));
  Serial.print(have_pot ? F("yes") : F("no"));
  Serial.print(F(" | LCD "));  Serial.print(lcd ? F("yes") : F("no"));
  Serial.print(F(" | mode "));  Serial.print(MODE_NAME[mode]);
  Serial.println(hot_day ? F(" | HOT DAY (+15 C)") : F(""));

  mcp2515.reset();
  if (mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ) != MCP2515::ERROR_OK || mcp2515.setNormalMode() != MCP2515::ERROR_OK) {
    Serial.println(F("MCP2515 init FAILED"));
    while (1) { }
  }
  twin_init(&tw, 0.80f, 25.0f, DT);
  can_monitor_init(&mon_status, millis());
  can_monitor_init(&mon_ctrl, millis());
  can_monitor_init(&mon_bal, millis());
  t_tx = t_rep = t_lcd = millis();
}

void loop()
{
  uint32_t now = millis();
  if (digitalRead(PIN_INT) == LOW) receiveAll(now);
  serialCommands(now);
  if ((int32_t)(now - t_tx) >= 0) {
    t_tx += TX_PERIOD;
    receiveAll(now);
    step(now);
    t_step = now = millis();
    cell_k = 0;
  }
  /* döngü içi pencereler (dosya başındaki "Zamanlama") */
  const uint32_t ph = now - t_step;
  if (cell_k < CAN_N_CELL_FRAMES && ph >= 10u + 5u * cell_k) sendCell(cell_k++, now);
  if (cell_k >= CAN_N_CELL_FRAMES && ph >= 35u && ph < 55u) {     /* hücreler gittikten sonra, tek iş */
    static uint8_t row = 0;
    if ((int32_t)(now - t_rep) >= 0) { t_rep += 1000; report(now); }
    else if (lcd && (int32_t)(now - t_lcd) >= 0) {
      /* sayfalar kendiliğinden döner (sürüş / ECU izni / sıcaklık), her biri 3 s */
      if (!row && (int32_t)(now - t_page) >= 0) { t_page = now + 3000; page = (page + 1) % 3; }
      t_lcd = now + 500;  lcdUpdate(row);  row ^= 1;
    }
  }
}
