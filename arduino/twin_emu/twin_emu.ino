/* twin_emu.ino -- Arduino Uno + MCP2515: paket dijital ikizi emülatörü (HIL adım 3, "EMU" düğümü).
 *
 * mcp2515_node'un sabit değerli taklidi yerine C:\bitirme\twin ikizinin C portunu (twin_model.c)
 * 10 Hz çalıştırır: araç modeli -> akım talebi -> ECU izni (0x201 I_allow) ile sınırla -> ikiz ->
 * gürültü ekle -> 0x101 / 0x102. Gürültü twin/closed_loop.py ile aynı: V sigma 0,25 V, I +0,5 A
 * sapma + sigma 0,5 A, T sigma 0,3 °C, V_min/V_max sigma 5 mV. 0x101'deki T = yüzey sıcaklığı.
 *
 * Kütüphaneler : "mcp2515" (autowp), LiquidCrystal_I2C (Library Manager)
 * Bağlantı     : MCP2515 CS = D10, INT = D2, SPI D11/D12/D13; 500 kbit/s, kristal 8 MHz
 *                A0 pedal potu (uçlar 5 V / GND, orta uç A0)      yoksa: hazır sürüş profili
 *                A1 LM35 çıkışı (ortam sıcaklığı, 10 mV/°C)       yoksa: 25 °C
 *                D3 mod butonu, A2 fren butonu, A3 LCD sayfa butonu (GND'ye basar, dahili pull-up)
 *                16x2 I2C LCD (PCF8574, adres 0x27 ya da 0x3F) SDA A4, SCL A5   yoksa: yalnız seri
 * Algılama     : pedal açılışta (A0'da pull-up açıkken < 1000 okunursa var), LM35 her saniye
 *                (kısa pull-up darbesinden sonra makul aralıkta okunursa var), LCD açılışta I2C ACK.
 *
 * Sürüş modları (D3 ya da "m <n>"):
 *   0 PROF  twin/drive_cycle.py sentetik çevrimi (hız izleyen sürücü), sürekli tekrar
 *   1 PEDL  pedal = çekiş kuvveti (0..1200 N), A2 basılıyken fren (rejeneratif en çok 30 A,
 *           kalanı mekanik fren). Moda girince pedal bir kez bırakılmalı (< %5).
 *   2 SARJ  araç durur, 0,7C (40,7 A) şarj talebi; ECU şarj izni sınırlar (CV bölgesi ECU'da)
 *   3 BEKL  araç durur, yalnız yardımcı yük (250 W)
 * Her durumda paket akımı = kırp(talep, -I_allow_chg, +I_allow_dis); 0x201 zaman aşımında 0 A.
 *
 * Seri (115200, satır sonu Newline):
 *   c <cmd> <value>  0x301 komutu (mcp2515_node ile aynı; 0x10-0x1F EMU hataları burada da uygulanır)
 *   m <0..3>         sürüş modu       s <yüzde>  ikiz SOC'yi ayarla (ör. s 80)
 *   a <°C>           ortam sıcaklığını zorla (a 99 = kapat, LM35 / 25 °C'ye dön)
 * Her 1 s bir rapor satırı (tools/twin_emu_run.py bunu özetler):
 *   R,t_s,mode,v_kmh,I,V,soc%,Tc,Ts,Ta,ta_src,ecu_state,Idis,Ichg,cause,soc_ecu%,tc_ecu,ok201,ok202,err,TEC,REC,faults
 *
 * bms_can.c / bms_can.h = bms/common/bms_can.*'nin birebir kopyası; o değişirse bunlar da güncellenmeli.
 * twin_model.c parametreleri bms_config.h / bms_param_struct.m / battery_model.py ile aynı olmalı.
 */
#include <SPI.h>
#include <Wire.h>
#include <mcp2515.h>
#include <LiquidCrystal_I2C.h>
extern "C" {
#include "bms_can.h"
#include "twin_model.h"
}

const uint8_t  PIN_CS = 10, PIN_INT = 2;
const uint8_t  PIN_MODE = 3, PIN_BRAKE = A2, PIN_PAGE = A3, PIN_PEDAL = A0, PIN_LM35 = A1;
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
uint8_t  have_pedal = 0, pedal_armed = 0, brake = 0;

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
can_monitor_t mon_status, mon_ctrl;
can_status_t  last_status;
can_ctrl_t    last_ctrl;
uint8_t       have_status = 0, have_ctrl = 0;

/* ---------------------------------------------------------------- sensörler */
float    t_amb = 25.0f, t_force = 99.0f;   /* t_force < 90 -> zorla */
uint8_t  lm35_ok = 0, lm35_init = 0;
uint8_t  page = 0, btn_prev = 0;

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
  if (faultActive(F_NTC, now)) m1.T = -40.0f;
  const can_meas2_t m2 = { tw.v_min + 0.005f * grand(), tw.v_max + 0.005f * grand(), t_amb };
  const uint8_t c = faultActive(F_FREEZE, now) ? cnt_frozen : cnt_tx;
  const bool bad_crc = faultActive(F_CRC, now);
  uint8_t d[8];

  cnt_tx = (cnt_tx + 1) & 0x0F;
  if (faultActive(F_SILENT, now)) return;
  can_encode_meas1(&m1, c, d);
  if (bad_crc) d[7] ^= 0xFF;
  n_tx_err += !sendRaw(CAN_ID_EMU_MEAS1, d);
  can_encode_meas2(&m2, c, d);
  if (bad_crc) d[7] ^= 0xFF;
  n_tx_err += !sendRaw(CAN_ID_EMU_MEAS2, d);
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
void readInputs(uint8_t tick)
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
  if (t_force < 90.0f)  t_amb = t_force;
  else if (lm35_ok) {
    if (!lm35_init) { t_amb = t; lm35_init = 1; }
    t_amb += 0.01f * (t - t_amb);                  /* tau ~10 s */
  } else { t_amb = 25.0f; lm35_init = 0; }

  if (have_pedal) {
    const float p = (analogRead(PIN_PEDAL) - 30) * (1.0f / 960.0f);
    pedal = p < 0.0f ? 0.0f : (p > 1.0f ? 1.0f : p);
  }
  brake = digitalRead(PIN_BRAKE) == LOW;

  /* butonlar 100 ms'de bir örneklenir (sıçrama süzgeci yerine geçer); basılma kenarı */
  const uint8_t b = (digitalRead(PIN_MODE) == LOW) | ((digitalRead(PIN_PAGE) == LOW) << 1);
  const uint8_t edge = b & ~btn_prev;
  btn_prev = b;
  if (edge & 1) { mode = (mode + 1) % N_MODE; pedal_armed = 0; }
  if (edge & 2) { page = (page + 1) % 3; t_lcd = 0; }
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
  if (veh_v < 0.0f) veh_v = 0.0f;
}

/* ---------------------------------------------------------------- 10 Hz adım */
void step(uint32_t now)
{
  static uint8_t tick = 0;
  readInputs(tick);
  if (++tick >= 10) tick = 0;
  vehicleStep();
  tw.t_amb = t_amb;
  twin_step(&tw, I_act);
  sendMeasurements(now);
  can_monitor_tick(&mon_status, now);
  can_monitor_tick(&mon_ctrl, now);
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
    if (e == p) { Serial.println(F("usage: c <cmd> <val> | m <0-3> | s <soc%> | a <degC>")); continue; }
    if (k == 'c') {
      p = e;
      const long val = strtol(p, &e, 0);
      if (e == p || a < 0 || a > 255 || val < -32768L || val > 65535L) { Serial.println(F("bad c")); continue; }
      sendCommand((uint8_t)a, (int16_t)(uint16_t)val, now);
    } else if (k == 'm' && a >= 0 && a < N_MODE) { mode = a; pedal_armed = 0; Serial.print(F("mode ")); Serial.println(MODE_NAME[mode]); }
    else if (k == 's' && a >= 0 && a <= 100) { twin_set_soc(&tw, a * 0.01f); Serial.println(F("twin SOC set")); }
    else if (k == 'a') { t_force = a; Serial.println(F("T_amb override")); }
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
  Serial.print(',');
  for (uint8_t f = 0; f < N_FAULT; f++) if (faultActive(f, now)) { Serial.print(FAULT_NAME[f]); Serial.print(' '); }
  Serial.println();
}

/* ---------------------------------------------------------------- LCD (2 Hz) */
const char STATE_NAME[5][5] = { "----", "NORM", "DERT", "LIMP", "SAFE" };

void lcdNum(float x, int8_t w, uint8_t d) { char b[10]; dtostrf(x, w, d, b); lcd->print(b); }

void lcdUpdate()
{
  const uint8_t st = have_ctrl && !mon_ctrl.fault ? (last_ctrl.state <= 4 ? last_ctrl.state : 0) : 0;
  lcd->setCursor(0, 0);
  if (page == 0) {               /* PROF NORM  42kmh / I+ 23.4 SOC 78.2 */
    lcd->print(MODE_NAME[mode]); lcd->print(' '); lcd->print(STATE_NAME[st]); lcd->print(' ');
    lcdNum(veh_v * 3.6f, 4, 0); lcd->print(F("kmh"));
    lcd->setCursor(0, 1);
    lcd->print('I'); lcdNum(I_act, 6, 1); lcd->print(F(" S")); lcdNum(tw.soc_avg * 100.0f, 5, 1); lcd->print(F("% "));
  } else if (page == 1) {        /* ECU: izin ve kestirim */
    lcd->print(F("Id")); lcdNum(last_ctrl.I_allow_dis, 4, 0); lcd->print(F(" Ic")); lcdNum(last_ctrl.I_allow_chg, 3, 0);
    lcd->print(' '); lcd->print(STATE_NAME[st]); lcd->print(' ');
    lcd->setCursor(0, 1);
    lcd->print(F("S^")); lcdNum(last_status.soc_pct, 5, 1); lcd->print(F(" Tc^")); lcdNum(last_status.tc_hat, 5, 1); lcd->print(' ');
  } else {                       /* sıcaklıklar ve kaynak */
    lcd->print(F("Tc")); lcdNum(tw.tc, 5, 1); lcd->print(F(" Ts")); lcdNum(tw.ts, 5, 1); lcd->print(F("  "));
    lcd->setCursor(0, 1);
    lcd->print(F("Ta")); lcdNum(t_amb, 5, 1); lcd->print(' ');
    lcd->print(t_force < 90.0f ? F("SET ") : (lm35_ok ? F("LM35") : F("FIX ")));
    lcd->print(F(" T")); lcdNum(mcp2515.errorCountTX(), 3, 0);
  }
}

/* ---------------------------------------------------------------- kurulum / döngü */
void setup()
{
  Serial.begin(115200);
  pinMode(PIN_INT, INPUT_PULLUP);
  pinMode(PIN_MODE, INPUT_PULLUP);  pinMode(PIN_BRAKE, INPUT_PULLUP);  pinMode(PIN_PAGE, INPUT_PULLUP);

  /* pedal var mı: pull-up açıkken kopuk pin 1023 okur; pot herhangi bir konumda daha düşük */
  pinMode(PIN_PEDAL, INPUT_PULLUP);  delay(2);
  have_pedal = analogRead(PIN_PEDAL) < 1000;
  pinMode(PIN_PEDAL, INPUT);
  mode = have_pedal ? M_PEDAL : M_PROF;

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
  Serial.print(F("=== twin_emu: 10 Hz pack twin, TX 0x101/0x102, RX 0x201/0x202 | pedal "));
  Serial.print(have_pedal ? F("yes") : F("no"));
  Serial.print(F(" | LCD "));  Serial.print(lcd ? F("yes") : F("no"));
  Serial.print(F(" | mode "));  Serial.println(MODE_NAME[mode]);

  mcp2515.reset();
  if (mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ) != MCP2515::ERROR_OK || mcp2515.setNormalMode() != MCP2515::ERROR_OK) {
    Serial.println(F("MCP2515 init FAILED"));
    while (1) { }
  }
  twin_init(&tw, 0.80f, 25.0f, DT);
  can_monitor_init(&mon_status, millis());
  can_monitor_init(&mon_ctrl, millis());
  t_tx = t_rep = t_lcd = millis();
}

void loop()
{
  const uint32_t now = millis();
  if (digitalRead(PIN_INT) == LOW) receiveAll(now);
  serialCommands(now);
  if ((int32_t)(now - t_tx) >= 0) {
    t_tx += TX_PERIOD;
    receiveAll(now);
    step(now);
  }
  if ((int32_t)(now - t_rep) >= 0) { t_rep += 1000; report(now); }
  if (lcd && (int32_t)(now - t_lcd) >= 0) { t_lcd = now + 500; lcdUpdate(); }
}
