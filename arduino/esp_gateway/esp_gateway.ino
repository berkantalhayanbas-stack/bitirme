/* esp_gateway.ino -- ESP-01 (ESP8266) Wi-Fi telematik köprüsü: STM32 UART satırları -> HTTPS -> api.php
 *
 * tools/web/gateway.py'nin aynısı, PC yerine ESP'de. STM32 LPUART1 TX (PA2, 115200) -> ESP RX; ortak GND.
 * ESP'nin TX'i STM'e bağlanmaz (PA3 ST-LINK sanal portuna bağlı; iki çıkış çakışır). Bu yüzden
 * köprü tek yönlü: komut (RUN/ALL/STOP) yok, yalnız telemetri. ESP TX'i yalnız hata ayıklama içindir
 * (Uno köprüsüyle yüklerken Serial Monitor'de görünür).
 *
 * Satırlar (gateway.py ile aynı): D,<t>,<run>,<soc>,<soh>,<T>,<I>,<st>,<tec>,<rec> | E,<t>,<run>,<code>,<detail>
 *   | RS,<run>,<name> | RE,<run>,<PASS|FAIL>,<summary>. Diğer satırlar atılır.
 * Her SEND_MS'de bir JSON yığını POST edilir (X-API-Key). Bağlantı açık tutulur (TLS el sıkışması
 * ESP8266'da ~1-2 s). Gönderim sürerken UART 8 KB tamponda birikir; yığın sınırı aşılırsa D satırları
 * seyreltilir (RS/RE/E hiç atılmaz), sayaç debug satırında.
 *
 * Gizli bilgiler secrets.h'de (git dışı; secrets.h.example'dan kopyala): Wi-Fi adı/parolası, API anahtarı.
 * Kart: Generic ESP8266 Module (esp8266:esp8266:generic), Flash 1MB, yükleme 115200.
 */
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClientSecureBearSSL.h>
#include "secrets.h"            /* WIFI_SSID, WIFI_PASS, API_KEY, API_URL */

#ifndef API_URL
#define API_URL "https://hizlipdf.net/bitirmeprj/api.php"
#endif
const uint32_t SEND_MS = 1000;
const uint16_t MAX_ROWS = 120;   /* yığın başına en çok D satırı (fazlası seyreltilir) */

BearSSL::WiFiClientSecure tls;
HTTPClient http;

String rows, events, runs;       /* JSON dizilerinin içleri (virgülle) */
uint16_t n_rows = 0;
uint32_t t_send = 0, n_sent = 0, n_fail = 0, n_drop = 0, n_lines = 0;
char line[200];
uint16_t line_n = 0;

/* ---------------------------------------------------------------- JSON yardımcıları */
static void jstr(String &s, const char *v)          /* tırnaklı, kaçışlı metin */
{
  s += '"';
  for (const char *p = v; *p; p++) {
    if (*p == '"' || *p == '\\') s += '\\';
    if ((uint8_t)*p >= 0x20) s += *p;
  }
  s += '"';
}
static void sep(String &s) { if (s.length()) s += ','; }

/* virgülle ayrılmış alanlar: f[0..n) (satır yerinde bölünür); kalan = son alandan itibaren tüm metin */
static uint8_t split(char *s, char **f, uint8_t max)
{
  uint8_t n = 0;
  f[n++] = s;
  for (char *p = s; *p && n < max; p++)
    if (*p == ',') { *p = 0; f[n++] = p + 1; }
  return n;
}
static bool num(const char *s) { if (*s == '-') s++; if (!*s) return false; for (; *s; s++) if ((*s < '0' || *s > '9') && *s != '.') return false; return true; }

/* ---------------------------------------------------------------- satır ayrıştırma */
static void handle(char *s)
{
  char *f[12];
  n_lines++;
  if (s[0] == 'D' && s[1] == ',') {
    const uint8_t n = split(s, f, 11);
    if (n != 10) return;
    for (uint8_t k = 1; k < 10; k++) if (!num(f[k])) return;
    if (n_rows >= MAX_ROWS) { n_drop++; return; }
    sep(rows);
    rows += "{\"t\":"; rows += f[1]; rows += ",\"run\":"; rows += f[2]; rows += ",\"soc\":"; rows += f[3];
    rows += ",\"soh\":"; rows += f[4]; rows += ",\"T\":"; rows += f[5]; rows += ",\"I\":"; rows += f[6];
    rows += ",\"st\":"; rows += f[7]; rows += ",\"tec\":"; rows += f[8]; rows += ",\"rec\":"; rows += f[9]; rows += '}';
    n_rows++;
  } else if (s[0] == 'E' && s[1] == ',') {
    const uint8_t n = split(s, f, 5);           /* detay virgül içerebilir: 5. alan kalanı tutar */
    if (n < 4 || !num(f[1]) || !num(f[2])) return;
    sep(events);
    events += "{\"t\":"; events += f[1]; events += ",\"run\":"; events += f[2]; events += ",\"code\":";
    jstr(events, f[3]);  events += ",\"detail\":";  jstr(events, n > 4 ? f[4] : "");  events += '}';
  } else if (s[0] == 'R' && s[1] == 'S' && s[2] == ',') {
    const uint8_t n = split(s, f, 3);
    if (n < 3 || !num(f[1])) return;
    sep(runs);
    runs += "{\"op\":\"start\",\"id\":"; runs += f[1]; runs += ",\"name\":"; jstr(runs, f[2]); runs += '}';
  } else if (s[0] == 'R' && s[1] == 'E' && s[2] == ',') {
    const uint8_t n = split(s, f, 4);
    if (n < 3 || !num(f[1])) return;
    sep(runs);
    runs += "{\"op\":\"end\",\"id\":"; runs += f[1]; runs += ",\"result\":"; jstr(runs, f[2]);
    runs += ",\"summary\":"; jstr(runs, n > 3 ? f[3] : ""); runs += '}';
  }
}

static void pump_uart()
{
  while (Serial.available()) {
    const char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') { line[line_n] = 0; if (line_n) handle(line); line_n = 0; }
    else if (line_n < sizeof(line) - 1) line[line_n++] = c;
  }
}

/* ---------------------------------------------------------------- gönderim */
static void send_batch()
{
  if (!rows.length() && !events.length() && !runs.length()) return;
  String body = "{\"rows\":[" + rows + "],\"events\":[" + events + "],\"runs\":[" + runs + "]}";
  rows = "";  events = "";  runs = "";
  const uint16_t nr = n_rows;
  n_rows = 0;
  if (WiFi.status() != WL_CONNECTED) { n_fail++; return; }   /* çevrimdışı: yığın atılır (RAM küçük) */
  http.setReuse(true);
  if (!http.begin(tls, API_URL)) { n_fail++; return; }
  http.addHeader("Content-Type", "application/json");
  http.addHeader("X-API-Key", API_KEY);
  const int code = http.POST(body);
  if (code == 200) n_sent += nr; else n_fail++;
  Serial.printf("[esp] POST %d rows %u sent %lu fail %lu drop %lu lines %lu heap %u\r\n",
                code, nr, n_sent, n_fail, n_drop, n_lines, ESP.getFreeHeap());
}

void setup()
{
  Serial.setRxBufferSize(8192);   /* POST sürerken (~0,3-2 s) gelen satırlar burada bekler */
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  tls.setInsecure();              /* sertifika doğrulanmaz (gösterim; anahtar yine TLS ile şifreli) */
  tls.setBufferSizes(16384, 512); /* sunucu MFLN desteklemiyor: alma tamponu 16 KB olmalı */
  http.setTimeout(5000);
  Serial.println(F("\r\n[esp] esp_gateway: connecting Wi-Fi"));
}

void loop()
{
  static bool was_up = false;
  pump_uart();
  const bool up = WiFi.status() == WL_CONNECTED;
  if (up != was_up) {
    Serial.printf("[esp] Wi-Fi %s %s\r\n", up ? "up" : "down", up ? WiFi.localIP().toString().c_str() : "");
    was_up = up;
  }
  if (millis() - t_send >= SEND_MS) {
    t_send = millis();
    send_batch();
  }
  yield();
}
