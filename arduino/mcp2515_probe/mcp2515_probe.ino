/* mcp2515_probe: MCP2515 SPI bağlantı tanısı (Arduino Uno). Kütüphane kullanmaz, register'ları doğrudan okur.
 * Bağlantı twin_emu ile aynı: CS = D10, SI (MOSI) = D11, SO (MISO) = D12, SCK = D13, INT = D2, VCC 5 V.
 * Seri Monitor 115200. Her 2 s'de bir: reset sonrası CANSTAT (beklenen 0x80 = config modu), CNF1 yaz/oku
 * testi ve bulguya göre olası arıza. Sonuç satırları İngilizce (log kuralı). */
#include <SPI.h>

const uint8_t CS = 10;
const uint8_t INT_PIN = 2;

static uint8_t rd(uint8_t a)
{
  digitalWrite(CS, LOW);
  SPI.transfer(0x03);            /* READ */
  SPI.transfer(a);
  const uint8_t v = SPI.transfer(0x00);
  digitalWrite(CS, HIGH);
  return v;
}

static void wr(uint8_t a, uint8_t v)
{
  digitalWrite(CS, LOW);
  SPI.transfer(0x02);            /* WRITE */
  SPI.transfer(a);
  SPI.transfer(v);
  digitalWrite(CS, HIGH);
}

void setup()
{
  Serial.begin(115200);
  pinMode(CS, OUTPUT);
  digitalWrite(CS, HIGH);
  pinMode(INT_PIN, INPUT_PULLUP);
  SPI.begin();
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));   /* yavaş: uzun kablo affedilsin */
  Serial.println(F("=== mcp2515_probe: CS D10, SI D11, SO D12, SCK D13, INT D2"));
}

void loop()
{
  /* SO (D12) hattı testi: CS yüksekken çip SO'yu bırakır (yüksek empedans). İç pull-up açıkken 1 okunmalı;
   * 0 ise hattı bir şey aşağı çekiyor. Sonra SPI okumaları da pull-up açıkken yapılır: çip hiç sürmüyorsa
   * 0xFF (CS / SCK / SI çipe ulaşmıyor ya da çip bozuk), 0x00 kalırsa SO gerçekten aşağıda tutuluyor. */
  digitalWrite(CS, HIGH);
  pinMode(12, INPUT_PULLUP);
  delay(2);
  const int so_idle = digitalRead(12);
  Serial.print(F("SO idle with pull-up: ")); Serial.print(so_idle);
  Serial.println(so_idle ? F(" (line free)") : F(" (something pulls D12 low!)"));
  digitalWrite(CS, LOW);
  SPI.transfer(0xC0);            /* RESET */
  digitalWrite(CS, HIGH);
  delay(10);
  const uint8_t canstat = rd(0x0E), canctrl = rd(0x0F);
  wr(0x2A, 0x55);                /* CNF1 (yalnız config modunda yazılabilir) */
  const uint8_t a = rd(0x2A);
  wr(0x2A, 0xAA);
  const uint8_t b = rd(0x2A);
  wr(0x2A, 0x00);

  Serial.print(F("CANSTAT 0x")); Serial.print(canstat, HEX);
  Serial.print(F("  CANCTRL 0x")); Serial.print(canctrl, HEX);
  Serial.print(F("  CNF1 write 55/AA -> read ")); Serial.print(a, HEX); Serial.print('/'); Serial.print(b, HEX);
  Serial.print(F("  INT pin ")); Serial.print(digitalRead(INT_PIN));
  Serial.print(F("  -> "));
  if (canstat == 0x80 && a == 0x55 && b == 0xAA)
    Serial.println(F("OK: SPI works, chip in config mode (problem elsewhere: crystal / bitrate)"));
  else if (canstat == 0xFF && a == 0xFF && b == 0xFF)
    Serial.println(F("all 0xFF: chip never drives SO -> CS (D10), SCK (D13) or SI (D11) not reaching the chip, or SO (D12) open, or dead chip"));
  else if (canstat == 0x00 && a == 0x00 && b == 0x00)
    Serial.println(F("all 0x00: SO (MISO, D12) shorted to GND, or no clock (SCK D13), or chip without power"));
  else if (a != 0x55 || b != 0xAA)
    Serial.println(F("reads vary but write test fails: SI (MOSI, D11) open/swapped with SO, or noisy SCK"));
  else
    Serial.println(F("SPI answers but chip did not reset into config mode: check crystal / power"));
  delay(2000);
}
