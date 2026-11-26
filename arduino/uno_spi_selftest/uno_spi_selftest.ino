/* uno_spi_selftest: MCP2515 olmadan Uno'nun SPI pinlerini sınar. Seri Monitor 115200, her 1 s bir satır.
 *   Köprü 1: D11 (MOSI) - D12 (MISO)  -> SPI geri döngüsü: gönderilen bayt geri okunmalı
 *   Köprü 2: D10 (CS)   - A0          -> D10 çıkışı A0'dan okunur
 *   Köprü 3: D13 (SCK)  - A0          -> D13 çıkışı A0'dan okunur
 *   Köprü 4: D11 (MOSI) - A0          -> D11 çıkışı A0'dan okunur
 *   Köprü 5: D12 (MISO) - A0          -> A0'ın sürdüğü seviye D12'den okunur
 * Köprüleri tek tek tak (aynı anda yalnız biri A0'a). Sonuç satırları İngilizce (log kuralı). */
#include <SPI.h>

static bool toggles(uint8_t pin)
{   /* pini çıkış olarak sür, A0'dan oku */
    pinMode(pin, OUTPUT);
    digitalWrite(pin, HIGH);  delayMicroseconds(50);
    const int h = digitalRead(A0);
    digitalWrite(pin, LOW);   delayMicroseconds(50);
    const int l = digitalRead(A0);
    return h == 1 && l == 0;
}

void setup()
{
    Serial.begin(115200);
    pinMode(A0, INPUT);
    Serial.println(F("=== uno_spi_selftest: bridge D11-D12, then D10-A0, then D13-A0 (one at a time)"));
}

void loop()
{
    /* 1) SPI geri döngüsü */
    SPI.begin();
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    const uint8_t r1 = SPI.transfer(0x55), r2 = SPI.transfer(0xAA);
    SPI.endTransaction();
    SPI.end();
    Serial.print(F("MOSI->MISO 55/AA -> ")); Serial.print(r1, HEX); Serial.print('/'); Serial.print(r2, HEX);
    Serial.print(r1 == 0x55 && r2 == 0xAA ? F(" OK") : F(" no"));
    /* 2) ve 3) D10 / D13 -> A0 */
    Serial.print(F("  |  D10->A0 ")); Serial.print(toggles(10) ? F("OK") : F("no"));
    Serial.print(F("  |  D13->A0 ")); Serial.print(toggles(13) ? F("OK") : F("no"));
    Serial.print(F("  |  D11->A0 ")); Serial.print(toggles(11) ? F("OK") : F("no"));
    /* Köprü 4: A0 - D12. A0 çıkış, D12 giriş: D12'nin okuyabildiği sınanır */
    pinMode(12, INPUT);
    pinMode(A0, OUTPUT);
    digitalWrite(A0, HIGH);  delayMicroseconds(50);
    const int h = digitalRead(12);
    digitalWrite(A0, LOW);   delayMicroseconds(50);
    const int l = digitalRead(12);
    pinMode(A0, INPUT);
    Serial.print(F("  |  A0->D12 ")); Serial.println(h == 1 && l == 0 ? F("OK") : F("no"));
    delay(1000);
}
