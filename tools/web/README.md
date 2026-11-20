# Uzaktan izleme (telemetri → gateway → web paneli)

```
STM32 (LPUART1, COM9) --UART--> gateway.py (PC) --HTTPS POST--> server/api.php --> MySQL
                                                         tarayıcı <-- index.html + data.php
```

## Kart tarafı
- **Canlı ECU:** `python tools/fw_build.py --app 4 --define BMS_SAFETY=1 --define BMS_OUTPUTS=1 --define BMS_TELEMETRY=1 --crc-ref --flash`
  - 10 Hz `D,<t_ms>,0,<soc>,<soh>,<Tc>,<I_allow>,<state>,<tec>,<rec>` satırları gelir; durum kodları 1 NORMAL, 2 DERATE, 3 LIMP-HOME, 4 SAFE-STOP.
  - Değişimlerde `E,<t_ms>,0,<code>,<detail>` olayları gelir: BOOT, durum adları, THERMAL_DERATE, COOLING_ALARM, SOP_*, DATASHEET_T_LIMIT, SENSOR_FAULT, SOC_EMPTY, CAN_TIMEOUT, CONTACTOR_OPEN/CLOSE.
  - `BMS_TELEMETRY=2` ayrıca `tools/live_panel.py` için geniş `DX,...` satırını ekler.
- **Senaryo merkezi (Monte Carlo):** `python tools/fw_build.py --app 8 --define MC_N=200 --define MC_AUTO=0 --flash`
  - Gateway penceresine `RUN <id>`, `ALL [n]` ya da `STOP` yazılır.
  - Her koşu `RS`/`RE` satırı üretir. İzli koşular (her `MC_TRACE_EVERY`. koşu ve `RUN`) 1 Hz `D` satırları gönderir; `run` alanı koşu numarasıdır.

## PC: gateway
```
pip install pyserial requests
set BMS_API_KEY=<config.php'deki API_KEY>
python tools/web/gateway.py COM9
```
- Anahtar ortam değişkeninden ya da `tools/web/api_key.txt` dosyasından okunur. Bu dosya git dışıdır.
- Adres `BMS_URL` ile değiştirilebilir; varsayılan `https://hizlipdf.net/bitirmeprj/api.php`.
- İnternet yoksa gönderilecekler `build/web/spool.jsonl` dosyasında kuyruğa alınır. Ham satırlar `build/web/log_*.txt` dosyasına yazılır.

## Sunucu (paylaşımlı hosting, PHP + MySQL)
1. `server/schema.sql` dosyasını phpMyAdmin'de çalıştır.
2. `server/config.sample.php` dosyasını `config.php` adıyla kopyala. Veritabanı bilgilerini ve uzun, rastgele bir `API_KEY` gir. **`config.php` git dışıdır, depoya koyma.**
3. `server/` içeriğini FTP ile yükle: `.htaccess`, `api.php`, `data.php`, `index.html`, `config.php`.
   - `.htaccess`, `config.php`, `config.sample.php` ve `schema.sql` dosyalarına dışarıdan erişimi kapatır.

## Yerel deneme (hosting olmadan, sunum yedeği)
```
python tools/web/mock_server.py            # http://127.0.0.1:8080/ , anahtar "test", build/web/mock.sqlite
set BMS_URL=http://127.0.0.1:8080/api.php
set BMS_API_KEY=test
python tools/web/gateway.py COM9
```
`mock_server.py`, `api.php` ve `data.php` ile aynı alanları ve aynı işlem sırasını kullanır. Yeniden başlayan bir senaryonun eski izi silinir.
