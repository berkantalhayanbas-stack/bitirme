# Uzaktan izleme (telemetri → gateway → web paneli)

```
STM32 (LPUART1, COM9) --UART--> gateway.py (PC) --HTTPS POST--> server/api.php --> MySQL
                                                         tarayıcı <-- index.html + data.php
```

## Kart tarafı
- **Canlı ECU:** `python tools/fw_build.py --app 4 --define BMS_SAFETY=1 --define BMS_OUTPUTS=1 --define BMS_TELEMETRY=1 --crc-ref --flash`
  - 10 Hz `D,<t_ms>,0,<soc>,<soh>,<Tc>,<I_allow>,<state>,<tec>,<rec>` satırları gelir; durum kodları 1 NORMAL, 2 DERATE, 3 LIMP-HOME, 4 SAFE-STOP.
  - Değişimlerde `E,<t_ms>,0,<code>,<detail>` olayları gelir: BOOT, durum adları, THERMAL_DERATE, COOLING_ALARM, SOP_*, DATASHEET_T_LIMIT, SENSOR_FAULT, SOC_EMPTY, CAN_TIMEOUT, CONTACTOR_OPEN/CLOSE.
  - `BMS_TELEMETRY=2` ayrıca geniş `DX,...` satırını ekler (`tools/live_panel.py` ve gateway). Gateway DX'i aynı adımın D satırına ekler (`t_ms` eşleşmesi); panel ölçülen akımı, paket gerilimini, min/maks grup gerilimini, yüzey ve ölçülen sıcaklığı, şarj I_allow'u, ψ'yi ve neden bitlerini ancak bununla gösterir (=1'de bu alanlar boş, panel uyarı yazar). Panel için önerilen: `--define BMS_TELEMETRY=2`.
  - Sunucu: geniş alanlar `telemetry` tablosunun ek kolonlarıdır (`schema.sql`); eski veritabanına `config.php` `db()` ilk istekte `ALTER TABLE` ile ekler (`TELEMETRY_X`). `data.php?q=linkhist` yığın başına RTT verir (panel grafiği).
- **Senaryo merkezi (Monte Carlo):** `python tools/fw_build.py --app 8 --define MC_N=200 --define MC_AUTO=0 --flash`
  - Gateway penceresine `RUN <id>`, `ALL [n]` ya da `STOP` yazılır.
  - Her koşu `RS`/`RE` satırı üretir. İzli koşular (her `MC_TRACE_EVERY`. koşu ve `RUN`) 1 Hz `D` satırları gönderir; `run` alanı koşu numarasıdır.

## Mod butonları (panelde "Button tests")
```
python tools/web/gateway.py COM9 --emu COM10
```
- Gateway twin_emu'nun portunu da okur: her basışta Uno `B,<olay>,<1|0>` yazar → "BUTTON" olayı; gateway ECU'nun D satırlarındaki durumu izleyip beklenen moda geçiş süresini "REACTION" olayı olarak ekler (PC saati, 0,1 s çözünürlük; NORMAL servisinde önce ECU'nun yeniden başlaması beklenir; ECU zaten o moddaysa "already in ..."). ECU senaryoyu bilmez, CAN değişmez.
- Panel: Overview'da "Button tests" kartı (`data.php?q=tests`).
- COM10'u açmak Uno'yu sıfırlar (~2 s susar; ECU SAFE'e düşer, NORMAL butonuyla dönülür). Klavyeden `emu <komut>` Uno'ya gider (ör. `emu k 2`). `--emu` açıkken `demo_run.py` ve Arduino IDE (yükleme, Serial Monitor) COM10'u kullanamaz: önce gateway'i kapat.
- `--emu-init "y 88"`: Uno açılınca gönderilecek twin_emu komutları (`;` ile ayrılır). Örnek: ikizi gerçek paketin tahmini SOH'una (%88, `C:itirmeerieal_pack_age.py`) ayarlamak; COM10 her açılışta Uno'yu sıfırladığı için `y` ayarı başka türlü kaybolur.

## PC: gateway
```
pip install pyserial requests
set BMS_API_KEY=<config.php'deki API_KEY>
python tools/web/gateway.py COM9
```
- Anahtar ortam değişkeninden ya da `tools/web/api_key.txt` dosyasından okunur. Bu dosya git dışıdır.
- Adres `BMS_URL` ile değiştirilebilir; varsayılan `https://hizlipdf.net/bitirmeprj/api.php`.
- İnternet yoksa gönderilecekler `build/web/spool.jsonl` dosyasında kuyruğa alınır. Ham satırlar `build/web/log_*.txt` dosyasına yazılır.

## Bağlantı kalitesi (gecikme, kayıp)
Ölçüm gateway'de yapılır; ECU yazılımı değişmez (D satırındaki `t_ms` yeterli).
- **UART kaybı:** aynı run'da ardışık `t_ms` farkı, periyodun (son 21 aralığın medyanı) 1,5 katını aşarsa aradaki satırlar eksik sayılır. 1 s'den uzun boşluk kayıp sayılmaz (ECU duraklaması: ECU adımı ölçüm çiftiyle atar; EMU sıfırlanınca ya da hat kopunca satır da gelmez). İlk sürüm bunu da sayıyordu: senaryo denemesinde (COM10 açılışı Uno'yu ~2 s susturur) %1,2 "kayıp" görünmüştü. Periyot önce "görülen en küçük aralık"tı: Uno yeniden başlayınca ECU adımının fazı kayıp tek bir 38 ms aralık bırakınca sonraki her 100 ms aralığı 2 kayıp satır sayılıyordu (2026-11-21, kartta). `D,` ile başlayıp ayrıştırılamayan satır bozuk sayılır.
- **UART titreşimi** (yalnız canlı mod, run 0): PC alış zamanı ile `t_ms` farkının son 5 s'deki en küçüğünden sapması. İlk sürüm oturumun en küçüğünü alıyordu; ECU ile PC saati arasında ~0,5 ms/s kayma olduğu için (2026-11-21, kartta) değer zamanla doğrusal büyüyüp 20 dakikada ~535 ms'ye çıktı, bu titreşim değil saat kaymasıydı.
- **Upload RTT** ve **uçtan uca gecikme** (satırın UART'tan alınması → sunucu onayı): ikisi de PC saatiyle ölçülür, iki saat eşlemesi gerekmez. Uçtan uca gecikmenin ~1 s'si 1 s'lik yığınlamadan gelir (tasarım gereği).
- Her yığına `meta` eklenir. `api.php` bunu `link` tablosuna yazar; `seq` en son kabul edilenden büyük değilse yığını **tekrar oynatma** sayıp 409 ile reddeder. `meta`'sız istemci (`arduino/esp_gateway`) yine kabul edilir.
- Çıktılar: `build/web/link_*.csv` (yığın başına satır), Ctrl+C ya da port kapanınca özet (p50/p95/max), panelde "Link quality (last 5 min)" kartı (`data.php?q=link&min=5`).
- Yerel deneme (2026-11-21, sahte UART + mock sunucu, 200 satır, bilinen 3 kayıp + 1 bozuk): sayaçlar birebir tuttu; 4 s sunucu kesintisinde 197 satırın hepsi kuyruktan yeniden gönderildi, hiçbiri reddedilmedi; aynı `seq` ile tekrar → 409.
- Gerçek sunucu (2026-11-21, hizlipdf.net, sahte UART 10 Hz, 300 satır, bilinen 3 kayıp + 1 bozuk): 296/296 satır ulaştı, kayıp 4 (bozuk satır da `t_ms` boşluğu açar) = %1,333, bozuk 1; upload RTT p50/p95 98/157 ms, uçtan uca 1154/1248 ms, titreşim p95 1 ms. Tekrar `seq` → 409, meta'sız → 200, yanlış anahtar → 401. `oneway_p50` -959 ms çıktı: PC saati NTP'siz (Local CMOS) ve sunucudan ~1 s ileride; bu yüzden esas ölçüt RTT ve uçtan uca gecikmedir.

## Panel
- **Overview** (varsayılan): sürücü ekranı gibi az ve büyük. Durum sade dille ("Power limited — because the battery is warm"), şarj, güç ölçeri (açık şerit = BMS'in izin verdiği, koyu = kullanılan; sol şarj, sağ sürüş; ölçek veri sayfası nominali 87,3 / 40,7 A), çekirdek sıcaklığı 45 / 55 / 60 °C bölgeleriyle, hücre dengesi, son 1 dk güç eğrisi, son uyarılar, bağlantı özeti.
- **Engineering**: ham değerler, durum şeridi, akım / sıcaklık / gerilim / SOC grafikleri (1 / 5 dk), CSV, bağlantı ayrıntısı, senaryo merkezi.

## Sunucu (paylaşımlı hosting, PHP + SQLite)
Veritabanı kurulumu gerekmez. PHP ilk istekte SQLite dosyasını kendisi oluşturur: tercihen web kökünün dışında `~/bitirmeprj_data/bms.sqlite`, olmazsa `data/` alt klasöründe, dışarıya kapalı. Tek gizli ayar `config.php` içindeki `API_KEY`'dir ve gateway'deki anahtarla aynı olmalıdır.
1. Paketi hazırla: `python tools/web/make_upload.py` → `build/web/bitirmeprj_upload.zip` (anahtar yoksa üretir, `config.php`'yi `config.sample.php`'den yazar; ikisi de git dışıdır).
2. cPanel > File Manager > `public_html/bitirmeprj/` > Upload > zip'i seç > zip'e sağ tık > Extract > zip'i sil.
3. Kontrol: `https://hizlipdf.net/bitirmeprj/data.php?q=ping` -> `{"ok":true,...}`
   - `.htaccess`, sitenin `.php` uzantısını kaldıran 301 yönlendirmesini bu klasörde kapatır (301, POST'u GET'e çevirir).
   - `config.php`, `config.sample.php` ve `schema.sql` dosyalarına dışarıdan erişim kapalıdır.

## Yerel deneme (hosting olmadan, sunum yedeği)
```
python tools/web/mock_server.py            # http://127.0.0.1:8080/ , anahtar "test", build/web/mock.sqlite
set BMS_URL=http://127.0.0.1:8080/api.php
set BMS_API_KEY=test
python tools/web/gateway.py COM9
```
`mock_server.py`, `api.php` ve `data.php` ile aynı alanları ve aynı işlem sırasını kullanır. Yeniden başlayan bir senaryonun eski izi silinir.
