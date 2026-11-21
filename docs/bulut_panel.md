# Bulut paneli: tasarım ve kaynaklar

Durum: plan (2026-11-20). Kod yok.
Kaynak doğrulama durumu her kaynağın yanında: **[TM]** tam metin okundu, **[Ö]** yalnız özet/arama
sonucu (yayıncı otomatik erişimi engelledi; tezde atıftan önce tam metinden doğrulanmalı).

## Mimari
```
twin_emu (Uno) --CAN--> STM32 ECU --LPUART1 "D,..." (PA2)--> ESP-01 --Wi-Fi/HTTPS--> PHP (ingest.php) --> MySQL
                                                                                       panel (index.html) <-- api.php
```
- Kontrol araçta, analiz bulutta: kapalı döngü (ECU <-> ikiz) internete bağlanmaz.
- **ESP-01 isteğe bağlı bir eklentidir.** Çıkarılırsa ya da Wi-Fi koparsa ECU aynen çalışır (ölçüm,
  EKF, stres endeksi, NORMAL/DERATE/LIMP/SAFE, kontaktör); yalnız uzak panel güncellenmez.
  Yerel izleme COM9 + `tools/live_panel.py` ile sürer. ESP-01 yalnız dinler (TX bağlı değil),
  ECU'ya geri etki edemez.
- Tek yön: web'den ECU'ya komut yolu yok; yazma yalnız gizli token ile. Gotion [9] da güvenlik
  kritik ESS'leri "tek yönlü yerel ağ geçidi" ile bağlıyor.
- Veri hızı: ECU 10 Hz üretir, ESP-01 1 s'de bir 10'lu paket gönderir; ham veri 30 gün, sonrası dakikalık ortalama.
- Araçta ESP-01 telefon hotspot'una bağlanır; gerçek üründe BMS'e bağlı 4G/5G IoT birimi [9]
  (prototip olarak anlatılır).

## Panel sekmeleri
1. **Canlı**: SOC, akım, sıcaklık (Ts ölçülen, Tc tahmini), mod rozeti (NORMAL/DERATE/LIMP/SAFE),
   kontaktör, I_allow, 19 grup gerilim çubuğu + dengeleme bitleri. Eşik aşımında uyarı [9].
2. **Stres endeksi**: toplam Psi göstergesi, alt endeks çubukları (Psi_T, Psi_C, Psi_Tdot, Psi_I, Psi_H,
   Psi_B, Psi_U), baskın kriter etiketi ("neden kısıldı?"), Psi ve I_allow zaman serisi.
   Tasarım onay bekliyor (çok kriterli Psi); panel ECU'nun D satırında gelen Psi_i değerlerini gösterir.
3. **Geçmiş**: MySQL zaman serileri, olay günlüğü (mod geçişleri, neden bitleri, hatalar).
4. **Simülasyon**: `bms/common` + `twin_model.c` WebAssembly'de; 5 stres senaryosu seçilir,
   aynı göstergeler (stres endeksi dahil). Gerçek/simüle veri açıkça etiketli.
5. **Karşılaştırma**: kayıtlı gerçek veri ile aynı girdiyle koşturulan simülasyon üst üste + artık
   (can_replay PIL/SIL karşılaştırmasının web karşılığı; DT anomali/SOH fikrinin görseli).

## Tasarım ilkeleri ve dayanağı
| İlke | Dayanak |
|---|---|
| Ağır hesap/uzun dönem analiz bulutta, güvenlik kararı araçta; bulutta eğitilen model araca gömülür | [1] TM, [3] TM, [5] TM, [6] TM, [9] TM, [10] TM, [2] [4] Ö |
| Tek yönlü bağlantı, token, HTTPS | [1] TM, [6] TM, [9] TM, [5] TM |
| Bağlantı yokken araç tam çalışır (ESP isteğe bağlı) | [5] TM, [6] TM, [3] TM |
| Uçtan uca gecikme ve paket kaybı ölçülüp raporlanır (uygulandı: `tools/web` link ölçümü, yöntem `tools/web/README.md`) | [11] TM (iddia var, yöntem yok) |
| Tekrar oynatma reddi (artan `seq`, 409) | [6] TM (bütünlük saldırıları) |
| Web tabanlı panel, eşik alarmı, geçmiş veri, zaman serisi veritabanı | [1] TM, [9] TM |
| Seyreltilmiş kayıt (0,2-1 Hz) ve sıkıştırma | [7], [8] Ö |
| İkiz/filo karşılaştırmasıyla anomali ve SOH | [1] TM, [9] TM, [10] TM |
| Gerçek ve simülasyon aynı arayüzde ama ayrı etiketli | [11]-[14] Ö |
| Tarayıcıda gerçek ECU kodu (WebAssembly) | taranan BMS literatüründe görülmedi: olası özgünlük, hedefli arama gerekli |

## Kaynaklar

### Tam metni okunanlar
9. **X. Li, D. Jauernig, M. Gao, T. Jones**, "Battery Cloud with Advanced Algorithms," arXiv:2203.03737v2 [eess.SY], 2022 (Gotion Inc.; Elsevier'e gönderilmiş ön baskı). [PDF](https://export.arxiv.org/pdf/2203.03737) **[TM]**
   - EV paketinde BMS + kablosuz IoT birimi, veriyi 4G/5G ile buluta gönderir; şebeke ESS'leri siber güvenlik gereği **tek yönlü yerel ağ geçidi** ile bağlanır.
   - Veritabanı: Hadoop veya zaman serisi veritabanı (InfluxDB, Timescale, Prometheus); küçük Ar-Ge veritabanı için TSDB önerir.
   - Görselleştirme: web tabanlı (Grafana, Datadog, Kibana), gerçek zamanlı, etkileşimli; eşik aşımı alarmı; panelden ML hattı tetikleme.
   - Bulut verisiyle eğitilen YSA şarjda paket SOC'u tahmin eder, **araç BMS'ine gömülür**, HIL + araçta test: RMSE %1,9 (hedef ort. < %3, maks. < %5).
   - SOH: DVA/ICA öznitelikleri.
   - Termal anomali: kümeleme tabanlı, tek bir EV vaka çalışmasında araç BMS'inden **~90 dakika önce** (14:15'e karşı 15:45; BMS 55 °C üstünde yakaladı). Genel bir başarım iddiası değil, tek vaka.
10. **T. Zhu, H. Wang, Z. Cao, S. See, Y. Wen**, "Towards Intelligent Systems for Battery Management: A Five-Tier Digital Twin Architecture," arXiv:2509.02366v2, 2026 (v1 2025, başlığı "...Five-Tier Digital Twin Framework", 4. yazar J. Xi). Nanyang Teknoloji Üniv. + NVIDIA. **[TM]**
   - 5 katman: geometrik (3D model) -> betimleyici (canlı telemetri 3D modele işlenir, sıcaklık dağılımı görselleştirilir) -> fizik bilgili tahmin -> öneri/optimizasyon -> otonom kontrol.
   - Ağır modelleme bulut/HPC'de, sonuç kenar BMS'e geri senkronize edilir.
   - v2: gerilim tahmin hatası %0,92, sıcaklık %0,18, SOH MAPE %1,09 (v1 özetinde farklı: < %1,57 / %0,39 / < %3; sürüm belirtilerek atıf verilmeli).
   - Görselleştirme NVIDIA Omniverse 3D sahnesiyle: bizim web panelinden çok daha ağır; "betimleyici katman" kavramı panelimizi konumlandırmak için kullanılabilir.
16. **M. Aydın, İ. Gürbüz**, "IoT based a low cost battery monitoring system using ESP8266 and Arduino IoT cloud platform," *Int. J. Automotive Engineering and Technologies* (IJAET), 2024 (yayın 31.12.2024). DOI 10.18245/ijaet.1553298. [PDF](https://dergipark.org.tr/en/download/article-file/4228618) **[TM]**
   - 6S modül (NCR18650PF 3300 mAh), 500 şarj-deşarj çevrimi; kapasite düşüşü ölçüldü, SOC/SOH hesaplanıp Arduino IoT Cloud mobil uygulamasında gösterildi.
   - Türk ve otomotiv odaklı yakın çalışma: karşılaştırma tablosunda iyi bir referans.
17. **J. Lianda, A. Hadi, A. Adam, H. Amri, G. Eviani**, "IoT Based Battery Capacity Monitoring System on Solar Panels in Electric Vehicle," Proc. iCAST-ES 2023, *Advances in Engineering Research* 230, Atlantis Press. DOI 10.2991/978-94-6463-364-1_45. [PDF](https://www.atlantis-press.com/article/125998353.pdf) **[TM]**
   - NodeMCU ESP8266 + Blynk + OLED; 12 V / 7 Ah kurşun-asit akü, 50 Wp panel; izleme hatası ort. %0,97.
   - Blynk'ten şarjı kapatma (web'den donanıma komut): bizim tek yön kararımızın tersi; karşılaştırmada kullanılabilir.
18. **S. S. Madani, Y. Shabeer, M. Fowler, S. Panchal, C. Ziebert, H. Chaoui, F. Allard**, "Digital twin technologies for battery systems: advancements, applications, and future directions," *Front. Batteries Electrochem.* 5, 1764210, 2026. DOI 10.3389/fbael.2026.1764210. [bağlantı](https://www.frontiersin.org/journals/batteries-and-electrochemistry/articles/10.3389/fbael.2026.1764210/full) **[TM kısmi]**
   - Okunan bölümler: DT için bulut-kenar iş birliği gerekli; çift yönlü veri akışı saldırı yüzeyini büyütür (bizim tek yön kararını destekler); araç üstü hesap yükü bir zorluk.
   - Bulut/araç iş bölümü tablosu (Tablo 6) okunamadı.

1. **W. Li, M. Rentemeister, J. Badeda, D. Jöst, D. Schulte, D. U. Sauer**, "Digital twin for battery systems: Cloud battery management system with online state-of-charge and state-of-health estimation," *J. Energy Storage* 30, 101557, 2020. DOI 10.1016/j.est.2020.101557. CC BY-NC-ND 4.0. RWTH Aachen ISEA. **[TM]** Temel referans.
   - 6 alt sistem: batarya -> BMS-Slave (LTC6804 + LPC11C14 Cortex-M0, < 1,2 mV) -> IoT birimi (Raspberry Pi; CAN'dan okur, Python ile fiziksel değere çevirir, TCP/IP + MQTT ile gönderir) -> bulut (Almanya'da veritabanı, DDoS korumalı ağ geçidi) -> API (Matlab/Python algoritmaları) -> tarayıcı tabanlı arayüz.
   - **Anahtar ilke (Bölüm 2.3):** her an gereken işlevler sistem güvenliği için yerelde de çalışmalı; bulutta bunların daha doğru ama ağır sürümü çalışır; geçmiş veri isteyen işlevler yalnız bulutta. Bizim "karar araçta, analiz bulutta" ayrımımızın doğrudan dayanağı.
   - Arayüz: gerçek zamanlı ölçüm + iç durumlar (SOC, SOH), geçmiş veri, ham ve sanal veri noktalarına alarm; uç noktalar yalnız HTTPS + kullanıcı doğrulaması.
   - Algoritmalar: genişletilmiş Thevenin modeli; SOC için uyarlamalı genişletilmiş H-sonsuz süzgeç (AEHF), SOH için parçacık sürüsü optimizasyonu (PSO) belirli periyotlarda geçmiş veriyle (kapasite ve güç kaybı birlikte).
   - Sonuçlar: SOC MAE kurşun-asit %0,06, Li-iyon (Samsung INR18650-35E, gerçek sürüş çevrimi, %90 DOD) %0,49, gerilim MAE 0,01 V; SOH 540 çevrimde kapasite MAE %0,74, güç (direnç) %1,70; gürültü altında %0,89 / %2,3.
   - **Dikkat:** sahada doğrulama yalnız Danimarka'daki bir UPS (4 x 12 V 92 Ah AGM kurşun-asit, float şarj), Almanya'dan izlendi. "Mobil" doğrulama laboratuvarda test cihazı + sürüş çevrimi, gerçek araç değil. Önceki notlardaki "sahada mobil prototip" ifadesi yanlıştı.
   - Sınırlılık: PSO'nun hesap yükü yüksek (bu yüzden bulutta). Gelecek çalışma: bulut verisiyle ML ömür tahmini.

3. **S. K. Mulpuri, B. Sah, P. Kumar**, "An intelligent battery management system (BMS) with end-edge-cloud connectivity – a perspective," *Sustainable Energy & Fuels* 9, 1142-1159, 2025. DOI 10.1039/d4se01238k. **[TM]**
   - İşlevleri ikiye ayırır: **zaman kritik** (temel durum kestirimi, hücre izleme ve dengeleme, termal izleme ve koruma) en düşük gecikmeyle araçta; zaman kritik olmayanlar (prognoz, yaşlanma, filo) kenar/bulutta. Araca ağır hesap eklemek gerçek zamanlı tepkiye gecikme ekler.
   - Önerilen IBMS: uç-kenar-bulut çok katmanlı paralel hesap, dijital ikiz, blokzincir, yeniden yapılandırılabilir paket (görüş makalesi, deney yok).
   - Bizim için: ECU'daki EKF + stres endeksi + koruma = "zaman kritik" katman; panel ve analiz = bulut katmanı. Sınıflandırmayı tezde doğrudan kullan.
5. **M. Ismail, R. Ahmed** (McMaster), "A Comprehensive Review of Cloud-Based Lithium-Ion Battery Management Systems for Electric Vehicle Applications," *IEEE Access* 12, 116259-116273, 2024. DOI 10.1109/ACCESS.2024.3446880. **[TM]**
   - Bulutun sorunları: gecikme (gerçek zamanlı uygulamalar için), güvenlik/gizlilik, erişilebilirlik (kesinti).
   - Araştırma boşlukları: (1) çevrimiçi öğrenme, gerçek araç verisiyle kavram kanıtı eksik; (2) **bağlantı**: her yerde internet yok, araçta daha basit modeller bağlantı gelene dek çalışmalı; (3) filo yönetimi; (4) **güvenlik**: dinleme, araca komut gönderilmesi, buluta sahte veri; (5) kenar/sis/sunucusuz hesap.
   - Endüstri: Bosch (8 yıl ileri yaşlanma tahmini, kullanım sertifikası), Elysia vb. (Tablo 4).
   - Bizim için: (2) ve (4) bizim tasarımın gerekçesi (ESP yokken ECU tam çalışır; web'den araca komut yolu yok). (1) proje vizyonundaki ML hedefine "boşluk" olarak bağlanır.
6. **F. Naseri, Z. Kazemi, P. G. Larsen, M. M. Arefi, E. Schaltz**, "Cyber-Physical Cloud Battery Management Systems: Review of Security Aspects," *Batteries* 9(7), 382, 2023. DOI 10.3390/batteries9070382. **[TM]**
   - **Sonuç:** araçtaki BMS işlevsel güvenlik için kritik; bulut BMS çekirdeği yalnız tamamlar. (Tezde en net dayanak cümlesi.)
   - Saldırı sınıfları (CIA): gizlilik (dinleme, ortadaki adam), bütünlük (sahte veri enjeksiyonu, tekrar oynatma, rastgele gecikme), erişilebilirlik (DoS). Saldırı yolları: CAN, OBD-II, IoT ağ geçidi, bulut.
   - Etkiler: şarjın engellenmesi, paketin boşaltılması, hızlandırılmış yaşlanma, en kötü durumda termal kaçak.
   - Önlemler: şifreleme + kimlik doğrulama, dağıtık mimari, dayanıklı algoritmalar, blokzincir; standartlar SAE J3061, ISO/SAE 21434 (TARA). CBMS siber güvenliği için özel standart yok.
   - Bizim için: HTTPS + token (gizlilik/kimlik), tek yön (araca komut yok = en tehlikeli bütünlük saldırısı kapalı); tekrar oynatma PC gateway yolunda kapatıldı (2026-11-21: artan `seq`, sunucu 409). **Açık kalan**: ESP gateway henüz `seq` göndermiyor; anahtarı ele geçiren sahte veri yazabilir (panel yanıltılır, araç etkilenmez) -> sınırlılık olarak yaz.
11. **S. Y. Nejakar, R. Y. Nejakar, Raganna A., Devanathan M.**, "Cloud-Integrated Digital Twin for Intelligent Battery Monitoring and SOC Prediction," 2025 IEEE 4th Int. Conf. TEMSMET. DOI 10.1109/TEMSMET65536.2025.11467356. **[TM]** (5 sayfa konferans bildirisi)
   - Donanım: ESP32 + gerilim bölücü + ACS712 (±30 A) + DS18B20 + NEO-6M GPS; tek 12 V Li-iyon batarya.
   - Örnekleme: V, I 1 Hz; T, GPS 5 s. ESP32 -> Firebase Realtime DB, HTTPS POST, JSON (V, I, T, SOC, zaman, GPS); Firebase Authentication.
   - Panel: Firebase Web SDK + Google Maps; canlı grafikler, eşik alarmı (SOC < %20, T > 50 °C), 5 km içinde servis önerisi.
   - SOC (Tablo I, MAE): Coulomb sayımı %6,5, Kalman %4,3, RNN %3,8, Kalman + RNN ağırlıklı füzyon %2,6.
   - 48 saat test: gecikme < 1 s (kararlı Wi-Fi), sıfır paket kaybı, çevrimdışı önbellek.
   - **Zayıf yönler (eleştirel okuma):**
     - Kalman modeli (durumlar, parametreler), RNN mimarisi ve eğitim verisi, füzyon ağırlıkları verilmemiş; tekrarlanamaz.
     - Ölçüm doğruluğu kaba: gerilim ±0,5 V, akım ±1 A. Bu belirsizlikle %2,6 SOC hatasının nasıl ölçüldüğü (referans?) belirtilmemiş.
     - Tablo I'de "doğruluk" sütunu 100 - MAE ile tutarsız (RNN 3,8 -> 96,1; hibrit 2,6 -> 97,9).
     - Gecikme ve paket kaybının nasıl ölçüldüğü yok.
     - Yalnız izleme: araçta karar/koruma yok; sabit eşik alarmları.
     - **Kaynakça güvenilmez:** [1] "Cloud battery management system with online state-of-charge and state-of-health estimation" başlığı Li ve ark. 2020'nin (bizim [1]) başlığına benziyor ama farklı yazar/dergiyle verilmiş; [5] "Application of digital twin in smart battery management systems" yazarları da farklı görünüyor; metinde Baccouche [7] denirken listede [7] Zhang. Bu kaynakları [11] üzerinden atıf olarak kullanma.
   - **Bizim için değeri:** mimari olarak en yakın akran (ESP -> HTTPS JSON -> bulut DB -> web panel, eşik alarmı, 1 Hz); karşılaştırma tablosunda en iyi "akran sistem". Bizden alınacak ölçüt fikri: uçtan uca gecikme ve paket kaybını **ölçülebilir** şekilde raporlamak (D satırındaki t_ms ile sunucu zamanını karşılaştır; sayaçla kayıp say). Bizim farklar: 19S12P paket, CAN, gerçek zamanlı ECU kararı, HIL, belgelenmiş EKF, çok kriterli stres endeksi.

### Yalnız özetten (doğrulanmalı)
2. ML ile batarya verisi, uç-kenar-bulut çerçevesi, *RSC Advances*, 2023. [bağlantı](https://pubs.rsc.org/en/content/articlehtml/2023/ra/d2ra06004c) **[Ö]**
4. "A Brief Review of Key Technologies for Cloud-Based Battery Management Systems," *J. Electron. Mater.*, 2024. [bağlantı](https://ui.adsabs.harvard.edu/abs/2024JEMat.tmp..611W/abstract) **[Ö]**
7. EV hücre verisinde kayıt frekansı/sıkıştırma (0,2 Hz, %28, RMSE 9,7 mV). [PDF](https://athene-forschung.rz.unibw-muenchen.de/doc/149613/149613.pdf) **[Ö]**
8. Bulut-kenar BESS izleme, hücre verisi kayıpsız sıkıştırma, *Batteries* 12, 396. [PDF](https://orca.cardiff.ac.uk/id/eprint/190237/1/batteries-12-00396.pdf) **[Ö]**
12. Dharani ve ark., DT ile SOC/SOH çift yol tahmini, Azure IoT, filo paneli (panel simüle veriyle). [ResearchGate](https://www.researchgate.net/publication/399712505) **[Ö]**
13. Uzaktan deneylerde WebAssembly hesap defterleri, *IEEE Access*, 2023. [bağlantı](https://doaj.org/article/2f6b00ef9a10434189e0eb0b3cadc456) **[Ö]**
14. Sanal + uzaktan laboratuvar (Valladolid pilot tesis), Springer, 2025. [bağlantı](https://link.springer.com/article/10.1007/s44217-025-00848-1) **[Ö]**
15. STM32 ile Cloud-IoT uzaktan mikrodenetleyici laboratuvarı, *Technologies* 14(5), 281. [bağlantı](https://www.mdpi.com/2227-7080/14/5/281/htm) **[Ö]**

## Bu projenin farkı (tezde vurgulanacak)
- [16], [17]: tek modül / kurşun-asit akü, basit sensör, ECU yok; [17] web'den donanıma komut veriyor.
  Burada: 19S12P paket, CAN, gerçek zamanlı ECU, HIL, çok kriterli stres endeksi, tek yönlü telemetri.
- [9]: endüstriyel ölçek (4G/5G, TSDB, Grafana); bizim sistem aynı iş bölümünün (araçta karar,
  bulutta analiz) düşük maliyetli, uçtan uca açık bir prototipi.
- [10]: 3D/Omniverse ağır görselleştirme; bizim panel tarayıcıda hafif, ama aynı ECU kodu simülasyonda da çalışır.
- [12] Ö: panel simüle veriyle. Burada: kart verisi canlı, simülasyon ayrı ve etiketli.
- [1]: aynı ilke (güvenlik yerelde, ağır algoritma bulutta), ama IoT birimi Raspberry Pi + MQTT, sahada
  yalnız kurşun-asit UPS, araçta karar mekanizması yok. Burada: araçta tam ECU (EKF + çok kriterli stres
  endeksi + NORMAL/DERATE/LIMP/SAFE), ~2 $'lık ESP-01 köprüsü, HIL ile doğrulanmış Li-iyon paket.
