r"""STM32 UART -> web sunucusu (tools/web/server/api.php) köprüsü.

    pip install pyserial requests
    python tools/web/gateway.py [COM9]
    yerel deneme: python tools/web/mock_server.py   ve   BMS_URL=http://127.0.0.1:8080/api.php BMS_API_KEY=test
    port yerine pyserial URL'si de olur (ör. socket://127.0.0.1:7777, test için).

Anahtar ve adres kaynakta YOK: BMS_API_KEY (ya da tools/web/api_key.txt, git dışı) ve BMS_URL
(varsayılan hizlipdf.net/bitirmeprj/api.php). config.php'deki API_KEY ile aynı olmalı.

STM32 UART satırları (115200, \n, ondalık nokta):
  D,<t_ms>,<run>,<soc>,<soh>,<T>,<I_allow>,<state>,<tec>,<rec>   state 1 NORMAL, 2 DERATE, 3 LIMP-HOME, 4 SAFE-STOP
  E,<t_ms>,<run>,<code>,<detail>                                 olay
  RS,<run>,<name>                                                senaryo (Monte Carlo koşusu) başladı
  RE,<run>,<PASS|FAIL>,<summary...>                              senaryo bitti
  DX,<ms>,<state>,<cause>,<soc>,<Tc>,<Ts>,<Idis>,<Ichg>,<psi>,<soh>,<I>,<V>,<T>,<Vmin>,<Vmax>
     [,<Psi_T>,<Psi_C>,<Psi_Tdot>,<Psi_H>,<Psi>,<dom>,<dT/dt C/min>,<k_R>]
                                    geniş satır (BMS_TELEMETRY=2): aynı adımın D satırına eklenir (t_ms eşleşmesi).
                                    psi = toplam kısıtlama 1 - I_allow/I_nom; Psi = çok kriterli stres indeksi,
                                    dom = baskın kriter (0 yok, 1 T, 2 C, 3 Tdot, 4 H); son 8 alan yoksa NULL
Diğer satırlar (ECU 1 s raporu, MC,READY/DONE) yalnız log'a yazılır.
PC -> STM32 komutları (klavyeden, APP_MC):  RUN <id>  |  ALL [n]  |  STOP

Bağlantı kalitesi (tez: gecikme ve kayıp ölçülerek raporlanır):
  - UART kaybı: aynı run'da ardışık D satırlarının t_ms farkı, son 21 aralığın medyanının 1,5 katını
    aşarsa eksik satır sayılır (1 s'den uzun boşluk kayıp değil ECU duraklamasıdır: ECU adımı ölçüm
    çiftiyle atar, EMU susunca satır da gelmez); "D," ile başlayıp ayrıştırılamayan satır bozuk sayılır.
  - UART titreşimi (yalnız run 0, t_ms gerçek zaman): (PC alış zamanı - t_ms) - son 5 s'deki en küçüğü
    (oturum boyu en küçüğü alınırsa ECU-PC saat kayması, ~0,5 ms/s, titreşim gibi birikir).
  - Gidiş-dönüş (RTT) ve uçtan uca gecikme (UART'tan alış -> sunucu onayı) tek saatle (PC) ölçülür.
  - Her yığına meta eklenir: seq (artan, ms), sent_ms, rx_first/rx_last, uart_missing/uart_bad,
    jit_ms ve önceki yığının rtt/age değerleri; api.php bunları link tablosuna yazar, seq <= son seq
    olan yığını tekrar oynatma sayıp reddeder (409).
  - build/web/link_*.csv yığın başına satır; Ctrl+C ile özet (p50/p95/max).

Mod butonları (isteğe bağlı, --emu COM10): twin_emu'nun seri portu da okunur. Her basışta Uno
"B,<olay>,<1|0>" yazar; gateway bunu "BUTTON" olayı olarak gönderir ve ECU'nun D satırlarındaki
durumu izleyip beklenen moda geçiş süresini "REACTION" olayı olarak ekler (PC saatiyle, 0,1 s
çözünürlük). ECU senaryoyu bilmez. COM10'u açmak Uno'yu sıfırlar (~2 s susar: ECU SAFE'e düşer,
NORMAL butonuyla dönülür). Klavyeden "emu <komut>" Uno'ya gider (ör. emu k 2), diğerleri ECU'ya.
demo_run.py COM10'u kullandığı için --emu ile aynı anda çalışmaz.
"""
import collections
import csv
import json
import os
import pathlib
import statistics
import sys
import threading
import time

import requests
import serial

HERE = pathlib.Path(__file__).resolve().parent
URL = os.environ.get("BMS_URL", "https://hizlipdf.net/bitirmeprj/api.php")
KEY_FILE = HERE / "api_key.txt"
API_KEY = os.environ.get("BMS_API_KEY") or (KEY_FILE.read_text().strip() if KEY_FILE.exists() else "")
ARGS = [a for a in sys.argv[1:]]
EMU_PORT = None
if "--emu" in ARGS:                                     # gateway.py COM9 --emu COM10
    i = ARGS.index("--emu")
    EMU_PORT = ARGS[i + 1]
    del ARGS[i:i + 2]
PORT = ARGS[0] if ARGS else "COM9"
BAUD = 115200
SEND_PERIOD = 1.0                                       # s
JIT_WIN = 5000                                          # ms, titreşim taban penceresi (saat kaymasını ayıklar)
PAUSE_MS = 1000                                         # ms, bundan uzun t_ms boşluğu = ECU duraklaması (kayıp değil)
OUT = HERE.parent.parent / "build" / "web"              # git dışı
OUT.mkdir(parents=True, exist_ok=True)
SPOOL = OUT / "spool.jsonl"                             # internet yoksa yerel kuyruk
STAMP = time.strftime('%Y%m%d_%H%M%S')
LOG = OUT / f"log_{STAMP}.txt"
LINK_CSV = OUT / f"link_{STAMP}.csv"
EMU_LOG = OUT / f"emu_{STAMP}.txt"                     # --emu: Uno satırları (R raporu, B basışları)

lock = threading.Lock()
buf = {"rows": [], "events": [], "runs": []}
stats = collections.Counter()
now_ms = lambda: int(time.time() * 1000)


# ---------------------------------------------------------------- UART kaybı ve titreşimi
class Uart:
    """Yığın süresince UART tarafı sayaçları (sender her yığında sıfırlar)."""
    def __init__(self):
        self.last = {}              # run -> [son t_ms, son 21 aralık (periyot = medyan)]
        self.offs = collections.deque()   # run 0: son JIT_WIN ms'nin (alış, alış - t_ms) çiftleri
        self.reset()

    def reset(self):
        self.rx_first = self.rx_last = None
        self.missing = self.bad = self.jit_max = 0

    def row(self, r, rx):
        self.rx_first = rx if self.rx_first is None else self.rx_first
        self.rx_last = rx
        run, t = r["run"], r["t"]
        prev = self.last.get(run)
        if prev is not None:
            dt = t - prev[0]
            if dt <= 0:                                 # ECU yeniden başladı / koşu tekrarlandı
                prev[1].clear()
                if run == 0:
                    self.offs.clear()
            else:
                # periyot = son aralıkların medyanı (en küçüğü alınırsa EMU yeniden başlayınca kayan faz
                # tek bir kısa aralık, ör. 38 ms, bırakır ve sonraki her 100 ms aralığı "kayıp" sayılırdı)
                d = sorted(prev[1])
                per = d[len(d) // 2] if len(d) >= 3 else None
                prev[1].append(dt)
                if dt > PAUSE_MS:                       # ECU adım atmadı (ölçüm gelmedi: EMU sıfırlandı,
                    stats["ecu_pause"] += 1             # hat koptu): satır kaybı değil, ayrı sayılır
                elif per is not None and dt > 1.5 * per:
                    m = round(dt / per) - 1
                    self.missing += m
                    stats["uart_missing"] += m
            prev[0] = t
        else:
            self.last[run] = [t, collections.deque(maxlen=21)]
        if run == 0:                                    # t_ms yalnız canlı modda gerçek zaman
            # titreşim = gecikme - son 5 s'nin en küçüğü: ECU ve PC saatleri arasındaki kayma
            # (ölçülen ~0,5 ms/s) oturum boyu birikip titreşim gibi görünmesin
            off = rx - t
            self.offs.append((rx, off))
            while rx - self.offs[0][0] > JIT_WIN:
                self.offs.popleft()
            self.jit_max = max(self.jit_max, off - min(o for _, o in self.offs))


uart = Uart()
seq_last = 0
prev_link = {"rtt": None, "age": None}
rtts, ages, jits = [], [], []                           # özet için (ms)


def next_seq():
    global seq_last
    seq_last = max(seq_last + 1, now_ms())              # artan; yeniden başlatmada da geriye gitmez
    return seq_last


def parse(line: str):
    p = line.split(",")
    try:
        if p[0] == "D" and len(p) == 10:
            return "rows", dict(t=int(p[1]), run=int(p[2]), soc=float(p[3]), soh=float(p[4]),
                                T=float(p[5]), I=float(p[6]), st=int(p[7]), tec=int(p[8]), rec=int(p[9]))
        if p[0] == "E" and len(p) >= 4:
            return "events", dict(t=int(p[1]), run=int(p[2]), code=p[3], detail=",".join(p[4:]))
        if p[0] == "RS" and len(p) >= 3:
            return "runs", dict(op="start", id=int(p[1]), name=",".join(p[2:]))
        if p[0] == "DX" and len(p) in (16, 24, 30, 36):
            # soc ve T (Tc) D'de 1, DX'te 2 ondalık: DX değeri D'ninkinin yerine geçer
            d = dict(t=int(p[1]), cause=int(p[3]), soc=float(p[4]), T=float(p[5]), ts=float(p[6]),
                     i_chg=float(p[8]), psi=float(p[9]),
                     i_meas=float(p[11]), v_pack=float(p[12]), t_meas=float(p[13]),
                     v_min=float(p[14]), v_max=float(p[15]))
            if len(p) >= 24:                # çok kriterli stres indeksi (eski ECU görüntüsünde yok: NULL)
                d.update(psi_t=float(p[16]), psi_c=float(p[17]), psi_tdot=float(p[18]), psi_h=float(p[19]),
                         psi_s=float(p[20]), psi_dom=int(p[21]), tdot=float(p[22]), k_r=float(p[23]))
            if len(p) >= 30:                # kapasite SOH'u (2026-11-25): öğrenilen %, ölçüm, EFC, çapa, dinlenme s, son oran %
                d.update(soh_c=float(p[24]), soh_n=int(p[25]), efc=float(p[26]), soh_anchor=int(p[27]),
                         rest_s=int(p[28]), soh_ratio=float(p[29]))
            if len(p) == 36:                # stres yeniden tasarımı (2026-11-26): Ψ_A, Ψ_B, Ψ_P, R_cyc, R_cal, hasar
                d.update(psi_a=float(p[30]), psi_b=float(p[31]), psi_p=float(p[32]), r_cyc=float(p[33]),
                         r_cal=float(p[34]), dmg=float(p[35]))
            return "dx", d
        if p[0] == "RE" and len(p) >= 3:
            return "runs", dict(op="end", id=int(p[1]), result=p[2], summary=",".join(p[3:]))
    except ValueError:
        pass
    if p[0] == "D":
        return "bad", None
    return None, None


def post(payload):
    """Döner: (durum kodu ya da None, sunucunun yazdığı satır sayısı, rtt ms, onay anı ms)."""
    m = payload.get("meta")
    if m is not None:
        m["sent_ms"] = now_ms()
        m["prev_rtt_ms"], m["prev_age_ms"] = prev_link["rtt"], prev_link["age"]
    t0 = time.perf_counter()
    try:
        r = requests.post(URL, json=payload, headers={"X-API-Key": API_KEY}, timeout=5)
    except requests.RequestException:
        return None, 0, None, None
    rtt = round((time.perf_counter() - t0) * 1000)
    if r.status_code == 401:
        stats["unauthorized"] += 1
    try:
        n = int(r.json().get("rows", 0)) if r.ok else 0
    except ValueError:
        n = 0
    return r.status_code, n, rtt, now_ms()


def account(payload, code, n, rtt, ack, writer):
    """Başarılı/başarısız yığını say, gecikmeleri kaydet, CSV satırı yaz."""
    m = payload.get("meta", {})
    nr = len(payload["rows"])
    age_first = ack - m["rx_first"] if ack and m.get("rx_first") else None
    age_last = ack - m["rx_last"] if ack and m.get("rx_last") else None
    if code is not None and 200 <= code < 300:
        stats["sent"] += n
        stats["ev"] += len(payload["events"]) + len(payload["runs"])
        if n != nr:
            stats["server_short"] += nr - n
        prev_link["rtt"], prev_link["age"] = rtt, age_first
        rtts.append(rtt)
        if age_first is not None:
            ages.append(age_first)
        status = "ok"
    elif code in (400, 409):
        stats["rejected"] += nr                          # tekrar/bozuk: kuyruğa alma, at
        status = "rejected" if code == 409 else "bad_request"
    else:
        status = "fail" if code is None else f"http_{code}"
    writer.writerow([m.get("seq"), nr, len(payload["events"]) + len(payload["runs"]), status, rtt,
                     age_first, age_last, m.get("uart_missing"), m.get("uart_bad"), m.get("jit_ms"),
                     m.get("resent", 0)])
    return status


def sender(writer):
    while True:
        time.sleep(SEND_PERIOD)
        with lock:
            # DX açıksa: son D satırının DX'i henüz gelmediyse (aynı adım, birkaç ms sonra) onu sonraki yığına bırak
            hold = []
            if (stats["dx_seen"] and buf["rows"] and "v_pack" not in buf["rows"][-1]
                    and uart.rx_last is not None and now_ms() - uart.rx_last < 200):
                hold = [buf["rows"].pop()]
            payload = {k: v[:] for k, v in buf.items()}
            for v in buf.values():
                v.clear()
            buf["rows"].extend(hold)
            if any(payload.values()) or uart.missing or uart.bad:
                payload["meta"] = dict(seq=next_seq(), rx_first=uart.rx_first, rx_last=uart.rx_last,
                                       uart_missing=uart.missing, uart_bad=uart.bad, jit_ms=uart.jit_max)
                jits.append(uart.jit_max)
            uart.reset()
        spool_left = False
        if SPOOL.exists():                              # önce bekleyen kuyruk (seq sırası korunur)
            pending = SPOOL.read_text(encoding="utf-8").splitlines()
            for i, ln in enumerate(pending):
                old = json.loads(ln)
                old.setdefault("meta", {})["resent"] = 1
                code, n, rtt, ack = post(old)
                st = account(old, code, n, rtt, ack, writer)
                if st in ("fail",) or st.startswith("http_"):
                    SPOOL.write_text("\n".join(pending[i:]) + "\n", encoding="utf-8")
                    spool_left = True
                    break
                stats["resent"] += 1
            if not spool_left:
                SPOOL.unlink()
        if "meta" not in payload:
            continue
        st = "fail"
        if not spool_left:                              # kuyruk boşalmadan yenisini gönderme (seq sırası)
            code, n, rtt, ack = post(payload)
            st = account(payload, code, n, rtt, ack, writer)
        if st == "fail" or st.startswith("http_"):
            stats["spooled"] += len(payload["rows"])
            with SPOOL.open("a", encoding="utf-8") as f:
                f.write(json.dumps(payload) + "\n")
        rtt_s = f"{prev_link['rtt']} ms" if prev_link["rtt"] is not None else "-"
        print(f"\r rows sent={stats['sent']} events/runs={stats['ev']} spooled={stats['spooled']} "
              f"resent={stats['resent']} uart missing={stats['uart_missing']} bad={stats['uart_bad']} "
              f"rtt={rtt_s} other lines={stats['other']}"
              + (" UNAUTHORIZED (check API key)" if stats["unauthorized"] else "") + "   ", end="", flush=True)


def keyboard(ser, emu):
    for cmd in sys.stdin:
        cmd = cmd.strip()
        if cmd.lower().startswith("emu ") and emu is not None:
            emu.write((cmd[4:] + "\n").encode())
        elif cmd:
            ser.write((cmd + "\n").encode())


# ---------------------------------------------------------------- mod butonları (twin_emu B satırı)
# olay -> (başladığında ad, kaldırıldığında ad, beklenen durum dizisi, zaman aşımı s)
# durumlar: 1 NORMAL, 2 DERATE, 3 LIMP-HOME, 4 SAFE-STOP
EV = {0: ("NORMAL: service restart", None, [1], 60),
      1: ("DERATE: full-battery charge", "DERATE: charging stopped", [2], 60),
      2: ("LIMP: temperature sensor cut", "LIMP: sensor repaired", [3], 60),
      3: ("STOP: measurement line cut", "STOP: line repaired", [4], 60),
      4: ("WATCHDOG: software hang", None, [4], 60),
      5: ("DERATE (long): cooling fault", "DERATE (long): cooling repaired", [2], 900),
      6: ("STOP (long): overheating", "STOP (long): heat source removed", [3, 4], 300)}
ST_NAME = {1: "NORMAL", 2: "DERATE", 3: "LIMP-HOME", 4: "SAFE-STOP"}
test = None                                             # bekleyen tepki ölçümü
last_t = 0                                              # son D satırının t_ms'i (olay zamanı için)
last_st = 0                                             # son D satırındaki ECU durumu


def emu_reader(emu):
    """Uno satırları: yalnız B,<olay>,<on> kullanılır (diğerleri yok sayılır)."""
    global test
    elog = EMU_LOG.open("w", encoding="utf-8", buffering=1)
    while True:
        try:
            line = emu.readline().decode(errors="ignore").strip()
        except serial.SerialException as e:
            print(f"\nemu port closed: {e}")
            return
        if line:
            elog.write(f"{now_ms()} {line}\n")
        if not line.startswith("B,"):
            continue
        try:
            _, ev, on = line.split(",")
            ev, on = int(ev), int(on)
        except ValueError:
            continue
        if ev not in EV:
            continue
        name_on, name_off, expect, tmo = EV[ev]
        label = name_on if on or name_off is None else name_off
        now = now_ms()
        with lock:
            exp = expect if on else []
            detail = label + (f" (expect {ST_NAME[exp[0]]})" if exp else "")
            buf["events"].append(dict(t=last_t, run=0, code="BUTTON", detail=detail))
            # NORMAL servisi ECU'yu yeniden başlatır: önce yeniden başlamayı (t_ms geri gider) bekle
            test = dict(label=label, expect=list(exp), t0=now, tmo=tmo, need_reset=(ev == 0)) if exp else None
            if test and ev != 0 and last_st == exp[0]:          # ECU zaten o modda (ör. SAFE'teyken STOP)
                buf["events"].append(dict(t=last_t, run=0, code="REACTION",
                                          detail=f"{label} -> already in {ST_NAME[exp[0]]}"))
                test = None
        print(f"\n[button] {detail}")


def track_reaction(row, rx):
    """D satırında bekleyen basışın beklenen durumu geldi mi (sırayla; ör. LIMP sonra SAFE)."""
    global test, last_t, last_st
    restarted = row["t"] < last_t
    last_t, last_st = row["t"], row["st"]
    if test is None or row["run"] != 0:
        return
    dt = (rx - test["t0"]) / 1000.0
    if test["need_reset"] and restarted:
        test["need_reset"] = False
    if not test["need_reset"] and row["st"] == test["expect"][0]:
        name = ST_NAME[test["expect"].pop(0)]
        buf["events"].append(dict(t=row["t"], run=0, code="REACTION",
                                  detail=f"{test['label']} -> {name} in {dt:.1f} s"))
        print(f"\n[reaction] {test['label']} -> {name} in {dt:.1f} s")
        if not test["expect"]:
            test = None
    elif dt > test["tmo"]:
        buf["events"].append(dict(t=row["t"], run=0, code="REACTION",
                                  detail=f"{test['label']} -> no {ST_NAME[test['expect'][0]]} within {test['tmo']} s"))
        test = None


def pct(v, q):
    if not v:
        return None
    s = sorted(v)
    return s[min(len(s) - 1, int(round(q / 100 * (len(s) - 1))))]


def summary():
    rows = stats["rows_parsed"]
    lost = stats["uart_missing"]
    print("\n\nLink quality summary")
    print(f"  D rows parsed {rows}, delivered {stats['sent']}, spooled {stats['spooled']}, "
          f"resent {stats['resent']}, rejected {stats['rejected']}, server short {stats['server_short']}"
          + (f", DX without D {stats['dx_orphan']}" if stats['dx_orphan'] else "")
          + (f", ECU pauses > 1 s {stats['ecu_pause']}" if stats['ecu_pause'] else ""))
    if rows + lost:
        print(f"  UART: missing {lost} ({100 * lost / (rows + lost):.3f} %), corrupt lines {stats['uart_bad']}")
    for name, v in (("upload RTT", rtts), ("end-to-end (UART rx -> server ack)", ages), ("UART jitter", jits)):
        if v:
            print(f"  {name} [ms]: p50 {pct(v, 50)}, p95 {pct(v, 95)}, max {max(v)}, mean {statistics.mean(v):.0f}, n {len(v)}")
    print(f"  per-batch log: {LINK_CSV}")


def main():
    if not API_KEY:
        print("warning: no API key (set BMS_API_KEY or create tools/web/api_key.txt)")
    ser = serial.serial_for_url(PORT, BAUD, timeout=1)
    emu = serial.serial_for_url(EMU_PORT, BAUD, timeout=1) if EMU_PORT else None
    if emu is not None:
        threading.Thread(target=emu_reader, args=(emu,), daemon=True).start()
    fcsv = LINK_CSV.open("w", newline="", encoding="utf-8", buffering=1)
    writer = csv.writer(fcsv)
    writer.writerow(["seq", "rows", "events", "status", "rtt_ms", "age_first_ms", "age_last_ms",
                     "uart_missing", "uart_bad", "jitter_ms", "resent"])
    threading.Thread(target=sender, args=(writer,), daemon=True).start()
    threading.Thread(target=keyboard, args=(ser, emu), daemon=True).start()
    print(f"{PORT} -> {URL}  (commands: RUN <id> | ALL [n] | STOP), log {LOG}")
    try:
        with LOG.open("w", encoding="utf-8", buffering=1) as log:     # satır satır (kesilirse kaybolmasın)
            while True:
                line = ser.readline().decode(errors="ignore").strip()
                if not line:
                    continue
                rx = now_ms()
                log.write(line + "\n")
                kind, item = parse(line)
                with lock:
                    if kind is None:
                        stats["other"] += 1
                    elif kind == "dx":                  # D satırından hemen sonra gelir (aynı adım)
                        last = buf["rows"][-1] if buf["rows"] else None
                        stats["dx_seen"] = 1
                        if last is not None and last["run"] == 0 and last["t"] == item.pop("t"):
                            last.update(item)
                        else:
                            stats["dx_orphan"] += 1     # D'si önceki yığınla gitti (nadir): alanlar NULL kalır
                    elif kind == "bad":
                        stats["uart_bad"] += 1
                        uart.bad += 1
                    else:
                        if kind == "rows":
                            stats["rows_parsed"] += 1
                            uart.row(item, rx)
                            track_reaction(item, rx)
                        buf[kind].append(item)
    except KeyboardInterrupt:
        pass
    except serial.SerialException as e:                 # kablo çekildi / port kapandı: özeti yine yaz
        print(f"\nserial port closed: {e}")
    finally:
        summary()
        fcsv.close()


if __name__ == "__main__":
    main()
