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
                                    geniş satır (BMS_TELEMETRY=2): aynı adımın D satırına eklenir (t_ms eşleşmesi)
Diğer satırlar (ECU 1 s raporu, MC,READY/DONE) yalnız log'a yazılır.
PC -> STM32 komutları (klavyeden, APP_MC):  RUN <id>  |  ALL [n]  |  STOP

Bağlantı kalitesi (tez: gecikme ve kayıp ölçülerek raporlanır):
  - UART kaybı: aynı run'da ardışık D satırlarının t_ms farkı, görülen en küçük periyodun 1,5 katını
    aşarsa eksik satır sayılır (1 s'den uzun boşluk kayıp değil ECU duraklamasıdır: ECU adımı ölçüm
    çiftiyle atar, EMU susunca satır da gelmez); "D," ile başlayıp ayrıştırılamayan satır bozuk sayılır.
  - UART titreşimi (yalnız run 0, t_ms gerçek zaman): (PC alış zamanı - t_ms) - son 5 s'deki en küçüğü
    (oturum boyu en küçüğü alınırsa ECU-PC saat kayması, ~0,5 ms/s, titreşim gibi birikir).
  - Gidiş-dönüş (RTT) ve uçtan uca gecikme (UART'tan alış -> sunucu onayı) tek saatle (PC) ölçülür.
  - Her yığına meta eklenir: seq (artan, ms), sent_ms, rx_first/rx_last, uart_missing/uart_bad,
    jit_ms ve önceki yığının rtt/age değerleri; api.php bunları link tablosuna yazar, seq <= son seq
    olan yığını tekrar oynatma sayıp reddeder (409).
  - build/web/link_*.csv yığın başına satır; Ctrl+C ile özet (p50/p95/max).
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
PORT = sys.argv[1] if len(sys.argv) > 1 else "COM9"
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

lock = threading.Lock()
buf = {"rows": [], "events": [], "runs": []}
stats = collections.Counter()
now_ms = lambda: int(time.time() * 1000)


# ---------------------------------------------------------------- UART kaybı ve titreşimi
class Uart:
    """Yığın süresince UART tarafı sayaçları (sender her yığında sıfırlar)."""
    def __init__(self):
        self.last = {}              # run -> [son t_ms, en küçük periyot]
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
                prev[1] = None
                if run == 0:
                    self.offs.clear()
            else:
                per = dt if prev[1] is None else min(prev[1], dt)
                if dt > PAUSE_MS:                       # ECU adım atmadı (ölçüm gelmedi: EMU sıfırlandı,
                    stats["ecu_pause"] += 1             # hat koptu): satır kaybı değil, ayrı sayılır
                elif dt > 1.5 * per:
                    m = round(dt / per) - 1
                    self.missing += m
                    stats["uart_missing"] += m
                prev[1] = per
            prev[0] = t
        else:
            self.last[run] = [t, None]
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
        if p[0] == "DX" and len(p) == 16:
            # soc ve T (Tc) D'de 1, DX'te 2 ondalık: DX değeri D'ninkinin yerine geçer
            return "dx", dict(t=int(p[1]), cause=int(p[3]), soc=float(p[4]), T=float(p[5]), ts=float(p[6]),
                              i_chg=float(p[8]), psi=float(p[9]),
                              i_meas=float(p[11]), v_pack=float(p[12]), t_meas=float(p[13]),
                              v_min=float(p[14]), v_max=float(p[15]))
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


def keyboard(ser):
    for cmd in sys.stdin:
        cmd = cmd.strip()
        if cmd:
            ser.write((cmd + "\n").encode())


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
    fcsv = LINK_CSV.open("w", newline="", encoding="utf-8", buffering=1)
    writer = csv.writer(fcsv)
    writer.writerow(["seq", "rows", "events", "status", "rtt_ms", "age_first_ms", "age_last_ms",
                     "uart_missing", "uart_bad", "jitter_ms", "resent"])
    threading.Thread(target=sender, args=(writer,), daemon=True).start()
    threading.Thread(target=keyboard, args=(ser,), daemon=True).start()
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
