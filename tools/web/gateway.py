r"""STM32 UART -> web sunucusu (tools/web/server/api.php) köprüsü.

    pip install pyserial requests
    python tools/web/gateway.py [COM9]
    yerel deneme: python tools/web/mock_server.py   ve   BMS_URL=http://127.0.0.1:8080/api.php BMS_API_KEY=test

Anahtar ve adres kaynakta YOK: BMS_API_KEY (ya da tools/web/api_key.txt, git dışı) ve BMS_URL
(varsayılan hizlipdf.net/bitirmeprj/api.php). config.php'deki API_KEY ile aynı olmalı.

STM32 UART satırları (115200, \n, ondalık nokta):
  D,<t_ms>,<run>,<soc>,<soh>,<T>,<I_allow>,<state>,<tec>,<rec>   state 1 NORMAL, 2 DERATE, 3 LIMP-HOME, 4 SAFE-STOP
  E,<t_ms>,<run>,<code>,<detail>                                 olay
  RS,<run>,<name>                                                senaryo (Monte Carlo koşusu) başladı
  RE,<run>,<PASS|FAIL>,<summary...>                              senaryo bitti
Diğer satırlar (ECU 1 s raporu, MC,READY/DONE) yalnız log'a yazılır.
PC -> STM32 komutları (klavyeden, APP_MC):  RUN <id>  |  ALL [n]  |  STOP
"""
import collections
import json
import os
import pathlib
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
OUT = HERE.parent.parent / "build" / "web"              # git dışı
OUT.mkdir(parents=True, exist_ok=True)
SPOOL = OUT / "spool.jsonl"                             # internet yoksa yerel kuyruk
LOG = OUT / f"log_{time.strftime('%Y%m%d_%H%M%S')}.txt"

lock = threading.Lock()
buf = {"rows": [], "events": [], "runs": []}
stats = collections.Counter()


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
        if p[0] == "RE" and len(p) >= 3:
            return "runs", dict(op="end", id=int(p[1]), result=p[2], summary=",".join(p[3:]))
    except ValueError:
        pass
    return None, None


def post(payload) -> bool:
    try:
        r = requests.post(URL, json=payload, headers={"X-API-Key": API_KEY}, timeout=5)
        if r.status_code == 401:
            stats["unauthorized"] += 1
        return r.ok
    except requests.RequestException:
        return False


def sender():
    while True:
        time.sleep(SEND_PERIOD)
        with lock:
            payload = {k: v[:] for k, v in buf.items()}
            for v in buf.values():
                v.clear()
        if SPOOL.exists():                              # önce bekleyen kuyruk
            pending = SPOOL.read_text(encoding="utf-8").splitlines()
            ok_all = True
            for i, ln in enumerate(pending):
                if not post(json.loads(ln)):
                    SPOOL.write_text("\n".join(pending[i:]) + "\n", encoding="utf-8")
                    ok_all = False
                    break
                stats["resent"] += 1
            if ok_all:
                SPOOL.unlink()
        if not any(payload.values()):
            continue
        if post(payload):
            stats["sent"] += len(payload["rows"])
            stats["ev"] += len(payload["events"]) + len(payload["runs"])
        else:
            stats["spooled"] += len(payload["rows"])
            with SPOOL.open("a", encoding="utf-8") as f:
                f.write(json.dumps(payload) + "\n")
        print(f"\r rows sent={stats['sent']} events/runs={stats['ev']} spooled={stats['spooled']} "
              f"resent={stats['resent']} other lines={stats['other']}"
              + (" UNAUTHORIZED (check API key)" if stats["unauthorized"] else "") + "   ", end="", flush=True)


def keyboard(ser):
    for cmd in sys.stdin:
        cmd = cmd.strip()
        if cmd:
            ser.write((cmd + "\n").encode())


def main():
    if not API_KEY:
        print("warning: no API key (set BMS_API_KEY or create tools/web/api_key.txt)")
    ser = serial.Serial(PORT, BAUD, timeout=1)
    threading.Thread(target=sender, daemon=True).start()
    threading.Thread(target=keyboard, args=(ser,), daemon=True).start()
    print(f"{PORT} -> {URL}  (commands: RUN <id> | ALL [n] | STOP), log {LOG}")
    with LOG.open("w", encoding="utf-8", buffering=1) as log:     # satır satır (kesilirse kaybolmasın)
        while True:
            line = ser.readline().decode(errors="ignore").strip()
            if not line:
                continue
            log.write(line + "\n")
            kind, item = parse(line)
            if kind is None:
                stats["other"] += 1
                continue
            with lock:
                buf[kind].append(item)


if __name__ == "__main__":
    main()
