r"""Yerel deneme sunucusu: tools/web/server'daki PHP uçlarını (api.php, data.php) Python + SQLite ile
taklit eder, index.html'i sunar. Hosting olmadan uçtan uca deneme ve sunum yedeği için.

    python tools/web/mock_server.py [--port 8080] [--key test] [--db build/web/mock.sqlite]
    set BMS_URL=http://127.0.0.1:8080/api.php & set BMS_API_KEY=test & python tools/web/gateway.py COM9
    tarayıcı: http://127.0.0.1:8080/

Şema ve yanıt biçimi schema.sql / api.php / data.php ile aynı (alan adları birebir).
"""
import argparse
import json
import os
import sqlite3
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

HERE = os.path.dirname(os.path.abspath(__file__))
ap = argparse.ArgumentParser()
ap.add_argument("--port", type=int, default=8080)
ap.add_argument("--key", default="test")
ap.add_argument("--db", default=os.path.join(HERE, "..", "..", "build", "web", "mock.sqlite"))
a = ap.parse_args()
os.makedirs(os.path.dirname(os.path.abspath(a.db)), exist_ok=True)

db = sqlite3.connect(a.db, check_same_thread=False)
db.row_factory = sqlite3.Row
lock = threading.Lock()
db.executescript("""
CREATE TABLE IF NOT EXISTS runs (run_id INTEGER PRIMARY KEY, name TEXT NOT NULL,
  started_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP, ended_at TEXT, result TEXT NOT NULL DEFAULT 'RUNNING', summary TEXT);
CREATE TABLE IF NOT EXISTS telemetry (id INTEGER PRIMARY KEY AUTOINCREMENT, run_id INTEGER NOT NULL DEFAULT 0,
  t_ms INTEGER NOT NULL, soc REAL, soh REAL, temp REAL, i_allow REAL, state INTEGER, tec INTEGER, rec INTEGER,
  rx_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP);
CREATE TABLE IF NOT EXISTS events (id INTEGER PRIMARY KEY AUTOINCREMENT, run_id INTEGER NOT NULL DEFAULT 0,
  t_ms INTEGER NOT NULL, code TEXT NOT NULL, detail TEXT, rx_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP);
""")


def rows(sql, args=()):
    with lock:
        return [dict(r) for r in db.execute(sql, args).fetchall()]


class H(BaseHTTPRequestHandler):
    def out(self, data, code=200, ctype="application/json; charset=utf-8"):
        body = data if isinstance(data, bytes) else json.dumps(data).encode()
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *args):
        pass

    def do_POST(self):
        if urlparse(self.path).path != "/api.php":
            return self.out({"error": "not found"}, 404)
        if self.headers.get("X-API-Key", "") != a.key:
            return self.out({"error": "unauthorized"}, 401)
        try:
            j = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0))))
        except ValueError:
            return self.out({"error": "bad json"}, 400)
        with lock:      # sıra api.php ile aynı: başlayan senaryo (eski izi sil) -> satırlar -> olaylar -> bitenler
            for r in j.get("runs", []):
                if r["op"] == "start":
                    db.execute("DELETE FROM telemetry WHERE run_id=? AND run_id>0", (int(r["id"]),))
                    db.execute("DELETE FROM events WHERE run_id=? AND run_id>0", (int(r["id"]),))
                    db.execute("INSERT INTO runs (run_id,name) VALUES (?,?) ON CONFLICT(run_id) DO UPDATE SET name=excluded.name, "
                               "started_at=CURRENT_TIMESTAMP, ended_at=NULL, result='RUNNING', summary=NULL", (int(r["id"]), r["name"][:64]))
            for r in j.get("rows", []):
                db.execute("INSERT INTO telemetry (run_id,t_ms,soc,soh,temp,i_allow,state,tec,rec) VALUES (?,?,?,?,?,?,?,?,?)",
                           (int(r["run"]), int(r["t"]), r["soc"], r["soh"], r["T"], r["I"], int(r["st"]), int(r["tec"]), int(r["rec"])))
            for e in j.get("events", []):
                db.execute("INSERT INTO events (run_id,t_ms,code,detail) VALUES (?,?,?,?)",
                           (int(e["run"]), int(e["t"]), e["code"][:32], (e.get("detail") or "")[:255]))
            for r in j.get("runs", []):
                if r["op"] == "end":
                    db.execute("UPDATE runs SET ended_at=CURRENT_TIMESTAMP, result=?, summary=? WHERE run_id=?",
                               ("PASS" if r["result"] == "PASS" else "FAIL", (r.get("summary") or "")[:255], int(r["id"])))
            db.commit()
        self.out({"ok": True, "rows": len(j.get("rows", []))})

    def do_GET(self):
        u = urlparse(self.path)
        if u.path in ("/", "/index.html"):
            return self.out(open(os.path.join(HERE, "server", "index.html"), "rb").read(), ctype="text/html; charset=utf-8")
        if u.path != "/data.php":
            return self.out({"error": "not found"}, 404)
        q = parse_qs(u.query)
        k = q.get("q", ["live"])[0]
        if k == "live":
            since = int(q.get("since", ["0"])[0])
            r = (rows("SELECT * FROM (SELECT * FROM telemetry ORDER BY id DESC LIMIT 600) ORDER BY id") if since == 0
                 else rows("SELECT * FROM telemetry WHERE id > ? ORDER BY id LIMIT 600", (since,)))
            return self.out({"rows": r, "events": rows("SELECT * FROM events ORDER BY id DESC LIMIT 20")})
        if k == "runs":
            return self.out(rows("SELECT r.*, CAST(strftime('%s', ended_at) - strftime('%s', started_at) AS INTEGER) AS dur_s "
                                 "FROM runs r ORDER BY started_at DESC, run_id DESC LIMIT 200"))
        if k == "run":
            i = int(q.get("id", ["0"])[0])
            return self.out({"rows": rows("SELECT t_ms,soc,soh,temp,i_allow,state FROM telemetry WHERE run_id=? ORDER BY t_ms", (i,)),
                             "events": rows("SELECT t_ms,code,detail FROM events WHERE run_id=? ORDER BY t_ms", (i,))})
        self.out({"error": "bad q"}, 400)


print(f"mock server http://127.0.0.1:{a.port}/  (API key '{a.key}', db {os.path.abspath(a.db)})")
ThreadingHTTPServer(("127.0.0.1", a.port), H).serve_forever()
