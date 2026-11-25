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
import time
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
with open(os.path.join(HERE, "server", "schema.sql"), encoding="utf-8") as fs:
    db.executescript(fs.read())                     # api.php ile aynı şema (tek kaynak)
# geniş satır kolonları (config.php TELEMETRY_X ile aynı): eski tabloya ekle
TELEMETRY_X = ["cause", "ts", "i_chg", "psi", "i_meas", "v_pack", "t_meas", "v_min", "v_max",
               "psi_t", "psi_c", "psi_tdot", "psi_h", "psi_s", "psi_dom", "tdot", "k_r",
               "soh_c", "soh_n", "efc", "soh_anchor", "rest_s", "soh_ratio"]
_have = {r[1] for r in db.execute("PRAGMA table_info(telemetry)")}
for _c in TELEMETRY_X:
    if _c not in _have:
        db.execute(f"ALTER TABLE telemetry ADD COLUMN {_c} {'INTEGER' if _c in ('cause', 'psi_dom') else 'REAL'}")


def rows(sql, args=()):
    with lock:
        return [dict(r) for r in db.execute(sql, args).fetchall()]


def iv(m, k):
    v = m.get(k)
    return int(v) if isinstance(v, (int, float)) and not isinstance(v, bool) else None


def pct(v, q):                                       # data.php pct() ile aynı
    v = sorted(x for x in v if x is not None)
    return int(v[min(len(v) - 1, int(round(q / 100 * (len(v) - 1))))]) if v else None


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
        m = j.get("meta") if isinstance(j.get("meta"), dict) else {}
        seq = iv(m, "seq")
        with lock:      # sıra api.php ile aynı: tekrar kontrolü -> başlayan senaryo -> satırlar -> olaylar -> bitenler -> link
            if seq is not None:
                last = db.execute("SELECT MAX(seq) FROM link").fetchone()[0]
                if last is not None and seq <= last:
                    return self.out({"error": "replay", "last_seq": last}, 409)
            for r in j.get("runs", []):
                if r["op"] == "start":
                    db.execute("DELETE FROM telemetry WHERE run_id=? AND run_id>0", (int(r["id"]),))
                    db.execute("DELETE FROM events WHERE run_id=? AND run_id>0", (int(r["id"]),))
                    db.execute("INSERT INTO runs (run_id,name) VALUES (?,?) ON CONFLICT(run_id) DO UPDATE SET name=excluded.name, "
                               "started_at=CURRENT_TIMESTAMP, ended_at=NULL, result='RUNNING', summary=NULL", (int(r["id"]), r["name"][:64]))
            for r in j.get("rows", []):
                x = [r.get(c) if isinstance(r.get(c), (int, float)) else None for c in TELEMETRY_X]
                db.execute(f"INSERT INTO telemetry (run_id,t_ms,soc,soh,temp,i_allow,state,tec,rec,{','.join(TELEMETRY_X)}) "
                           f"VALUES (?,?,?,?,?,?,?,?,?{',?' * len(TELEMETRY_X)})",
                           (int(r["run"]), int(r["t"]), r["soc"], r["soh"], r["T"], r["I"], int(r["st"]), int(r["tec"]),
                            int(r["rec"]), *x))
            for e in j.get("events", []):
                db.execute("INSERT INTO events (run_id,t_ms,code,detail) VALUES (?,?,?,?)",
                           (int(e["run"]), int(e["t"]), e["code"][:32], (e.get("detail") or "")[:255]))
            for r in j.get("runs", []):
                if r["op"] == "end":
                    db.execute("UPDATE runs SET ended_at=CURRENT_TIMESTAMP, result=?, summary=? WHERE run_id=?",
                               ("PASS" if r["result"] == "PASS" else "FAIL", (r.get("summary") or "")[:255], int(r["id"])))
            db.execute("INSERT INTO link (seq,srv_ms,sent_ms,rx_first_ms,rx_last_ms,n_rows,n_events,uart_missing,uart_bad,"
                       "jit_ms,prev_rtt_ms,prev_age_ms,resent) VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?)",
                       (seq, int(time.time() * 1000), iv(m, "sent_ms"), iv(m, "rx_first"), iv(m, "rx_last"),
                        len(j.get("rows", [])), len(j.get("events", [])) + len(j.get("runs", [])),
                        iv(m, "uart_missing"), iv(m, "uart_bad"), iv(m, "jit_ms"), iv(m, "prev_rtt_ms"),
                        iv(m, "prev_age_ms"), iv(m, "resent") or 0))
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
            n = max(1, min(3000, int(q.get("n", ["600"])[0])))
            r = (rows(f"SELECT * FROM (SELECT * FROM telemetry ORDER BY id DESC LIMIT {n}) ORDER BY id") if since == 0
                 else rows("SELECT * FROM telemetry WHERE id > ? ORDER BY id LIMIT 600", (since,)))
            return self.out({"rows": r, "events": rows("SELECT * FROM events ORDER BY id DESC LIMIT 20")})
        if k == "runs":
            return self.out(rows("SELECT r.*, CAST(strftime('%s', ended_at) - strftime('%s', started_at) AS INTEGER) AS dur_s "
                                 "FROM runs r ORDER BY started_at DESC, run_id DESC LIMIT 200"))
        if k == "run":
            i = int(q.get("id", ["0"])[0])
            return self.out({"rows": rows("SELECT t_ms,soc,soh,temp,i_allow,state,cause,ts,i_chg,i_meas,v_min,v_max FROM telemetry "
                                     "WHERE run_id=? ORDER BY t_ms", (i,)),
                             "events": rows("SELECT t_ms,code,detail FROM events WHERE run_id=? ORDER BY t_ms", (i,))})
        if k == "tests":
            return self.out(rows("SELECT id,t_ms,code,detail,rx_at FROM events WHERE code IN ('BUTTON','REACTION') "
                                 "ORDER BY id DESC LIMIT 16"))
        if k == "linkhist":
            mn = max(1, min(1440, int(q.get("min", ["5"])[0])))
            now = int(time.time() * 1000)
            return self.out({"now_ms": now, "rows": rows("SELECT srv_ms,prev_rtt_ms,prev_age_ms,n_rows,uart_missing,uart_bad,resent "
                                                         "FROM link WHERE srv_ms >= ? ORDER BY id LIMIT 2000", (now - mn * 60000,))})
        if k == "link":
            mn = max(1, min(1440, int(q.get("min", ["5"])[0])))
            L = rows("SELECT * FROM link WHERE srv_ms >= ? ORDER BY id", (int(time.time() * 1000) - mn * 60000,))
            nr = sum(r["n_rows"] for r in L)
            miss = sum(r["uart_missing"] or 0 for r in L)
            col = lambda c: [r[c] for r in L]
            return self.out({"minutes": mn, "batches": len(L), "rows": nr, "uart_missing": miss,
                             "uart_bad": sum(r["uart_bad"] or 0 for r in L), "resent": sum(r["resent"] for r in L),
                             "loss_pct": round(100 * miss / (nr + miss), 3) if nr + miss else None,
                             "rtt_p50": pct(col("prev_rtt_ms"), 50), "rtt_p95": pct(col("prev_rtt_ms"), 95),
                             "age_p50": pct(col("prev_age_ms"), 50), "age_p95": pct(col("prev_age_ms"), 95),
                             "jit_p95": pct(col("jit_ms"), 95),
                             "oneway_p50": pct([None if r["sent_ms"] is None else r["srv_ms"] - r["sent_ms"] for r in L], 50),
                             "last_srv_ms": L[-1]["srv_ms"] if L else None})
        self.out({"error": "bad q"}, 400)


print(f"mock server http://127.0.0.1:{a.port}/  (API key '{a.key}', db {os.path.abspath(a.db)})")
ThreadingHTTPServer(("127.0.0.1", a.port), H).serve_forever()
