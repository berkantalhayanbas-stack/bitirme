"""Hosting yükleme paketi: build/web/bitirmeprj_upload.zip

    python tools/web/make_upload.py

- Anahtar tools/web/api_key.txt'ten okunur; yoksa üretilir (git dışı).
- server/config.php, config.sample.php'den bu anahtarla yazılır (git dışı).
- Zip: .htaccess, api.php, data.php, index.html, schema.sql, config.php (kök düzeyde;
  cPanel'de public_html/bitirmeprj/ içinde Extract edilir).
Anahtar hiçbir çıktıya yazdırılmaz.
"""
import re
import secrets
import zipfile
from pathlib import Path

WEB = Path(__file__).resolve().parent
SRV = WEB / "server"
OUT = WEB.parents[1] / "build" / "web" / "bitirmeprj_upload.zip"
FILES = [".htaccess", "api.php", "data.php", "index.html", "schema.sql", "config.php"]


def main():
    key_file = WEB / "api_key.txt"
    if not key_file.exists():
        key_file.write_text(secrets.token_urlsafe(32) + "\n", encoding="ascii")
        print("new API key written to tools/web/api_key.txt")
    key = key_file.read_text(encoding="ascii").strip()

    # config.php her seferinde örnekten üretilir: örnekteki değişiklikler pakete girer
    sample = (SRV / "config.sample.php").read_text(encoding="utf-8")
    conf, n = re.subn(r"const API_KEY = '[^']*';", lambda m: f"const API_KEY = '{key}';", sample)
    if n != 1:
        raise SystemExit("config.sample.php: API_KEY line not found")
    (SRV / "config.php").write_text(conf, encoding="utf-8", newline="\n")

    OUT.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(OUT, "w", zipfile.ZIP_DEFLATED) as z:
        for f in FILES:
            z.write(SRV / f, f)
    print(f"{OUT} ({OUT.stat().st_size} B): {', '.join(FILES)}")


if __name__ == "__main__":
    main()
