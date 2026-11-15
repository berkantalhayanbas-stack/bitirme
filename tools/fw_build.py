"""
Firmware'i CubeIDE'den bağımsız derler (aynı derleyici ve bayraklarla) ve isteğe bağlı ST-LINK ile yükler.

Test adımlarının her biri ayrı bir APP_MODE / optimizasyonla derlenir; CubeIDE'nin Debug/
makefile'larına dokunulmaz (onlar yeni kaynak dosyalarını CubeIDE yeniden üretene kadar görmez).
Kaynaklar: Src/*.c, Startup/*.s, bms/**/*.c. Derleyici: CubeIDE'nin gnu-tools-for-stm32 eklentisi.

Kullanım:
  python tools/fw_build.py --app 5                       # APP_CAN_SOAK, -O2 -> build/app5_O2/bms_ecu.elf
  python tools/fw_build.py --app 1 --opt O0 --flash      # PIL, -O0, derle + yükle + sıfırla
  python tools/fw_build.py --app 4 --flash               # varsayılan ECU yazılımını geri yükle
  python tools/fw_build.py --app 4 --define FOO=1        # ek derleme bayrağı
Çıktı: build/app<N>_<opt>[_<define>]/ (git dışı). CubeIDE'de hata ayıklama oturumu açıkken yükleme çalışmaz.
"""
import argparse
import glob
import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CUBE = r"C:\ST\STM32CubeIDE_*\STM32CubeIDE\plugins"
ARCH = ["-mcpu=cortex-m4", "-mfpu=fpv4-sp-d16", "-mfloat-abi=hard", "-mthumb"]
INCS = ["Inc", "Drivers/CMSIS/Device", "Drivers/CMSIS/Core", "bms/bsp", "bms/common", "bms/pil"]
DEFS = ["-DDEBUG", "-DSTM32", "-DSTM32G4", "-DNUCLEO_G431RB", "-DSTM32G431RBTx", "-DSTM32G431xx"]


def tool(pattern):
    hits = sorted(glob.glob(os.path.join(CUBE, pattern)))
    if not hits:
        sys.exit(f"not found: {pattern}")
    return hits[-1]


def run(cmd):
    r = subprocess.run(cmd, capture_output=True, text=True)
    out = (r.stdout + r.stderr).strip()
    if r.returncode != 0:
        sys.exit(f"FAILED: {' '.join(cmd[:3])} ...\n{out}")
    return out


def build(app, opt, defines):
    gcc = tool(r"*gnu-tools-for-stm32*\tools\bin\arm-none-eabi-gcc.exe")
    size = gcc.replace("arm-none-eabi-gcc.exe", "arm-none-eabi-size.exe")
    tag = f"app{app}_{opt}" + "".join("_" + d.replace("=", "") for d in defines)
    out = os.path.join(ROOT, "build", tag)
    os.makedirs(out, exist_ok=True)
    inc = [f"-I{os.path.join(ROOT, i)}" for i in INCS]
    cflags = ARCH + DEFS + inc + [f"-DAPP_MODE={app}", f"-{opt}", "-std=gnu11", "-g3", "-Wall",
                                  "-ffunction-sections", "-fdata-sections", "--specs=nano.specs"]
    cflags += [f"-D{d}" for d in defines]
    srcs = (glob.glob(os.path.join(ROOT, "Src", "*.c")) + glob.glob(os.path.join(ROOT, "Startup", "*.s"))
            + glob.glob(os.path.join(ROOT, "bms", "**", "*.c"), recursive=True))
    objs, warn = [], []
    for s in srcs:
        o = os.path.join(out, os.path.relpath(s, ROOT).replace(os.sep, "_").rsplit(".", 1)[0] + ".o")
        if s.endswith(".s"):
            cmd = [gcc] + ARCH + ["-g3", "-x", "assembler-with-cpp", "-c", s, "-o", o]
        else:
            cmd = [gcc] + cflags + ["-c", s, "-o", o]
        msg = run(cmd)
        if msg:
            warn.append(msg)
        objs.append(o)
    elf = os.path.join(out, "bms_ecu.elf")
    run([gcc, "-o", elf] + objs + ARCH + [f"-T{os.path.join(ROOT, 'STM32G431RBTX_FLASH.ld')}",
         "--specs=nosys.specs", "--specs=nano.specs", f"-Wl,-Map={os.path.join(out, 'bms_ecu.map')}",
         "-Wl,--gc-sections", "-static", "-Wl,--start-group", "-lc", "-lm", "-Wl,--end-group"])
    print(f"built {os.path.relpath(elf, ROOT)}  (APP_MODE={app}, -{opt}{', ' + ', '.join(defines) if defines else ''})")
    print(run([size, elf]))
    for w in warn:
        print("WARNING:", w)
    return elf


def flash(elf):
    cli = tool(r"*cubeprogrammer*\tools\bin\STM32_Programmer_CLI.exe")
    out = run([cli, "-c", "port=SWD", "mode=UR", "-w", elf, "-v", "-rst"])
    ok = "File download complete" in out and ("Download verified successfully" in out or "verified" in out)
    print("flash:", "OK (written, verified, reset)" if ok else "UNEXPECTED OUTPUT\n" + out[-600:])
    if not ok:
        sys.exit(1)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--app", type=int, default=4, help="APP_MODE (1 PIL, 2 CAN loopback, 3 CAN node, 4 ECU, 5 CAN soak)")
    ap.add_argument("--opt", default="O2", choices=["O0", "O1", "O2", "O3", "Os"])
    ap.add_argument("--define", action="append", default=[], help="extra -D (repeatable)")
    ap.add_argument("--flash", action="store_true")
    a = ap.parse_args()
    e = build(a.app, a.opt, a.define)
    if a.flash:
        flash(e)
