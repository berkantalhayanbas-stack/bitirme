r"""bms.dbc -> Wireshark Lua çözücüsü (tools/can/bms_dbc.lua, ÜRETİLMİŞ; elle düzenleme).

Her mesaj kimliği Wireshark'ın "can.id" tablosuna bağlanır. Çözücü:
  - sinyaller: başlangıç biti / uzunluk / işaret / ölçek / ofset / birim (yalnız Intel, @1; DBC'nin tamamı Intel)
  - VAL_ tabloları (State, Cmd, 4095 = Invalid) ad olarak, "Bit field:" yorumlu sinyaller bayrak listesi olarak
  - E2E: CRC-8 SAE J1850 [ID düşük, ID yüksek, veri 0..6] = bayt 7, sayaç (bayt 6 alt nibble) adımı 1
    (2..3 kayıp, 0 ya da > 3 ret; bms_can.c can_monitor ile aynı), olay mesajında (GenMsgSendType Event) yalnız tekrar
  - periyot: GenMsgCycleTime'ın 1,5 katını aşan boşluk uyarı
Hatalar Wireshark "expert info" olarak işaretlenir (Analyze > Expert Information, kırmızı/sarı satırlar).
Alanlar: bms.<Sinyal>, bms.msg, bms.crc_ok, bms.cnt_step, bms.period_ms, bms.frame_bits (doldurma dahil,
+ 3 IFS; I/O Graph SUM(bms.frame_bits) / 1 s = hat yükü bit/s); ör. filtre  bms.State == 3

    python tools/can/dbc_to_lua.py                 # bms.dbc değişince yeniden üret
Kurulum: Wireshark > Help > About > Folders > Personal Lua Plugins klasörüne kopyala (ya da
tshark -X lua_script:tools/can/bms_dbc.lua). Analyze > Reload Lua Plugins (Ctrl+Shift+L).
"""
import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))
DBC = os.path.join(HERE, "bms.dbc")
OUT = os.path.join(HERE, "bms_dbc.lua")

BO = re.compile(r'^BO_ (\d+) (\w+): (\d+) (\w+)')
SG = re.compile(r'^ SG_ (\w+) : (\d+)\|(\d+)@([01])([+-]) \(([^,]+),([^)]+)\) \[[^\]]*\] "([^"]*)"')
VAL = re.compile(r'^VAL_ (\d+) (\w+) (.*);')
CM_SG = re.compile(r'^CM_ SG_ (\d+) (\w+) "(.*)";')
CYC = re.compile(r'^BA_ "GenMsgCycleTime" BO_ (\d+) (\d+);')
SEND = re.compile(r'^BA_ "GenMsgSendType" BO_ (\d+) (\d+);')


def lua_str(s):
    return '"' + s.replace("\\", "\\\\").replace('"', '\\"') + '"'


def main():
    msgs, order = {}, []
    cur = None
    with open(DBC) as f:
        lines = f.read().splitlines()
    for ln in lines:
        m = BO.match(ln)
        if m:
            cur = int(m[1])
            msgs[cur] = dict(name=m[2], dlc=int(m[3]), tx=m[4], sigs=[], cyc=0, event=False)
            order.append(cur)
            continue
        m = SG.match(ln)
        if m and cur is not None:
            if m[4] != "1":
                raise SystemExit(f"{m[1]}: Motorola byte order not supported by this generator")
            msgs[cur]["sigs"].append(dict(name=m[1], start=int(m[2]), len=int(m[3]), signed=m[5] == "-",
                                          scale=float(m[6]), offset=float(m[7]), unit=m[8], vals={}, bits={}))
        elif not ln.startswith(" "):
            cur = None
    sig = {(i, s["name"]): s for i, m in msgs.items() for s in m["sigs"]}
    for ln in lines:
        if m := VAL.match(ln):
            for k, v in re.findall(r'(\d+) "([^"]*)"', m[3]):
                sig[(int(m[1]), m[2])]["vals"][int(k)] = v
        elif m := CM_SG.match(ln):
            if m[3].startswith("Bit field:"):
                body = m[3].split(";")[0].split(".")[0][len("Bit field:"):]
                for k, v in re.findall(r'(\d+) ([^,]+)', body):
                    if int(k) < (1 << sig[(int(m[1]), m[2])]["len"]):
                        sig[(int(m[1]), m[2])]["bits"][int(k)] = v.strip()
        elif m := CYC.match(ln):
            msgs[int(m[1])]["cyc"] = int(m[2])
        elif m := SEND.match(ln):
            msgs[int(m[1])]["event"] = m[2] == "1"

    L = []
    w = L.append
    w("-- bms_dbc.lua -- ÜRETİLMİŞ DOSYA (tools/can/dbc_to_lua.py, kaynak tools/can/bms.dbc); elle düzenleme.")
    w("-- Wireshark CAN çözücüsü: sinyaller, değer tabloları, CRC-8 / sayaç / periyot kontrolü (expert info).")
    w('local p = Proto("bms", "BMS ECU CAN (bms.dbc)")')
    w("local F = {}")
    w('F.msg = ProtoField.string("bms.msg", "Message")')
    w('F.crc_ok = ProtoField.bool("bms.crc_ok", "CRC-8 ok")')
    w('F.cnt_step = ProtoField.int8("bms.cnt_step", "Counter step")')
    w('F.period_ms = ProtoField.double("bms.period_ms", "Gap since previous [ms]")')
    w('F.frame_bits = ProtoField.uint16("bms.frame_bits", "Bits on wire (stuffing incl., + 3 IFS)")')
    for i in order:
        for s in msgs[i]["sigs"]:
            label = s["name"] + (f' [{s["unit"]}]' if s["unit"] else "")
            if s["vals"] and s["scale"] == 1 and not s["offset"]:
                vs = "{" + ", ".join(f"[{k}] = {lua_str(v)}" for k, v in s["vals"].items()) + "}"
                w(f'F.{s["name"]} = ProtoField.uint32("bms.{s["name"]}", {lua_str(label)}, base.DEC, {vs})')
            elif s["scale"] == 1 and not s["offset"] and not s["signed"]:
                w(f'F.{s["name"]} = ProtoField.uint32("bms.{s["name"]}", {lua_str(label)})')
            else:
                w(f'F.{s["name"]} = ProtoField.double("bms.{s["name"]}", {lua_str(label)})')
    w("local fl = {}")
    w("for _, v in pairs(F) do fl[#fl + 1] = v end")
    w("p.fields = fl")
    w('local E_CRC = ProtoExpert.new("bms.crc_bad", "CRC-8 mismatch", expert.group.CHECKSUM, expert.severity.ERROR)')
    w('local E_CNT = ProtoExpert.new("bms.cnt_bad", "Counter step rejected (0 or > 3)", expert.group.SEQUENCE, expert.severity.ERROR)')
    w('local E_LOSS = ProtoExpert.new("bms.cnt_loss", "Counter step 2..3 (frame loss)", expert.group.SEQUENCE, expert.severity.WARN)')
    w('local E_GAP = ProtoExpert.new("bms.gap", "Gap > 1.5 x cycle time", expert.group.SEQUENCE, expert.severity.WARN)')
    w('local E_INV = ProtoExpert.new("bms.invalid", "Signal marked invalid", expert.group.MALFORMED, expert.severity.NOTE)')
    w('local E_STATE = ProtoExpert.new("bms.state_change", "ECU state change", expert.group.SEQUENCE, expert.severity.CHAT)')
    w("p.experts = { E_CRC, E_CNT, E_LOSS, E_GAP, E_INV, E_STATE }")
    w("")
    w("-- mesaj tablosu: ad, periyot [ms] (0 = olay), sinyaller {alan, ad, başlangıç, uzunluk, işaretli, ölçek, ofset, birim, değerler, bitler}")
    w("local M = {")
    for i in order:
        m = msgs[i]
        w(f'  [{i}] = {{ name = {lua_str(m["name"])}, cyc = {0 if m["event"] else m["cyc"]}, sigs = {{')
        for s in m["sigs"]:
            vals = "{" + ", ".join(f"[{k}] = {lua_str(v)}" for k, v in s["vals"].items()) + "}"
            bits = "{" + ", ".join(f"{{{k}, {lua_str(v)}}}" for k, v in sorted(s["bits"].items())) + "}"
            w(f'    {{ f = F.{s["name"]}, n = {lua_str(s["name"])}, st = {s["start"]}, ln = {s["len"]}, sg = {str(s["signed"]).lower()}, '
              f'sc = {s["scale"]!r}, of = {s["offset"]!r}, u = {lua_str(s["unit"])}, vals = {vals}, bits = {bits} }},')
        w("  } },")
    w("}")
    w(r"""
local STATE_VALS = {}
for _, s in ipairs(M[513].sigs) do if s.n == "State" then STATE_VALS = s.vals end end

local function crc8(b, n)
  local c = 0xFF
  for i = 1, n do
    c = c ~ b[i]
    for _ = 1, 8 do
      if c & 0x80 ~= 0 then c = ((c << 1) ~ 0x1D) & 0xFF else c = (c << 1) & 0xFF end
    end
  end
  return c ~ 0xFF
end

-- hattaki bit sayısı: SOF..CRC doldurmalı + CRC del + ACK + EOF + 3 IFS (can_log.frame_bits, LA ile doğrulandı)
local function frame_bits(id, b)
  local bits = { 0 }
  for i = 10, 0, -1 do bits[#bits + 1] = (id >> i) & 1 end
  bits[#bits + 1] = 0; bits[#bits + 1] = 0; bits[#bits + 1] = 0           -- RTR IDE r0
  for i = 3, 0, -1 do bits[#bits + 1] = (8 >> i) & 1 end                 -- DLC 8
  for k = 1, 8 do for i = 7, 0, -1 do bits[#bits + 1] = (b[k] >> i) & 1 end end
  local crc = 0
  for _, x in ipairs(bits) do
    local nx = x ~ ((crc >> 14) & 1)
    crc = (crc << 1) & 0x7FFF
    if nx ~= 0 then crc = crc ~ 0x4599 end
  end
  for i = 14, 0, -1 do bits[#bits + 1] = (crc >> i) & 1 end
  local stuffed, run, lastb = 0, 0, -1
  for _, x in ipairs(bits) do
    if x == lastb then run = run + 1 else run = 1 end
    lastb = x
    if run == 5 then stuffed = stuffed + 1; run = 1; lastb = 1 - x end
  end
  return #bits + stuffed + 10 + 3
end

-- ilk geçişte (pinfo.visited = false) çerçeve sırasıyla hesaplanır, sonra çerçeve no ile önbellekten okunur
local last, cache = {}, {}
function p.init() last, cache = {}, {} end

local function fmt(v, s)
  if s.sc == 1 and s.of == 0 then return tostring(math.tointeger(v) or v) end
  local d = s.sc < 0.01 and 4 or (s.sc < 0.1 and 2 or 1)
  return string.format("%." .. d .. "f", v)
end

function p.dissector(tvb, pinfo, tree)
  local id = pinfo.match_uint
  local m = M[id]
  if not m or tvb:len() < 8 then return 0 end
  pinfo.cols.protocol = "BMS"
  local b = {}
  for i = 0, 7 do b[i + 1] = tvb(i, 1):uint() end
  local raw = 0
  for i = 8, 1, -1 do raw = (raw << 8) | b[i] end

  local c = cache[pinfo.number]
  if not c then
    c = {}
    c.crc_ok = crc8({ id & 0xFF, id >> 8, b[1], b[2], b[3], b[4], b[5], b[6], b[7] }, 9) == b[8]
    local cnt = b[7] & 0x0F
    local L = last[id]
    if L then
      c.step = (cnt - L.cnt) & 0x0F
      c.gap = math.floor((pinfo.abs_ts - L.ts) * 1e6 + 0.5) / 1000.0   -- µs çözünürlüğe yuvarla
    end
    c.bits = frame_bits(id, b)
    if id == 513 then
      local st = (raw >> 32) & 0x0F
      c.state_change = L ~= nil and L.state ~= st
      c.state = st
    end
    last[id] = { cnt = cnt, ts = pinfo.abs_ts, state = c.state }
    cache[pinfo.number] = c
  end

  local t = tree:add(p, tvb(0, 8), "BMS " .. m.name .. string.format(" (0x%03X)", id))
  t:add(F.msg, m.name)
  local info = {}
  for _, s in ipairs(m.sigs) do
    local v = (raw >> s.st) & ((1 << s.ln) - 1)
    local rng = tvb(s.st // 8, math.min(8 - s.st // 8, (s.st % 8 + s.ln + 7) // 8))
    if s.vals[v] and s.sc ~= 1 then
      local it = t:add(s.f, rng, 0.0)
      it:set_text(s.n .. ": " .. s.vals[v] .. string.format(" (raw %d)", v))
      if s.vals[v] == "Invalid" then it:add_proto_expert_info(E_INV) end
    else
      if s.sg and v >= (1 << (s.ln - 1)) then v = v - (1 << s.ln) end
      local phys = v * s.sc + s.of
      local it = t:add(s.f, rng, (s.sc == 1 and s.of == 0) and v or phys)
      local txt = fmt(phys, s) .. (s.u ~= "" and (" " .. s.u) or "")
      if s.vals[v] then txt = s.vals[v] .. " (" .. v .. ")" end
      if #s.bits > 0 then
        local on = {}
        for _, bt in ipairs(s.bits) do if v & bt[1] ~= 0 then on[#on + 1] = bt[2] end end
        txt = txt .. (#on > 0 and (" = " .. table.concat(on, " | ")) or " = none")
      end
      it:set_text(s.n .. ": " .. txt)
      if not s.n:find("_CRC$") and not s.n:find("_Counter$") and not s.n:find("_BalFb$")
         and not s.n:find("^Bal_Group") and #info < 4 then
        info[#info + 1] = s.n .. "=" .. (s.vals[v] or fmt(phys, s))
      end
    end
  end

  t:add(F.frame_bits, c.bits)
  local ci = t:add(F.crc_ok, tvb(7, 1), c.crc_ok)
  if not c.crc_ok then ci:add_proto_expert_info(E_CRC) end
  if c.step then
    local si = t:add(F.cnt_step, tvb(6, 1), c.step)
    if m.cyc > 0 then
      if c.step == 0 or c.step > 3 then si:add_proto_expert_info(E_CNT)
      elseif c.step > 1 then si:add_proto_expert_info(E_LOSS) end
    elseif c.step == 0 then si:add_proto_expert_info(E_CNT) end
  end
  if c.gap and m.cyc > 0 then
    local gi = t:add(F.period_ms, c.gap)
    if c.gap > 1.5 * m.cyc then gi:add_proto_expert_info(E_GAP) end
  end
  if c.state_change then t:add_proto_expert_info(E_STATE, "ECU state -> " .. (STATE_VALS[c.state] or c.state)) end

  pinfo.cols.info = m.name .. "  " .. table.concat(info, "  ") .. (c.crc_ok and "" or "  [CRC ERROR]")
  return 8
end

local tbl = DissectorTable.get("can.id")""")
    for i in order:
        w(f"tbl:add({i}, p)   -- 0x{i:03X} {msgs[i]['name']}")
    with open(OUT, "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(L) + "\n")
    n = sum(len(m["sigs"]) for m in msgs.values())
    print(f"{OUT}: {len(order)} messages, {n} signals")


if __name__ == "__main__":
    main()
