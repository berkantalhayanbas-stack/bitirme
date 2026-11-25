-- bms_dbc.lua -- ÜRETİLMİŞ DOSYA (tools/can/dbc_to_lua.py, kaynak tools/can/bms.dbc); elle düzenleme.
-- Wireshark CAN çözücüsü: sinyaller, değer tabloları, CRC-8 / sayaç / periyot kontrolü (expert info).
local p = Proto("bms", "BMS ECU CAN (bms.dbc)")
local F = {}
F.msg = ProtoField.string("bms.msg", "Message")
F.crc_ok = ProtoField.bool("bms.crc_ok", "CRC-8 ok")
F.cnt_step = ProtoField.int8("bms.cnt_step", "Counter step")
F.period_ms = ProtoField.double("bms.period_ms", "Gap since previous [ms]")
F.frame_bits = ProtoField.uint16("bms.frame_bits", "Bits on wire (stuffing incl., + 3 IFS)")
F.Pack_V = ProtoField.double("bms.Pack_V", "Pack_V [V]")
F.Pack_I = ProtoField.double("bms.Pack_I", "Pack_I [A]")
F.Pack_T = ProtoField.double("bms.Pack_T", "Pack_T [degC]")
F.Meas1_Counter = ProtoField.uint32("bms.Meas1_Counter", "Meas1_Counter")
F.Meas1_CRC = ProtoField.uint32("bms.Meas1_CRC", "Meas1_CRC")
F.Cell_V_min = ProtoField.double("bms.Cell_V_min", "Cell_V_min [V]")
F.Cell_V_max = ProtoField.double("bms.Cell_V_max", "Cell_V_max [V]")
F.Ambient_T = ProtoField.double("bms.Ambient_T", "Ambient_T [degC]")
F.Meas2_Counter = ProtoField.uint32("bms.Meas2_Counter", "Meas2_Counter")
F.Meas2_CRC = ProtoField.uint32("bms.Meas2_CRC", "Meas2_CRC")
F.Group01_V = ProtoField.double("bms.Group01_V", "Group01_V [V]")
F.Group02_V = ProtoField.double("bms.Group02_V", "Group02_V [V]")
F.Group03_V = ProtoField.double("bms.Group03_V", "Group03_V [V]")
F.Group04_V = ProtoField.double("bms.Group04_V", "Group04_V [V]")
F.Cells0_Counter = ProtoField.uint32("bms.Cells0_Counter", "Cells0_Counter")
F.Cells0_CRC = ProtoField.uint32("bms.Cells0_CRC", "Cells0_CRC")
F.Group01_BalFb = ProtoField.uint32("bms.Group01_BalFb", "Group01_BalFb")
F.Group02_BalFb = ProtoField.uint32("bms.Group02_BalFb", "Group02_BalFb")
F.Group03_BalFb = ProtoField.uint32("bms.Group03_BalFb", "Group03_BalFb")
F.Group04_BalFb = ProtoField.uint32("bms.Group04_BalFb", "Group04_BalFb")
F.Group05_V = ProtoField.double("bms.Group05_V", "Group05_V [V]")
F.Group06_V = ProtoField.double("bms.Group06_V", "Group06_V [V]")
F.Group07_V = ProtoField.double("bms.Group07_V", "Group07_V [V]")
F.Group08_V = ProtoField.double("bms.Group08_V", "Group08_V [V]")
F.Cells1_Counter = ProtoField.uint32("bms.Cells1_Counter", "Cells1_Counter")
F.Cells1_CRC = ProtoField.uint32("bms.Cells1_CRC", "Cells1_CRC")
F.Group05_BalFb = ProtoField.uint32("bms.Group05_BalFb", "Group05_BalFb")
F.Group06_BalFb = ProtoField.uint32("bms.Group06_BalFb", "Group06_BalFb")
F.Group07_BalFb = ProtoField.uint32("bms.Group07_BalFb", "Group07_BalFb")
F.Group08_BalFb = ProtoField.uint32("bms.Group08_BalFb", "Group08_BalFb")
F.Group09_V = ProtoField.double("bms.Group09_V", "Group09_V [V]")
F.Group10_V = ProtoField.double("bms.Group10_V", "Group10_V [V]")
F.Group11_V = ProtoField.double("bms.Group11_V", "Group11_V [V]")
F.Group12_V = ProtoField.double("bms.Group12_V", "Group12_V [V]")
F.Cells2_Counter = ProtoField.uint32("bms.Cells2_Counter", "Cells2_Counter")
F.Cells2_CRC = ProtoField.uint32("bms.Cells2_CRC", "Cells2_CRC")
F.Group09_BalFb = ProtoField.uint32("bms.Group09_BalFb", "Group09_BalFb")
F.Group10_BalFb = ProtoField.uint32("bms.Group10_BalFb", "Group10_BalFb")
F.Group11_BalFb = ProtoField.uint32("bms.Group11_BalFb", "Group11_BalFb")
F.Group12_BalFb = ProtoField.uint32("bms.Group12_BalFb", "Group12_BalFb")
F.Group13_V = ProtoField.double("bms.Group13_V", "Group13_V [V]")
F.Group14_V = ProtoField.double("bms.Group14_V", "Group14_V [V]")
F.Group15_V = ProtoField.double("bms.Group15_V", "Group15_V [V]")
F.Group16_V = ProtoField.double("bms.Group16_V", "Group16_V [V]")
F.Cells3_Counter = ProtoField.uint32("bms.Cells3_Counter", "Cells3_Counter")
F.Cells3_CRC = ProtoField.uint32("bms.Cells3_CRC", "Cells3_CRC")
F.Group13_BalFb = ProtoField.uint32("bms.Group13_BalFb", "Group13_BalFb")
F.Group14_BalFb = ProtoField.uint32("bms.Group14_BalFb", "Group14_BalFb")
F.Group15_BalFb = ProtoField.uint32("bms.Group15_BalFb", "Group15_BalFb")
F.Group16_BalFb = ProtoField.uint32("bms.Group16_BalFb", "Group16_BalFb")
F.Group17_V = ProtoField.double("bms.Group17_V", "Group17_V [V]")
F.Group18_V = ProtoField.double("bms.Group18_V", "Group18_V [V]")
F.Group19_V = ProtoField.double("bms.Group19_V", "Group19_V [V]")
F.Cells4_Spare = ProtoField.uint32("bms.Cells4_Spare", "Cells4_Spare")
F.Cells4_Counter = ProtoField.uint32("bms.Cells4_Counter", "Cells4_Counter")
F.Cells4_CRC = ProtoField.uint32("bms.Cells4_CRC", "Cells4_CRC")
F.Group17_BalFb = ProtoField.uint32("bms.Group17_BalFb", "Group17_BalFb")
F.Group18_BalFb = ProtoField.uint32("bms.Group18_BalFb", "Group18_BalFb")
F.Group19_BalFb = ProtoField.uint32("bms.Group19_BalFb", "Group19_BalFb")
F.I_allow_dis = ProtoField.double("bms.I_allow_dis", "I_allow_dis [A]")
F.I_allow_chg = ProtoField.double("bms.I_allow_chg", "I_allow_chg [A]")
F.State = ProtoField.uint32("bms.State", "State", base.DEC, {[1] = "NORMAL", [2] = "DERATE", [3] = "LIMP_HOME", [4] = "SAFE_STOP"})
F.Cause = ProtoField.uint32("bms.Cause", "Cause")
F.Ctrl_Counter = ProtoField.uint32("bms.Ctrl_Counter", "Ctrl_Counter")
F.Ctrl_CRC = ProtoField.uint32("bms.Ctrl_CRC", "Ctrl_CRC")
F.SOC = ProtoField.double("bms.SOC", "SOC [%]")
F.Tcore_est = ProtoField.double("bms.Tcore_est", "Tcore_est [degC]")
F.SOH = ProtoField.double("bms.SOH", "SOH [%]")
F.Psi = ProtoField.double("bms.Psi", "Psi")
F.Status_Counter = ProtoField.uint32("bms.Status_Counter", "Status_Counter")
F.Status_CRC = ProtoField.uint32("bms.Status_CRC", "Status_CRC")
F.Bal_Group01 = ProtoField.uint32("bms.Bal_Group01", "Bal_Group01")
F.Bal_Group02 = ProtoField.uint32("bms.Bal_Group02", "Bal_Group02")
F.Bal_Group03 = ProtoField.uint32("bms.Bal_Group03", "Bal_Group03")
F.Bal_Group04 = ProtoField.uint32("bms.Bal_Group04", "Bal_Group04")
F.Bal_Group05 = ProtoField.uint32("bms.Bal_Group05", "Bal_Group05")
F.Bal_Group06 = ProtoField.uint32("bms.Bal_Group06", "Bal_Group06")
F.Bal_Group07 = ProtoField.uint32("bms.Bal_Group07", "Bal_Group07")
F.Bal_Group08 = ProtoField.uint32("bms.Bal_Group08", "Bal_Group08")
F.Bal_Group09 = ProtoField.uint32("bms.Bal_Group09", "Bal_Group09")
F.Bal_Group10 = ProtoField.uint32("bms.Bal_Group10", "Bal_Group10")
F.Bal_Group11 = ProtoField.uint32("bms.Bal_Group11", "Bal_Group11")
F.Bal_Group12 = ProtoField.uint32("bms.Bal_Group12", "Bal_Group12")
F.Bal_Group13 = ProtoField.uint32("bms.Bal_Group13", "Bal_Group13")
F.Bal_Group14 = ProtoField.uint32("bms.Bal_Group14", "Bal_Group14")
F.Bal_Group15 = ProtoField.uint32("bms.Bal_Group15", "Bal_Group15")
F.Bal_Group16 = ProtoField.uint32("bms.Bal_Group16", "Bal_Group16")
F.Bal_Group17 = ProtoField.uint32("bms.Bal_Group17", "Bal_Group17")
F.Bal_Group18 = ProtoField.uint32("bms.Bal_Group18", "Bal_Group18")
F.Bal_Group19 = ProtoField.uint32("bms.Bal_Group19", "Bal_Group19")
F.Bal_Active = ProtoField.uint32("bms.Bal_Active", "Bal_Active")
F.Bal_Stale = ProtoField.uint32("bms.Bal_Stale", "Bal_Stale")
F.Bal_Drive = ProtoField.uint32("bms.Bal_Drive", "Bal_Drive")
F.Bal_Blocked = ProtoField.uint32("bms.Bal_Blocked", "Bal_Blocked")
F.Cell_dV = ProtoField.double("bms.Cell_dV", "Cell_dV [V]")
F.Bal_Counter = ProtoField.uint32("bms.Bal_Counter", "Bal_Counter")
F.Bal_CRC = ProtoField.uint32("bms.Bal_CRC", "Bal_CRC")
F.Cmd = ProtoField.uint32("bms.Cmd", "Cmd", base.DEC, {[1] = "ECU_RESTART", [2] = "ECU_SET_SOC", [3] = "ECU_CLR_STATS", [4] = "ECU_HANG", [16] = "EMU_CLEAR", [17] = "EMU_CRC", [18] = "EMU_FREEZE", [19] = "EMU_SILENT", [20] = "EMU_NTC_OPEN"})
F.Cmd_Value = ProtoField.double("bms.Cmd_Value", "Cmd_Value")
F.Cmd_Counter = ProtoField.uint32("bms.Cmd_Counter", "Cmd_Counter")
F.Cmd_CRC = ProtoField.uint32("bms.Cmd_CRC", "Cmd_CRC")
local fl = {}
for _, v in pairs(F) do fl[#fl + 1] = v end
p.fields = fl
local E_CRC = ProtoExpert.new("bms.crc_bad", "CRC-8 mismatch", expert.group.CHECKSUM, expert.severity.ERROR)
local E_CNT = ProtoExpert.new("bms.cnt_bad", "Counter step rejected (0 or > 3)", expert.group.SEQUENCE, expert.severity.ERROR)
local E_LOSS = ProtoExpert.new("bms.cnt_loss", "Counter step 2..3 (frame loss)", expert.group.SEQUENCE, expert.severity.WARN)
local E_GAP = ProtoExpert.new("bms.gap", "Gap > 1.5 x cycle time", expert.group.SEQUENCE, expert.severity.WARN)
local E_INV = ProtoExpert.new("bms.invalid", "Signal marked invalid", expert.group.MALFORMED, expert.severity.NOTE)
local E_STATE = ProtoExpert.new("bms.state_change", "ECU state change", expert.group.SEQUENCE, expert.severity.CHAT)
p.experts = { E_CRC, E_CNT, E_LOSS, E_GAP, E_INV, E_STATE }

-- mesaj tablosu: ad, periyot [ms] (0 = olay), sinyaller {alan, ad, başlangıç, uzunluk, işaretli, ölçek, ofset, birim, değerler, bitler}
local M = {
  [257] = { name = "EMU_Meas1", cyc = 100, sigs = {
    { f = F.Pack_V, n = "Pack_V", st = 0, ln = 16, sg = false, sc = 0.1, of = 0.0, u = "V", vals = {}, bits = {} },
    { f = F.Pack_I, n = "Pack_I", st = 16, ln = 16, sg = true, sc = 0.1, of = 0.0, u = "A", vals = {}, bits = {} },
    { f = F.Pack_T, n = "Pack_T", st = 32, ln = 16, sg = true, sc = 0.1, of = 0.0, u = "degC", vals = {}, bits = {} },
    { f = F.Meas1_Counter, n = "Meas1_Counter", st = 48, ln = 4, sg = false, sc = 1.0, of = 0.0, u = "", vals = {}, bits = {} },
    { f = F.Meas1_CRC, n = "Meas1_CRC", st = 56, ln = 8, sg = false, sc = 1.0, of = 0.0, u = "", vals = {}, bits = {} },
  } },
  [258] = { name = "EMU_Meas2", cyc = 100, sigs = {
    { f = F.Cell_V_min, n = "Cell_V_min", st = 0, ln = 16, sg = false, sc = 0.001, of = 0.0, u = "V", vals = {}, bits = {} },
    { f = F.Cell_V_max, n = "Cell_V_max", st = 16, ln = 16, sg = false, sc = 0.001, of = 0.0, u = "V", vals = {}, bits = {} },
    { f = F.Ambient_T, n = "Ambient_T", st = 32, ln = 16, sg = true, sc = 0.1, of = 0.0, u = "degC", vals = {}, bits = {} },
    { f = F.Meas2_Counter, n = "Meas2_Counter", st = 48, ln = 4, sg = false, sc = 1.0, of = 0.0, u = "", vals = {}, bits = {} },
    { f = F.Meas2_CRC, n = "Meas2_CRC", st = 56, ln = 8, sg = false, sc = 1.0, of = 0.0, u = "", vals = {}, bits = {} },
  } },
  [272] = { name = "EMU_Cells0", cyc = 100, sigs = {
    { f = F.Group01_V, n = "Group01_V", st = 0, ln = 12, sg = false, sc = 0.001, of = 1.0, u = "V", vals = {[4095] = "Invalid"}, bits = {} },
    { f = F.Group02_V, n = "Group02_V", st = 12, ln = 12, sg = false, sc = 0.001, of = 1.0, u = "V", vals = {[4095] = "Invalid"}, bits = {} },
    { f = F.Group03_V, n = "Group03_V", st = 24, ln = 12, sg = false, sc = 0.001, of = 1.0, u = "V", vals = {[4095] = "Invalid"}, bits = {} },
    { f = F.Group04_V, n = "Group04_V", st = 36, ln = 12, sg = false, sc = 0.001, of = 1.0, u = "V", vals = {[4095] = "Invalid"}, bits = {} },
    { f = F.Cells0_Counter, n = "Cells0_Counter", st = 48, ln = 4, sg = false, sc = 1.0, of = 0.0, u = "", vals = {}, bits = {} },
    { f = F.Cells0_CRC, n = "Cells0_CRC", st = 56, ln = 8, sg = false, sc = 1.0, of = 0.0, u = "", vals = {}, bits = {} },
    { f = F.Group01_BalFb, n = "Group01_BalFb", st = 52, ln = 1, sg = false, sc = 1.0, of = 0.0, u = "", vals = {}, bits = {} },
    { f = F.Group02_BalFb, n = "Group02_BalFb", st = 53, ln = 1, sg = false, sc = 1.0, of = 0.0, u = "", vals = {}, bits = {} },
    { f = F.Group03_BalFb, n = "Group03_BalFb", st = 54, ln = 1, sg = false, sc = 1.0, of = 0.0, u = "", vals = {}, bits = {} },
    { f = F.Group04_BalFb, n = "Group04_BalFb", st = 55, ln = 1, sg = false, sc = 1.0, of = 0.0, u = "", vals = {}, bits = {} },
  } },
  [273] = { name = "EMU_Cells1", cyc = 100, sigs = {
    { f = F.Group05_V, n = "Group05_V", st = 0, ln = 12, sg = false, sc = 0.001, of = 1.0, u = "V", vals = {[4095] = "Invalid"}, bits = {} },
    { f = F.Group06_V, n = "Group06_V", st = 12, ln = 12, sg = false, sc = 0.001, of = 1.0, u = "V", vals = {[4095] = "Invalid"}, bits = {} },
    { f = F.Group07_V, n = "Group07_V", st = 24, ln = 12, sg = false, sc = 0.001, of = 1.0, u = "V", vals = {[4095] = "Invalid"}, bits = {} },
    { f = F.Group08_V, n = "Group08_V", st = 36, ln = 12, sg = false, sc = 0.001, of = 1.0, u = "V", vals = {[4095] = "Invalid"}, bits = {} },
    { f = F.Cells1_Counter, n = "Cells1_Counter", st = 48, ln = 4, sg = false, sc = 1.0, of = 0.0, u = "", vals = {}, bits = {} },
    { f = F.Cells1_CRC, n = "Cells1_CRC", st = 56, ln = 8, sg = false, sc = 1.0, of = 0.0, u = "", vals = {}, bits = {} },
    { f = F.Group05_BalFb, n = "Group05_BalFb", st = 52, ln = 1, sg = false, sc = 1.0, of = 0.0, u = "", vals = {}, bits = {} },
    { f = F.Group06_BalFb, n = "Group06_BalFb", st = 53, ln = 1, sg = false, sc = 1.0, of = 0.0, u = "", vals = {}, bits = {} },
    { f = F.Group07_BalFb, n = "Group07_BalFb", st = 54, ln = 1, sg = false, sc = 1.0, of = 0.0, u = "", vals = {}, bits = {} },
    { f = F.Group08_BalFb, n = "Group08_BalFb", st = 55, ln = 1, sg = false, sc = 1.0, of = 0.0, u = "", vals = {}, bits = {} },
  } },
  [274] = { name = "EMU_Cells2", cyc = 100, sigs = {
    { f = F.Group09_V, n = "Group09_V", st = 0, ln = 12, sg = false, sc = 0.001, of = 1.0, u = "V", vals = {[4095] = "Invalid"}, bits = {} },
    { f = F.Group10_V, n = "Group10_V", st = 12, ln = 12, sg = false, sc = 0.001, of = 1.0, u = "V", vals = {[4095] = "Invalid"}, bits = {} },
    { f = F.Group11_V, n = "Group11_V", st = 24, ln = 12, sg = false, sc = 0.001, of = 1.0, u = "V", vals = {[4095] = "Invalid"}, bits = {} },
    { f = F.Group12_V, n = "Group12_V", st = 36, ln = 12, sg = false, sc = 0.001, of = 1.0, u = "V", vals = {[4095] = "Invalid"}, bits = {} },
    { f = F.Cells2_Counter, n = "Cells2_Counter", st = 48, ln = 4, sg = false, sc = 1.0, of = 0.0, u = "", vals = {}, bits = {} },
    { f = F.Cells2_CRC, n = "Cells2_CRC", st = 56, ln = 8, sg = false, sc = 1.0, of = 0.0, u = "", vals = {}, bits = {} },
    { f = F.Group09_BalFb, n = "Group09_BalFb", st = 52, ln = 1, sg = false, sc = 1.0, of = 0.0, u = "", vals = {}, bits = {} },
    { f = F.Group10_BalFb, n = "Group10_BalFb", st = 53, ln = 1, sg = false, sc = 1.0, of = 0.0, u = "", vals = {}, bits = {} },
    { f = F.Group11_BalFb, n = "Group11_BalFb", st = 54, ln = 1, sg = false, sc = 1.0, of = 0.0, u = "", vals = {}, bits = {} },
    { f = F.Group12_BalFb, n = "Group12_BalFb", st = 55, ln = 1, sg = false, sc = 1.0, of = 0.0, u = "", vals = {}, bits = {} },
  } },
  [275] = { name = "EMU_Cells3", cyc = 100, sigs = {
    { f = F.Group13_V, n = "Group13_V", st = 0, ln = 12, sg = false, sc = 0.001, of = 1.0, u = "V", vals = {[4095] = "Invalid"}, bits = {} },
    { f = F.Group14_V, n = "Group14_V", st = 12, ln = 12, sg = false, sc = 0.001, of = 1.0, u = "V", vals = {[4095] = "Invalid"}, bits = {} },
    { f = F.Group15_V, n = "Group15_V", st = 24, ln = 12, sg = false, sc = 0.001, of = 1.0, u = "V", vals = {[4095] = "Invalid"}, bits = {} },
    { f = F.Group16_V, n = "Group16_V", st = 36, ln = 12, sg = false, sc = 0.001, of = 1.0, u = "V", vals = {[4095] = "Invalid"}, bits = {} },
    { f = F.Cells3_Counter, n = "Cells3_Counter", st = 48, ln = 4, sg = false, sc = 1.0, of = 0.0, u = "", vals = {}, bits = {} },
    { f = F.Cells3_CRC, n = "Cells3_CRC", st = 56, ln = 8, sg = false, sc = 1.0, of = 0.0, u = "", vals = {}, bits = {} },
    { f = F.Group13_BalFb, n = "Group13_BalFb", st = 52, ln = 1, sg = false, sc = 1.0, of = 0.0, u = "", vals = {}, bits = {} },
    { f = F.Group14_BalFb, n = "Group14_BalFb", st = 53, ln = 1, sg = false, sc = 1.0, of = 0.0, u = "", vals = {}, bits = {} },
    { f = F.Group15_BalFb, n = "Group15_BalFb", st = 54, ln = 1, sg = false, sc = 1.0, of = 0.0, u = "", vals = {}, bits = {} },
    { f = F.Group16_BalFb, n = "Group16_BalFb", st = 55, ln = 1, sg = false, sc = 1.0, of = 0.0, u = "", vals = {}, bits = {} },
  } },
  [276] = { name = "EMU_Cells4", cyc = 100, sigs = {
    { f = F.Group17_V, n = "Group17_V", st = 0, ln = 12, sg = false, sc = 0.001, of = 1.0, u = "V", vals = {[4095] = "Invalid"}, bits = {} },
    { f = F.Group18_V, n = "Group18_V", st = 12, ln = 12, sg = false, sc = 0.001, of = 1.0, u = "V", vals = {[4095] = "Invalid"}, bits = {} },
    { f = F.Group19_V, n = "Group19_V", st = 24, ln = 12, sg = false, sc = 0.001, of = 1.0, u = "V", vals = {[4095] = "Invalid"}, bits = {} },
    { f = F.Cells4_Spare, n = "Cells4_Spare", st = 36, ln = 12, sg = false, sc = 1.0, of = 0.0, u = "", vals = {}, bits = {} },
    { f = F.Cells4_Counter, n = "Cells4_Counter", st = 48, ln = 4, sg = false, sc = 1.0, of = 0.0, u = "", vals = {}, bits = {} },
    { f = F.Cells4_CRC, n = "Cells4_CRC", st = 56, ln = 8, sg = false, sc = 1.0, of = 0.0, u = "", vals = {}, bits = {} },
    { f = F.Group17_BalFb, n = "Group17_BalFb", st = 52, ln = 1, sg = false, sc = 1.0, of = 0.0, u = "", vals = {}, bits = {} },
    { f = F.Group18_BalFb, n = "Group18_BalFb", st = 53, ln = 1, sg = false, sc = 1.0, of = 0.0, u = "", vals = {}, bits = {} },
    { f = F.Group19_BalFb, n = "Group19_BalFb", st = 54, ln = 1, sg = false, sc = 1.0, of = 0.0, u = "", vals = {}, bits = {} },
  } },
  [513] = { name = "ECU_Ctrl", cyc = 100, sigs = {
    { f = F.I_allow_dis, n = "I_allow_dis", st = 0, ln = 16, sg = false, sc = 0.1, of = 0.0, u = "A", vals = {}, bits = {} },
    { f = F.I_allow_chg, n = "I_allow_chg", st = 16, ln = 16, sg = false, sc = 0.1, of = 0.0, u = "A", vals = {}, bits = {} },
    { f = F.State, n = "State", st = 32, ln = 4, sg = false, sc = 1.0, of = 0.0, u = "", vals = {[1] = "NORMAL", [2] = "DERATE", [3] = "LIMP_HOME", [4] = "SAFE_STOP"}, bits = {} },
    { f = F.Cause, n = "Cause", st = 40, ln = 8, sg = false, sc = 1.0, of = 0.0, u = "", vals = {}, bits = {{1, "thermal derate"}, {2, "cooling alarm"}, {4, "SOP discharge"}, {8, "SOP charge"}, {16, "datasheet T limit"}, {32, "sensor fault"}, {64, "SOC empty"}, {128, "hard limit (SAFE)"}} },
    { f = F.Ctrl_Counter, n = "Ctrl_Counter", st = 48, ln = 4, sg = false, sc = 1.0, of = 0.0, u = "", vals = {}, bits = {} },
    { f = F.Ctrl_CRC, n = "Ctrl_CRC", st = 56, ln = 8, sg = false, sc = 1.0, of = 0.0, u = "", vals = {}, bits = {} },
  } },
  [514] = { name = "ECU_Status", cyc = 100, sigs = {
    { f = F.SOC, n = "SOC", st = 0, ln = 16, sg = false, sc = 0.01, of = 0.0, u = "%", vals = {}, bits = {} },
    { f = F.Tcore_est, n = "Tcore_est", st = 16, ln = 16, sg = true, sc = 0.1, of = 0.0, u = "degC", vals = {}, bits = {} },
    { f = F.SOH, n = "SOH", st = 32, ln = 8, sg = false, sc = 0.5, of = 0.0, u = "%", vals = {}, bits = {} },
    { f = F.Psi, n = "Psi", st = 40, ln = 8, sg = false, sc = 0.005, of = 0.0, u = "", vals = {}, bits = {} },
    { f = F.Status_Counter, n = "Status_Counter", st = 48, ln = 4, sg = false, sc = 1.0, of = 0.0, u = "", vals = {}, bits = {} },
    { f = F.Status_CRC, n = "Status_CRC", st = 56, ln = 8, sg = false, sc = 1.0, of = 0.0, u = "", vals = {}, bits = {} },
  } },
  [528] = { name = "ECU_Bal", cyc = 100, sigs = {
    { f = F.Bal_Group01, n = "Bal_Group01", st = 0, ln = 1, sg = false, sc = 1.0, of = 0.0, u = "", vals = {}, bits = {} },
    { f = F.Bal_Group02, n = "Bal_Group02", st = 1, ln = 1, sg = false, sc = 1.0, of = 0.0, u = "", vals = {}, bits = {} },
    { f = F.Bal_Group03, n = "Bal_Group03", st = 2, ln = 1, sg = false, sc = 1.0, of = 0.0, u = "", vals = {}, bits = {} },
    { f = F.Bal_Group04, n = "Bal_Group04", st = 3, ln = 1, sg = false, sc = 1.0, of = 0.0, u = "", vals = {}, bits = {} },
    { f = F.Bal_Group05, n = "Bal_Group05", st = 4, ln = 1, sg = false, sc = 1.0, of = 0.0, u = "", vals = {}, bits = {} },
    { f = F.Bal_Group06, n = "Bal_Group06", st = 5, ln = 1, sg = false, sc = 1.0, of = 0.0, u = "", vals = {}, bits = {} },
    { f = F.Bal_Group07, n = "Bal_Group07", st = 6, ln = 1, sg = false, sc = 1.0, of = 0.0, u = "", vals = {}, bits = {} },
    { f = F.Bal_Group08, n = "Bal_Group08", st = 7, ln = 1, sg = false, sc = 1.0, of = 0.0, u = "", vals = {}, bits = {} },
    { f = F.Bal_Group09, n = "Bal_Group09", st = 8, ln = 1, sg = false, sc = 1.0, of = 0.0, u = "", vals = {}, bits = {} },
    { f = F.Bal_Group10, n = "Bal_Group10", st = 9, ln = 1, sg = false, sc = 1.0, of = 0.0, u = "", vals = {}, bits = {} },
    { f = F.Bal_Group11, n = "Bal_Group11", st = 10, ln = 1, sg = false, sc = 1.0, of = 0.0, u = "", vals = {}, bits = {} },
    { f = F.Bal_Group12, n = "Bal_Group12", st = 11, ln = 1, sg = false, sc = 1.0, of = 0.0, u = "", vals = {}, bits = {} },
    { f = F.Bal_Group13, n = "Bal_Group13", st = 12, ln = 1, sg = false, sc = 1.0, of = 0.0, u = "", vals = {}, bits = {} },
    { f = F.Bal_Group14, n = "Bal_Group14", st = 13, ln = 1, sg = false, sc = 1.0, of = 0.0, u = "", vals = {}, bits = {} },
    { f = F.Bal_Group15, n = "Bal_Group15", st = 14, ln = 1, sg = false, sc = 1.0, of = 0.0, u = "", vals = {}, bits = {} },
    { f = F.Bal_Group16, n = "Bal_Group16", st = 15, ln = 1, sg = false, sc = 1.0, of = 0.0, u = "", vals = {}, bits = {} },
    { f = F.Bal_Group17, n = "Bal_Group17", st = 16, ln = 1, sg = false, sc = 1.0, of = 0.0, u = "", vals = {}, bits = {} },
    { f = F.Bal_Group18, n = "Bal_Group18", st = 17, ln = 1, sg = false, sc = 1.0, of = 0.0, u = "", vals = {}, bits = {} },
    { f = F.Bal_Group19, n = "Bal_Group19", st = 18, ln = 1, sg = false, sc = 1.0, of = 0.0, u = "", vals = {}, bits = {} },
    { f = F.Bal_Active, n = "Bal_Active", st = 24, ln = 1, sg = false, sc = 1.0, of = 0.0, u = "", vals = {}, bits = {} },
    { f = F.Bal_Stale, n = "Bal_Stale", st = 25, ln = 1, sg = false, sc = 1.0, of = 0.0, u = "", vals = {}, bits = {} },
    { f = F.Bal_Drive, n = "Bal_Drive", st = 26, ln = 1, sg = false, sc = 1.0, of = 0.0, u = "", vals = {}, bits = {} },
    { f = F.Bal_Blocked, n = "Bal_Blocked", st = 27, ln = 1, sg = false, sc = 1.0, of = 0.0, u = "", vals = {}, bits = {} },
    { f = F.Cell_dV, n = "Cell_dV", st = 32, ln = 16, sg = false, sc = 0.0001, of = 0.0, u = "V", vals = {}, bits = {} },
    { f = F.Bal_Counter, n = "Bal_Counter", st = 48, ln = 4, sg = false, sc = 1.0, of = 0.0, u = "", vals = {}, bits = {} },
    { f = F.Bal_CRC, n = "Bal_CRC", st = 56, ln = 8, sg = false, sc = 1.0, of = 0.0, u = "", vals = {}, bits = {} },
  } },
  [769] = { name = "PC_Cmd", cyc = 0, sigs = {
    { f = F.Cmd, n = "Cmd", st = 0, ln = 8, sg = false, sc = 1.0, of = 0.0, u = "", vals = {[1] = "ECU_RESTART", [2] = "ECU_SET_SOC", [3] = "ECU_CLR_STATS", [4] = "ECU_HANG", [16] = "EMU_CLEAR", [17] = "EMU_CRC", [18] = "EMU_FREEZE", [19] = "EMU_SILENT", [20] = "EMU_NTC_OPEN"}, bits = {} },
    { f = F.Cmd_Value, n = "Cmd_Value", st = 8, ln = 16, sg = true, sc = 1.0, of = 0.0, u = "", vals = {}, bits = {} },
    { f = F.Cmd_Counter, n = "Cmd_Counter", st = 48, ln = 4, sg = false, sc = 1.0, of = 0.0, u = "", vals = {}, bits = {} },
    { f = F.Cmd_CRC, n = "Cmd_CRC", st = 56, ln = 8, sg = false, sc = 1.0, of = 0.0, u = "", vals = {}, bits = {} },
  } },
}

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

local tbl = DissectorTable.get("can.id")
tbl:add(257, p)   -- 0x101 EMU_Meas1
tbl:add(258, p)   -- 0x102 EMU_Meas2
tbl:add(272, p)   -- 0x110 EMU_Cells0
tbl:add(273, p)   -- 0x111 EMU_Cells1
tbl:add(274, p)   -- 0x112 EMU_Cells2
tbl:add(275, p)   -- 0x113 EMU_Cells3
tbl:add(276, p)   -- 0x114 EMU_Cells4
tbl:add(513, p)   -- 0x201 ECU_Ctrl
tbl:add(514, p)   -- 0x202 ECU_Status
tbl:add(528, p)   -- 0x210 ECU_Bal
tbl:add(769, p)   -- 0x301 PC_Cmd
