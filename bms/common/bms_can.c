/* bms_can.c -- bms.dbc kodlama/çözme, CRC-8 SAE J1850, alım izleyicisi. */
#include <math.h>
#include "bms_can.h"

#define MAX_DELTA 3     /* sayaç farkı 2..3: kayıp çerçeve(ler) ama kabul; >3: dizi hatası */

uint8_t crc8_j1850(const uint8_t *data, int len)
{
    uint8_t crc = 0xFF;
    for (int i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x1D) : (uint8_t)(crc << 1);
    }
    return (uint8_t)(crc ^ 0xFF);
}

/* CRC kapsamı: [ID düşük bayt, ID yüksek bayt, veri 0..6] (AUTOSAR E2E "data ID" fikri:
 * aynı veri başka bir kimlikle gelirse CRC tutmaz) */
static uint8_t frame_crc(uint16_t id, const uint8_t d[8])
{
    uint8_t buf[9] = { (uint8_t)(id & 0xFF), (uint8_t)(id >> 8) };
    for (int i = 0; i < 7; i++) buf[2 + i] = d[i];
    return crc8_j1850(buf, 9);
}

static int32_t sat_round(float x, float scale, int32_t lo, int32_t hi)
{
    float r = roundf(x / scale);
    if (r < (float)lo) return lo;
    if (r > (float)hi) return hi;
    return (int32_t)r;
}
static void put16(uint8_t *d, int pos, int32_t v) { d[pos] = (uint8_t)(v & 0xFF); d[pos + 1] = (uint8_t)((v >> 8) & 0xFF); }
static uint16_t getu16(const uint8_t *d, int pos) { return (uint16_t)(d[pos] | (d[pos + 1] << 8)); }
static int16_t gets16(const uint8_t *d, int pos) { return (int16_t)getu16(d, pos); }

static void finish(uint16_t id, uint8_t counter, uint8_t d[8])
{
    d[6] = (uint8_t)(counter & 0x0F);       /* üst 4 bit ayrılmış (0) */
    d[7] = frame_crc(id, d);
}
static int check(uint16_t id, const uint8_t d[8], uint8_t *counter)
{
    if (frame_crc(id, d) != d[7]) return 0;
    *counter = d[6] & 0x0F;
    return 1;
}

/* ---------------------------------------------------------------- kodlama */
void can_encode_meas1(const can_meas1_t *m, uint8_t counter, uint8_t d[8])
{
    put16(d, 0, sat_round(m->V, 0.1f, 0, 65535));
    put16(d, 2, sat_round(m->I, 0.1f, -32768, 32767));
    put16(d, 4, sat_round(m->T, 0.1f, -32768, 32767));
    finish(CAN_ID_EMU_MEAS1, counter, d);
}
void can_encode_meas2(const can_meas2_t *m, uint8_t counter, uint8_t d[8])
{
    put16(d, 0, sat_round(m->V_min, 0.001f, 0, 65535));
    put16(d, 2, sat_round(m->V_max, 0.001f, 0, 65535));
    put16(d, 4, sat_round(m->T_amb, 0.1f, -32768, 32767));
    finish(CAN_ID_EMU_MEAS2, counter, d);
}
void can_encode_ctrl(const can_ctrl_t *m, uint8_t counter, uint8_t d[8])
{
    put16(d, 0, sat_round(m->I_allow_dis, 0.1f, 0, 65535));
    put16(d, 2, sat_round(m->I_allow_chg, 0.1f, 0, 65535));
    d[4] = (uint8_t)(m->state & 0x0F);
    d[5] = m->cause;
    finish(CAN_ID_ECU_CTRL, counter, d);
}
void can_encode_status(const can_status_t *m, uint8_t counter, uint8_t d[8])
{
    put16(d, 0, sat_round(m->soc_pct, 0.01f, 0, 65535));
    put16(d, 2, sat_round(m->tc_hat, 0.1f, -32768, 32767));
    d[4] = (uint8_t)sat_round(m->soh_pct, 0.5f, 0, 255);
    d[5] = (uint8_t)sat_round(m->psi, 0.005f, 0, 255);
    finish(CAN_ID_ECU_STATUS, counter, d);
}
void can_encode_cmd(const can_cmd_t *m, uint8_t counter, uint8_t d[8])
{
    d[0] = m->cmd;
    put16(d, 1, m->value);
    d[3] = d[4] = d[5] = 0;
    finish(CAN_ID_PC_CMD, counter, d);
}

/* ---------------------------------------------------------------- çözme */
int can_decode_meas1(const uint8_t d[8], can_meas1_t *m, uint8_t *counter)
{
    if (!check(CAN_ID_EMU_MEAS1, d, counter)) return 0;
    m->V = 0.1f * getu16(d, 0);  m->I = 0.1f * gets16(d, 2);  m->T = 0.1f * gets16(d, 4);
    return 1;
}
int can_decode_meas2(const uint8_t d[8], can_meas2_t *m, uint8_t *counter)
{
    if (!check(CAN_ID_EMU_MEAS2, d, counter)) return 0;
    m->V_min = 0.001f * getu16(d, 0);  m->V_max = 0.001f * getu16(d, 2);  m->T_amb = 0.1f * gets16(d, 4);
    return 1;
}
int can_decode_ctrl(const uint8_t d[8], can_ctrl_t *m, uint8_t *counter)
{
    if (!check(CAN_ID_ECU_CTRL, d, counter)) return 0;
    m->I_allow_dis = 0.1f * getu16(d, 0);  m->I_allow_chg = 0.1f * getu16(d, 2);
    m->state = d[4] & 0x0F;  m->cause = d[5];
    return 1;
}
int can_decode_status(const uint8_t d[8], can_status_t *m, uint8_t *counter)
{
    if (!check(CAN_ID_ECU_STATUS, d, counter)) return 0;
    m->soc_pct = 0.01f * getu16(d, 0);  m->tc_hat = 0.1f * gets16(d, 2);
    m->soh_pct = 0.5f * d[4];  m->psi = 0.005f * d[5];
    return 1;
}
int can_decode_cmd(const uint8_t d[8], can_cmd_t *m, uint8_t *counter)
{
    if (!check(CAN_ID_PC_CMD, d, counter)) return 0;
    m->cmd = d[0];  m->value = (int16_t)(d[1] | (d[2] << 8));
    return 1;
}

/* ---------------------------------------------------------------- izleyici */
void can_monitor_init(can_monitor_t *mon, uint32_t now_ms)
{
    *mon = (can_monitor_t){0};
    mon->last_ok_ms = now_ms;
}

int can_monitor_rx(can_monitor_t *mon, uint32_t now_ms, int crc_ok, uint8_t counter)
{
    if (!crc_ok) { mon->n_crc_err++; return 0; }
    if (mon->have_counter) {
        uint8_t delta = (uint8_t)((counter - mon->last_counter) & 0x0F);
        mon->last_counter = counter;
        if (delta == 0) { mon->n_cnt_err++; return 0; }            /* donmuş sayaç / tekrar */
        if (delta > MAX_DELTA) { mon->n_cnt_err++; return 0; }    /* dizi hatası (yeniden eşitlenir) */
        mon->n_lost += (uint32_t)(delta - 1);
    } else {
        mon->have_counter = 1;
        mon->last_counter = counter;
    }
    mon->n_ok++;
    mon->last_ok_ms = now_ms;
    return 1;
}

void can_monitor_tick(can_monitor_t *mon, uint32_t now_ms)
{
    uint32_t age = now_ms - mon->last_ok_ms;
    mon->fault = age >= CAN_TIMEOUT_FAULT_MS;
    if (age >= CAN_TIMEOUT_SAFE_MS) mon->safe = 1;               /* kalıcı */
}
