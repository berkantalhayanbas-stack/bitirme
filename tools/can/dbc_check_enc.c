/* dbc_check_enc.c -- bms_can.c ile rastgele çerçeveler (tools/can/dbc_check.py derler): "id hex v1 v2 ..." */
#include <stdio.h>
#include <stdlib.h>
#include "bms_can.h"
static float u(float a, float b) { return a + (b - a) * (float)rand() / (float)RAND_MAX; }
static void pr(unsigned id, const uint8_t *d) { printf("%u ", id); for (int i = 0; i < 8; i++) printf("%02X", d[i]); }
int main(void) {
  srand(7); uint8_t d[8];
  for (int n = 0; n < 2000; n++) {
    uint8_t c = (uint8_t)(n & 15);
    can_meas1_t m1 = { u(0, 100), u(-400, 400), u(-40, 85) }; can_encode_meas1(&m1, c, d); pr(0x101, d); printf(" %f %f %f\n", m1.V, m1.I, m1.T);
    can_meas2_t m2 = { u(2.5f, 4.2f), u(2.5f, 4.2f), u(-40, 60) }; can_encode_meas2(&m2, c, d); pr(0x102, d); printf(" %f %f %f\n", m2.V_min, m2.V_max, m2.T_amb);
    can_ctrl_t ct = { u(0, 90), u(0, 41), (uint8_t)(1 + rand() % 4), (uint8_t)(rand() & 255) }; can_encode_ctrl(&ct, c, d); pr(0x201, d); printf(" %f %f %u %u\n", ct.I_allow_dis, ct.I_allow_chg, ct.state, ct.cause);
    can_status_t st = { u(0, 100), u(-20, 80), u(60, 100), u(0, 1) }; can_encode_status(&st, c, d); pr(0x202, d); printf(" %f %f %f %f\n", st.soc_pct, st.tc_hat, st.soh_pct, st.psi);
    can_cmd_t cm = { (uint8_t)(rand() & 31), (int16_t)(rand() - 16384) }; can_encode_cmd(&cm, c, d); pr(0x301, d); printf(" %u %d\n", cm.cmd, cm.value);
    uint8_t k = (uint8_t)(n % 5); can_cell_t ce = { { u(2.8f, 4.2f), u(2.8f, 4.2f), u(2.8f, 4.2f), u(2.8f, 4.2f) }, (uint8_t)(rand() & 15), (uint8_t)(rand() & 15) };
    if (k == 4) ce.valid &= 7;
    can_encode_cell(k, &ce, c, d); pr(0x110 + k, d); printf(" %f %f %f %f %u %u\n", ce.v[0], ce.v[1], ce.v[2], ce.v[3], ce.valid, ce.bal);
    can_bal_t b = { (uint32_t)(rand() * 31u) & 0x7FFFF, (uint8_t)(rand() & 15), u(0, 0.2f) }; can_encode_bal(&b, c, d); pr(0x210, d); printf(" %lu %u %f\n", (unsigned long)b.mask, b.flags, b.dv);
  }
  return 0;
}
