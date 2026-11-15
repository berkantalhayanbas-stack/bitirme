/* bsp_cordic.c -- CORDIC ile expf / logf (bkz. bsp_cordic.h). */
#include <string.h>
#include "stm32g4xx.h"
#include "bsp_cordic.h"

#define FUNC_HCOS   5u
#define FUNC_LN     8u
#define Q31         2147483648.0f
#define LN2_HI      0.693145751953125f          /* ln2'nin ilk 16 biti: k·LN2_HI tam çıkar */
#define LN2_LO      1.42860682030941723212e-6f
#define LOG2E       1.44269504088896341f
#define LN2         0.693147180559945309f

static uint32_t s_prec = 6u;

void bsp_cordic_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_CORDICEN;
    (void)RCC->AHB1ENR;
}

void bsp_cordic_precision(uint32_t cycles)
{
    s_prec = cycles < 1u ? 1u : cycles > 15u ? 15u : cycles;
}

static inline float pow2i(int32_t k)             /* 2^k, k ∈ [-126, 127] (normal float) */
{
    const uint32_t b = (uint32_t)(k + 127) << 23;
    float f;
    memcpy(&f, &b, 4);
    return f;
}

float bsp_cordic_expf(float x)
{
    if (x > 88.0f)  x = 88.0f;
    if (x < -87.0f) x = -87.0f;
    const float kf = x * LOG2E;
    const int32_t k = (int32_t)(kf + (kf >= 0.0f ? 0.5f : -0.5f));
    const float r = (x - (float)k * LN2_HI) - (float)k * LN2_LO;     /* |r| <= 0,347 */

    CORDIC->CSR = FUNC_HCOS | (s_prec << CORDIC_CSR_PRECISION_Pos) | (1u << CORDIC_CSR_SCALE_Pos)
                | CORDIC_CSR_NRES;                                    /* 1 argüman, 2 sonuç, q1.31 */
    CORDIC->WDATA = (uint32_t)(int32_t)(r * (0.5f * Q31));
    const int32_t ch = (int32_t)CORDIC->RDATA;                        /* cosh(r)/2 (hazır olana dek bekler) */
    const int32_t sh = (int32_t)CORDIC->RDATA;                        /* sinh(r)/2 */
    const float er = (float)(ch + sh) * (2.0f / Q31);
    return er * pow2i(k);
}

float bsp_cordic_logf(float x)
{
    if (!(x > 0.0f)) return -1.0e30f;                                 /* tanımsız: çok negatif */
    uint32_t b;
    memcpy(&b, &x, 4);
    const int32_t e = (int32_t)((b >> 23) & 0xFFu) - 126;             /* x = m·2^e, m ∈ [0,5, 1) */
    b = (b & 0x007FFFFFu) | (126u << 23);
    float m;
    memcpy(&m, &b, 4);

    CORDIC->CSR = FUNC_LN | (s_prec << CORDIC_CSR_PRECISION_Pos) | (1u << CORDIC_CSR_SCALE_Pos);
    CORDIC->WDATA = (uint32_t)(int32_t)(m * (0.5f * Q31));            /* m·2^-1 */
    const int32_t res = (int32_t)CORDIC->RDATA;                       /* ln(m)·2^-2 */
    return (float)res * (4.0f / Q31) + (float)e * LN2;
}
