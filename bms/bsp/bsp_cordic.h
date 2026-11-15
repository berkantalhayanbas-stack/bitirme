/* bsp_cordic.h -- STM32G4 CORDIC hızlandırıcısıyla expf / logf (kayıt seviyesinde, HAL yok).
 *
 * exp(x): argüman indirgeme x = k·ln2 + r, |r| <= ln2/2 (= 0,347; hiperbolik CORDIC |r| < 1,118 ister);
 *         CORDIC "hyperbolic cosine" (FUNC 5, SCALE 1): ARG = r·2^-1 (q1.31), RES1 = cosh(r)·2^-1,
 *         RES2 = sinh(r)·2^-1  ->  e^r = cosh + sinh;  e^x = 2^k · e^r (üs bitleriyle).
 * ln(x):  x = m·2^e, m ∈ [0,5, 1);  CORDIC "natural logarithm" (FUNC 8, SCALE 1):
 *         ARG = m·2^-1 (q1.31), RES1 = ln(m)·2^-2  ->  ln x = 4·RES1 + e·ln2.
 * Kaynak: RM0440 CORDIC bölümü, AN5325; ölçek kuralı HAL: girdi ·2^-n, çıktı ·2^n (ln'de 2^(n+1)).
 * Hassasiyet: PRECISION = döngü sayısı (1 döngü = 4 yineleme); q1.31'de ~1 döngüde 4 bit.
 * Geçerlilik: expf için x ∈ [-87, 88] (normal float), logf için x > 0 normal float.
 */
#ifndef BSP_CORDIC_H
#define BSP_CORDIC_H
#include <stdint.h>

void  bsp_cordic_init(void);
void  bsp_cordic_precision(uint32_t cycles);   /* 1..15, varsayılan 6 */
float bsp_cordic_expf(float x);
float bsp_cordic_logf(float x);

#endif
