/*
 * alloc_v2.c - improved allocators. NOT FLOWN; see alloc_v2.h.
 */
#include "alloc_v2.h"
#include "alloc_matrix.h"

#include <math.h>

static int reject_non_finite(const float U[ALLOC_N_DOF], float T[ALLOC_N_THR])
{
    for (int k = 0; k < ALLOC_N_DOF; k++) {
        if (!isfinite(U[k])) {
            for (int i = 0; i < ALLOC_N_THR; i++) T[i] = 0.0f;
            return 1;
        }
    }
    return 0;
}

static float max_abs(const float T[ALLOC_N_THR])
{
    float m = 0.0f;
    for (int i = 0; i < ALLOC_N_THR; i++) if (fabsf(T[i]) > m) m = fabsf(T[i]);
    return m;
}

uint8_t alloc_v2_wrench_to_thrust(const float U[ALLOC_N_DOF], float T[ALLOC_N_THR])
{
    if (reject_non_finite(U, T)) return ALLOC_REJECTED;

    for (int i = 0; i < ALLOC_N_THR; i++) {
        float t = 0.0f;
        for (int k = 0; k < ALLOC_N_DOF; k++) t += ALLOC_B_PINV_FULL[i][k] * U[k];
        T[i] = t;
    }

    float m = max_abs(T);
    if (m > 1.0f) {
        for (int i = 0; i < ALLOC_N_THR; i++) T[i] /= m;
        return ALLOC_V2_SCALED;
    }
    return 0;
}

uint8_t alloc_v2p_wrench_to_thrust(const float U[ALLOC_N_DOF], float T[ALLOC_N_THR])
{
    if (reject_non_finite(U, T)) return ALLOC_REJECTED;

    /* Primary: heave, roll, pitch. Secondary: surge, sway, yaw. */
    float Tp[ALLOC_N_THR], Ts[ALLOC_N_THR];
    for (int i = 0; i < ALLOC_N_THR; i++) {
        const float *b = ALLOC_B_PINV_FULL[i];
        Tp[i] = b[2]*U[2] + b[3]*U[3] + b[4]*U[4];
        Ts[i] = b[0]*U[0] + b[1]*U[1] + b[5]*U[5];
    }

    uint8_t flags = 0;
    float sum[ALLOC_N_THR];
    for (int i = 0; i < ALLOC_N_THR; i++) sum[i] = Tp[i] + Ts[i];

    float s = 1.0f;
    float m = max_abs(Tp);
    if (m > 1.0f) {
        for (int i = 0; i < ALLOC_N_THR; i++) Tp[i] /= m;
        s = 0.0f;
        flags = ALLOC_V2_SCALED;
    } else if (max_abs(sum) > 1.0f) {
        /* Largest s with -1 <= Tp_i + s Ts_i <= 1 for every thruster. Each
         * limit is >= 0 because |Tp_i| <= 1. Only searched when the full
         * command does not fit, so rounding cannot flag a command that does. */
        for (int i = 0; i < ALLOC_N_THR; i++) {
            float lim;
            if      (Ts[i] > 0.0f) lim = ( 1.0f - Tp[i]) / Ts[i];
            else if (Ts[i] < 0.0f) lim = (-1.0f - Tp[i]) / Ts[i];
            else continue;
            if (lim < s) s = lim;
        }
        if (s < 1.0f) flags = ALLOC_V2_SECONDARY;
    }

    for (int i = 0; i < ALLOC_N_THR; i++) {
        float t = Tp[i] + s * Ts[i];
        /* Rounding in the limit can leave t one ulp outside the range. */
        if      (t >  1.0f) t =  1.0f;
        else if (t < -1.0f) t = -1.0f;
        T[i] = t;
    }
    return flags;
}
