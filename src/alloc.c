/*
 * alloc.c - thruster allocation, ported from the vehicle firmware
 * (Pico_Main.ino SECTION 22-23). The group logic is kept line-for-line so it
 * can be diffed against the original; see alloc.h for the deviations.
 */
#include "alloc.h"
#include "alloc_matrix.h"

#include <math.h>

/* Per-thruster polarity flip, applied before PWM mapping. T1's ESC/propeller
 * is mounted reversed on the vehicle (REVERSE_THRUST in the firmware). */
static const uint8_t REVERSE_THRUST[ALLOC_N_THR] = {
    1, 0, 0, 0,     /* T1 T2 T3 T4 */
    0, 0, 0, 0      /* T5 T6 T7 T8 */
};

/* Largest step the slew limiter may take: the whole PWM range. Bounds the
 * float->int conversion for very large dt. */
#define SLEW_STEP_MAX (ALLOC_PWM_MAX - ALLOC_PWM_MIN)

uint8_t alloc_wrench_to_thrust(const float U[ALLOC_N_DOF],
                               float T_out[ALLOC_N_THR])
{
    /* Deviation: the firmware has no guard here; NaN would reach the ESCs
     * through an undefined float->int conversion. */
    for (int k = 0; k < ALLOC_N_DOF; k++) {
        if (!isfinite(U[k])) {
            for (int i = 0; i < ALLOC_N_THR; i++) T_out[i] = 0.0f;
            return ALLOC_REJECTED;
        }
    }

    float T_trans[ALLOC_N_THR] = {0}, T_yaw[ALLOC_N_THR] = {0}, T_vert[ALLOC_N_THR] = {0};

    for (int i = 0; i < ALLOC_N_THR; i++) {
        T_trans[i] = ALLOC_B_PINV[i][0]*U[0] + ALLOC_B_PINV[i][1]*U[1];
        T_yaw[i]   = ALLOC_B_PINV[i][5]*U[5];
        T_vert[i]  = ALLOC_B_PINV[i][2]*U[2] + ALLOC_B_PINV[i][3]*U[3] + ALLOC_B_PINV[i][4]*U[4];
    }

    uint8_t sat = 0;

    /* Vertical group: thrusters 0..3 */
    float maxV = 0.0f;
    for (int i = 0; i < 4; i++) if (fabsf(T_vert[i]) > maxV) maxV = fabsf(T_vert[i]);
    if (maxV > 1.0f) {
        for (int i = 0; i < 4; i++) T_vert[i] /= maxV;
        sat |= ALLOC_SAT_VERT;
    }

    /* Translation group: thrusters 4..7 */
    float maxTn = 0.0f;
    for (int i = 4; i < 8; i++) if (fabsf(T_trans[i]) > maxTn) maxTn = fabsf(T_trans[i]);
    if (maxTn > 1.0f) {
        for (int i = 4; i < 8; i++) T_trans[i] /= maxTn;
        sat |= ALLOC_SAT_TRANS;
    }

    /* Yaw group: thrusters 4..7 */
    float maxY = 0.0f;
    for (int i = 4; i < 8; i++) if (fabsf(T_yaw[i]) > maxY) maxY = fabsf(T_yaw[i]);
    if (maxY > 1.0f) {
        for (int i = 4; i < 8; i++) T_yaw[i] /= maxY;
        sat |= ALLOC_SAT_YAW;
    }

    /* Combine and clamp */
    for (int i = 0; i < ALLOC_N_THR; i++) {
        T_out[i] = (i < 4) ? T_vert[i] : (T_trans[i] + T_yaw[i]);
        if      (T_out[i] >  1.0f) T_out[i] =  1.0f;
        else if (T_out[i] < -1.0f) T_out[i] = -1.0f;
    }

    return sat;
}

static int thrust_to_pwm(float u, int idx)
{
    if (!isfinite(u)) return ALLOC_PWM_NEUTRAL;     /* deviation, see alloc.h */
    if (REVERSE_THRUST[idx]) u = -u;

    if (u >  1.0f) u =  1.0f;
    if (u < -1.0f) u = -1.0f;
    if (fabsf(u) < ALLOC_DEADZONE) return ALLOC_PWM_NEUTRAL;

    return (u >= 0.0f)
        ? ALLOC_PWM_NEUTRAL + (int)(u * (float)(ALLOC_PWM_MAX - ALLOC_PWM_NEUTRAL))
        : ALLOC_PWM_NEUTRAL + (int)(u * (float)(ALLOC_PWM_NEUTRAL - ALLOC_PWM_MIN));
}

void alloc_thrust_to_pwm(const float thrust[ALLOC_N_THR],
                         int16_t pwm_target[ALLOC_N_THR])
{
    for (int i = 0; i < ALLOC_N_THR; i++)
        pwm_target[i] = (int16_t)thrust_to_pwm(thrust[i], i);
}

void alloc_slew_step(alloc_slew_t *s, const int16_t pwm_target[ALLOC_N_THR],
                     float dt)
{
    /* Firmware: fixed PWM_RAMP_STEP = 50 us per 20 ms tick. Here the step
     * scales with dt; dt <= 0 and NaN fail the comparison and hold. */
    int step = 0;
    if (dt > 0.0f) {
        float f = ALLOC_SLEW_US_PER_S * dt + 0.5f;
        step = (f >= (float)SLEW_STEP_MAX) ? SLEW_STEP_MAX : (int)f;
    }

    for (int i = 0; i < ALLOC_N_THR; i++) {
        int cur   = s->pwm_us[i];
        int delta = pwm_target[i] - cur;
        if      (delta >  step) delta =  step;
        else if (delta < -step) delta = -step;
        cur += delta;

        if      (cur > ALLOC_PWM_MAX) cur = ALLOC_PWM_MAX;
        else if (cur < ALLOC_PWM_MIN) cur = ALLOC_PWM_MIN;
        s->pwm_us[i] = (int16_t)cur;
    }
}

void alloc_neutral(alloc_slew_t *s)
{
    for (int i = 0; i < ALLOC_N_THR; i++) s->pwm_us[i] = ALLOC_PWM_NEUTRAL;
}
