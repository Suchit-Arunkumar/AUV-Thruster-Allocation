/*
 * alloc.h - thruster allocation for the Team Tiburon 8-thruster AUV.
 *
 * Pipeline, called once per control tick (50 Hz on the vehicle):
 *
 *   wrench U[6] --alloc_wrench_to_thrust--> thrust T[8] in [-1, 1]
 *               --alloc_thrust_to_pwm-----> ESC target pwm[8] in us
 *               --alloc_slew_step---------> ESC output pwm[8] in us
 *
 * alloc_neutral() is the failsafe: every output to neutral at once, no ramp.
 *
 * Behaviour is the vehicle firmware's computeAllocation() / thrust_to_pwm() /
 * applyPWM() with the same constants. Deliberate deviations, each only
 * reachable with inputs the firmware never guarded against:
 *   - a non-finite wrench element is rejected: zero thrust, ALLOC_REJECTED;
 *   - a non-finite thrust maps to neutral PWM;
 *   - the slew step is computed from dt (round(2500 us/s * dt), exactly the
 *     firmware's 50 us at dt = 0.02 s); dt <= 0 or NaN holds the outputs.
 *
 * C99, float only, no heap, no HAL. Not reentrant across a shared
 * alloc_slew_t; everything else is pure.
 */
#ifndef ALLOC_H
#define ALLOC_H

#include <stdint.h>

#define ALLOC_N_DOF         6     /* surge sway heave roll pitch yaw */
#define ALLOC_N_THR         8     /* T1-T4 vertical, T5-T8 horizontal */

#define ALLOC_PWM_MIN       1100  /* us, full reverse */
#define ALLOC_PWM_NEUTRAL   1500  /* us, stopped */
#define ALLOC_PWM_MAX       1900  /* us, full forward */

#define ALLOC_DEADZONE      0.02f /* |thrust| below this -> neutral */
#define ALLOC_SLEW_US_PER_S 2500.0f /* 50 us per 20 ms tick */

/* Return flags of alloc_wrench_to_thrust(). Bits 0-2 match the firmware's
 * telemetry sat_flags byte. */
#define ALLOC_SAT_VERT      0x01u /* heave/roll/pitch group renormalised */
#define ALLOC_SAT_TRANS     0x02u /* surge/sway group renormalised */
#define ALLOC_SAT_YAW       0x04u /* yaw group renormalised */
#define ALLOC_REJECTED      0x80u /* non-finite wrench, thrust zeroed */

typedef struct {
    int16_t pwm_us[ALLOC_N_THR];  /* last value written to each ESC */
} alloc_slew_t;

/* Wrench (surge, sway, heave, roll, pitch, yaw) to normalised thrust T1..T8.
 * Returns ALLOC_SAT_* / ALLOC_REJECTED flags. */
uint8_t alloc_wrench_to_thrust(const float wrench[ALLOC_N_DOF],
                               float thrust[ALLOC_N_THR]);

/* Normalised thrust to ESC target pulse width: polarity flip, clamp,
 * deadzone, linear map to [PWM_MIN, PWM_MAX]. */
void alloc_thrust_to_pwm(const float thrust[ALLOC_N_THR],
                         int16_t pwm_target[ALLOC_N_THR]);

/* Move each output toward its target by at most round(2500 * dt) us, then
 * clamp to [PWM_MIN, PWM_MAX]. */
void alloc_slew_step(alloc_slew_t *s, const int16_t pwm_target[ALLOC_N_THR],
                     float dt);

/* Failsafe and initial state: every output to PWM_NEUTRAL, no ramp. */
void alloc_neutral(alloc_slew_t *s);

#endif /* ALLOC_H */
