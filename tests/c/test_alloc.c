/*
 * test_alloc.c - edge-case tests for src/alloc.c. Prints "N/M checks passed"
 * and exits non-zero on any failure.
 */
#include "alloc.h"
#include "alloc_matrix.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static int g_total, g_failed;

#define CHECK(cond) do {                                                    \
    g_total++;                                                              \
    if (!(cond)) { g_failed++;                                              \
        printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); }            \
} while (0)

static float max_abs(const float *v, int lo, int hi)
{
    float m = 0.0f;
    for (int i = lo; i < hi; i++) if (fabsf(v[i]) > m) m = fabsf(v[i]);
    return m;
}

static int all_pwm(const int16_t *p, int v)
{
    for (int i = 0; i < ALLOC_N_THR; i++) if (p[i] != v) return 0;
    return 1;
}

/* One normalised thrust value through the PWM map on thruster idx. */
static int pwm_of(float u, int idx)
{
    float t[ALLOC_N_THR] = {0};
    int16_t p[ALLOC_N_THR];
    t[idx] = u;
    alloc_thrust_to_pwm(t, p);
    return p[idx];
}

static void test_zero(void)
{
    float w[6] = {0}, t[8];
    int16_t p[8];
    CHECK(alloc_wrench_to_thrust(w, t) == 0);
    CHECK(max_abs(t, 0, 8) == 0.0f);
    alloc_thrust_to_pwm(t, p);
    CHECK(all_pwm(p, ALLOC_PWM_NEUTRAL));
}

/* A small single-axis command is the B+ column on that axis's group and
 * exactly zero on the other group. */
static void test_single_axis(void)
{
    for (int k = 0; k < 6; k++) {
        float w[6] = {0}, t[8];
        w[k] = 0.1f;
        CHECK(alloc_wrench_to_thrust(w, t) == 0);
        int vert = (k >= 2 && k <= 4);
        for (int i = 0; i < 8; i++) {
            int driven = vert ? (i < 4) : (i >= 4);
            float expect = driven ? ALLOC_B_PINV[i][k] * 0.1f : 0.0f;
            CHECK(t[i] == expect);
        }
    }
}

/* Group renormalisation: the saturated group is scaled uniformly, so the
 * ratios between its thrusters are those of the B+ column. */
static void test_extreme_saturation(void)
{
    float w[6] = {1e6f, 0, 0, 0, 0, 0}, t[8];
    CHECK(alloc_wrench_to_thrust(w, t) == ALLOC_SAT_TRANS);
    CHECK(fabsf(max_abs(t, 4, 8) - 1.0f) < 1e-6f);
    for (int i = 4; i < 8; i++)
        CHECK(fabsf(t[i] - ALLOC_B_PINV[i][0] / ALLOC_B_PINV[6][0]) < 1e-6f);

    float h[6] = {0, 0, 100.0f, 0, 0, 0};
    CHECK(alloc_wrench_to_thrust(h, t) == ALLOC_SAT_VERT);
    CHECK(fabsf(max_abs(t, 0, 4) - 1.0f) < 1e-6f);
    CHECK(max_abs(t, 4, 8) == 0.0f);

    float y[6] = {0, 0, 0, 0, 0, -50.0f};
    CHECK(alloc_wrench_to_thrust(y, t) == ALLOC_SAT_YAW);
    CHECK(fabsf(t[7] + 1.0f) < 1e-6f);          /* T8 has the largest yaw gain */

    /* Every group saturated at once; translation + yaw then clipped per
     * thruster. Nothing may leave [-1, 1]. */
    float all[6] = {80, -80, 80, 20, -20, 20};
    CHECK(alloc_wrench_to_thrust(all, t) ==
          (ALLOC_SAT_VERT | ALLOC_SAT_TRANS | ALLOC_SAT_YAW));
    CHECK(max_abs(t, 0, 8) <= 1.0f);
    CHECK(max_abs(t, 4, 8) == 1.0f);
}

static void test_non_finite(void)
{
    const float bad[3] = {NAN, INFINITY, -INFINITY};
    for (int b = 0; b < 3; b++) {
        for (int k = 0; k < 6; k++) {
            float w[6] = {0.3f, -0.2f, 0.5f, 0.1f, -0.1f, 0.05f}, t[8];
            int16_t p[8];
            w[k] = bad[b];
            CHECK(alloc_wrench_to_thrust(w, t) == ALLOC_REJECTED);
            CHECK(max_abs(t, 0, 8) == 0.0f);
            alloc_thrust_to_pwm(t, p);
            CHECK(all_pwm(p, ALLOC_PWM_NEUTRAL));
        }
        CHECK(pwm_of(bad[b], 3) == ALLOC_PWM_NEUTRAL);
    }
}

static void test_deadzone_and_mapping(void)
{
    CHECK(pwm_of(0.0199f, 2) == 1500);
    CHECK(pwm_of(0.02f, 2) == 1508);
    CHECK(pwm_of(-0.0199f, 2) == 1500);
    CHECK(pwm_of(-0.02f, 2) == 1492);
    CHECK(pwm_of(0.5049f, 2) == 1701);          /* 201.96 truncated toward zero */
    CHECK(pwm_of(-0.5049f, 2) == 1299);
    CHECK(pwm_of(0.5f, 0) == 1300);             /* T1 polarity reversed */
    CHECK(pwm_of(-0.5f, 0) == 1700);
    CHECK(pwm_of(0.0199f, 0) == 1500);
}

static void test_pwm_clamping(void)
{
    CHECK(pwm_of(1.0f, 4) == ALLOC_PWM_MAX);
    CHECK(pwm_of(-1.0f, 4) == ALLOC_PWM_MIN);
    CHECK(pwm_of(5.0f, 4) == ALLOC_PWM_MAX);
    CHECK(pwm_of(-5.0f, 4) == ALLOC_PWM_MIN);
    CHECK(pwm_of(5.0f, 0) == ALLOC_PWM_MIN);
}

static void test_slew(void)
{
    alloc_slew_t s;
    int16_t up[8], down[8];
    for (int i = 0; i < 8; i++) { up[i] = ALLOC_PWM_MAX; down[i] = ALLOC_PWM_MIN; }

    alloc_neutral(&s);
    CHECK(all_pwm(s.pwm_us, 1500));

    alloc_slew_step(&s, up, 0.02f);             /* firmware: 50 us per tick */
    CHECK(all_pwm(s.pwm_us, 1550));
    for (int n = 0; n < 7; n++) alloc_slew_step(&s, up, 0.02f);
    CHECK(all_pwm(s.pwm_us, 1900));             /* 400 us in 8 ticks */
    alloc_slew_step(&s, up, 0.02f);
    CHECK(all_pwm(s.pwm_us, 1900));

    alloc_slew_step(&s, down, 0.02f);
    CHECK(all_pwm(s.pwm_us, 1850));

    alloc_slew_step(&s, down, 0.0f);            /* hold */
    CHECK(all_pwm(s.pwm_us, 1850));
    alloc_slew_step(&s, down, -1.0f);
    CHECK(all_pwm(s.pwm_us, 1850));
    alloc_slew_step(&s, down, NAN);
    CHECK(all_pwm(s.pwm_us, 1850));

    alloc_slew_step(&s, down, 0.0199f);         /* round(49.75) = 50 */
    CHECK(all_pwm(s.pwm_us, 1800));
    alloc_slew_step(&s, down, 0.01f);           /* 25 us */
    CHECK(all_pwm(s.pwm_us, 1775));

    alloc_slew_step(&s, down, 10.0f);           /* step capped, lands on target */
    CHECK(all_pwm(s.pwm_us, ALLOC_PWM_MIN));
    alloc_slew_step(&s, up, INFINITY);
    CHECK(all_pwm(s.pwm_us, ALLOC_PWM_MAX));

    int16_t wild[8] = {3000, -3000, 1500, 1500, 1500, 1500, 1500, 1500};
    alloc_slew_step(&s, wild, 10.0f);           /* output clamp */
    CHECK(s.pwm_us[0] == ALLOC_PWM_MAX);
    CHECK(s.pwm_us[1] == ALLOC_PWM_MIN);

    alloc_neutral(&s);                          /* failsafe: no ramp */
    CHECK(all_pwm(s.pwm_us, ALLOC_PWM_NEUTRAL));
}

int main(void)
{
    test_zero();
    test_single_axis();
    test_extreme_saturation();
    test_non_finite();
    test_deadzone_and_mapping();
    test_pwm_clamping();
    test_slew();

    printf("%d/%d checks passed\n", g_total - g_failed, g_total);
    return g_failed ? 1 : 0;
}
