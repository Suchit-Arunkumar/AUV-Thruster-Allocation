/*
 * alloc_vectors.c - runs the C pipeline over a file of wrenches for the
 * NumPy comparison (tests/test_c_vs_numpy.py).
 *
 *   alloc_vectors <in.bin> <out.bin> <dt>
 *
 * in.bin : N x 6 float32 wrenches
 * out.bin: per wrench, 8 float32 thrust, 8 int16 PWM target, 8 int16 slewed
 *          PWM, 1 uint8 flags (57 bytes). The slew state starts at neutral
 *          and carries across rows, one tick of dt per row.
 */
#include "alloc.h"

#include <stdio.h>
#include <stdlib.h>

int main(int argc, char **argv)
{
    if (argc != 4) {
        fprintf(stderr, "usage: %s in.bin out.bin dt\n", argv[0]);
        return 2;
    }
    FILE *in = fopen(argv[1], "rb");
    FILE *out = fopen(argv[2], "wb");
    if (!in || !out) { perror("fopen"); return 2; }
    float dt = strtof(argv[3], NULL);

    alloc_slew_t s;
    alloc_neutral(&s);

    float w[ALLOC_N_DOF], t[ALLOC_N_THR];
    int16_t target[ALLOC_N_THR];
    long n = 0;
    while (fread(w, sizeof w, 1, in) == 1) {
        uint8_t flags = alloc_wrench_to_thrust(w, t);
        alloc_thrust_to_pwm(t, target);
        alloc_slew_step(&s, target, dt);
        fwrite(t, sizeof t, 1, out);
        fwrite(target, sizeof target, 1, out);
        fwrite(s.pwm_us, sizeof s.pwm_us, 1, out);
        fwrite(&flags, 1, 1, out);
        n++;
    }
    fclose(in);
    fclose(out);
    printf("%ld wrenches\n", n);
    return 0;
}
