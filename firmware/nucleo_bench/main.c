/*
 * main.c - cycle cost of src/alloc.c on the NUCLEO-F446RE (Cortex-M4F,
 * 180 MHz, -O2, no interrupts enabled).
 *
 * 1000 wrenches from a seeded xorshift32 are pushed through the flown
 * pipeline; each stage is timed with DWT->CYCCNT. The same wrenches then go
 * through the v2 and v2p allocators (NOT FLOWN), timed the same way. Every
 * output byte is folded into an FNV-1a hash per allocator that
 * read_bench.py recomputes with the NumPy reference, so the same run shows
 * the on-target results are correct.
 *
 * Output on USART2 (ST-Link VCP, 115200 8N1), repeated every ~2 s:
 *   BEGIN run=<n>
 *   <key> <value> ...
 *   END
 */
#include "board.h"
#include "alloc.h"
#include "alloc_v2.h"

#define N_WRENCH 1000u
#define SEED     2026u
#define DT       0.02f

typedef struct { uint32_t min, max, sum; } stat_t;
typedef uint8_t (*alloc_fn)(const float *, float *);

static float W[N_WRENCH][ALLOC_N_DOF];

static uint32_t xorshift32(uint32_t *s)
{
    uint32_t x = *s;
    x ^= x << 13; x ^= x >> 17; x ^= x << 5;
    return *s = x;
}

/* Uniform in [0, 1): top 24 bits, exact in float. */
static float u01(uint32_t *s) { return (float)(xorshift32(s) >> 8) * (1.0f / 16777216.0f); }

/* Same distribution as python/compare_c.py: per-row scale in [0, 3), each
 * axis uniform in +-scale x single-axis authority (1 / largest |B+| entry). */
static void make_wrenches(void)
{
    static const float auth[ALLOC_N_DOF] = {
        1.0f / 0.7072f, 1.0f / 0.8347f, 1.0f / 0.3060f,
        1.0f / 2.0482f, 1.0f / 2.0306f, 1.0f / 2.9767f };
    uint32_t s = SEED;
    for (uint32_t i = 0; i < N_WRENCH; i++) {
        float r = 3.0f * u01(&s);
        for (int k = 0; k < ALLOC_N_DOF; k++)
            W[i][k] = ((2.0f * u01(&s) - 1.0f) * r) * auth[k];
    }
}

static uint32_t fnv1a(uint32_t h, const void *p, uint32_t n)
{
    const uint8_t *b = (const uint8_t *)p;
    while (n--) { h ^= *b++; h *= 16777619u; }
    return h;
}

static void stat_reset(stat_t *s) { s->min = 0xFFFFFFFFu; s->max = 0; s->sum = 0; }

static void stat_add(stat_t *s, uint32_t v)
{
    if (v < s->min) s->min = v;
    if (v > s->max) s->max = v;
    s->sum += v;
}

static void put_u32(uint32_t v)
{
    char buf[11];
    int i = 10;
    buf[i] = '\0';
    do { buf[--i] = (char)('0' + v % 10u); v /= 10u; } while (v);
    board_puts(&buf[i]);
}

static void put_hex(uint32_t v)
{
    char buf[11] = "0x";
    for (int i = 0; i < 8; i++) {
        uint32_t d = (v >> (28 - 4 * i)) & 0xFu;
        buf[2 + i] = (char)(d < 10 ? '0' + d : 'a' + d - 10);
    }
    buf[10] = '\0';
    board_puts(buf);
}

static void put_stat(const char *name, const stat_t *s)
{
    board_puts(name);
    board_puts(" min "); put_u32(s->min);
    board_puts(" max "); put_u32(s->max);
    board_puts(" sum "); put_u32(s->sum);
    board_puts("\r\n");
}

#define BARRIER() __asm volatile("" ::: "memory")

/* Time one wrench -> thrust allocator over all wrenches; hash its output. */
static void time_alloc(alloc_fn fn, stat_t *st, uint32_t *hash, uint32_t *n_scaled)
{
    stat_reset(st);
    *hash = 2166136261u;
    *n_scaled = 0;
    for (uint32_t i = 0; i < N_WRENCH; i++) {
        float t[ALLOC_N_THR];
        uint32_t c0 = board_cycles(); BARRIER();
        uint32_t c1 = board_cycles(); BARRIER();
        uint8_t flags = fn(W[i], t);
        BARRIER(); uint32_t c2 = board_cycles();
        stat_add(st, c2 - c1 - (c1 - c0));
        if (flags) (*n_scaled)++;
        *hash = fnv1a(*hash, t, sizeof t);
        *hash = fnv1a(*hash, &flags, 1);
    }
}

int main(void)
{
    board_clock_180mhz();
    board_uart2_init();
    board_dwt_init();
    make_wrenches();

    for (uint32_t run = 1;; run++) {
        stat_t st_thr, st_pwm, st_slew, st_total, st_ovh;
        stat_reset(&st_thr); stat_reset(&st_pwm);
        stat_reset(&st_slew); stat_reset(&st_total); stat_reset(&st_ovh);

        alloc_slew_t s;
        alloc_neutral(&s);
        uint32_t hash = 2166136261u, n_sat = 0;

        for (uint32_t i = 0; i < N_WRENCH; i++) {
            float t[ALLOC_N_THR];
            int16_t target[ALLOC_N_THR];

            uint32_t c0 = board_cycles(); BARRIER();
            uint32_t c1 = board_cycles(); BARRIER();
            uint8_t flags = alloc_wrench_to_thrust(W[i], t);
            BARRIER(); uint32_t c2 = board_cycles(); BARRIER();
            alloc_thrust_to_pwm(t, target);
            BARRIER(); uint32_t c3 = board_cycles(); BARRIER();
            alloc_slew_step(&s, target, DT);
            BARRIER(); uint32_t c4 = board_cycles();

            uint32_t ovh = c1 - c0;
            stat_add(&st_ovh, ovh);
            stat_add(&st_thr, c2 - c1 - ovh);
            stat_add(&st_pwm, c3 - c2 - ovh);
            stat_add(&st_slew, c4 - c3 - ovh);
            stat_add(&st_total, c4 - c1 - 3u * ovh);

            if (flags & 0x07u) n_sat++;
            hash = fnv1a(hash, t, sizeof t);
            hash = fnv1a(hash, target, sizeof target);
            hash = fnv1a(hash, s.pwm_us, sizeof s.pwm_us);
            hash = fnv1a(hash, &flags, 1);
        }

        board_puts("BEGIN run="); put_u32(run); board_puts("\r\n");
        board_puts("sysclk_hz "); put_u32(BOARD_SYSCLK_HZ); board_puts("\r\n");
        board_puts("n "); put_u32(N_WRENCH); board_puts("\r\n");
        board_puts("seed "); put_u32(SEED); board_puts("\r\n");
        put_stat("overhead", &st_ovh);
        put_stat("wrench_to_thrust", &st_thr);
        put_stat("thrust_to_pwm", &st_pwm);
        put_stat("slew_step", &st_slew);
        put_stat("total", &st_total);
        board_puts("saturated "); put_u32(n_sat); board_puts("\r\n");
        board_puts("hash "); put_hex(hash); board_puts("\r\n");

        stat_t st_v2;
        uint32_t h2, n2;
        time_alloc(alloc_v2_wrench_to_thrust, &st_v2, &h2, &n2);
        put_stat("v2_wrench_to_thrust", &st_v2);
        board_puts("v2_scaled "); put_u32(n2); board_puts("\r\n");
        board_puts("v2_hash "); put_hex(h2); board_puts("\r\n");
        time_alloc(alloc_v2p_wrench_to_thrust, &st_v2, &h2, &n2);
        put_stat("v2p_wrench_to_thrust", &st_v2);
        board_puts("v2p_scaled "); put_u32(n2); board_puts("\r\n");
        board_puts("v2p_hash "); put_hex(h2); board_puts("\r\n");
        board_puts("END\r\n");

        board_delay_cycles(2u * BOARD_SYSCLK_HZ);
    }
}
