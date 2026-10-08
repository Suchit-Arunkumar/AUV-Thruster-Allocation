# AUV Thruster Allocation

[![ci](https://github.com/Suchit-Arunkumar/AUV-Thruster-Allocation/actions/workflows/ci.yml/badge.svg)](https://github.com/Suchit-Arunkumar/AUV-Thruster-Allocation/actions/workflows/ci.yml)

Thruster allocation (motor mixing) for Team Tiburon's 8-thruster AUV: the 6×8
allocation matrix, its pseudo-inverse, per-group saturation renormalisation,
deadzone, slew limiting and PWM mapping for eight ESCs. This is the allocator
that ran on the vehicle's RP2350 (Pico 2) firmware at SAUVC 2026, ported to
portable C99 (`src/alloc.c`), with a NumPy reference that matches it bit for
bit, and a measured analysis of how it behaves.

Measuring the flown allocator found two defects. Its pseudo-inverse is used
block-diagonally, which couples yaw and translation into roll and pitch
(up to {{matrix.block_max_cross_coupling:.3f}} per unit command, {{reconstruction.flown.direction_error_deg.mean:.1f}}° mean direction
error with nothing saturated). Its groups are scaled independently, which
distorts the commanded direction under saturation: on saturated commands it is
worse than naive per-thruster clipping {{derived.flown_worse_pct:.0f}}% of the time. `alloc_v2`
(**not flown**) uses the full pseudo-inverse and one uniform scale factor:
direction error goes to {{direction.saturated.v2.direction_deg.max:sci0}}° with or without saturation, at the cost of
{{derived.v2_extra_loss_pts:.1f}} percentage points more magnitude loss than the flown allocator.

---

## Contents

1. [Provenance and scope](#provenance-and-scope)
2. [The math](#the-math)
3. [Pipeline](#pipeline)
4. [Results](#results)
5. [Key figures](#key-figures)
6. [Open questions](#open-questions)
7. [Not verified](#not-verified)
8. [Build and test](#build-and-test)
9. [Repository layout](#repository-layout)

---

## Provenance and scope

| | |
|---|---|
| Flown | RP2350 (Pico 2) vehicle firmware, SAUVC 2026, 50 Hz control loop |
| Ported to | [TIBURON-AUV-FreeRTOS](https://github.com/Suchit-Arunkumar/TIBURON-AUV-FreeRTOS) and [Nucleo_AUV_Bare_Metal](https://github.com/Suchit-Arunkumar/Nucleo_AUV_Bare_Metal) (STM32F446RE) |
| This repo | The allocation stage only: wrench in, eight ESC pulse widths out |
| Out of scope | The upstream 6-DOF controller (co-developed) that produces the wrench |
| Thrust units | Normalised to full ESC command, \|T<sub>i</sub>\| ≤ 1. Thrust per ESC has not been calibrated in a tank, so no result here is in newtons |

`A`, `B⁺`, the group logic and all PWM constants are identical in the team's
tuning sketch, the vehicle firmware and both STM32 ports. The one port
difference is in TIBURON-AUV-FreeRTOS, which applies the slew limit only while
recovering from boot or failsafe; the vehicle firmware and the bare-metal port
apply it on every armed tick, and so does `src/alloc.c`.

`src/alloc.c` keeps the firmware's group logic line for line. Deliberate
deviations, each reachable only with inputs the firmware never guarded:

- a non-finite wrench element is rejected (zero thrust, `ALLOC_REJECTED`); the
  firmware would pass NaN through an undefined float→int conversion to the ESCs;
- a non-finite thrust maps to neutral PWM;
- the slew step is `round(2500 µs/s · dt)` instead of a fixed 50 µs, identical
  at the vehicle's dt = 0.02 s; dt ≤ 0 or NaN holds the outputs.

`src/alloc_v2.c` is new work, verified against the NumPy reference and in
simulation, and **has not flown or run on a board**. The NUCLEO-F446RE bench
firmware that would time it builds in CI but has not been run yet.

---

## The math

### Allocation matrix

Thruster *j* at position **r**<sub>j</sub> (body frame) pushing along unit
direction **d**<sub>j</sub> contributes a force and a moment about the reference
point. Stacking all eight gives the wrench:

$$
\boldsymbol\tau=\begin{bmatrix}\mathbf F\\ \mathbf M\end{bmatrix}=A\,\mathbf T,
\qquad
\mathbf a_j=\begin{bmatrix}\mathbf d_j\\ \mathbf r_j\times\mathbf d_j\end{bmatrix},
\qquad
A\in\mathbb R^{6\times 8}
$$

For the vertical thrusters T1–T4, **d** = (0, 0, 1), so the moment column is
(r<sub>y</sub>, −r<sub>x</sub>, 0): the roll and pitch rows are the lever arms
read straight off the CAD. For the horizontal thrusters T5–T8 at 45°,
**d** = (d<sub>x</sub>, d<sub>y</sub>, 0) and

$$
\mathbf r\times\mathbf d=\begin{bmatrix}-r_z d_y\\ r_z d_x\\ r_x d_y-r_y d_x\end{bmatrix}.
$$

`A` has rank {{matrix.rank}}, singular values
{{matrix.singular_values.0:.3f}}, {{matrix.singular_values.1:.3f}}, {{matrix.singular_values.2:.3f}}, {{matrix.singular_values.3:.3f}}, {{matrix.singular_values.4:.3f}}, {{matrix.singular_values.5:.3f}}
and condition number {{matrix.condition_number:.2f}}. Full row rank means every wrench is
reachable, and the 8 − 6 = 2 dimensional null space is the freedom left over.

### Why the pseudo-inverse

With more thrusters than DOFs, `A T = τ` has infinitely many solutions. The
pseudo-inverse picks the one with the least total squared thrust:

$$
\min_{\mathbf T}\ \tfrac12\lVert\mathbf T\rVert_2^2\quad\text{s.t.}\quad A\mathbf T=\boldsymbol\tau
\quad\Longrightarrow\quad
\mathbf T^\star=A^\top\!\left(AA^\top\right)^{-1}\boldsymbol\tau=A^{+}\boldsymbol\tau .
$$

Setting the gradient of the Lagrangian to zero gives **T** = Aᵀ**λ**, and the
constraint gives A Aᵀ**λ** = **τ**; A Aᵀ is invertible because A has full row
rank. **T**\* lies in the row space of A, orthogonal to the null space, so any
other exact solution **T**\* + **n** has ‖**T**\*‖² + ‖**n**‖² > ‖**T**\*‖². It
is computed offline once (SVD, `python/model.py`) and stored as constants: one
8×6 matrix-vector product per tick. It minimises the 2-norm, not the largest
single thrust, so it is not the solution that delays saturation longest.

### Saturation: why group renormalisation keeps a direction

Each thruster is limited to \|T<sub>i</sub>\| ≤ 1. Allocation is linear, so
dividing the thrusts of a group by the same factor *m* divides that group's
output by *m*:

$$
A_g\!\left(\frac{P_g\boldsymbol\tau_g}{m}\right)=\frac{\boldsymbol\tau_g}{m},
\qquad m=\max_i\lvert(P_g\boldsymbol\tau_g)_i\rvert>1 .
$$

Direction is kept and only magnitude is lost, whereas clipping each thruster on
its own changes the ratios between thrusters and therefore the direction.

### What the flown allocator actually computes

The firmware forms three groups, each with its own slice of B⁺:

| Group | DOFs | Thrusters |
|---|---|---|
| vertical | heave, roll, pitch | T1–T4 |
| translation | surge, sway | T5–T8 |
| yaw | yaw | T5–T8 |

Each group is renormalised on its own; translation and yaw are then **added**
on T5–T8 and each thruster is clipped to ±1. Three consequences, all measured
below:

1. **Block truncation.** Only the block-diagonal part P of B⁺ is used, and the
   dropped entries reach {{matrix.dropped_pinv_max_abs:.3f}}. So A P ≠ I: a pure yaw command also
   produces roll and pitch moment, because nothing compensates the horizontal
   thrusters' ±0.014 roll/pitch lever arms.
2. **Independent scale factors.** Each group is scaled by its own *m*, so the
   direction of the full 6-D wrench changes whenever only some groups saturate.
3. **Sum, then clip.** Translation and yaw are each within limits after
   renormalisation, but their sum on T5–T8 can exceed 1 and is clipped per
   thruster, which loses direction inside the horizontal group.

### v2 (not flown): full B⁺, one scale factor

$$
\mathbf T=A^{+}\boldsymbol\tau,\qquad
m=\max_i\lvert T_i\rvert,\qquad
\mathbf T\leftarrow\mathbf T/m\ \text{ if } m>1
\quad\Longrightarrow\quad
A\mathbf T=\boldsymbol\tau/m .
$$

The achieved wrench is exactly parallel to the request; the only error left is
float rounding. The cost is magnitude: one saturated thruster shrinks the whole
wrench. Uniform scaling of the minimum-norm solution is also not the largest
achievable wrench along **τ**; that would use the 2-D null space and needs a
small LP or QP per tick.

**v2p** splits the wrench into a primary part (heave, roll, pitch) and a
secondary part (surge, sway, yaw). The primary part is allocated first, scaled
uniformly only if it alone saturates; the secondary part gets the largest
common factor that still fits:

$$
\mathbf T=\mathbf T_p+s\,\mathbf T_s,\qquad
s=\min\!\Bigl(1,\ \min_{i:\,T_{s,i}\neq0}\frac{\operatorname{sign}(T_{s,i})-T_{p,i}}{T_{s,i}}\Bigr).
$$

---

## Pipeline

Per 50 Hz tick, in `src/alloc.c` (flown) or with the wrench→thrust step swapped
for `src/alloc_v2.c`:

| Stage | Function | What it does |
|---|---|---|
| 1 | `alloc_wrench_to_thrust` | wrench (surge, sway, heave, roll, pitch, yaw) → thrust T1–T8 in [−1, 1]; group renormalisation, sum, clip; returns saturation flags |
| 2 | `alloc_thrust_to_pwm` | polarity flip (T1 is mounted reversed), clamp, deadzone \|u\| < 0.02 → neutral, `1500 + (int)(u·400)` µs, truncated toward zero |
| 3 | `alloc_slew_step` | each output moves at most 50 µs per 20 ms tick (2500 µs/s), then clamp to 1100–1900 µs |
| — | `alloc_neutral` | failsafe: every output to 1500 µs in the same tick, no ramp |

| Constant | Value |
|---|---|
| PWM min / neutral / max | 1100 / 1500 / 1900 µs |
| Deadzone | \|u\| < 0.02 |
| Slew | 50 µs per tick at 50 Hz (full scale in 8 ticks, 160 ms) |
| Saturation flags | bit 0 vertical, bit 1 translation, bit 2 yaw (same as the firmware telemetry byte) |

Thruster index → RP2350 GPIO in the vehicle firmware: T1 9, T2 22, T3 14,
T4 16; T5–T8 are on placeholder pins 0–3 in the source (see
[Not verified](#not-verified)). The allocation code never touches pins.

`include/alloc_matrix.h` is generated by `python/model.py` from `A`, the single
source of truth, and carries the SHA-256 of that matrix. It holds B⁺ rounded to
the 4 decimals the firmware stores (used by the flown allocator) and at full
float32 precision (used by v2).

---

## Results

All numbers come from `bench/analysis.py` (fixed seeds) unless stated. Each DOF
is divided by its **authority** (the largest single-axis command the flown
allocator delivers without saturating) before angles and magnitudes are
measured, so one degree means the same on surge as on yaw:

{{@authority_table}}

### Pseudo-inverse vs firmware

| | |
|---|---|
| max \|pinv(A) − firmware B_pinv\| | {{matrix.pinv_vs_firmware_max_abs_diff:sci2}} (at {{matrix.pinv_vs_firmware_argmax}}), below the 5 × 10⁻⁵ of 4-decimal rounding |
| pinv(A) rounded to 4 decimals = firmware literals | exactly (`tests/test_model.py`) |
| max \|A · B_pinv − I\| | {{matrix.firmware_pinv_residual_max_abs:sci1}} (the rounding) |
| Column norms of A | {{matrix.column_norms.0:.3f}} – {{matrix.column_norms.6:.3f}} |

### Unsaturated commands

{{reconstruction.n_unsaturated:,}} commands that no allocator saturates (each DOF uniform in
±0.4 × authority):

| | Flown | v2 (not flown) |
|---|---|---|
| max \|A T − τ\| | {{reconstruction.flown.max_abs_error:.4f}} (My) | {{reconstruction.v2.max_abs_error:sci1}} |
| Direction error mean / p95 / max | {{reconstruction.flown.direction_error_deg.mean:.2f}}° / {{reconstruction.flown.direction_error_deg.p95:.2f}}° / {{reconstruction.flown.direction_error_deg.max:.2f}}° | {{reconstruction.v2.direction_error_deg.max:sci0}}° max |

The flown error is the block truncation. Its coupling terms, from A P:

{{@coupling_table}}

### Saturated commands: direction and magnitude

{{direction.n_wrenches:,}} wrenches, direction uniform on the 6-sphere, magnitude uniform in
0–3 × authority; {{direction.n_saturated:,}} ({{derived.sat_pct:.0f}}%) saturate at least one allocator.
*Magnitude lost* is the share of the request missing along its own direction.

{{@direction_table}}

![Direction error and magnitude loss](results/direction_error.png)

- On saturated commands the flown allocator beats per-thruster clipping
  {{derived.flown_better_pct:.0f}}% of the time and is worse {{derived.flown_worse_pct:.0f}}% of the time (ties within 0.01°
  excluded). Its advantage is in the tail: max {{direction.saturated.flown.direction_deg.max:.1f}}° against {{direction.saturated.clip.direction_deg.max:.1f}}°.
- Inside each flown group, renormalisation does what it claims for the vertical
  group and not for the horizontal one, where translation + yaw is clipped:

{{@group_table}}

- v2 removes the direction error and pays in magnitude: mean loss
  {{direction.saturated.v2.magnitude_loss.mean:.1%}} against {{direction.saturated.flown.magnitude_loss.mean:.1%}} (flown) and {{direction.saturated.clip.magnitude_loss.mean:.1%}} (clip). Its worst case is lower
  than the flown allocator's ({{direction.saturated.v2.magnitude_loss.max:.1%}} against {{direction.saturated.flown.magnitude_loss.max:.1%}}).
- The flown allocator's heave-roll-pitch column includes the roll and pitch
  moment that saturated horizontal thrusters leak through the block truncation,
  which is why its maximum there is large even though the vertical group itself
  keeps direction.
- v2p keeps heave, roll and pitch exact and gives up surge, sway and yaw to do
  it, so its full-wrench direction error is the largest of the four by design.

### Saturation rate per DOF (flown allocator)

No logged command data from the vehicle was available, so this is a sweep: all
six DOFs commanded at once, each uniform in ±level × authority. A DOF counts as
saturated when it is delivered more than 1% (of its authority) short of what
the flown allocator would deliver unsaturated.

{{@saturation_table}}

![Saturation rate per DOF](results/saturation_rate.png)

Heave, roll and pitch share the four vertical thrusters and saturate first.

### C vs NumPy, and tests

- `src/alloc.c`, `src/alloc_v2.c` and `python/reference.py` agree **bit for bit**
  (thrust, PWM target, slewed PWM, flags) over 10,000 seeded wrenches of which
  over a thousand saturate (`tests/test_c.py`, `python/compare_c.py`). Both sides
  avoid fused multiply-add (`-ffp-contract=off`) so the operation order is the same.
- {{@pytest_count}} pytest tests plus the C unit runner (`tests/c/test_alloc.c`: zero,
  single-axis, extreme saturation, NaN/±inf on every input, deadzone
  boundaries, truncation, PWM clamping, slew steps and holds, failsafe, v2 and
  v2p). All run in [CI](https://github.com/Suchit-Arunkumar/AUV-Thruster-Allocation/actions/workflows/ci.yml)
  on ubuntu-latest with gcc and `-Werror`; the C build and the C-vs-NumPy tests
  are required there, not skipped.
- v2 direction error is asserted ≤ 10⁻⁴° on every saturated command, on both
  the NumPy and the C outputs.

### Cycle cost on a Cortex-M4F

{{@bench}}

---

## Key figures

| | |
|---|---|
| Rank / condition number of A | {{matrix.rank}} / {{matrix.condition_number:.2f}} |
| pinv(A) vs firmware B_pinv | {{matrix.pinv_vs_firmware_max_abs_diff:sci2}} max abs difference |
| Flown, unsaturated: max error / direction error | {{reconstruction.flown.max_abs_error:.4f}} / {{reconstruction.flown.direction_error_deg.mean:.1f}}° mean, {{reconstruction.flown.direction_error_deg.max:.1f}}° max |
| Flown, saturated: direction error mean / p95 / max | {{direction.saturated.flown.direction_deg.mean:.1f}}° / {{direction.saturated.flown.direction_deg.p95:.1f}}° / {{direction.saturated.flown.direction_deg.max:.1f}}° |
| Per-thruster clip, saturated | {{direction.saturated.clip.direction_deg.mean:.1f}}° / {{direction.saturated.clip.direction_deg.p95:.1f}}° / {{direction.saturated.clip.direction_deg.max:.1f}}° |
| Flown worse than clipping | {{derived.flown_worse_pct:.0f}}% of saturated commands |
| v2 (not flown), saturated direction error | {{direction.saturated.v2.direction_deg.max:sci0}}° max |
| v2 extra magnitude loss vs flown | {{derived.v2_extra_loss_pts:.1f}} percentage points (mean {{direction.saturated.v2.magnitude_loss.mean:.1%}} vs {{direction.saturated.flown.magnitude_loss.mean:.1%}}) |
| C vs NumPy | bit-identical, 10,000 wrenches (CI) |
| Flown pipeline on STM32F446 @ 180 MHz | {{@bench_key}} |

---

## Open questions

**Origin of the ±0.014 roll/pitch entries for T5–T8.** A moment produced by a
thrust along **d** at a lever arm is **r** × **d**, which is always orthogonal
to **d**. With **d** in the horizontal plane that forces
M<sub>x</sub> = −r<sub>z</sub> d<sub>y</sub> and M<sub>y</sub> = r<sub>z</sub> d<sub>x</sub>.
T5's entries are consistent with a vertical offset r<sub>z</sub> = {{matrix.lever_arm_check.rz_from_T5_m:.4f}} m (about
2 cm). T6–T8 are not, for that or any other r<sub>z</sub>: one entry each has the
opposite sign (bold), and M·d ≠ 0.

{{@lever_table}}

This pattern suggests sign errors in hand-entered values. Propeller reaction
torque would add a component along **d**, but it would also appear on T5. The
entries set the size of the flown allocator's block-truncation coupling, so
they matter for the unsaturated error above. `A` is used as-is here; it has not
been changed.

**Two asymmetries not yet checked against the CAD.** They may be real geometry,
but nothing here confirms it:

- **Yaw arms.** M<sub>z</sub> for T5 and T6 is 0.0346, for T7 and T8 ±0.325: a
  ninefold difference between thrusters at the same 45° angle. That fits T5/T6
  pointing almost through the reference point, or a mis-entered value.
- **T1's roll arm.** M<sub>x</sub> is +0.17 for T1 and +0.12 for T3, on the same
  side of the vehicle, while T2 and T4 are both −0.12. Either T1 is mounted
  further out, or one entry is off.

Both set the allocator's yaw and roll authority directly, so they matter as
much as the sign question above.

---

## Not verified

- **Thrust calibration.** Per-ESC thrust vs PWM was not measured in a tank;
  everything assumes linear, symmetric (forward = reverse) thrust and no
  thruster–thruster interaction.
- **v2 and v2p have not flown or run on a board.** They are verified against
  the NumPy reference and in the analysis. The NUCLEO-F446RE cycle bench builds
  in CI but has not been run.
- **Horizontal thruster wiring.** In the vehicle firmware, T5–T8 sit on
  placeholder GPIO 0–3 pending a wiring check, and the index → physical
  thruster order for that group has not been re-verified.
- **Saturation rates** come from a uniform sweep, not from logged commands.
- **No closed-loop result.** How the direction errors measured here affect
  tracking depends on the upstream controller, which is out of scope.

---

## Build and test

```bash
pip install -r python/requirements.txt
python python/model.py            # regenerate include/alloc_matrix.h (or --check)
cmake -S . -B build && cmake --build build
ctest --test-dir build --output-on-failure
ALLOC_REQUIRE_C=1 python -m pytest -v
python bench/analysis.py          # results/*.json, results/*.png
python bench/render_readme.py     # README.md from docs/README.in.md
```

Bench on a NUCLEO-F446RE (bare board, ST-Link VCP):

```bash
python firmware/nucleo_bench/build.py
STM32_Programmer_CLI -c port=SWD -w firmware/nucleo_bench/build/alloc_bench.elf -v -rst
python firmware/nucleo_bench/read_bench.py --port COM12 --save
```

`read_bench.py` waits for the second run (warm flash cache), converts cycles to
µs, recomputes every output hash with the NumPy reference and writes
`results/nucleo_bench.json`. Clock tree, startup, linker script and DWT setup
come from [Nucleo_AUV_Bare_Metal](https://github.com/Suchit-Arunkumar/Nucleo_AUV_Bare_Metal).

---

## Repository layout

```
include/alloc.h          flown allocator API, constants, flags
include/alloc_v2.h       v2 / v2p API (not flown)
include/alloc_matrix.h   generated: A, B⁺ (4-decimal firmware values and full precision)
src/alloc.c              flown allocator, ported from the vehicle firmware
src/alloc_v2.c           v2 / v2p (not flown)
python/model.py          A → B⁺, rank, singular values; generates the header
python/reference.py      NumPy reference of every allocator, bit-exact in float32
python/metrics.py        authority, direction error, magnitude loss
python/compare_c.py      C vs NumPy over seeded wrenches
tests/                   pytest suite, C unit runner, C vector harness
bench/                   analysis and README renderer
results/                 JSON and plots behind every number above
firmware/nucleo_bench/   STM32F446 cycle bench (DWT CYCCNT, USART2)
docs/README.in.md        README template
```

MIT licensed. CMSIS headers in `firmware/nucleo_bench/cmsis/` keep their own
licences.
