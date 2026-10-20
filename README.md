# AUV Thruster Allocation

Thruster allocation (motor mixing) for Team Tiburon's 8-thruster AUV. Covers
the 6×8 allocation matrix, its minimum-norm pseudo-inverse, per-group
saturation renormalisation so yaw never starves depth, deadzone, slew limiting
and PWM mapping for eight ESCs. It ran in the RP2350 (Pico 2) firmware at
SAUVC 2026 and is ported here to portable C (`src/alloc.c`).

---

## Contents

1. [Provenance and scope](#provenance-and-scope)
2. [The math](#the-math)
3. [Pipeline](#pipeline)
4. [Not verified](#not-verified)
5. [Build](#build)
6. [Repository layout](#repository-layout)

---

## Provenance and scope

| | |
|---|---|
| Flown | RP2350 (Pico 2) vehicle firmware, SAUVC 2026, 50 Hz control loop |
| Ported to | [TIBURON-AUV-FreeRTOS](https://github.com/Suchit-Arunkumar/TIBURON-AUV-FreeRTOS) and [Nucleo_AUV_Bare_Metal](https://github.com/Suchit-Arunkumar/Nucleo_AUV_Bare_Metal) (STM32F446RE) |
| This repo | The allocation stage only: wrench in, eight ESC pulse widths out |
| Out of scope | The upstream 6-DOF controller (co-developed) that produces the wrench |
| Thrust units | Normalised to full ESC command, \|T<sub>i</sub>\| ≤ 1. Thrust per ESC has not been calibrated in a tank, so nothing here is in newtons |

`src/alloc.c` keeps the firmware's group logic line for line. Deliberate
deviations, each reachable only with inputs the firmware never guarded:

- a non-finite wrench element is rejected (zero thrust, `ALLOC_REJECTED`); the
  firmware would pass NaN through an undefined float→int conversion to the ESCs;
- a non-finite thrust maps to neutral PWM.

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

The roll and pitch coupling terms of the horizontal thrusters (the +/-0.014 entries of B for T5-T8) have not been verified against the CAD.

`A` has full row rank (6), so every wrench is reachable, and the
8 − 6 = 2 dimensional null space is the freedom left over.

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
is computed offline once (SVD, `tools/pseudoinverse.m`) and stored as constants:
one 8×6 matrix-vector product per tick. It minimises the 2-norm, not the largest
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

In the flown allocator this holds exactly for the vertical group (T1–T4). On
T5–T8, translation and yaw are each renormalised, then summed and clipped per
thruster, so direction can shift when the sum saturates.

### What the flown allocator computes

The firmware forms three groups, each with its own slice of B⁺:

| Group | DOFs | Thrusters |
|---|---|---|
| vertical | heave, roll, pitch | T1–T4 |
| translation | surge, sway | T5–T8 |
| yaw | yaw | T5–T8 |

Each group is renormalised on its own; translation and yaw are then **added**
on T5–T8 and each thruster is clipped to ±1. Because the vertical group has its
own thrusters and its own scale factor, a large yaw or translation command
never reduces heave, roll or pitch authority.

The pseudo-inverse is used per group (block-diagonal), so the small roll/pitch
lever arms of the horizontal thrusters (±0.014) are not compensated.

---

## Pipeline

Per 50 Hz tick, in `src/alloc.c`:

| Stage | Function | What it does |
|---|---|---|
| 1 | `alloc_wrench_to_thrust` | wrench (surge, sway, heave, roll, pitch, yaw) → thrust T1–T8 in [−1, 1]; group renormalisation, sum, clip; returns saturation flags |
| 2 | `alloc_thrust_to_pwm` | polarity flip (T1 is mounted reversed), clamp, deadzone \|u\| < 0.02 → neutral, `1500 + (int)(u·400)` µs, truncated toward zero |
| 3 | `alloc_slew_step` | each output moves at most 50 µs per call (one 20 ms tick), then clamp to 1100–1900 µs |
| — | `alloc_neutral` | failsafe: every output to 1500 µs in the same tick, no ramp |

| Constant | Value |
|---|---|
| PWM min / neutral / max | 1100 / 1500 / 1900 µs |
| Deadzone | \|u\| < 0.02 |
| Slew | 50 µs per tick at 50 Hz (full scale in 8 ticks, 160 ms) |
| Saturation flags | bit 0 vertical, bit 1 translation, bit 2 yaw (same as the firmware telemetry byte) |

Thruster index → RP2350 GPIO in the vehicle firmware (pin map of 2026-10-07):
T1 9, T2 19, T3 14, T4 16, T5 22, T6 18, T7 15, T8 17. The allocation code
never touches pins.

`include/alloc_matrix.h` holds `A` and `B_pinv`, rounded to the 4 decimals the
firmware stores. `B_pinv` was computed with `tools/pseudoinverse.m`.

---

## Not verified

- **Thrust calibration.** Per-ESC thrust vs PWM was not measured in a tank;
  everything assumes linear, symmetric (forward = reverse) thrust.
- **No closed-loop result.** Tracking performance depends on the upstream
  controller, which is out of scope.

---

## Build

```bash
gcc -std=c99 -Wall -Wextra -c src/alloc.c -Iinclude
```

or, as a static library:

```bash
cmake -S . -B build && cmake --build build
```

`B_pinv` in `include/alloc_matrix.h` comes from `tools/pseudoinverse.m`
(MATLAB); rerun it if the thruster geometry in `A` changes.

---

## Repository layout

```
include/alloc.h          allocator API, constants, flags
include/alloc_matrix.h   A and B⁺ (4-decimal firmware values)
src/alloc.c              allocator, ported from the vehicle firmware
tools/pseudoinverse.m    MATLAB: B⁺ from A via SVD
CMakeLists.txt           builds src/alloc.c as a static library
```

MIT licensed.
