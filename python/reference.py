"""NumPy reference of the allocation pipeline in src/alloc.c.

Vectorised over N wrenches. With dtype=np.float32 every arithmetic step is
done in the same order and precision as the C code, so the two agree
bit-for-bit. dtype=np.float64 is used by the analysis.
"""

from __future__ import annotations

import numpy as np

import model

PWM_MIN, PWM_NEUTRAL, PWM_MAX = 1100, 1500, 1900
DEADZONE = 0.02
SLEW_US_PER_S = 2500.0
REVERSE_THRUST = np.array([1, 0, 0, 0, 0, 0, 0, 0], dtype=bool)

SAT_VERT, SAT_TRANS, SAT_YAW, REJECTED = 0x01, 0x02, 0x04, 0x80

B_PINV = model.firmware_pinv()


def _renormalise(T: np.ndarray, rows: slice) -> tuple[np.ndarray, np.ndarray]:
    """Divide the group by its largest |T| when that exceeds 1."""
    m = np.max(np.abs(T[:, rows]), axis=1)
    sat = m > 1
    T = T.copy()
    T[sat, rows] = T[sat, rows] / m[sat, None]
    return T, sat


def wrench_to_thrust(W, dtype=np.float32, B: np.ndarray | None = None):
    """Returns (thrust (N,8), flags (N,) uint8). Mirrors alloc_wrench_to_thrust()."""
    W = np.atleast_2d(np.asarray(W, dtype=dtype))
    B = (B_PINV if B is None else B).astype(dtype)
    one = dtype(1)
    with np.errstate(invalid="ignore", over="ignore"):
        return _wrench_to_thrust(W, B, one, dtype)


def _wrench_to_thrust(W, B, one, dtype):

    def col(k):  # B[i][k] * U[k] for all i, shape (N, 8)
        return B[:, k][None, :] * W[:, k][:, None]

    T_trans = col(0) + col(1)
    T_yaw = col(5)
    T_vert = (col(2) + col(3)) + col(4)

    T_vert, sv = _renormalise(T_vert, slice(0, 4))
    T_trans, st = _renormalise(T_trans, slice(4, 8))
    T_yaw, sy = _renormalise(T_yaw, slice(4, 8))

    T = np.concatenate([T_vert[:, :4], T_trans[:, 4:] + T_yaw[:, 4:]], axis=1)
    T = np.clip(T, -one, one).astype(dtype)

    flags = (sv * SAT_VERT | st * SAT_TRANS | sy * SAT_YAW).astype(np.uint8)

    bad = ~np.all(np.isfinite(W), axis=1)
    T[bad] = 0
    flags[bad] = REJECTED
    return T, flags


def thrust_to_pwm(T) -> np.ndarray:
    """Mirrors alloc_thrust_to_pwm(). Returns int16 (N,8)."""
    u = np.atleast_2d(np.asarray(T, dtype=np.float32)).copy()
    finite = np.isfinite(u)
    u = np.where(finite, u, np.float32(0))
    u[:, REVERSE_THRUST] = -u[:, REVERSE_THRUST]
    u = np.clip(u, np.float32(-1), np.float32(1))
    span = np.float32(PWM_MAX - PWM_NEUTRAL)        # == PWM_NEUTRAL - PWM_MIN
    pwm = PWM_NEUTRAL + np.trunc(u * span).astype(np.int32)
    pwm[np.abs(u) < np.float32(DEADZONE)] = PWM_NEUTRAL
    pwm[~finite] = PWM_NEUTRAL
    return pwm.astype(np.int16)


def slew_step_size(dt: float) -> int:
    dt = np.float32(dt)
    if not dt > 0:
        return 0
    f = np.float32(SLEW_US_PER_S) * dt + np.float32(0.5)
    return PWM_MAX - PWM_MIN if f >= PWM_MAX - PWM_MIN else int(f)


def slew_step(current, target, dt: float) -> np.ndarray:
    """Mirrors alloc_slew_step() for one tick."""
    step = slew_step_size(dt)
    cur = np.asarray(current, dtype=np.int32)
    delta = np.clip(np.asarray(target, dtype=np.int32) - cur, -step, step)
    return np.clip(cur + delta, PWM_MIN, PWM_MAX).astype(np.int16)


def neutral() -> np.ndarray:
    return np.full(model.N_THR, PWM_NEUTRAL, dtype=np.int16)
