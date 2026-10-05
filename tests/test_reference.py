"""Behaviour of the NumPy reference pipeline (same cases as tests/c)."""

import numpy as np
import pytest

import model
import reference as ref

B = model.firmware_pinv()


def test_zero():
    t, f = ref.wrench_to_thrust(np.zeros(6))
    assert not t.any() and f[0] == 0
    assert (ref.thrust_to_pwm(t) == 1500).all()


@pytest.mark.parametrize("k", range(6))
def test_single_axis_uses_one_group(k):
    w = np.zeros(6)
    w[k] = 0.1
    t, f = ref.wrench_to_thrust(w)
    vert = 2 <= k <= 4
    driven = np.arange(8) < 4 if vert else np.arange(8) >= 4
    assert f[0] == 0
    assert np.allclose(t[0, driven], B[driven, k] * 0.1, atol=1e-7)
    assert not t[0, ~driven].any()


def test_group_renormalisation_keeps_direction():
    w = np.array([3.0, -2.0, 0, 0, 0, 0])
    t, f = ref.wrench_to_thrust(w, dtype=np.float64)
    assert f[0] == ref.SAT_TRANS
    unsat = model.block_pinv() @ w
    k = np.max(np.abs(unsat[4:]))
    assert np.allclose(t[0, 4:], unsat[4:] / k)


def test_everything_saturated_stays_in_range():
    t, f = ref.wrench_to_thrust([80, -80, 80, 20, -20, 20])
    assert f[0] == ref.SAT_VERT | ref.SAT_TRANS | ref.SAT_YAW
    assert np.max(np.abs(t)) <= 1.0


@pytest.mark.parametrize("bad", [np.nan, np.inf, -np.inf])
@pytest.mark.parametrize("k", range(6))
def test_non_finite_rejected(bad, k):
    w = np.array([0.3, -0.2, 0.5, 0.1, -0.1, 0.05])
    w[k] = bad
    t, f = ref.wrench_to_thrust(w)
    assert f[0] == ref.REJECTED and not t.any()
    assert (ref.thrust_to_pwm(t) == 1500).all()


@pytest.mark.parametrize("u, idx, pwm", [
    (0.0199, 2, 1500), (0.02, 2, 1508), (-0.0199, 2, 1500), (-0.02, 2, 1492),
    (0.5049, 2, 1701), (-0.5049, 2, 1299), (0.5, 0, 1300), (-0.5, 0, 1700),
    (1.0, 4, 1900), (-1.0, 4, 1100), (5.0, 4, 1900), (-5.0, 4, 1100),
    (5.0, 0, 1100), (np.nan, 3, 1500),
])
def test_pwm_map(u, idx, pwm):
    t = np.zeros(8, dtype=np.float32)
    t[idx] = u
    assert ref.thrust_to_pwm(t)[0, idx] == pwm


@pytest.mark.parametrize("dt, step", [
    (0.02, 50), (0.0199, 50), (0.01, 25), (0.0, 0), (-1.0, 0), (np.nan, 0),
    (10.0, 800), (np.inf, 800),
])
def test_slew_step_size(dt, step):
    assert ref.slew_step_size(dt) == step


def test_slew_reaches_full_scale_in_8_ticks():
    cur = ref.neutral()
    up = np.full(8, 1900)
    for n in range(8):
        cur = ref.slew_step(cur, up, 0.02)
        assert (cur == 1500 + 50 * (n + 1)).all()
