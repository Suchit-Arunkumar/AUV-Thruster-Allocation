"""v2 allocators (NOT FLOWN): NumPy reference properties."""

import numpy as np
import pytest

import compare_c
import metrics
import model
import reference as ref


@pytest.fixture(scope="module")
def W():
    return compare_c.wrenches(10_000, 2026)


def test_v2_unsaturated_has_no_cross_coupling(W):
    T, f = ref.v2_wrench_to_thrust(W)
    ok = f == 0
    assert ok.sum() > 1000
    err = np.abs(metrics.achieved(T[ok]) - W[ok].astype(np.float64))
    assert err.max() < 1e-5


def test_v2_direction_exact_whenever_it_scales(W):
    T, f = ref.v2_wrench_to_thrust(W)
    sc = f == ref.V2_SCALED
    assert sc.sum() > 1000
    assert np.max(np.abs(T[sc])) == 1.0
    assert metrics.angle_deg(W[sc], T[sc]).max() <= 1e-4
    # Also in plain (unscaled) wrench units.
    a, w = metrics.achieved(T[sc]), W[sc].astype(np.float64)
    c = np.sum(a * w, 1) / np.linalg.norm(a, axis=1) / np.linalg.norm(w, axis=1)
    assert np.degrees(np.arccos(np.clip(c, -1, 1))).max() <= 1e-4


def test_v2p_delivers_attitude_in_full_when_it_fits(W):
    T, f = ref.v2p_wrench_to_thrust(W)
    sec = f == ref.V2_SECONDARY
    assert sec.sum() > 100
    a = metrics.achieved(T[sec])[:, 2:5]
    assert np.abs(a - W[sec, 2:5]).max() < 1e-5
    assert np.abs(T).max() <= 1.0


def test_v2p_equals_v2_when_nothing_saturates(W):
    T2, f2 = ref.v2_wrench_to_thrust(W)
    Tp, fp = ref.v2p_wrench_to_thrust(W)
    ok = (f2 == 0) & (fp == 0)
    assert np.abs(T2[ok] - Tp[ok]).max() < 1e-6


@pytest.mark.parametrize("fn", [ref.v2_wrench_to_thrust, ref.v2p_wrench_to_thrust])
def test_v2_rejects_non_finite(fn):
    T, f = fn([[0.1, np.inf, 0, 0, 0, 0], [np.nan] * 6])
    assert (f == ref.REJECTED).all() and not T.any()


def test_full_pinv_is_right_inverse():
    assert np.abs(model.A @ model.full_pinv_f32().astype(np.float64) - np.eye(6)).max() < 1e-6
