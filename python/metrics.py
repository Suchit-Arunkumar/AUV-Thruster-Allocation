"""Shared measures for the analysis and the tests.

Authority of a DOF = the largest single-axis command the FLOWN allocation
delivers without saturating: 1 / max_i |B+_ik| over the thrusters that DOF
drives. Wrenches are divided by it before measuring angles or magnitudes, so
one degree means the same thing on surge as on yaw.
"""

from __future__ import annotations

import numpy as np

import model

AUTH = 1.0 / np.max(np.abs(model.block_pinv()), axis=0)


def achieved(T) -> np.ndarray:
    """Wrench produced by thrusts T (N,8), computed in float64."""
    return np.asarray(T, dtype=np.float64) @ model.A.T


def _scaled(W, dof):
    return np.asarray(W, dtype=np.float64)[:, dof] / AUTH[dof]


def angle_deg(W, T, dof=slice(None)) -> np.ndarray:
    """Angle between requested W and achieved A T, authority-scaled."""
    w, a = _scaled(W, dof), _scaled(achieved(T), dof)
    c = np.sum(a * w, axis=1) / (np.linalg.norm(a, axis=1) * np.linalg.norm(w, axis=1))
    return np.degrees(np.arccos(np.clip(c, -1.0, 1.0)))


def magnitude_loss(W, T, dof=slice(None)) -> np.ndarray:
    """1 - (component of the achieved wrench along the request) / |request|,
    authority-scaled. 0 = delivered in full; 0.4 = 40 % of it lost."""
    w, a = _scaled(W, dof), _scaled(achieved(T), dof)
    n = np.linalg.norm(w, axis=1)
    return 1.0 - np.sum(a * w, axis=1) / n**2


def stats(x) -> dict:
    x = np.asarray(x)
    return {"mean": float(np.mean(x)), "p95": float(np.percentile(x, 95)),
            "max": float(np.max(x))}
