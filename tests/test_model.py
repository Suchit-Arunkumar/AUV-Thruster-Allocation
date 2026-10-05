"""The matrix, its pseudo-inverse, and the generated header."""

import numpy as np

import model
from firmware_constants import FIRMWARE_B_PINV

FW = np.array(FIRMWARE_B_PINV)


def test_full_row_rank():
    assert model.properties()["rank"] == 6


def test_pinv_matches_firmware_to_4_decimals():
    # Rounding to 4 decimals moves a value by at most 5e-5.
    assert np.max(np.abs(model.pinv() - FW)) < 5e-5


def test_rounded_pinv_is_firmware_literal():
    assert np.array_equal(model.firmware_pinv(), FW)


def test_pinv_is_right_inverse():
    assert np.allclose(model.A @ model.pinv(), np.eye(6), atol=1e-12)
    # The 4-decimal firmware literals leave a 1e-4 residual.
    assert np.max(np.abs(model.A @ FW - np.eye(6))) < 2e-4


def test_pinv_is_minimum_norm():
    # Any other exact solution differs by a null-space vector and is longer.
    rng = np.random.default_rng(0)
    null = np.linalg.svd(model.A)[2][6:]
    for _ in range(100):
        w = rng.normal(size=6)
        t = model.pinv() @ w
        t2 = t + null.T @ rng.normal(size=2)
        assert np.allclose(model.A @ t2, w)
        assert np.linalg.norm(t2) >= np.linalg.norm(t)


def test_header_up_to_date():
    assert model.HEADER.read_text() == model.render_header()
    assert model.source_hash() in model.HEADER.read_text()
