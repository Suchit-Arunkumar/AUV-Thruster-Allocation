"""C build: unit-test runner and C-vs-NumPy comparison.

Skipped when the C targets have not been built, unless ALLOC_REQUIRE_C=1
(set in CI), which turns a missing build into a failure.
"""

import os
import re
import subprocess

import pytest

import compare_c

BUILD = compare_c.ROOT / os.environ.get("ALLOC_BUILD_DIR", "build")


def _binary(name):
    for p in (BUILD / name, BUILD / f"{name}.exe",
              BUILD / "Release" / name, BUILD / "Release" / f"{name}.exe"):
        if p.exists():
            return p
    if os.environ.get("ALLOC_REQUIRE_C") == "1":
        pytest.fail(f"{name} not built under {BUILD}")
    pytest.skip(f"{name} not built under {BUILD}")


def test_c_unit_runner():
    r = subprocess.run([str(_binary("test_alloc"))], capture_output=True, text=True)
    assert r.returncode == 0, r.stdout
    m = re.search(r"(\d+)/(\d+) checks passed", r.stdout)
    assert m and m.group(1) == m.group(2)


def test_c_matches_numpy_bit_for_bit():
    _binary("alloc_vectors")
    r = compare_c.compare(compare_c.find_binary(BUILD), n=10_000, seed=2026)
    assert r["n_saturated"] > 1000          # the set exercises saturation
    assert r["thrust_bit_identical"]
    assert r["pwm_target_max_abs_diff_us"] == 0
    assert r["pwm_slewed_max_abs_diff_us"] == 0
    assert r["flags_mismatches"] == 0
