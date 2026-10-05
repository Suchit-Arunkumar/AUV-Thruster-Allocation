"""Run the C pipeline (alloc_vectors) and the NumPy reference on the same
seeded wrenches and report the largest deviation.

    python python/compare_c.py [--build build] [--n 10000] [--seed 2026]
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

import model
import reference as ref

ROOT = Path(__file__).resolve().parents[1]
DT = 0.02
RECORD = np.dtype([("thrust", "<f4", 8), ("target", "<i2", 8),
                   ("slewed", "<i2", 8), ("flags", "u1")])


def find_binary(build: Path) -> Path | None:
    for name in ("alloc_vectors", "alloc_vectors.exe"):
        for p in (build / name, build / "Release" / name):
            if p.exists():
                return p
    return None


def wrenches(n: int, seed: int) -> np.ndarray:
    """Per-axis uniform in +-scale x single-axis authority, with the scale
    drawn per row from [0, 3] so roughly half the rows saturate. Every 50th
    row is exactly zero. Authority = 1 / max_i |B+_ik| over the driven group.
    """
    rng = np.random.default_rng(seed)
    auth = 1.0 / np.max(np.abs(model.block_pinv()), axis=0)
    scale = rng.uniform(0.0, 3.0, size=(n, 1))
    w = rng.uniform(-1.0, 1.0, size=(n, 6)) * scale * auth
    w[::50] = 0.0
    return w.astype(np.float32)


def reference_records(w: np.ndarray, dt: float = DT) -> np.ndarray:
    t, flags = ref.wrench_to_thrust(w)
    target = ref.thrust_to_pwm(t)
    out = np.zeros(len(w), dtype=RECORD)
    cur = ref.neutral()
    for i in range(len(w)):
        cur = ref.slew_step(cur, target[i], dt)
        out["slewed"][i] = cur
    out["thrust"], out["target"], out["flags"] = t, target, flags
    return out


def run_c(binary: Path, w: np.ndarray, dt: float = DT) -> np.ndarray:
    with tempfile.TemporaryDirectory() as d:
        src, dst = Path(d) / "in.bin", Path(d) / "out.bin"
        w.astype("<f4").tofile(src)
        subprocess.run([str(binary), str(src), str(dst), repr(dt)],
                       check=True, capture_output=True)
        return np.fromfile(dst, dtype=RECORD)


def compare(binary: Path, n: int = 10_000, seed: int = 2026) -> dict:
    w = wrenches(n, seed)
    c, py = run_c(binary, w), reference_records(w)
    assert len(c) == n
    sat = (py["flags"] & 0x07) != 0
    return {
        "n_wrenches": n,
        "seed": seed,
        "n_saturated": int(sat.sum()),
        "thrust_max_abs_diff": float(np.max(np.abs(c["thrust"] - py["thrust"]))),
        "thrust_bit_identical": bool(np.array_equal(
            c["thrust"].view(np.uint32), py["thrust"].view(np.uint32))),
        "pwm_target_max_abs_diff_us": int(np.max(np.abs(
            c["target"].astype(int) - py["target"].astype(int)))),
        "pwm_slewed_max_abs_diff_us": int(np.max(np.abs(
            c["slewed"].astype(int) - py["slewed"].astype(int)))),
        "flags_mismatches": int(np.sum(c["flags"] != py["flags"])),
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--build", type=Path, default=ROOT / "build")
    ap.add_argument("--n", type=int, default=10_000)
    ap.add_argument("--seed", type=int, default=2026)
    ap.add_argument("--out", type=Path, default=ROOT / "results" / "c_vs_numpy.json")
    a = ap.parse_args()
    binary = find_binary(a.build)
    if binary is None:
        print(f"alloc_vectors not found under {a.build}; build with CMake first")
        return 1
    r = compare(binary, a.n, a.seed)
    a.out.parent.mkdir(exist_ok=True)
    a.out.write_text(json.dumps(r, indent=2) + "\n", newline="\n")
    print(json.dumps(r, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
