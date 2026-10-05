"""Read the allocation bench from the NUCLEO-F446RE and check its output.

    python firmware/nucleo_bench/read_bench.py --port COM12
    python firmware/nucleo_bench/read_bench.py --from-file pasted.txt --save

Waits for the second complete BEGIN..END block (the first run includes cold
flash-cache effects), prints it, converts cycles to microseconds at the
reported SYSCLK, and recomputes the output hash with the NumPy reference.
--save writes results/nucleo_bench.json.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT / "python"))

import compare_c  # noqa: E402

N, SEED, DT = 1000, 2026, 0.02
AUTH_DEN = (0.7072, 0.8347, 0.3060, 2.0482, 2.0306, 2.9767)   # as in main.c


def wrenches() -> np.ndarray:
    """main.c make_wrenches(), float32 op for op."""
    f = np.float32
    auth = [f(1.0) / f(d) for d in AUTH_DEN]
    s = SEED

    def u01():
        nonlocal s
        s ^= (s << 13) & 0xFFFFFFFF
        s ^= s >> 17
        s ^= (s << 5) & 0xFFFFFFFF
        return f(s >> 8) * f(1.0 / 16777216.0)

    w = np.zeros((N, 6), dtype=np.float32)
    for i in range(N):
        r = f(3.0) * u01()
        for k in range(6):
            w[i, k] = ((f(2.0) * u01() - f(1.0)) * r) * auth[k]
    return w


def expected_hash() -> int:
    rec = compare_c.reference_records(wrenches(), DT)
    h = 2166136261
    for row in rec:
        for b in (row["thrust"].tobytes(), row["target"].tobytes(),
                  row["slewed"].tobytes(), row["flags"].tobytes()):
            for x in b:
                h = ((h ^ x) * 16777619) & 0xFFFFFFFF
    return h


def parse(text: str) -> dict | None:
    blocks = re.findall(r"BEGIN run=(\d+)\r?\n(.*?)END", text, re.S)
    warm = [b for b in blocks if int(b[0]) >= 2]
    if not warm:
        return None
    run, body = warm[-1]
    kv, stages = {"run": int(run)}, {}
    for line in body.strip().splitlines():
        parts = line.split()
        if len(parts) == 7 and parts[1] == "min":
            stages[parts[0]] = {"min": int(parts[2]), "max": int(parts[4]), "sum": int(parts[6])}
        elif len(parts) == 2:
            kv[parts[0]] = parts[1]
    hz = int(kv["sysclk_hz"])
    n = int(kv["n"])
    out = {"run": kv["run"], "sysclk_hz": hz, "n_calls": n, "seed": int(kv["seed"]),
           "saturated_calls": int(kv["saturated"]), "hash": kv["hash"], "stages": {}}
    for name, s in stages.items():
        mean = s["sum"] / n
        out["stages"][name] = {
            "cycles_min": s["min"], "cycles_mean": round(mean, 1), "cycles_max": s["max"],
            "us_mean": round(mean / hz * 1e6, 3), "us_max": round(s["max"] / hz * 1e6, 3)}
    return out


def read_serial(port: str, baud: int, timeout_s: float) -> str:
    import time

    import serial  # pyserial

    buf, t0 = "", time.time()
    with serial.Serial(port, baud, timeout=0.5) as ser:
        while time.time() - t0 < timeout_s:
            buf += ser.read(4096).decode("ascii", errors="replace")
            if parse(buf):
                return buf
    return buf


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM12")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--timeout", type=float, default=15.0)
    ap.add_argument("--from-file", type=Path)
    ap.add_argument("--save", action="store_true")
    a = ap.parse_args()

    text = a.from_file.read_text() if a.from_file else read_serial(a.port, a.baud, a.timeout)
    r = parse(text)
    if r is None:
        print("no complete warm run found; raw output:\n" + text)
        return 1
    exp = f"0x{expected_hash():08x}"
    r["expected_hash"] = exp
    r["output_matches_reference"] = r["hash"] == exp
    print(json.dumps(r, indent=2))
    if a.save:
        (ROOT / "results" / "nucleo_bench.json").write_text(json.dumps(r, indent=2) + "\n", newline="\n")
        print("wrote results/nucleo_bench.json")
    return 0 if r["output_matches_reference"] else 2


if __name__ == "__main__":
    sys.exit(main())
