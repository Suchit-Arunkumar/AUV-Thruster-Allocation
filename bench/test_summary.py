"""Build the C targets, run every test, and record the counts.

    python bench/test_summary.py   -> results/tests.json, results/c_vs_numpy.json

Needs a host C compiler and CMake (pip install cmake ninja) on PATH.
"""

from __future__ import annotations

import json
import os
import re
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BUILD = ROOT / "build"


def run(cmd, check=True, env=None):
    print("$", " ".join(map(str, cmd)))
    return subprocess.run(cmd, cwd=ROOT, check=check, text=True,
                          capture_output=True, env=env)


def main() -> int:
    run(["cmake", "-S", ".", "-B", str(BUILD), "-G", "Ninja", "-DCMAKE_BUILD_TYPE=Release"])
    run(["cmake", "--build", str(BUILD)])

    cache = (BUILD / "CMakeCache.txt").read_text()
    cc = re.search(r"^CMAKE_C_COMPILER:\w+=(.+)$", cache, re.M).group(1)
    compiler = run([cc, "--version"]).stdout.splitlines()[0]

    exe = next(p for p in (BUILD / "test_alloc", BUILD / "test_alloc.exe") if p.exists())
    out = run([str(exe)], check=False).stdout
    passed, total = map(int, re.search(r"(\d+)/(\d+) checks passed", out).groups())

    junit = BUILD / "pytest.xml"
    run([sys.executable, "-m", "pytest", "-q", f"--junitxml={junit}"], check=False,
        env={**os.environ, "ALLOC_REQUIRE_C": "1"})
    suite = ET.parse(junit).getroot()
    suite = suite if suite.tag == "testsuite" else suite[0]
    n, fail, err, skip = (int(suite.get(k)) for k in ("tests", "failures", "errors", "skipped"))

    run([sys.executable, "python/compare_c.py", "--build", str(BUILD)])

    res = {
        "c_unit_checks": {"passed": passed, "total": total},
        "pytest": {"tests": n, "passed": n - fail - err - skip,
                   "failed": fail + err, "skipped": skip},
        "host_compiler": compiler,
    }
    (ROOT / "results" / "tests.json").write_text(json.dumps(res, indent=2) + "\n", newline="\n")
    print(json.dumps(res, indent=2))
    return 0 if res["pytest"]["failed"] == 0 and passed == total else 1


if __name__ == "__main__":
    sys.exit(main())
