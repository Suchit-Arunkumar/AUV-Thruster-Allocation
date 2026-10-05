"""Fill README.md from docs/README.in.md and results/*.json.

    python bench/render_readme.py           # write README.md
    python bench/render_readme.py --check   # exit 1 if README.md is stale

Placeholders:
  {{file.key.0.key:fmt}}  a value from results/<file>.json, Python format spec
  {{@name}}               a generated block (tables, test count, bench)
No figure in the README is typed by hand.
"""

from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
RES = ROOT / "results"
TEMPLATE = ROOT / "docs" / "README.in.md"
README = ROOT / "README.md"

VARIANT_NAMES = {"flown": "Flown (group renormalisation)",
                 "clip": "Per-thruster clip (baseline)",
                 "v2": "v2: full B⁺, uniform scale (not flown)",
                 "v2p": "v2p: v2 + depth/attitude priority (not flown)"}


def load() -> dict:
    data = {p.stem: json.loads(p.read_text(encoding="utf-8")) for p in RES.glob("*.json")}
    d = data["direction"]["saturated"]
    data["derived"] = {
        "v2_extra_loss_pts": 100 * (d["v2"]["magnitude_loss"]["mean"]
                                    - d["flown"]["magnitude_loss"]["mean"]),
        "flown_better_pct": 100 * data["direction"]["flown_vs_clip"]["flown_better"],
        "flown_worse_pct": 100 * data["direction"]["flown_vs_clip"]["flown_worse"],
        "sat_pct": 100 * data["direction"]["n_saturated"] / data["direction"]["n_wrenches"],
    }
    return data


def lookup(data: dict, path: str):
    v = data
    for part in path.split("."):
        v = v[int(part)] if isinstance(v, list) else v[part]
    return v


SUP = str.maketrans("-0123456789", "⁻⁰¹²³⁴⁵⁶⁷⁸⁹")


def sci(x: float, digits: int = 1) -> str:
    """4.4e-08 -> '4.4 × 10⁻⁸' (real superscripts, no caret)."""
    m, e = f"{x:.{digits}e}".split("e")
    return f"{m} × 10{str(int(e)).translate(SUP)}"


def f(x, spec=None):
    if spec and spec.startswith("sci"):
        return sci(x, int(spec[3:] or 1))
    if spec is None:
        return format(x, ".3g") if isinstance(x, float) else str(x)
    return format(x, spec)


# ---- generated blocks -----------------------------------------------------
def direction_table(data) -> str:
    d = data["direction"]["saturated"]
    out = ["| Allocator | Direction error mean / p95 / max | Magnitude lost mean / p95 / max "
           "| Heave-roll-pitch direction max | Heave-roll-pitch magnitude lost mean |",
           "|---|---|---|---|---|"]
    for v, name in VARIANT_NAMES.items():
        s = d[v]
        dd, ml = s["direction_deg"], s["magnitude_loss"]
        out.append(
            f"| {name} | {dd['mean']:.1f}° / {dd['p95']:.1f}° / {dd['max']:.1f}° "
            f"| {ml['mean']:.1%} / {ml['p95']:.1%} / {ml['max']:.1%} "
            f"| {s['primary_direction_deg']['max']:.1f}° "
            f"| {s['primary_magnitude_loss']['mean']:.1%} |")
    return "\n".join(out)


def deg(x: float) -> str:
    return f"{sci(x)}°" if x < 1e-3 else f"{x:.1f}°"


def group_table(data) -> str:
    g = data["direction"]["flown_vs_clip"]["group_angle_deg"]
    out = ["| Group | Flown mean / max | Per-thruster clip mean / max |", "|---|---|---|"]
    for name, s in g.items():
        v = [deg(s[k][q]) for k in ("flown", "clip") for q in ("mean", "max")]
        out.append(f"| {name} | {v[0]} / {v[1]} | {v[2]} / {v[3]} |")
    return "\n".join(out)


def coupling_table(data) -> str:
    t = data["matrix"]["block_cross_coupling_terms"]
    out = ["| Unit command | Unwanted output |", "|---|---|"]
    for k, v in sorted(t.items(), key=lambda kv: -abs(kv[1])):
        src, dst = k.split("->")
        out.append(f"| {src} = 1 | {dst} = {v:+.4f} |")
    return "\n".join(out)


def lever_table(data) -> str:
    c = data["matrix"]["lever_arm_check"]["columns"]
    out = ["| Thruster | d (x, y) | A: (Mx, My) | r×d with T5's r<sub>z</sub>: (Mx, My) | M·d |",
           "|---|---|---|---|---|"]
    mdd = data["matrix"]["column_moment_dot_direction"]
    for j, (name, s) in enumerate(c.items()):
        a = [f"**{v:+.3f}**" if bad else f"{v:+.3f}"
             for v, bad in zip(s["A_Mx_My"], s["sign_mismatch"])]
        p = [f"{v:+.3f}" for v in s["predicted_Mx_My"]]
        out.append(f"| {name} | ({s['direction'][0]:+.3f}, {s['direction'][1]:+.3f}) "
                   f"| ({a[0]}, {a[1]}) | ({p[0]}, {p[1]}) | {mdd[4 + j]} |")
    return "\n".join(out)


def saturation_table(data) -> str:
    s = data["saturation"]
    lv = s["levels"]
    pick = [lv.index(x) for x in (0.4, 0.5, 0.6, 0.8, 1.0)]
    out = ["| Command range (× authority) | " + " | ".join(f"±{lv[i]:.1f}" for i in pick) + " |",
           "|---|" + "---|" * len(pick)]
    for dof, rates in s["per_dof_rate"].items():
        out.append(f"| {dof} | " + " | ".join(f"{100 * rates[i]:.1f}%" for i in pick) + " |")
    v2 = s["group_flag_rate"]["v2_scaled"]
    out.append("| *v2 scales (any DOF)* | " + " | ".join(f"{100 * v2[i]:.1f}%" for i in pick) + " |")
    return "\n".join(out)


def authority_table(data) -> str:
    a = data["matrix"]["authority"]
    return ("| " + " | ".join(a) + " |\n|" + "---|" * len(a) + "\n| "
            + " | ".join(f"{v:.3f}" for v in a.values()) + " |")


def bench_block(data) -> str:
    b = data.get("nucleo_bench")
    if b is None:
        return ("*Not yet measured.* The bench firmware is in `firmware/nucleo_bench/`; "
                "`results/nucleo_bench.json` is written only from the board's own output.")
    hz = b["sysclk_hz"] / 1e6
    out = [f"NUCLEO-F446RE, {hz:.0f} MHz, arm-none-eabi-gcc `-O2 -ffp-contract=off`, "
           f"{b['n_calls']} seeded wrenches ({b['saturated_calls']} saturated in the flown "
           f"allocator), run {b['run']} (warm flash cache), DWT CYCCNT, read overhead subtracted.",
           "", "| Stage | Cycles min / mean / max | µs mean / max |", "|---|---|---|"]
    names = {"wrench_to_thrust": "Flown: wrench → thrust", "thrust_to_pwm": "thrust → PWM (8 ESCs)",
             "slew_step": "slew step", "total": "**Flown pipeline total**",
             "v2_wrench_to_thrust": "v2: wrench → thrust (not flown)",
             "v2p_wrench_to_thrust": "v2p: wrench → thrust (not flown)"}
    for k, name in names.items():
        s = b["stages"].get(k)
        if s:
            out.append(f"| {name} | {s['cycles_min']} / {s['cycles_mean']} / {s['cycles_max']} "
                       f"| {s['us_mean']} / {s['us_max']} |")
    ok = b["output_matches_reference"]
    out += ["", "On-target output vs NumPy reference (FNV-1a over every thrust, PWM and flag byte): "
            + ", ".join(f"{k.replace('_hash', '') if k != 'hash' else 'flown'} "
                        f"{'match' if v else 'MISMATCH'}" for k, v in ok.items()) + "."]
    return "\n".join(out)


def bench_key(data) -> str:
    b = data.get("nucleo_bench")
    if b is None:
        return "pending bench run"
    t = b["stages"]["total"]
    return f"{t['cycles_mean']} cycles mean, {t['cycles_max']} max ({t['us_mean']} / {t['us_max']} µs)"


def pytest_count(_data) -> str:
    out = subprocess.run([sys.executable, "-m", "pytest", "--collect-only", "-q"],
                         cwd=ROOT, capture_output=True, text=True).stdout
    return re.search(r"(\d+) tests? collected", out).group(1)


BLOCKS = {"direction_table": direction_table, "group_table": group_table,
          "coupling_table": coupling_table, "lever_table": lever_table,
          "saturation_table": saturation_table, "authority_table": authority_table,
          "bench": bench_block, "bench_key": bench_key, "pytest_count": pytest_count}


def render() -> str:
    data = load()

    def sub(m):
        key, spec = m.group(1), m.group(2)
        if key.startswith("@"):
            return BLOCKS[key[1:]](data)
        return f(lookup(data, key), spec)

    body = re.sub(r"\{\{([^}:]+)(?::([^}]*))?\}\}", sub, TEMPLATE.read_text(encoding="utf-8"))
    return ("<!-- Generated by bench/render_readme.py from docs/README.in.md. Edit the template. -->\n"
            + body)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true")
    a = ap.parse_args()
    text = render()
    if a.check:
        if README.read_text(encoding="utf-8") != text:
            print("README.md is stale: run python bench/render_readme.py")
            return 1
        print("README.md is up to date")
        return 0
    README.write_text(text, encoding="utf-8", newline="\n")
    print("wrote README.md")
    return 0


if __name__ == "__main__":
    sys.exit(main())
