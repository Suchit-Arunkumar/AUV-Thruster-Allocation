"""Every number in the README comes from this script (plus compare_c.py,
the test suite, and the on-target bench). Deterministic: fixed seeds.

    python bench/analysis.py        -> results/*.json, results/*.png

Units: thrust normalised to full ESC authority (|T_i| <= 1); forces in the
same units, moments in those units x metres. Per-ESC thrust has not been
calibrated in a tank, so nothing here is in newtons.

Angles and magnitudes are measured after dividing each DOF by its authority
(python/metrics.py), so one degree means the same thing on surge as on yaw.

Variants, all on the same seeded wrenches:
  flown  the allocator that ran at SAUVC 2026 (src/alloc.c)
  clip   same block B+, each thruster clipped on its own (baseline)
  v2     full B+, one uniform scale (src/alloc_v2.c, NOT FLOWN)
  v2p    v2 with heave/roll/pitch before surge/sway/yaw (NOT FLOWN)
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python"))
sys.path.insert(0, str(ROOT / "tests"))

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

import metrics  # noqa: E402
import model  # noqa: E402
import reference as ref  # noqa: E402
from firmware_constants import FIRMWARE_B_PINV  # noqa: E402
from metrics import AUTH, angle_deg, magnitude_loss, stats  # noqa: E402

OUT = ROOT / "results"
A = model.A
P_FW = model.block_pinv()                     # what computeAllocation() uses
DOF = model.DOF
PRIMARY = [2, 3, 4]                           # heave, roll, pitch (Fz, Mx, My)

COLORS = {"flown": "#4f81bd", "clip": "#c0504d", "v2": "#2e8b57", "v2p": "#8e6bb8"}
LABELS = {"flown": "flown (group renormalisation)", "clip": "per-thruster clip",
          "v2": "v2 uniform scale (not flown)", "v2p": "v2p prioritised (not flown)"}


def r6(x):
    """Round floats for stable JSON."""
    if isinstance(x, dict):
        return {k: r6(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [r6(v) for v in x]
    if isinstance(x, (float, np.floating)):
        return float(f"{float(x):.6g}")
    if isinstance(x, np.integer):
        return int(x)
    return x


def dump(name: str, data: dict) -> None:
    (OUT / name).write_text(json.dumps(r6(data), indent=2) + "\n", newline="\n")
    print(f"wrote results/{name}")


def allocate(W) -> tuple[dict, np.ndarray]:
    """Thrusts for every variant (float64), and which rows saturate in any."""
    T_fw, f_fw = ref.wrench_to_thrust(W, dtype=np.float64)
    T_v2, f_v2 = ref.v2_wrench_to_thrust(W, dtype=np.float64)
    T_v2p, _ = ref.v2p_wrench_to_thrust(W, dtype=np.float64)
    pre = W @ P_FW.T
    T = {"flown": T_fw, "clip": np.clip(pre, -1.0, 1.0), "v2": T_v2, "v2p": T_v2p}
    sat = (f_fw != 0) | np.any(np.abs(pre) > 1.0, axis=1) | (f_v2 != 0)
    return T, sat


def group_angle_deg(W, T, thr, dof):
    """Angle inside one flown group: what its thrusters produce on its DOFs
    vs what they would produce unsaturated."""
    Ag = A[np.ix_(dof, thr)]
    want = (W @ P_FW.T)[:, thr] @ Ag.T / AUTH[dof]
    got = T[:, thr] @ Ag.T / AUTH[dof]
    c = np.sum(want * got, axis=1) / (np.linalg.norm(want, axis=1) * np.linalg.norm(got, axis=1))
    return np.degrees(np.arccos(np.clip(c, -1.0, 1.0)))


# --------------------------------------------------------------------------
def matrix() -> dict:
    fw = np.array(FIRMWARE_B_PINV)
    p = model.pinv()
    d = np.abs(p - fw)
    i, k = np.unravel_index(np.argmax(d), d.shape)
    coupling = A @ P_FW
    off = coupling - np.diag(np.diag(coupling))
    # A moment produced by a force d at position r is r x d, so it is
    # orthogonal to d. A nonzero M.d means the column is not of that form.
    m_dot_d = [float(A[3:, j] @ A[:3, j] / np.linalg.norm(A[:3, j])) for j in range(8)]
    out = model.properties()
    out.update({
        "pinv_vs_firmware_max_abs_diff": float(d.max()),
        "pinv_vs_firmware_argmax": f"T{i + 1} / {DOF[k]}",
        "firmware_pinv_residual_max_abs": float(np.max(np.abs(A @ fw - np.eye(6)))),
        "authority": dict(zip(DOF, AUTH.tolist())),
        "block_A_times_P": coupling.tolist(),
        "block_max_cross_coupling": float(np.max(np.abs(off))),
        "block_cross_coupling_terms": {
            f"{DOF[c]}->{model.WRENCH[r]}": float(off[r, c])
            for r in range(6) for c in range(6) if abs(off[r, c]) > 1e-3},
        "dropped_pinv_max_abs": float(np.max(np.abs(fw - P_FW))),
        "column_moment_dot_direction": [f"{v:+.4f}" for v in m_dot_d],
        "lever_arm_check": lever_arm_check(),
    })
    return out


def lever_arm_check() -> dict:
    """Horizontal thrusters, d = (dx, dy, 0): r x d gives Mx = -rz dy and
    My = rz dx. Take rz from T5 and predict Mx, My for T5-T8."""
    d = A[:2, 4:]
    rz = -A[3, 4] / d[1, 0]
    rows = {}
    for j in range(4):
        dx, dy = d[:, j]
        pred = (-rz * dy, rz * dx)
        rows[f"T{j + 5}"] = {
            "direction": [float(dx), float(dy)],
            "A_Mx_My": [float(A[3, 4 + j]), float(A[4, 4 + j])],
            "predicted_Mx_My": [float(pred[0]), float(pred[1])],
            "sign_mismatch": [bool(np.sign(pred[0]) != np.sign(A[3, 4 + j])),
                              bool(np.sign(pred[1]) != np.sign(A[4, 4 + j]))],
        }
    return {"rz_from_T5_m": float(rz), "columns": rows}


def reconstruction(n=100_000, seed=1) -> dict:
    """Commands no variant saturates: achieved vs requested wrench."""
    rng = np.random.default_rng(seed)
    W = rng.uniform(-1, 1, size=(n, 6)) * AUTH * 0.4
    T, sat = allocate(W)
    W = W[~sat]
    out = {"n_unsaturated": int((~sat).sum()),
           "sampling": "each DOF uniform in +-0.4 x authority, seed 1"}
    for v in ("flown", "v2"):
        err = np.abs(metrics.achieved(T[v][~sat]) - W)
        out[v] = {"max_abs_error": float(err.max()),
                  "max_abs_error_per_axis": dict(zip(model.WRENCH, err.max(axis=0).tolist())),
                  "direction_error_deg": stats(angle_deg(W, T[v][~sat]))}
    full = np.abs(W @ np.array(FIRMWARE_B_PINV).T @ A.T - W)
    out["full_4dp_pinv_max_abs_error"] = float(full.max())
    return out


def direction(n=100_000, seed=2) -> dict:
    rng = np.random.default_rng(seed)
    u = rng.normal(size=(n, 6))
    u /= np.linalg.norm(u, axis=1, keepdims=True)
    r = rng.uniform(0.0, 3.0, size=(n, 1))
    W = u * r * AUTH
    T, sat = allocate(W)
    Ws = W[sat]

    res = {"n_wrenches": n, "n_saturated": int(sat.sum()),
           "sampling": "direction uniform on the unit 6-sphere in authority-scaled "
                       "space, magnitude uniform in [0, 3] x authority, seed 2",
           "saturated": {}, "unsaturated_direction_deg": {}}
    ang, loss = {}, {}
    for v, Tv in T.items():
        ang[v] = angle_deg(Ws, Tv[sat])
        loss[v] = magnitude_loss(Ws, Tv[sat])
        res["saturated"][v] = {
            "direction_deg": stats(ang[v]),
            "magnitude_loss": stats(loss[v]),
            "primary_direction_deg": stats(angle_deg(Ws, Tv[sat], PRIMARY)),
            "primary_magnitude_loss": stats(magnitude_loss(Ws, Tv[sat], PRIMARY)),
        }
        res["unsaturated_direction_deg"][v] = stats(angle_deg(W[~sat], Tv[~sat]))

    res["flown_vs_clip"] = {
        "flown_better": float(np.mean(ang["flown"] < ang["clip"] - 0.01)),
        "flown_worse": float(np.mean(ang["flown"] > ang["clip"] + 0.01)),
        "group_angle_deg": {
            name: {"flown": stats(group_angle_deg(Ws, T["flown"][sat], thr, dof)),
                   "clip": stats(group_angle_deg(Ws, T["clip"][sat], thr, dof))}
            for name, thr, dof in (("vertical (T1-T4 on heave/roll/pitch)", [0, 1, 2, 3], [2, 3, 4]),
                                   ("horizontal (T5-T8 on surge/sway/yaw)", [4, 5, 6, 7], [0, 1, 5]))},
    }

    bins = np.linspace(0, 3, 13)
    idx = np.digitize(r[:, 0], bins) - 1
    centres = (bins[:-1] + bins[1:]) / 2
    curve = {"r": centres.tolist()}
    for v, Tv in T.items():
        e = angle_deg(W, Tv)
        curve[v] = [float(np.median(e[idx == b])) for b in range(len(centres))]
    res["median_direction_vs_magnitude"] = curve

    fig, ax = plt.subplots(1, 3, figsize=(15, 4.4))
    for v in T:
        for a, data in ((ax[0], ang[v]), (ax[1], 100 * loss[v])):
            x = np.sort(data)
            a.plot(x, np.linspace(0, 100, len(x)), color=COLORS[v], label=LABELS[v])
        ax[2].plot(centres, curve[v], "-o", ms=3, color=COLORS[v], label=LABELS[v])
    ax[0].set_xlabel("direction error (deg)")
    ax[0].set_ylabel(f"% of {int(sat.sum())} saturated commands")
    ax[0].set_title("Direction error, saturated (CDF)")
    ax[0].set_xlim(left=-1)
    ax[1].set_xlabel("magnitude lost along the request (%)")
    ax[1].set_title("Magnitude loss, saturated (CDF)")
    ax[2].axvline(1.0, color="grey", lw=0.8)
    ax[2].set_xlabel("command magnitude (x single-axis authority)")
    ax[2].set_ylabel("median direction error (deg)")
    ax[2].set_title("Direction error vs command magnitude")
    for a in ax:
        a.grid(alpha=0.3)
    ax[0].legend(fontsize=8, loc="lower right")
    fig.tight_layout()
    fig.savefig(OUT / "direction_error.png", dpi=130)
    plt.close(fig)
    print("wrote results/direction_error.png")
    return res


def saturation(n=20_000, seed=3) -> dict:
    """Flown allocator: per-DOF saturation rate with every DOF commanded."""
    rng = np.random.default_rng(seed)
    levels = [round(x, 2) for x in np.linspace(0.1, 1.0, 10)]
    per_dof = {d: [] for d in DOF}
    groups = {"vert": [], "trans": [], "yaw": [], "final_clip": [], "v2_scaled": []}
    for a in levels:
        W = rng.uniform(-1, 1, size=(n, 6)) * AUTH * a
        T, flags = ref.wrench_to_thrust(W, dtype=np.float64)
        want = W @ P_FW.T @ A.T          # what the flown allocator gives unsaturated
        lost = np.abs(T @ A.T - want) > 0.01 * AUTH
        for k, d in enumerate(DOF):
            per_dof[d].append(float(lost[:, k].mean()))
        groups["vert"].append(float(np.mean(flags & ref.SAT_VERT > 0)))
        groups["trans"].append(float(np.mean(flags & ref.SAT_TRANS > 0)))
        groups["yaw"].append(float(np.mean(flags & ref.SAT_YAW > 0)))
        groups["final_clip"].append(float(np.mean(np.any(np.abs(_pre_clip(W)) > 1.0, axis=1))))
        _, f2 = ref.v2_wrench_to_thrust(W, dtype=np.float64)
        groups["v2_scaled"].append(float(np.mean(f2 != 0)))

    fig, ax = plt.subplots(figsize=(6.5, 4.2))
    for d in DOF:
        ax.plot(levels, np.array(per_dof[d]) * 100, "-o", ms=3, label=d)
    ax.set_xlabel("per-DOF command range (+- fraction of single-axis authority)")
    ax.set_ylabel("commands where the DOF is cut by > 1% (%)")
    ax.set_title("Flown allocator: saturation rate per DOF")
    ax.grid(alpha=0.3)
    ax.legend(ncol=2)
    fig.tight_layout()
    fig.savefig(OUT / "saturation_rate.png", dpi=130)
    plt.close(fig)
    print("wrote results/saturation_rate.png")

    return {
        "n_per_level": n,
        "sampling": "each DOF independently uniform in +-level x authority, seed 3",
        "criterion": "achieved DOF differs from the unsaturated flown output "
                     "by more than 1% of that DOF's authority",
        "levels": levels,
        "per_dof_rate": per_dof,
        "group_flag_rate": groups,
    }


def _pre_clip(W):
    """T5-T8 after group renormalisation, before the final per-thruster clip."""
    trans = W[:, :2] @ P_FW[4:, :2].T
    yaw = W[:, 5:6] @ P_FW[4:, 5:6].T
    for g in (trans, yaw):
        m = np.max(np.abs(g), axis=1)
        s = m > 1
        g[s] /= m[s, None]
    return trans + yaw


def main() -> int:
    OUT.mkdir(exist_ok=True)
    dump("matrix.json", matrix())
    dump("reconstruction.json", reconstruction())
    dump("direction.json", direction())
    dump("saturation.json", saturation())
    return 0


if __name__ == "__main__":
    sys.exit(main())
