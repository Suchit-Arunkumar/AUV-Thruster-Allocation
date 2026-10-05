"""Every number in the README comes from this script (plus compare_c.py,
the test suite, and the on-target bench). Deterministic: fixed seeds.

    python bench/analysis.py        -> results/*.json, results/*.png

Units: thrust normalised to full ESC authority (|T_i| <= 1); forces in the
same units, moments in those units x metres. Per-ESC thrust has not been
calibrated in a tank, so nothing here is in newtons.

"Authority" of a DOF = the largest single-axis command the firmware
allocation delivers without saturating: 1 / max_i |B+_ik| over the thrusters
that DOF drives. Direction errors are measured after scaling each DOF by its
authority, so one degree means the same thing on surge as on yaw.
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

import model  # noqa: E402
import reference as ref  # noqa: E402
from firmware_constants import FIRMWARE_B_PINV  # noqa: E402

OUT = ROOT / "results"
A = model.A
P_FW = model.block_pinv()                     # what computeAllocation() uses
AUTH = 1.0 / np.max(np.abs(P_FW), axis=0)     # per-DOF single-axis authority
DOF = model.DOF


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


def firmware(W):
    T, flags = ref.wrench_to_thrust(W, dtype=np.float64)
    return T, flags


def naive(W):
    """Same allocation matrix, each thruster clipped on its own."""
    return np.clip(W @ P_FW.T, -1.0, 1.0)


def angle_deg(W, T):
    """Angle between requested and achieved wrench, authority-scaled."""
    a = (T @ A.T) / AUTH
    w = W / AUTH
    c = np.sum(a * w, axis=1) / (np.linalg.norm(a, axis=1) * np.linalg.norm(w, axis=1))
    return np.degrees(np.arccos(np.clip(c, -1.0, 1.0)))


def group_angle_deg(W, T, thr, dof):
    """Angle inside one group: what its thrusters produce on that group's
    DOFs vs what they would produce unsaturated."""
    Ag = A[np.ix_(dof, thr)]
    want = (W @ P_FW.T)[:, thr] @ Ag.T / AUTH[dof]
    got = T[:, thr] @ Ag.T / AUTH[dof]
    c = np.sum(want * got, axis=1) / (np.linalg.norm(want, axis=1) * np.linalg.norm(got, axis=1))
    return np.degrees(np.arccos(np.clip(c, -1.0, 1.0)))


def stats(x):
    return {"mean": float(np.mean(x)), "p95": float(np.percentile(x, 95)),
            "max": float(np.max(x))}


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
    })
    return out


def reconstruction(n=100_000, seed=1) -> dict:
    """Unsaturated commands only: firmware pipeline vs requested wrench."""
    rng = np.random.default_rng(seed)
    W = rng.uniform(-1, 1, size=(n, 6)) * AUTH * 0.4
    T, flags = firmware(W)
    unsat = (flags == 0) & np.all(np.abs(W @ P_FW.T) <= 1.0, axis=1)
    W, T = W[unsat], T[unsat]
    err = np.abs(T @ A.T - W)
    full = np.abs(W @ np.array(FIRMWARE_B_PINV).T @ A.T - W)
    return {
        "n_unsaturated": int(unsat.sum()),
        "sampling": "each DOF uniform in +-0.4 x authority, seed 1",
        "firmware_max_abs_error": float(err.max()),
        "firmware_max_abs_error_per_axis": dict(zip(model.WRENCH, err.max(axis=0).tolist())),
        "full_pinv_max_abs_error": float(full.max()),
        "firmware_direction_error_deg": stats(angle_deg(W, T)),
    }


def direction(n=100_000, seed=2) -> dict:
    rng = np.random.default_rng(seed)
    u = rng.normal(size=(n, 6))
    u /= np.linalg.norm(u, axis=1, keepdims=True)
    r = rng.uniform(0.0, 3.0, size=(n, 1))
    W = u * r * AUTH
    Tf, flags = firmware(W)
    Tn = naive(W)
    sat = (flags != 0) | np.any(np.abs(W @ P_FW.T) > 1.0, axis=1)
    ef, en = angle_deg(W, Tf), angle_deg(W, Tn)

    bins = np.linspace(0, 3, 13)
    idx = np.digitize(r[:, 0], bins) - 1
    curve = {"r": [], "fw_median": [], "fw_p95": [], "naive_median": [], "naive_p95": []}
    for b in range(len(bins) - 1):
        m = idx == b
        curve["r"].append(float((bins[b] + bins[b + 1]) / 2))
        for name, e in (("fw", ef), ("naive", en)):
            curve[f"{name}_median"].append(float(np.median(e[m])))
            curve[f"{name}_p95"].append(float(np.percentile(e[m], 95)))

    fig, ax = plt.subplots(1, 2, figsize=(11, 4.2))
    hb = np.linspace(0, max(ef[sat].max(), en[sat].max()), 60)
    ax[0].hist(en[sat], bins=hb, alpha=0.6, label="per-thruster clip", color="#c0504d")
    ax[0].hist(ef[sat], bins=hb, alpha=0.6, label="firmware group renormalisation", color="#4f81bd")
    ax[0].set_xlabel("direction error (deg)")
    ax[0].set_ylabel(f"count (of {int(sat.sum())} saturated)")
    ax[0].set_title("Saturated commands")
    ax[0].legend()
    for name, c, lab in (("fw", "#4f81bd", "firmware"), ("naive", "#c0504d", "per-thruster clip")):
        ax[1].plot(curve["r"], curve[f"{name}_median"], "-o", color=c, label=f"{lab} median")
        ax[1].plot(curve["r"], curve[f"{name}_p95"], "--", color=c, label=f"{lab} p95")
    ax[1].axvline(1.0, color="grey", lw=0.8)
    ax[1].set_xlabel("command magnitude (x single-axis authority)")
    ax[1].set_ylabel("direction error (deg)")
    ax[1].set_title("Error vs command magnitude")
    ax[1].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(OUT / "direction_error.png", dpi=130)
    plt.close(fig)
    print("wrote results/direction_error.png")

    return {
        "n_wrenches": n,
        "n_saturated": int(sat.sum()),
        "sampling": "direction uniform on the unit 6-sphere in authority-scaled "
                    "space, magnitude uniform in [0, 3] x authority, seed 2",
        "firmware_saturated_deg": stats(ef[sat]),
        "naive_clip_saturated_deg": stats(en[sat]),
        "firmware_unsaturated_deg": stats(ef[~sat]),
        "saturated_firmware_better": float(np.mean(ef[sat] < en[sat] - 0.01)),
        "saturated_firmware_worse": float(np.mean(ef[sat] > en[sat] + 0.01)),
        "group_angle_saturated_deg": {
            name: {"firmware": stats(group_angle_deg(W[sat], Tf[sat], thr, dof)),
                   "naive_clip": stats(group_angle_deg(W[sat], Tn[sat], thr, dof))}
            for name, thr, dof in (("vertical (T1-T4 on heave/roll/pitch)", [0, 1, 2, 3], [2, 3, 4]),
                                   ("horizontal (T5-T8 on surge/sway/yaw)", [4, 5, 6, 7], [0, 1, 5]))},
        "curve": curve,
    }


def saturation(n=20_000, seed=3) -> dict:
    """Per-DOF saturation rate when every DOF is commanded at once."""
    rng = np.random.default_rng(seed)
    levels = [round(x, 2) for x in np.linspace(0.1, 1.0, 10)]
    per_dof = {d: [] for d in DOF}
    groups = {"vert": [], "trans": [], "yaw": [], "final_clip": []}
    for a in levels:
        W = rng.uniform(-1, 1, size=(n, 6)) * AUTH * a
        T, flags = firmware(W)
        want = W @ P_FW.T @ A.T          # what the firmware gives unsaturated
        got = T @ A.T
        lost = np.abs(got - want) > 0.01 * AUTH
        for k, d in enumerate(DOF):
            per_dof[d].append(float(lost[:, k].mean()))
        groups["vert"].append(float(np.mean(flags & ref.SAT_VERT > 0)))
        groups["trans"].append(float(np.mean(flags & ref.SAT_TRANS > 0)))
        groups["yaw"].append(float(np.mean(flags & ref.SAT_YAW > 0)))
        groups["final_clip"].append(float(np.mean(np.any(np.abs(
            _pre_clip(W)) > 1.0, axis=1))))

    fig, ax = plt.subplots(figsize=(6.5, 4.2))
    for d in DOF:
        ax.plot(levels, np.array(per_dof[d]) * 100, "-o", ms=3, label=d)
    ax.set_xlabel("per-DOF command range (+- fraction of single-axis authority)")
    ax.set_ylabel("commands where the DOF is cut by > 1% (%)")
    ax.set_title("Saturation rate per DOF, all six DOFs commanded")
    ax.legend(ncol=2)
    fig.tight_layout()
    fig.savefig(OUT / "saturation_rate.png", dpi=130)
    plt.close(fig)
    print("wrote results/saturation_rate.png")

    return {
        "n_per_level": n,
        "sampling": "each DOF independently uniform in +-level x authority, seed 3",
        "criterion": "achieved DOF differs from the unsaturated firmware output "
                     "by more than 1% of that DOF's authority",
        "levels": levels,
        "per_dof_rate": per_dof,
        "group_flag_rate": groups,
    }


def _pre_clip(W):
    """T5-T8 after group renormalisation, before the final per-thruster clip."""
    Pt = P_FW.copy()
    trans = W[:, :2] @ Pt[4:, :2].T
    yaw = W[:, 5:6] @ Pt[4:, 5:6].T
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
