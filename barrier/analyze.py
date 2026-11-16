#!/usr/bin/env python3
"""기록(cmd.csv + state.csv) → 벽 거리·속도·가속도 시간 그래프 (조건별 겹침) + 요약표.

    python -m barrier.analyze logs/20261023_*_hard_slow logs/20261023_*_hard_fast ... [--out barrier_compare.png]
    python -m barrier.analyze logs/              # 하위 run 전부

측정 손끝은 state.csv 의 tcp_meas (= FK(jnt_ang, 엔코더) · T_flange_tcp). 시뮬레이션 모드나 가짜 로봇에서
tcp_meas 가 안 움직이면 tcp_ref(지령값)로 대체하고 표에 표시한다 (--source 로 강제).
벽 축: cmd.csv 에서 바리어가 가장 오래 작동한 면. 거리 s = 그 면까지 거리(양수 = 안쪽), 접근 속도 v = -ds/dt,
가속도 a = dv/dt (음수 = 감속). 속도·가속도는 균일 시간축으로 재표본 후 Savitzky-Golay 미분.
t=0 은 바리어가 처음 작동한 시각 (--no-align 으로 끔).
"""
import argparse
import csv
import json
from pathlib import Path

import numpy as np
from scipy.signal import savgol_filter

from .box import FACES, NAMES, Box

# dataviz 기준 팔레트 (categorical, light surface), 고정 순서
PALETTE = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#6250d6", "#e34948"]
INK, INK2, GRID = "#0b0b0b", "#52514e", "#e6e5e1"


def read_csv(path):
    with open(path, newline="") as f:
        rows = list(csv.reader(f))
    head, body = rows[0], rows[1:]
    out = {}
    for j, name in enumerate(head):
        col = [r[j] for r in body if len(r) == len(head)]
        if name in ("active", "safety_active"):   # "000100" 같은 비트열 — float 로 바꾸면 안 됨
            out[name] = np.array(col)
            continue
        try:
            out[name] = np.array(col, float)
        except ValueError:
            out[name] = np.array(col)
    return out


def load_run(run_dir, source="auto", smooth_s=0.08):
    run_dir = Path(run_dir)
    meta = json.loads((run_dir / "meta.json").read_text())
    cmd, st = read_csv(run_dir / "cmd.csv"), read_csv(run_dir / "state.csv")
    if "tracking" in cmd:                                   # 추적 중인 행만 (대기 행은 NaN)
        keep = cmd["tracking"] > 0
        cmd = {k: v[keep] for k, v in cmd.items()}
    box = Box.from_dict(meta["box"])
    if len(cmd["t"]) < 5 or len(st["t"]) < 5:
        raise ValueError(f"{run_dir}: 추적 기록이 너무 짧음 (추적 시작 전에 끝났거나 핀치가 안 됨)")

    # 벽 축: 가장 오래 작동한 면 (none 모드면 안전 박스 대신 실험 박스에서 거리가 최소였던 면)
    act = np.array([[c == "1" for c in s] for s in cmd["active"]], bool) if len(cmd["active"]) else np.zeros((0, 6), bool)
    names = [n for n in NAMES if f"d_{n}" in cmd]
    counts = act.sum(0) if act.size else np.zeros(len(names))
    face = int(np.argmax(counts)) if counts.max() > 0 else int(np.nanargmin(np.nanmin(np.stack([cmd[f"d_{f}"] for f in names]), 1)))
    r_min = float(meta.get("r_min", 0.0))

    def dist(pos):
        if names[face] == "rmin":
            return np.hypot(pos[:, 0], pos[:, 1]) - r_min
        ax, lo = face // 2, face % 2 == 0
        return (pos[:, ax] - box.lo[ax]) if lo else (box.hi[ax] - pos[:, ax])

    meas = np.stack([st["tcp_meas_x"], st["tcp_meas_y"], st["tcp_meas_z"]], 1)
    ref = np.stack([st["tcp_ref_x_mm"], st["tcp_ref_y_mm"], st["tcp_ref_z_mm"]], 1) / 1000.0
    used = source
    if source == "auto":
        used = "meas" if np.isfinite(meas).all() and np.ptp(meas, axis=0).max() > 1e-4 else "ref"
    pos = meas if used == "meas" else ref
    s_state = dist(pos)
    s_cmd = cmd[f"d_{names[face]}"]

    # 균일 재표본 + SG 미분
    t = st["t"]
    keep = np.concatenate([[True], np.diff(t) > 0])
    t, s_state = t[keep], s_state[keep]
    dt = float(np.median(np.diff(t)))
    tu = np.arange(t[0], t[-1], dt)
    su = np.interp(tu, t, s_state)
    win = max(5, int(round(smooth_s / dt)) | 1)
    if win > len(su) - 1:
        win = (len(su) - 1) | 1
    s_f = savgol_filter(su, win, 3)
    v = -savgol_filter(su, win, 3, deriv=1, delta=dt)      # 접근 속도 (벽 쪽 양수)
    a = -savgol_filter(su, win, 3, deriv=2, delta=dt)      # 접근 가속도 (음수 = 감속)
    j = -savgol_filter(su, win, 3, deriv=3, delta=dt)

    t_c = float(cmd["t"][act[:, face]][0]) if counts[face] > 0 else None
    return dict(name=run_dir.name, meta=meta, face=names[face], source=used, dt=dt, t=tu, s=s_f, v=v, a=a, j=j,
                t_cmd=cmd["t"], s_cmd=s_cmd, t_contact=t_c, hand_speed=meta.get("fake_quest"))


def metrics(r):
    t, s, v, a, j = r["t"], r["s"], r["v"], r["a"], r["j"]
    tc = r["t_contact"]
    out = dict(run=r["name"], mode=r["meta"]["mode"], alpha=r["meta"]["alpha"] if r["meta"]["mode"] == "cbf" else "",
               face=r["face"], source=r["source"], contact_s=tc)
    if tc is None:
        return out
    pre = (t > tc - 0.3) & (t <= tc)
    post = (t >= tc - 0.1) & (t <= tc + 0.6)
    out["approach_mps"] = float(v[pre].mean()) if pre.any() else np.nan
    out["peak_decel_mps2"] = float(a[post].min()) if post.any() else np.nan
    out["peak_jerk_mps3"] = float(np.abs(j[post]).max()) if post.any() else np.nan
    out["overshoot_mm"] = float(-s[t >= tc - 0.1].min() * 1000) if (t >= tc - 0.1).any() else np.nan
    after = np.where((t >= tc) & (np.abs(v) < 0.005))[0]
    out["stop_after_s"] = float(t[after[0]] - tc) if len(after) else np.nan
    return out


def plot(runs, out, align=True):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    if align and not any(r["t_contact"] is not None for r in runs):
        print("바리어 접촉이 한 번도 없어서 t=0 정렬 없이 그립니다")
        align = False
    fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True, dpi=120)
    fig.patch.set_facecolor("#fcfcfb")
    labels = ["distance to wall  s  [cm]", "approach speed  v = −ds/dt  [m/s]", "approach accel  a = dv/dt  [m/s²]"]
    for r, col in zip(runs, PALETTE):
        off = r["t_contact"] if (align and r["t_contact"] is not None) else 0.0
        tag = r["name"].split("_", 2)[-1]
        m = r["meta"]
        lab = f"{tag}  ({m['mode']}" + (f" α={m['alpha']:g}" if m["mode"] == "cbf" else "") + ")"
        axes[0].plot(r["t"] - off, r["s"] * 100, color=col, lw=2, label=lab)
        axes[0].plot(r["t_cmd"] - off, r["s_cmd"] * 100, color=col, lw=1, ls="--", alpha=0.7)
        axes[1].plot(r["t"] - off, r["v"], color=col, lw=2)
        axes[2].plot(r["t"] - off, r["a"], color=col, lw=2)
    axes[0].axhline(0, color=INK2, lw=1)
    axes[0].text(0.01, 0.03, "solid = measured TCP, dashed = command p_cmd", transform=axes[0].transAxes, ha="left", va="bottom", fontsize=8, color=INK2)
    for ax, lab in zip(axes, labels):
        ax.set_ylabel(lab, color=INK, fontsize=9)
        ax.set_facecolor("#fcfcfb")
        ax.grid(True, color=GRID, lw=0.8)
        for sp in ("top", "right"):
            ax.spines[sp].set_visible(False)
        for sp in ("left", "bottom"):
            ax.spines[sp].set_color(GRID)
        ax.tick_params(colors=INK2, labelsize=8)
        if align:
            ax.axvline(0, color=INK2, lw=1, ls=":")
    axes[2].set_xlabel("time since first barrier contact [s]" if align else "time [s]", color=INK, fontsize=9)
    axes[0].legend(frameon=False, fontsize=8, loc="upper right")
    if align:
        tmin = min((r["t"] - r["t_contact"])[0] for r in runs if r["t_contact"] is not None)
        axes[0].set_xlim(max(tmin, -1.5), 1.5)
    fig.suptitle("Barrier stop behaviour — robot TCP", color=INK, fontsize=12)
    fig.tight_layout()
    fig.savefig(out, facecolor=fig.get_facecolor())
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs", nargs="+", help="run 디렉터리들 또는 logs/ 디렉터리")
    ap.add_argument("--out", default=None, help="PNG 경로 (기본: 첫 run 의 부모/barrier_compare.png)")
    ap.add_argument("--source", choices=["auto", "meas", "ref"], default="auto")
    ap.add_argument("--smooth", type=float, default=0.08, help="SG 창 길이 (s)")
    ap.add_argument("--no-align", action="store_true")
    a = ap.parse_args()
    dirs = []
    for p in a.runs:
        p = Path(p)
        dirs += sorted(d for d in p.iterdir() if (d / "meta.json").exists()) if p.is_dir() and not (p / "meta.json").exists() else [p]
    if not dirs:
        raise SystemExit("run 디렉터리가 없음 (meta.json 이 있는 폴더를 주세요)")
    runs = [load_run(d, a.source, a.smooth) for d in dirs]
    out = Path(a.out) if a.out else dirs[0].parent / "barrier_compare.png"
    plot(runs, out, align=not a.no_align)
    rows = [metrics(r) for r in runs]
    keys = ["run", "mode", "alpha", "face", "source", "approach_mps", "peak_decel_mps2", "peak_jerk_mps3", "overshoot_mm", "stop_after_s"]
    with open(out.with_suffix(".csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)
    fmt = lambda v: f"{v:.3f}" if isinstance(v, float) else str(v)
    print("| " + " | ".join(keys) + " |\n|" + "---|" * len(keys))
    for r in rows:
        print("| " + " | ".join(fmt(r.get(k, "")) for k in keys) + " |")
    print(f"\n그래프 {out}\n표 {out.with_suffix('.csv')}")


if __name__ == "__main__":
    main()
