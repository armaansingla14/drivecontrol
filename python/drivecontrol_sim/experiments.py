"""Experiments for the drivecontrol library (all results are simulated).

    python -m drivecontrol_sim.experiments [--out ../results] [--quick]

Writes CSV files and summary.md into the output directory.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import itertools
import math
import random
import subprocess
from pathlib import Path

import numpy as np

from . import bridge, reference, sim, tracks
from .vehicle import VehicleParams

REPO = Path(__file__).resolve().parents[2]


def write_csv(path: Path, rows: list[dict]) -> None:
    if not rows:
        return
    with path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        for r in rows:
            w.writerow({k: (f"{v:.6g}" if isinstance(v, float) else v) for k, v in r.items()})


# ----------------------------------------------------------------------------- baseline tuning

def tune_naive(trs, out: Path) -> tuple[float, float]:
    rows, best = [], None
    for ke, kpsi in itertools.product([0.05, 0.1, 0.2, 0.3], [0.2, 0.4, 0.7, 1.0]):
        lat = [sim.tracking_metrics(sim.run(tr, "naive", naive_k_e=ke, naive_k_psi=kpsi), sim.LIMITS)["lat_rms"]
               for tr in trs]
        rows.append({"k_e": ke, "k_psi": kpsi, "mean_lat_rms": float(np.mean(lat)), "max_lat_rms": float(np.max(lat))})
        if best is None or np.mean(lat) < best[0]:
            best = (float(np.mean(lat)), ke, kpsi)
    write_csv(out / "naive_tuning.csv", rows)
    return best[1], best[2]


# ----------------------------------------------------------------------------- tracking

CONTROLLERS = ["stanley", "pure_pursuit", "naive"]


def exp_tracking(trs, naive_gains, out: Path, md: list[str]) -> None:
    rows = []
    for tr in trs:
        for c in CONTROLLERS:
            for dyn in (True, False):
                kw = {"naive_k_e": naive_gains[0], "naive_k_psi": naive_gains[1]} if c == "naive" else {}
                m = sim.tracking_metrics(sim.run(tr, c, dynamic=dyn, **kw), sim.LIMITS)
                rows.append({"track": tr.name, "controller": c, "plant": "dynamic" if dyn else "kinematic", **m})
    write_csv(out / "tracking.csv", rows)
    md.append("## Path tracking (dynamic bicycle plant)\n")
    md.append("Lateral and heading errors are measured at the centre of gravity against the reference path, "
              "excluding the final 5 m stopping zone. Steer-rate violations count 10 ms steps where the commanded "
              "road-wheel angle changed faster than 0.6 rad/s.\n")
    md.append("| Track | Controller | Lateral RMS (m) | Lateral max (m) | Heading RMS (deg) | Speed RMS (m/s) | "
              "Peak a_y (m/s^2) | Steer-rate violations | Pedal-rate violations |")
    md.append("|---|---|---:|---:|---:|---:|---:|---:|---:|")
    for r in rows:
        if r["plant"] != "dynamic":
            continue
        md.append(f"| {r['track']} | {r['controller']} | {r['lat_rms']:.3f} | {r['lat_max']:.3f} | "
                  f"{r['head_rms_deg']:.2f} | {r['speed_rms']:.3f} | {r['ay_max']:.2f} | {r['viol_steer_rate']} | "
                  f"{r['viol_pedal_rate']} |")
    md.append("")
    # aggregate
    md.append("Mean over the six tracks (dynamic plant / kinematic plant):\n")
    md.append("| Controller | Lateral RMS dyn. | Lateral RMS kin. | Worst lateral max dyn. | Speed RMS dyn. | "
              "Total constraint violations |")
    md.append("|---|---:|---:|---:|---:|---:|")
    for c in CONTROLLERS:
        d = [r for r in rows if r["controller"] == c and r["plant"] == "dynamic"]
        k = [r for r in rows if r["controller"] == c and r["plant"] == "kinematic"]
        viol = sum(r["viol_steer_range"] + r["viol_steer_rate"] + r["viol_pedal_rate"] + r["viol_overlap"] for r in d)
        md.append(f"| {c} | {np.mean([r['lat_rms'] for r in d]):.3f} | {np.mean([r['lat_rms'] for r in k]):.3f} | "
                  f"{max(r['lat_max'] for r in d):.3f} | {np.mean([r['speed_rms'] for r in d]):.3f} | {viol} |")
    md.append("")


# ----------------------------------------------------------------------------- anti-windup

def exp_windup(out: Path, md: list[str], naive_gains) -> None:
    tr = tracks.windup_track()
    rows = []
    cases = [("stanley", "backcalc"), ("stanley", "clamp"), ("stanley", "none"), ("naive", "none")]
    for c, aw in cases:
        kw = {"naive_k_e": naive_gains[0], "naive_k_psi": naive_gains[1]} if c == "naive" else {}
        log = sim.run(tr, c, anti_windup=aw, t_max=70.0, **kw)
        a = log.arrays()
        v, t = a["v"], a["t"]
        target = 30.0
        reach = np.argmax(v >= target - 0.5) if np.any(v >= target - 0.5) else len(v) - 1
        outside = np.where(np.abs(v[reach:] - target) > 0.5)[0]
        settle = float(t[reach + outside[-1]] - t[reach]) if outside.size else 0.0
        iae = float(np.sum(np.abs(v[reach:] - target)) * sim.DT)
        rows.append({"controller": c, "anti_windup": aw, "overshoot_mps": float(np.max(v) - target),
                     "time_to_29_5_s": float(t[reach]), "settle_after_reach_s": settle, "iae_after_reach": iae,
                     "max_brake_after_reach": float(np.max(a["brake"][reach:]))})
    write_csv(out / "windup.csv", rows)
    md.append("## Anti-windup: step to 30 m/s with an unmodelled 10 % grade (50-450 m)\n")
    md.append("| Controller | Anti-windup | Overshoot (m/s) | Time to 29.5 m/s (s) | Settling after reach (s) | "
              "IAE after reach (m) | Max brake after reach |")
    md.append("|---|---|---:|---:|---:|---:|---:|")
    for r in rows:
        md.append(f"| {r['controller']} | {r['anti_windup']} | {r['overshoot_mps']:.2f} | {r['time_to_29_5_s']:.2f} | "
                  f"{r['settle_after_reach_s']:.2f} | {r['iae_after_reach']:.2f} | {r['max_brake_after_reach']:.2f} |")
    md.append("")


# ----------------------------------------------------------------------------- e-stop

def straight_const(v0: float, length: float = 2500.0) -> tracks.Track:
    x = np.arange(0.0, length, tracks.DS)
    return tracks.Track(f"straight_{v0:.0f}", x, np.zeros_like(x), np.full_like(x, v0), "straight")


def circle_const(v0: float, r: float = 80.0) -> tracks.Track:
    th = np.arange(0.0, 2 * math.pi, tracks.DS / r)
    x, y = r * np.sin(th), r * (1 - np.cos(th))
    return tracks.Track(f"circle_r{int(r)}_{v0:.0f}", x, y, np.full_like(x, v0), "circle")


def stop_metrics(log: sim.RunLog, t_trig: float) -> dict:
    a = log.arrays()
    t, v = a["t"], a["v"]
    i0 = int(np.searchsorted(t, t_trig))
    brk = np.where(a["brake"][i0:] > 0.05)[0]
    react = float(t[i0 + brk[0]] - t_trig) if brk.size else float("nan")
    still = np.where(v[i0:] < 0.05)[0]
    stopped = bool(still.size)
    i1 = i0 + (still[0] if stopped else len(v) - i0 - 1)
    dist = float(np.sum(v[i0:i1]) * sim.DT)
    ax = a["ax"][i0:i1]
    jerk = np.abs(np.diff(ax)) / sim.DT if ax.size > 1 else np.array([0.0])
    return {"reaction_s": react, "stopped": stopped, "stop_time_s": float(t[i1] - t_trig) if stopped else float("nan"),
            "stop_distance_m": dist if stopped else float("nan"), "peak_decel": float(-np.min(ax)) if ax.size else 0.0,
            "peak_jerk": float(np.max(jerk)), "lat_dev_max_m": float(np.max(np.abs(a["lat_err"][i0:i1 + 1])))}


def exp_estop(out: Path, md: list[str], naive_gains) -> None:
    rows = []
    t_trig = 3.0
    scenarios = []
    for v0 in (10.0, 20.0, 30.0):
        scenarios.append(("straight", v0, straight_const(v0)))
    scenarios.append(("circle_r80", 17.0, circle_const(17.0)))
    triggers = {
        "estop_bit": sim.CanScript(estop_at=t_trig),
        "safety_heartbeat_lost": sim.CanScript(heartbeat_stop_at=t_trig),
        "planner_lost": sim.CanScript(command_stop_at=t_trig),
        "corrupted_frames": sim.CanScript(corrupt_from=t_trig),
    }
    p = VehicleParams()
    for (road, v0, tr), (trig, script), c in itertools.product(scenarios, triggers.items(), ["stanley", "naive"]):
        script = sim.CanScript(**{**script.__dict__, "target_speed": 60.0})
        kw = {"naive_k_e": naive_gains[0], "naive_k_psi": naive_gains[1]} if c == "naive" else {}
        log = sim.run(tr, c, can=script, v0=v0, t_max=t_trig + 40.0, stop_on="standstill", **kw)
        m = stop_metrics(log, t_trig)
        ideal = v0 * v0 / (2 * p.brake_force_max / p.mass)
        rows.append({"road": road, "v0": v0, "trigger": trig, "controller": c, "ideal_brake_distance_m": ideal, **m})
    write_csv(out / "estop.csv", rows)
    md.append("## Emergency stop and link-loss behaviour\n")
    md.append("Trigger at t = 3 s while cruising. `ideal` = v0^2 / (2 * 8 m/s^2), the distance with full brake force "
              "applied instantly (reference only, not achievable with actuator lag). Reaction = trigger to first "
              "commanded brake > 5 %. `-` = did not stop within 40 s.\n")
    md.append("| Road | v0 (m/s) | Trigger | Controller | Reaction (s) | Stop time (s) | Stop distance (m) | "
              "Ideal (m) | Peak decel (m/s^2) | Peak jerk (m/s^3) | Max lateral dev. (m) |")
    md.append("|---|---:|---|---|---:|---:|---:|---:|---:|---:|---:|")

    def f(x, fmt):
        return "-" if (isinstance(x, float) and math.isnan(x)) else format(x, fmt)

    for r in rows:
        md.append(f"| {r['road']} | {r['v0']:.0f} | {r['trigger']} | {r['controller']} | {f(r['reaction_s'], '.2f')} | "
                  f"{f(r['stop_time_s'], '.2f')} | {f(r['stop_distance_m'], '.1f')} | {r['ideal_brake_distance_m']:.1f} | "
                  f"{r['peak_decel']:.2f} | {r['peak_jerk']:.0f} | {r['lat_dev_max_m']:.2f} |")
    md.append("")


# ----------------------------------------------------------------------------- parity

def exp_parity(out: Path, md: list[str]) -> None:
    rng = random.Random(7)
    pid_err = 0.0
    for _ in range(200):
        g = [rng.uniform(0.1, 2), rng.uniform(0, 1), rng.uniform(0, 0.3), rng.uniform(0, 1.5), -rng.uniform(1, 5),
             rng.uniform(1, 5), rng.uniform(0.01, 0.2), rng.choice([0, 1, 2]), rng.uniform(0.5, 4)]
        n = 300
        ref = [math.sin(0.01 * i * rng.uniform(1, 5)) * 10 for i in range(n)]
        meas = [r * 0.7 + rng.gauss(0, 0.1) for r in ref]
        ff = [rng.uniform(-1, 1) for _ in range(n)]
        c = bridge.pid_run(g, ref, meas, ff, 0.01)
        py = reference.Pid(*g)
        p = [py.update(r, m_, f_, 0.01) for r, m_, f_ in zip(ref, meas, ff)]
        pid_err = max(pid_err, max(abs(a - b) for a, b in zip(c, p)))
    lat_err = {"pure_pursuit": 0.0, "stanley": 0.0}
    for _ in range(300):
        tr = rng.choice([tracks.s_curve(), tracks.hairpin(), tracks.double_lane_change()])
        i = rng.randrange(10, len(tr.x) - 200)
        state = (tr.x[i] + rng.gauss(0, 0.8), tr.y[i] + rng.gauss(0, 0.8),
                 math.atan2(tr.y[i + 1] - tr.y[i], tr.x[i + 1] - tr.x[i]) + rng.gauss(0, 0.1),
                 rng.uniform(0, 25), 0.0, rng.gauss(0, 0.1))
        rp = reference.RefPath(tr.x, tr.y, tr.v)
        for mode, cfg in (("pure_pursuit", [3.0, 0.5, 3.0, 25.0]), ("stanley", [1.5, 1.0, 0.05, 1.0])):
            c = bridge.lateral_once(mode, cfg, tr.x, tr.y, tr.v, state)
            p = reference.pure_pursuit(rp, state, cfg) if mode == "pure_pursuit" else reference.stanley(rp, state, cfg)
            lat_err[mode] = max(lat_err[mode], abs(c - p))
    rows = [{"block": "pid", "cases": 200, "max_abs_diff": pid_err},
            {"block": "pure_pursuit", "cases": 300, "max_abs_diff": lat_err["pure_pursuit"]},
            {"block": "stanley", "cases": 300, "max_abs_diff": lat_err["stanley"]}]
    write_csv(out / "parity.csv", rows)
    md.append("## C++ / Python parity\n")
    md.append("Independent Python re-implementations (`reference.py`) vs the C++ library through ctypes on random inputs.\n")
    md.append("| Block | Random cases | Max abs difference |")
    md.append("|---|---:|---:|")
    for r in rows:
        md.append(f"| {r['block']} | {r['cases']} | {r['max_abs_diff']:.2e} |")
    md.append("")


# ----------------------------------------------------------------------------- HIL replay

def make_log(path: Path, duration: float, estop_at=None, hb_stop_at=None, corrupt=None) -> None:
    lines = []
    ce = cc = 0
    for k in range(int(duration / 0.01) + 1):
        t = k * 0.01
        if k % 2 == 0 and (hb_stop_at is None or t < hb_stop_at):
            d = bytearray(bridge.encode_estop(estop_at is not None and t >= estop_at, ce))
            ce += 1
            if corrupt and corrupt[0] <= t < corrupt[1]:
                d[7] ^= 0xFF
            lines.append(f"({t:.6f}) can0 080#{d.hex().upper()}")
        if k % 5 == 0:
            d = bytearray(bridge.encode_command(20.0, True, False, cc))
            cc += 1
            lines.append(f"({t:.6f}) can0 100#{d.hex().upper()}")
    path.write_text("\n".join(lines) + "\n")


def exp_hil(out: Path, md: list[str]) -> None:
    exe = REPO / "build" / "dc_replay"
    if not exe.exists():
        md.append("## HIL replay\n\nSkipped: build/dc_replay not found.\n")
        return
    logdir = out / "hil"
    logdir.mkdir(exist_ok=True)
    work = REPO / "build" / "hil_out"
    work.mkdir(exist_ok=True)
    cases = {
        "nominal": dict(),
        "estop_at_6s": dict(estop_at=6.0),
        "heartbeat_lost_at_6s": dict(hb_stop_at=6.0),
        "crc_burst_6_to_6.2s": dict(corrupt=(6.0, 6.2)),
    }
    rows = []
    for name, kw in cases.items():
        log_in = logdir / f"{name}.log"
        make_log(log_in, 20.0, **kw)
        for naive in (False, True):
            digests, summary = [], ""
            for rep in range(2):
                out_log = work / f"{name}_{'naive' if naive else 'full'}_{rep}.log"
                cmd = [str(exe), "--log", str(log_in), "--out", str(out_log), "--v0", "15", "--t-end", "20"]
                if naive:
                    cmd.append("--naive")
                summary = subprocess.run(cmd, check=True, capture_output=True, text=True).stdout.strip()
                digests.append(hashlib.sha256(out_log.read_bytes()).hexdigest())
            kv = dict(item.split("=") for item in summary.split())
            rows.append({"log": name, "controller": "naive" if naive else "full", "deterministic": digests[0] == digests[1],
                         "stop_state_at": float(kv["stop_state_at"]), "first_brake_at": float(kv["first_brake_at"]),
                         "stopped_at": float(kv["stopped_at"]), "stop_distance": float(kv["stop_distance"]),
                         "final_speed": float(kv["final_speed"])})
    write_csv(out / "hil_replay.csv", rows)
    md.append("## HIL-style CAN replay (`build/dc_replay`)\n")
    md.append("Recorded planner/safety traffic (candump format, `results/hil/*.log`) replayed into the controller with "
              "a kinematic plant in the loop, run twice per case. Start speed 15 m/s, planner target 20 m/s, 20 s. "
              "The naive baseline does not send DC_Status, so it has no stop state to report.\n")
    md.append("| Log | Controller | Bit-identical output | Stop state reported (s) | First brake frame (s) | "
              "Standstill (s) | Distance braking -> standstill (m) | Final speed (m/s) |")
    md.append("|---|---|---|---:|---:|---:|---:|---:|")

    def g(x, fmt):
        return "-" if x < 0 else format(x, fmt)

    for r in rows:
        md.append(f"| {r['log']} | {r['controller']} | {'yes' if r['deterministic'] else 'no'} | "
                  f"{g(r['stop_state_at'], '.2f')} | {g(r['first_brake_at'], '.2f')} | {g(r['stopped_at'], '.2f')} | "
                  f"{g(r['stop_distance'], '.1f')} | {r['final_speed']:.2f} |")
    md.append("")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(REPO / "results"))
    args = ap.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    md = ["# drivecontrol results (simulated)\n",
          "Generated by `python -m drivecontrol_sim.experiments`. Controller code: C++ library via ctypes; "
          "plant: Python bicycle model (`vehicle.py`), 100 Hz control, 1 kHz plant integration.\n"]
    trs = tracks.all_tracks()
    gains = tune_naive(trs, out)
    md.append(f"Naive baseline gains chosen by a 4x4 grid search on the same tracks (`naive_tuning.csv`): "
              f"k_e = {gains[0]}, k_psi = {gains[1]}.\n")
    exp_tracking(trs, gains, out, md)
    exp_windup(out, md, gains)
    exp_estop(out, md, gains)
    exp_parity(out, md)
    exp_hil(out, md)
    (out / "summary.md").write_text("\n".join(md) + "\n")
    print("\n".join(md))


if __name__ == "__main__":
    main()
