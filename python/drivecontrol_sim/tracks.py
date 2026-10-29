"""Reference tracks with curvature-limited speed profiles."""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, Optional

import numpy as np

DS = 0.5  # path resolution, m


@dataclass
class Track:
    name: str
    x: np.ndarray
    y: np.ndarray
    v: np.ndarray
    description: str
    grade: Optional[Callable[[float], float]] = None  # grade as a function of travelled distance

    @property
    def length(self) -> float:
        return float(np.sum(np.hypot(np.diff(self.x), np.diff(self.y))))


def _resample(x: np.ndarray, y: np.ndarray, ds: float = DS) -> tuple[np.ndarray, np.ndarray]:
    seg = np.hypot(np.diff(x), np.diff(y))
    s = np.concatenate([[0.0], np.cumsum(seg)])
    si = np.arange(0.0, s[-1], ds)
    return np.interp(si, s, x), np.interp(si, s, y)


def curvature(x: np.ndarray, y: np.ndarray) -> np.ndarray:
    dx, dy = np.gradient(x), np.gradient(y)
    ddx, ddy = np.gradient(dx), np.gradient(dy)
    return (dx * ddy - dy * ddx) / np.power(dx * dx + dy * dy, 1.5)


def speed_profile(x, y, v_max: float, a_lat: float = 4.0, a_acc: float = 2.0, a_dec: float = 3.0,
                  v_start: float = 0.0, v_end: float = 0.0) -> np.ndarray:
    k = np.abs(curvature(x, y))
    # smooth curvature a little so the profile does not chatter on discretisation noise
    k = np.convolve(k, np.ones(9) / 9, mode="same")
    v = np.minimum(v_max, np.sqrt(a_lat / np.maximum(k, 1e-6)))
    ds = np.hypot(np.diff(x), np.diff(y))
    v[0] = max(min(v[0], v_start), 0.5)
    for i in range(1, len(v)):  # forward pass: acceleration limit
        v[i] = min(v[i], math.sqrt(v[i - 1] ** 2 + 2 * a_acc * ds[i - 1]))
    v[-1] = min(v[-1], v_end)
    for i in range(len(v) - 2, -1, -1):  # backward pass: deceleration limit
        v[i] = min(v[i], math.sqrt(v[i + 1] ** 2 + 2 * a_dec * ds[i]))
    return v


def straight_speed_steps() -> Track:
    x = np.arange(0.0, 1600.0, DS)
    y = np.zeros_like(x)
    target = np.where(x < 500, 15.0, np.where(x < 1000, 28.0, 12.0))
    v = target.copy()
    v[0] = 0.5
    ds = DS
    for i in range(1, len(v)):
        v[i] = min(v[i], math.sqrt(v[i - 1] ** 2 + 2 * 2.0 * ds))
    v[-1] = 0.0
    for i in range(len(v) - 2, -1, -1):
        v[i] = min(v[i], math.sqrt(v[i + 1] ** 2 + 2 * 3.0 * ds))
    v[0] = 0.5
    return Track("straight_speed_steps", x, y, v, "1.6 km straight, 15 -> 28 -> 12 m/s, accel 2 / decel 3 m/s^2")


def circle(radius: float = 40.0, laps: float = 1.25, v_max: float = 12.0) -> Track:
    th = np.arange(0.0, 2 * math.pi * laps, DS / radius)
    x = radius * np.sin(th)
    y = radius * (1 - np.cos(th))
    v = speed_profile(x, y, v_max, a_lat=4.5)
    return Track(f"circle_r{int(radius)}", x, y, v, f"circle R = {radius:.0f} m, {laps} laps, up to {v_max} m/s")


def s_curve(amplitude: float = 8.0, wavelength: float = 120.0, length: float = 720.0, v_max: float = 22.0) -> Track:
    xs = np.linspace(0.0, length, int(length * 4))
    ys = amplitude * np.sin(2 * math.pi * xs / wavelength)
    x, y = _resample(xs, ys)
    v = speed_profile(x, y, v_max)
    return Track("s_curve", x, y, v, f"sinusoid A = {amplitude} m, lambda = {wavelength} m, up to {v_max} m/s")


def double_lane_change(v_const: float = 16.0) -> Track:
    # ISO 3888-1 inspired: 3.5 m offset, 25 m transitions, 25 m hold
    xs = np.linspace(0.0, 400.0, 4000)

    def blend(x0, x1, xv):
        t = np.clip((xv - x0) / (x1 - x0), 0, 1)
        return 0.5 - 0.5 * np.cos(math.pi * t)

    ys = 3.5 * (blend(100, 125, xs) - blend(150, 175, xs))
    x, y = _resample(xs, ys)
    v = np.minimum(speed_profile(x, y, v_const, a_lat=8.0), v_const)
    return Track("double_lane_change", x, y, v, f"3.5 m double lane change at {v_const} m/s (~{v_const * 3.6:.0f} km/h)")


def hairpin(radius: float = 15.0, v_max: float = 18.0) -> Track:
    pts_x, pts_y = [], []
    for s in np.arange(0.0, 150.0, DS):
        pts_x.append(s)
        pts_y.append(0.0)
    for th in np.arange(0.0, math.pi, DS / radius):
        pts_x.append(150.0 + radius * math.sin(th))
        pts_y.append(radius * (1 - math.cos(th)))
    for s in np.arange(0.0, 150.0, DS):
        pts_x.append(150.0 - s)
        pts_y.append(2 * radius)
    x, y = np.array(pts_x), np.array(pts_y)
    v = speed_profile(x, y, v_max, a_lat=4.0)
    return Track("hairpin", x, y, v, f"150 m straight, 180 deg turn R = {radius} m, 150 m back, up to {v_max} m/s")


def figure_eight(radius: float = 30.0, v_max: float = 11.0) -> Track:
    th = np.arange(0.0, 2 * math.pi, DS / radius)
    x1, y1 = radius * np.sin(th), radius * (1 - np.cos(th))
    x2, y2 = radius * np.sin(th), -radius * (1 - np.cos(th))
    x = np.concatenate([x1, x2[1:]])
    y = np.concatenate([y1, y2[1:]])
    v = speed_profile(x, y, v_max, a_lat=4.5)
    return Track("figure_eight", x, y, v, f"two R = {radius} m loops, up to {v_max} m/s")


def all_tracks() -> list[Track]:
    return [straight_speed_steps(), circle(), s_curve(), double_lane_change(), hairpin(), figure_eight()]


def windup_track(v_target: float = 30.0, grade: float = 0.10) -> Track:
    """Straight road with a step speed request and an uphill section the
    controller does not know about: the pedal saturates for several seconds."""
    x = np.arange(0.0, 2500.0, DS)
    y = np.zeros_like(x)
    v = np.full_like(x, v_target)
    v[:2] = 0.5

    def g(s: float) -> float:
        return grade if 50.0 <= s <= 450.0 else 0.0

    return Track("windup_step_grade", x, y, v, f"step to {v_target} m/s, {grade * 100:.0f} % grade from 50 to 450 m", g)
