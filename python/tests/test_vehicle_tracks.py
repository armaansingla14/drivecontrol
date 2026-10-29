import math

import numpy as np
import pytest

from drivecontrol_sim import tracks
from drivecontrol_sim.vehicle import Bicycle, VehicleParams


def test_kinematic_circle_radius():
    car = Bicycle(dynamic=False)
    car.reset(0, 0, 0, 5.0)
    delta = 0.1
    xs, ys = [], []
    for _ in range(3000):
        car.step(delta, 0.0, 0.0, 0.01)
        car.st.vx = 5.0  # hold speed
        xs.append(car.st.X)
        ys.append(car.st.Y)
    # fit circle centre: rear-axle radius L / tan(delta), CG radius slightly larger
    r_rear = car.p.wheelbase / math.tan(delta)
    r_cg = math.hypot(r_rear, car.p.lr)
    cx, cy = np.mean(xs), np.mean(ys)
    r = np.mean(np.hypot(np.array(xs) - cx, np.array(ys) - cy))
    assert r == pytest.approx(r_cg, rel=0.02)


def test_straight_line_acceleration_matches_force_balance():
    p = VehicleParams()
    car = Bicycle(p=p, dynamic=True)
    car.reset(0, 0, 0, 10.0)
    for _ in range(300):  # 3 s full throttle
        car.step(0.0, 1.0, 0.0, 0.01)
    # after the drive lag settles, acceleration ~ (F - resistance) / m
    expected = (p.drive_force_max - p.resistance(car.st.vx)) / p.mass
    assert car.st.ax == pytest.approx(expected, rel=0.02)
    assert abs(car.st.Y) < 1e-9


def test_braking_stops_without_reversing():
    car = Bicycle()
    car.reset(0, 0, 0, 20.0)
    for _ in range(1000):
        car.step(0.0, 0.0, 1.0, 0.01)
    assert car.st.vx == 0.0


def test_steering_actuator_rate_and_saturation():
    car = Bicycle()
    car.reset(0, 0, 0, 10.0)
    car.step(1.0, 0.0, 0.0, 0.01)
    assert car.st.delta <= car.p.steer_rate_max * 0.01 + 1e-12
    for _ in range(300):
        car.step(1.0, 0.0, 0.0, 0.01)
    assert car.st.delta == pytest.approx(car.p.steer_max, abs=1e-3)


def test_dynamic_understeer_gradient_sign():
    # equal cornering stiffness with lf < lr -> understeer: needs more steer than kinematic
    dyn = Bicycle(dynamic=True)
    kin = Bicycle(dynamic=False)
    for car in (dyn, kin):
        car.reset(0, 0, 0, 20.0)
        for _ in range(500):
            car.step(0.05, 0.0, 0.0, 0.01)
            car.st.vx = 20.0
    assert dyn.st.r < kin.st.r


def test_grade_slows_vehicle():
    flat, hill = Bicycle(), Bicycle(grade=0.1)
    for car in (flat, hill):
        car.reset(0, 0, 0, 15.0)
        for _ in range(200):
            car.step(0.0, 0.3, 0.0, 0.01)
    assert hill.st.vx < flat.st.vx


@pytest.mark.parametrize("tr", tracks.all_tracks(), ids=lambda t: t.name)
def test_track_profiles(tr):
    assert len(tr.x) == len(tr.y) == len(tr.v)
    ds = np.hypot(np.diff(tr.x), np.diff(tr.y))
    assert np.all(ds > 0.1) and np.all(ds < 1.0)
    assert tr.v[0] >= 0.5
    assert tr.v[-1] <= 0.5
    # acceleration limits respected by the profile
    acc = np.diff(tr.v ** 2) / (2 * ds)
    assert acc.max() < 2.0 + 1e-6 or tr.name == "straight_speed_steps"
    assert acc.min() > -3.0 - 1e-6


def test_lateral_acceleration_bounded_by_profile():
    tr = tracks.s_curve()
    k = np.abs(tracks.curvature(tr.x, tr.y))
    k = np.convolve(k, np.ones(9) / 9, mode="same")
    assert np.max(tr.v ** 2 * k) <= 4.0 + 1e-6
