"""Drive a CARLA vehicle along a route with the drivecontrol C++ controller.

Not run as part of this repository (no CARLA server available here); kept as
the integration path for testing against a higher-fidelity simulator.

    python carla/run_carla_tracking.py --host localhost --town Town04 --controller stanley --length 800

Requires the `carla` Python package matching the server version and a built
build/libdrivecontrol_c.so.
"""
from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))
from drivecontrol_sim import bridge, tracks  # noqa: E402

try:
    import carla  # type: ignore
except ImportError:  # pragma: no cover
    carla = None


def route_from_map(world, start_tf, length_m: float, step: float = 0.5):
    """Follow lane centre waypoints from the spawn point. CARLA is left-handed
    (y to the right); convert to a right-handed frame by flipping y."""
    wp = world.get_map().get_waypoint(start_tf.location)
    xs, ys = [], []
    travelled = 0.0
    while travelled < length_m:
        loc = wp.transform.location
        xs.append(loc.x)
        ys.append(-loc.y)
        nxt = wp.next(step)
        if not nxt:
            break
        wp = nxt[0]
        travelled += step
    return np.array(xs), np.array(ys)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--town", default="Town04")
    ap.add_argument("--controller", default="stanley", choices=["stanley", "pure_pursuit", "naive"])
    ap.add_argument("--length", type=float, default=800.0)
    ap.add_argument("--v-max", type=float, default=20.0)
    ap.add_argument("--out", default="carla_run.csv")
    args = ap.parse_args()
    if carla is None:
        raise SystemExit("the carla package is not installed")

    client = carla.Client(args.host, args.port)
    client.set_timeout(20.0)
    world = client.load_world(args.town)
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.01  # controller runs at 100 Hz
    world.apply_settings(settings)

    bp = world.get_blueprint_library().find("vehicle.tesla.model3")
    spawn = world.get_map().get_spawn_points()[0]
    vehicle = world.spawn_actor(bp, spawn)
    physics = vehicle.get_physics_control()
    wheels = physics.wheels
    front = (wheels[0].position + wheels[1].position) / 2 / 100.0  # cm -> m
    rear = (wheels[2].position + wheels[3].position) / 2 / 100.0
    wheelbase = front.distance(rear)
    max_steer = math.radians(wheels[0].max_steer_angle)

    x, y = route_from_map(world, spawn, args.length)
    v = tracks.speed_profile(x, y, args.v_max)
    ctrl = bridge.Controller(args.controller, steer_max=min(0.52, max_steer))
    ctrl.set_path(x, y, v)

    rows = []
    t = 0.0
    try:
        for _ in range(int(240 / 0.01)):
            world.tick()
            tf = vehicle.get_transform()
            vel = vehicle.get_velocity()
            ang = vehicle.get_angular_velocity()
            yaw = -math.radians(tf.rotation.yaw)
            # reference point: rear axle, half a wheelbase behind the actor origin
            rx = tf.location.x - 0.5 * wheelbase * math.cos(-yaw)
            ry = -(tf.location.y - 0.5 * wheelbase * math.sin(-yaw))
            speed = math.hypot(vel.x, vel.y)
            steer, thr, brk = ctrl.step(rx, ry, yaw, speed, 0.0, -math.radians(ang.z), t, 0.01)
            vehicle.apply_control(carla.VehicleControl(throttle=float(thr), brake=float(brk),
                                                       steer=float(-steer / max_steer)))
            dbg = ctrl.debug()
            rows.append({"t": t, "x": rx, "y": ry, "v": speed, "lat_err": dbg.lateral_error, "steer": steer,
                         "throttle": thr, "brake": brk, "state": dbg.state})
            t += 0.01
            if dbg.s > x.size * 0.5 - 2.0 and speed < 0.3:
                break
    finally:
        vehicle.destroy()
        settings.synchronous_mode = False
        world.apply_settings(settings)
        ctrl.close()

    with open(args.out, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    lat = np.array([r["lat_err"] for r in rows])
    print(f"lateral RMS {np.sqrt(np.mean(lat ** 2)):.3f} m, max {np.max(np.abs(lat)):.3f} m")


if __name__ == "__main__":
    main()
