"""Vehicle plant models: kinematic and dynamic (linear-tyre) bicycle with
first-order actuator dynamics and physical actuator limits."""
from __future__ import annotations

import math
from dataclasses import dataclass, field


@dataclass
class VehicleParams:
    mass: float = 1500.0
    iz: float = 2250.0
    lf: float = 1.2
    lr: float = 1.5
    cf: float = 80000.0        # front axle cornering stiffness, N/rad
    cr: float = 80000.0
    mu: float = 1.0
    drive_force_max: float = 6000.0
    brake_force_max: float = 12000.0
    cd_a: float = 0.7
    rho: float = 1.2
    c_rr: float = 0.012
    # actuators (physical plant side)
    steer_tau: float = 0.08
    steer_max: float = 0.6
    steer_rate_max: float = 0.8
    drive_tau: float = 0.15
    brake_tau: float = 0.08

    @property
    def wheelbase(self) -> float:
        return self.lf + self.lr

    def resistance(self, v: float) -> float:
        drag = 0.5 * self.rho * self.cd_a * v * abs(v)
        roll = self.c_rr * self.mass * 9.81 if v > 0.05 else 0.0
        return drag + roll


@dataclass
class PlantState:
    X: float = 0.0     # CG position
    Y: float = 0.0
    psi: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    r: float = 0.0
    delta: float = 0.0   # actual road-wheel angle
    drive: float = 0.0   # actual normalised drive force
    brake: float = 0.0
    ax: float = 0.0      # last longitudinal acceleration
    s_travel: float = 0.0


@dataclass
class Bicycle:
    """Dynamic bicycle model (linear tyres with friction saturation) that
    falls back to kinematic behaviour below `v_kin` where the slip-angle
    model is ill-conditioned. Set `dynamic=False` for a purely kinematic plant."""

    p: VehicleParams = field(default_factory=VehicleParams)
    dynamic: bool = True
    v_kin: float = 3.0
    grade: float = 0.0  # road grade (rise/run), positive uphill; can be updated each step
    st: PlantState = field(default_factory=PlantState)

    def reset(self, x: float, y: float, psi: float, v: float) -> None:
        """Place the vehicle with its rear axle at (x, y)."""
        self.st = PlantState(X=x + self.p.lr * math.cos(psi), Y=y + self.p.lr * math.sin(psi), psi=psi, vx=v)

    # reference point used by the controllers: rear-axle centre
    def rear_axle(self) -> tuple[float, float]:
        s = self.st
        return s.X - self.p.lr * math.cos(s.psi), s.Y - self.p.lr * math.sin(s.psi)

    def step(self, steer_cmd: float, throttle: float, brake: float, dt: float, substeps: int = 10) -> None:
        p, s = self.p, self.st
        h = dt / substeps
        for _ in range(substeps):
            # actuators: rate limit + first-order lag + saturation
            target = max(-p.steer_max, min(p.steer_max, steer_cmd))
            d_delta = (target - s.delta) / p.steer_tau * h
            max_d = p.steer_rate_max * h
            s.delta += max(-max_d, min(max_d, d_delta))
            s.drive += (max(0.0, min(1.0, throttle)) - s.drive) * h / p.drive_tau
            s.brake += (max(0.0, min(1.0, brake)) - s.brake) * h / p.brake_tau

            fx = s.drive * p.drive_force_max - p.resistance(s.vx) - p.mass * 9.81 * math.sin(math.atan(self.grade))
            fb = s.brake * p.brake_force_max
            if s.vx > 0.01:
                fx -= fb
            elif abs(fx) <= fb:
                fx = 0.0  # brakes hold the car at standstill
                s.vx = 0.0
            else:
                fx -= math.copysign(fb, fx)

            L = p.wheelbase
            if not self.dynamic or s.vx < self.v_kin:
                ax = fx / p.mass
                s.vx = max(0.0, s.vx + ax * h)
                s.r = s.vx * math.tan(s.delta) / L
                s.vy = s.r * p.lr
            else:
                af = s.delta - math.atan2(s.vy + p.lf * s.r, s.vx)
                ar = -math.atan2(s.vy - p.lr * s.r, s.vx)
                fzf = p.mass * 9.81 * p.lr / L
                fzr = p.mass * 9.81 * p.lf / L
                fyf = max(-p.mu * fzf, min(p.mu * fzf, p.cf * af))
                fyr = max(-p.mu * fzr, min(p.mu * fzr, p.cr * ar))
                ax = (fx - fyf * math.sin(s.delta)) / p.mass + s.vy * s.r
                ay = (fyr + fyf * math.cos(s.delta)) / p.mass - s.vx * s.r
                rdot = (p.lf * fyf * math.cos(s.delta) - p.lr * fyr) / p.iz
                s.vx = max(0.0, s.vx + ax * h)
                s.vy += ay * h
                s.r += rdot * h
                ax = ax - s.vy * s.r
            s.ax = ax
            c, sn = math.cos(s.psi), math.sin(s.psi)
            s.X += (s.vx * c - s.vy * sn) * h
            s.Y += (s.vx * sn + s.vy * c) * h
            s.psi += s.r * h
            s.s_travel += math.hypot(s.vx, s.vy) * h
