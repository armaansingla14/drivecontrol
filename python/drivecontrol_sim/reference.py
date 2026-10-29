"""Pure-Python re-implementation of the C++ building blocks (PID, path
projection, pure pursuit, Stanley). Used only for parity tests against the
C++ library; the simulator itself drives the C++ code through the bridge."""
from __future__ import annotations

import math


def wrap(a: float) -> float:
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class Pid:
    NONE, CLAMP, BACKCALC = 0, 1, 2

    def __init__(self, kp, ki, kd, kff, out_min, out_max, d_tau, aw, kt, d_on_measurement=True):
        self.kp, self.ki, self.kd, self.kff = kp, ki, kd, kff
        self.lo, self.hi, self.tau, self.aw, self.kt = out_min, out_max, d_tau, int(aw), kt
        self.dom = d_on_measurement
        self.i = 0.0
        self.d = 0.0
        self.first = True
        self.pm = 0.0
        self.pe = 0.0

    def update(self, ref, meas, ff, dt):
        e = ref - meas
        if self.first:
            self.pm, self.pe, self.first = meas, e, False
        d_raw = -(meas - self.pm) / dt if self.dom else (e - self.pe) / dt
        self.d += dt / (self.tau + dt) * (d_raw - self.d)
        self.pm, self.pe = meas, e
        u_unsat = self.kp * e + self.i + self.kd * self.d + self.kff * ff
        u = min(max(u_unsat, self.lo), self.hi)
        if self.aw == self.NONE:
            self.i += self.ki * e * dt
        elif self.aw == self.CLAMP:
            sat = u != u_unsat
            helps = (u_unsat > u and e < 0) or (u_unsat < u and e > 0)
            if not sat or helps:
                self.i += self.ki * e * dt
        else:
            self.i += (self.ki * e + self.kt * (u - u_unsat)) * dt
        return u


class RefPath:
    def __init__(self, x, y, v):
        self.x, self.y, self.v = list(x), list(y), list(v)
        n = len(self.x)
        self.s = [0.0] * n
        for i in range(1, n):
            self.s[i] = self.s[i - 1] + math.hypot(self.x[i] - self.x[i - 1], self.y[i] - self.y[i - 1])
        self.yaw = []
        for i in range(n):
            a, b = max(i - 1, 0), min(i + 1, n - 1)
            self.yaw.append(math.atan2(self.y[b] - self.y[a], self.x[b] - self.x[a]))
        self.kappa = []
        for i in range(n):
            a, b = max(i - 1, 0), min(i + 1, n - 1)
            ds = self.s[b] - self.s[a]
            self.kappa.append(wrap(self.yaw[b] - self.yaw[a]) / ds if ds > 1e-9 else 0.0)

    @property
    def length(self):
        return self.s[-1]

    def project(self, px, py):
        cands = []
        for i in range(len(self.x) - 1):
            dx, dy = self.x[i + 1] - self.x[i], self.y[i + 1] - self.y[i]
            l2 = dx * dx + dy * dy
            t = ((px - self.x[i]) * dx + (py - self.y[i]) * dy) / l2 if l2 > 0 else 0.0
            t = min(max(t, 0.0), 1.0)
            qx, qy = self.x[i] + t * dx, self.y[i] + t * dy
            cands.append(((px - qx) ** 2 + (py - qy) ** 2, i, t, qx, qy))
        lim = (math.sqrt(min(c[0] for c in cands)) + 0.5) ** 2
        d = [c[0] for c in cands]

        def local_min(k):
            return (k == 0 or d[k] <= d[k - 1]) and (k + 1 == len(d) or d[k] <= d[k + 1])

        # earliest local minimum within 0.5 m of the best, as in the C++ global search
        best = next(cands[k] for k in range(len(d)) if local_min(k) and d[k] <= lim)
        d2, i, t, qx, qy = best
        s = self.s[i] + t * (self.s[i + 1] - self.s[i])
        yaw = math.atan2(self.y[i + 1] - self.y[i], self.x[i + 1] - self.x[i])
        kappa = self.kappa[i] + t * (self.kappa[i + 1] - self.kappa[i])
        cross = math.cos(yaw) * (py - qy) - math.sin(yaw) * (px - qx)
        e = (1.0 if cross >= 0 else -1.0) * math.sqrt(d2)
        return s, yaw, kappa, e

    def at(self, s):
        if s <= 0:
            return self.x[0], self.y[0], self.yaw[0]
        if s >= self.s[-1]:
            return self.x[-1], self.y[-1], self.yaw[-1]
        lo, hi = 0, len(self.s) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if self.s[mid] <= s:
                lo = mid
            else:
                hi = mid
        t = (s - self.s[lo]) / (self.s[hi] - self.s[lo])
        return (self.x[lo] + t * (self.x[hi] - self.x[lo]), self.y[lo] + t * (self.y[hi] - self.y[lo]),
                math.atan2(self.y[hi] - self.y[lo], self.x[hi] - self.x[lo]))


def pure_pursuit(path: RefPath, state, cfg4, wheelbase=2.7):
    x, y, yaw, v = state[:4]
    ld0, kv, ldmin, ldmax = cfg4
    s, _, _, _ = path.project(x, y)
    ld = min(max(ld0 + kv * v, ldmin), ldmax)
    tx, ty, tyaw = path.at(s + ld)
    rem = path.length - s
    if rem < ld:
        tx += (ld - rem) * math.cos(tyaw)
        ty += (ld - rem) * math.sin(tyaw)
    alpha = wrap(math.atan2(ty - y, tx - x) - yaw)
    dist = math.hypot(tx - x, ty - y)
    return math.atan2(2 * wheelbase * math.sin(alpha), max(dist, 1e-3))


def stanley(path: RefPath, state, cfg4, wheelbase=2.7):
    x, y, yaw, v, _steer, yaw_rate = state[:6]
    k, k_soft, k_yr, ff = cfg4
    fx, fy = x + wheelbase * math.cos(yaw), y + wheelbase * math.sin(yaw)
    _s, pyaw, kappa, e = path.project(fx, fy)
    psi_e = wrap(pyaw - yaw)
    delta = psi_e + math.atan2(-k * e, k_soft + abs(v))
    if ff > 0.5:
        delta += math.atan(wheelbase * kappa)
    delta -= k_yr * (yaw_rate - v * kappa)
    return delta
