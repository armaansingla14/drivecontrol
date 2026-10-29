"""ctypes bridge to libdrivecontrol_c.so (the C API in include/drivecontrol/capi.h)."""
from __future__ import annotations

import ctypes as C
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Optional

REPO = Path(__file__).resolve().parents[2]


def _find_library() -> str:
    env = os.environ.get("DRIVECONTROL_LIB")
    if env:
        return env
    for cand in (REPO / "build" / "libdrivecontrol_c.so",):
        if cand.exists():
            return str(cand)
    raise FileNotFoundError("libdrivecontrol_c.so not found; build the C++ project first (see README)")


class State(C.Structure):
    _fields_ = [("x", C.c_double), ("y", C.c_double), ("yaw", C.c_double), ("v", C.c_double),
                ("steer", C.c_double), ("yaw_rate", C.c_double), ("valid", C.c_int32)]


class Cmd(C.Structure):
    _fields_ = [("steer", C.c_double), ("throttle", C.c_double), ("brake", C.c_double)]


class Debug(C.Structure):
    _fields_ = [("state", C.c_int32), ("reasons", C.c_int32), ("lateral_error", C.c_double),
                ("heading_error", C.c_double), ("lookahead", C.c_double), ("s", C.c_double),
                ("v_ref", C.c_double), ("a_des", C.c_double), ("raw_steer", C.c_double),
                ("raw_throttle", C.c_double), ("raw_brake", C.c_double)]


class Config(C.Structure):
    _fields_ = [("controller", C.c_int32), ("lateral", C.c_int32), ("anti_windup", C.c_int32),
                ("use_can", C.c_int32), ("pp_ld0", C.c_double), ("pp_kv", C.c_double),
                ("pp_ld_min", C.c_double), ("pp_ld_max", C.c_double), ("st_k", C.c_double),
                ("st_k_soft", C.c_double), ("st_k_yaw_rate", C.c_double), ("st_curvature_ff", C.c_int32),
                ("v_kp", C.c_double), ("v_ki", C.c_double), ("v_kd", C.c_double), ("steer_max", C.c_double),
                ("steer_rate_max", C.c_double), ("estop_brake_rate", C.c_double),
                ("max_lateral_error", C.c_double), ("controlled_decel", C.c_double),
                ("naive_k_e", C.c_double), ("naive_k_psi", C.c_double), ("naive_kp_v", C.c_double),
                ("naive_ki_v", C.c_double)]


_lib: Optional[C.CDLL] = None


def lib() -> C.CDLL:
    global _lib
    if _lib is None:
        L = C.CDLL(_find_library())
        D = C.POINTER(C.c_double)
        L.dc_default_config.argtypes = [C.POINTER(Config)]
        L.dc_create.argtypes = [C.POINTER(Config)]
        L.dc_create.restype = C.c_void_p
        L.dc_destroy.argtypes = [C.c_void_p]
        L.dc_set_path.argtypes = [C.c_void_p, D, D, D, C.c_int]
        L.dc_set_path.restype = C.c_int
        L.dc_can_rx.argtypes = [C.c_void_p, C.c_uint32, C.POINTER(C.c_uint8), C.c_double]
        L.dc_step.argtypes = [C.c_void_p, C.POINTER(State), C.c_double, C.c_double, C.POINTER(Cmd)]
        L.dc_can_tx.argtypes = [C.c_void_p, C.POINTER(C.c_uint32), C.POINTER(C.c_uint8)]
        L.dc_can_tx.restype = C.c_int
        L.dc_get_debug.argtypes = [C.c_void_p, C.POINTER(Debug)]
        L.dc_encode_command.argtypes = [C.c_double, C.c_int, C.c_int, C.c_uint8, C.POINTER(C.c_uint8)]
        L.dc_encode_estop.argtypes = [C.c_int, C.c_uint8, C.POINTER(C.c_uint8)]
        L.dc_decode_actuator.argtypes = [C.POINTER(C.c_uint8), C.POINTER(Cmd)]
        L.dc_decode_actuator.restype = C.c_int
        L.dc_decode_status.argtypes = [C.POINTER(C.c_uint8), C.POINTER(C.c_int32), D, C.POINTER(C.c_int32)]
        L.dc_decode_status.restype = C.c_int
        L.dc_pid_run.argtypes = [D, D, D, D, C.c_int, C.c_double, D]
        L.dc_lateral_once.argtypes = [C.c_int, D, D, D, D, C.c_int, C.POINTER(State), C.c_double]
        L.dc_lateral_once.restype = C.c_double
        _lib = L
    return _lib


def _darr(vals: Iterable[float]):
    vals = list(vals)
    return (C.c_double * len(vals))(*vals)


DRIVE_STATES = {0: "disabled", 1: "active", 2: "controlled_stop", 3: "emergency_stop", 4: "stopped"}
ID_ESTOP, ID_COMMAND, ID_STATUS, ID_ACTUATOR = 0x080, 0x100, 0x300, 0x310


@dataclass
class StepDebug:
    state: int
    reasons: int
    lateral_error: float
    heading_error: float
    v_ref: float
    a_des: float
    raw_steer: float
    raw_throttle: float
    raw_brake: float


class Controller:
    """One controller instance in the C++ library."""

    def __init__(self, controller: str = "stanley", use_can: bool = False, anti_windup: str = "backcalc",
                 **overrides: float):
        L = lib()
        cfg = Config()
        L.dc_default_config(C.byref(cfg))
        cfg.controller = 1 if controller == "naive" else 0
        cfg.lateral = 0 if controller == "pure_pursuit" else 1
        cfg.anti_windup = {"none": 0, "clamp": 1, "backcalc": 2}[anti_windup]
        cfg.use_can = 1 if use_can else 0
        for k, v in overrides.items():
            setattr(cfg, k, v)
        self.config = cfg
        self._h = L.dc_create(C.byref(cfg))
        self._state = State()
        self._cmd = Cmd()
        self._dbg = Debug()

    def close(self) -> None:
        if self._h:
            lib().dc_destroy(self._h)
            self._h = None

    def __del__(self) -> None:  # pragma: no cover - interpreter shutdown order
        try:
            self.close()
        except Exception:
            pass

    def set_path(self, x, y, v) -> None:
        n = len(x)
        rc = lib().dc_set_path(self._h, _darr(x), _darr(y), _darr(v), n)
        if rc != 0:
            raise ValueError("path needs at least 2 points")

    def can_rx(self, can_id: int, data: bytes, t: float) -> None:
        buf = (C.c_uint8 * 8)(*data)
        lib().dc_can_rx(self._h, can_id, buf, t)

    def step(self, x, y, yaw, v, steer, yaw_rate, t, dt, valid=True):
        s = self._state
        s.x, s.y, s.yaw, s.v, s.steer, s.yaw_rate, s.valid = x, y, yaw, v, steer, yaw_rate, 1 if valid else 0
        lib().dc_step(self._h, C.byref(s), t, dt, C.byref(self._cmd))
        return self._cmd.steer, self._cmd.throttle, self._cmd.brake

    def debug(self) -> StepDebug:
        lib().dc_get_debug(self._h, C.byref(self._dbg))
        d = self._dbg
        return StepDebug(d.state, d.reasons, d.lateral_error, d.heading_error, d.v_ref, d.a_des, d.raw_steer,
                         d.raw_throttle, d.raw_brake)

    def can_tx(self):
        cid = C.c_uint32()
        buf = (C.c_uint8 * 8)()
        frames = []
        while lib().dc_can_tx(self._h, C.byref(cid), buf):
            frames.append((cid.value, bytes(buf)))
        return frames


def encode_command(speed: float, enable: bool, reset: bool, counter: int) -> bytes:
    buf = (C.c_uint8 * 8)()
    lib().dc_encode_command(speed, int(enable), int(reset), counter & 0x0F, buf)
    return bytes(buf)


def encode_estop(estop: bool, counter: int) -> bytes:
    buf = (C.c_uint8 * 8)()
    lib().dc_encode_estop(int(estop), counter & 0x0F, buf)
    return bytes(buf)


def decode_actuator(data: bytes):
    out = Cmd()
    ok = lib().dc_decode_actuator((C.c_uint8 * 8)(*data), C.byref(out))
    return (out.steer, out.throttle, out.brake) if ok else None


def decode_status(data: bytes):
    st, sp, fl = C.c_int32(), C.c_double(), C.c_int32()
    ok = lib().dc_decode_status((C.c_uint8 * 8)(*data), C.byref(st), C.byref(sp), C.byref(fl))
    return (st.value, sp.value, fl.value) if ok else None


def pid_run(gains, ref, meas, ff, dt):
    n = len(ref)
    out = (C.c_double * n)()
    lib().dc_pid_run(_darr(gains), _darr(ref), _darr(meas), _darr(ff), n, dt, out)
    return list(out)


def lateral_once(mode: str, cfg4, px, py, pv, state: tuple, wheelbase: float = 2.7) -> float:
    s = State(*state[:6], 1)
    return lib().dc_lateral_once(0 if mode == "pure_pursuit" else 1, _darr(cfg4), _darr(px), _darr(py), _darr(pv),
                                 len(px), C.byref(s), wheelbase)
