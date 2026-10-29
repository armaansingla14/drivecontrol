import math
import random

import pytest

from drivecontrol_sim import bridge, reference, tracks


@pytest.mark.parametrize("aw", [0, 1, 2])
def test_pid_parity(aw):
    rng = random.Random(aw)
    g = [0.8, 0.4, 0.05, 1.0, -2.0, 3.0, 0.05, aw, 2.0]
    n = 500
    ref = [5 * math.sin(0.02 * i) + (3 if i > 200 else 0) for i in range(n)]
    meas = [0.6 * r + rng.gauss(0, 0.05) for r in ref]
    ff = [rng.uniform(-0.5, 0.5) for _ in range(n)]
    c = bridge.pid_run(g, ref, meas, ff, 0.01)
    p = reference.Pid(*g)
    py = [p.update(r, m, f, 0.01) for r, m, f in zip(ref, meas, ff)]
    assert max(abs(a - b) for a, b in zip(c, py)) < 1e-12


@pytest.mark.parametrize("mode,cfg", [("pure_pursuit", [3.0, 0.5, 3.0, 25.0]), ("stanley", [1.5, 1.0, 0.05, 0.002])])
def test_lateral_parity(mode, cfg):
    rng = random.Random(1)
    tr = tracks.hairpin()
    rp = reference.RefPath(tr.x, tr.y, tr.v)
    for _ in range(40):
        i = rng.randrange(5, len(tr.x) - 100)
        yaw = math.atan2(tr.y[i + 1] - tr.y[i], tr.x[i + 1] - tr.x[i])
        st = (tr.x[i] + rng.gauss(0, 1), tr.y[i] + rng.gauss(0, 1), yaw + rng.gauss(0, 0.2), rng.uniform(0, 20), 0.0,
              rng.gauss(0, 0.1))
        c = bridge.lateral_once(mode, cfg, tr.x, tr.y, tr.v, st)
        p = reference.pure_pursuit(rp, st, cfg) if mode == "pure_pursuit" else reference.stanley(rp, st, cfg)
        assert c == pytest.approx(p, abs=1e-9)


def test_can_encode_decode_roundtrip():
    data = bridge.encode_command(12.5, True, False, 3)
    assert len(data) == 8
    ctrl = bridge.Controller("stanley", use_can=True)
    tr = tracks.straight_speed_steps()
    ctrl.set_path(tr.x, tr.y, tr.v)
    for k in range(10):
        ctrl.can_rx(bridge.ID_ESTOP, bridge.encode_estop(False, k), k * 0.02)
        if k % 2 == 0:
            ctrl.can_rx(bridge.ID_COMMAND, bridge.encode_command(10.0, True, False, k // 2), k * 0.02)
        ctrl.step(0, 0, 0, 0, 0, 0, k * 0.02, 0.01)
    frames = ctrl.can_tx()
    acts = [bridge.decode_actuator(d) for i, d in frames if i == bridge.ID_ACTUATOR]
    stats = [bridge.decode_status(d) for i, d in frames if i == bridge.ID_STATUS]
    assert acts and all(a is not None for a in acts)
    assert stats[-1][0] == 1  # active
    ctrl.close()


def test_corrupted_frame_rejected_by_decoder():
    data = bytearray(bridge.encode_command(1.0, True, False, 0))
    data[0] ^= 1
    assert bridge.decode_actuator(bytes(data)) is None


def test_set_path_rejects_single_point():
    ctrl = bridge.Controller()
    with pytest.raises(ValueError):
        ctrl.set_path([0.0], [0.0], [1.0])
    ctrl.close()
