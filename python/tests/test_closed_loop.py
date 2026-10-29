import numpy as np
import pytest

from drivecontrol_sim import sim, tracks
from drivecontrol_sim.experiments import stop_metrics, straight_const


@pytest.mark.parametrize("controller", ["stanley", "pure_pursuit"])
def test_lane_change_tracking(controller):
    log = sim.run(tracks.double_lane_change(), controller)
    m = sim.tracking_metrics(log, sim.LIMITS)
    assert m["completed"]
    assert m["lat_max"] < 1.0
    assert m["viol_steer_rate"] == 0 and m["viol_steer_range"] == 0
    assert m["viol_pedal_rate"] == 0 and m["viol_overlap"] == 0


def test_estop_bit_stops_within_bound():
    log = sim.run(straight_const(20.0), "stanley", can=sim.CanScript(estop_at=3.0), v0=20.0, t_max=30.0,
                  stop_on="standstill")
    m = stop_metrics(log, 3.0)
    assert m["stopped"]
    assert m["stop_distance_m"] < 35.0
    assert m["reaction_s"] <= 0.03


def test_heartbeat_loss_full_vs_naive():
    script = sim.CanScript(heartbeat_stop_at=3.0)
    full = stop_metrics(sim.run(straight_const(15.0), "stanley", can=script, v0=15.0, t_max=20.0,
                                stop_on="standstill"), 3.0)
    naive = stop_metrics(sim.run(straight_const(15.0), "naive", can=script, v0=15.0, t_max=12.0,
                                 stop_on="standstill"), 3.0)
    assert full["stopped"]
    assert full["reaction_s"] < 0.2  # 100 ms heartbeat timeout + one step
    assert not naive["stopped"]


def test_windup_overshoot_reduced():
    tr = tracks.windup_track()
    aw = sim.run(tr, "stanley", anti_windup="backcalc", t_max=40.0)
    no = sim.run(tr, "stanley", anti_windup="none", t_max=40.0)
    assert np.max(aw.arrays()["v"]) - 30.0 < 2.0
    assert np.max(no.arrays()["v"]) - 30.0 > 5.0


def test_simulation_deterministic():
    a = sim.run(tracks.hairpin(), "stanley").arrays()
    b = sim.run(tracks.hairpin(), "stanley").arrays()
    assert np.array_equal(a["lat_err"], b["lat_err"])
