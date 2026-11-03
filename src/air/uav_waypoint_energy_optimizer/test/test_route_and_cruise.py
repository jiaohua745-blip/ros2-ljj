"""Unit tests for the optimizer and cruise controller (no ROS required)."""
import math
import random

import pytest

from uav_waypoint_energy_optimizer.energy_model import Settings as EnergySettings
from uav_waypoint_energy_optimizer.route_optimizer import (
    nearest_neighbor_order, optimize, two_opt_improve)
from uav_waypoint_energy_optimizer.waypoint_cruise import (
    Settings as CruiseSettings, WaypointCruiseController)


# ----------------------------------------------------------------------
# 排序算法
# ----------------------------------------------------------------------
def test_nearest_neighbor_starts_from_start_index():
    points = [(0.0, 0.0), (1.0, 0.1), (5.0, 0.0), (1.2, -0.1)]
    order = nearest_neighbor_order(points, start_index=0)
    assert order[0] == 0
    assert sorted(order) == list(range(len(points)))


def test_two_opt_removes_crossings():
    # 构造一个明显交叉的初始顺序：正方形对角访问。
    points = [(0.0, 0.0), (10.0, 10.0), (0.0, 10.0), (10.0, 0.0)]
    improved = two_opt_improve(points, [0, 1, 2, 3], EnergySettings())
    from uav_waypoint_energy_optimizer.energy_model import route_energy
    e_before, _, _ = route_energy(points, [0, 1, 2, 3], EnergySettings())
    e_after, _, _ = route_energy(points, improved, EnergySettings())
    assert e_after <= e_before + 1e-9
    assert sorted(improved) == list(range(len(points)))


def test_optimize_never_worse_than_naive_on_random_cases():
    rng = random.Random(42)
    for _ in range(20):
        points = [(rng.uniform(-30, 30), rng.uniform(-30, 30))
                  for _ in range(rng.randint(4, 9))]
        result = optimize(points, EnergySettings())
        assert sorted(result.order) == list(range(len(points)))
        assert result.energy_optimized <= result.energy_naive + 1e-6


def test_optimize_result_fields_consistent():
    points = [(0.0, 0.0), (12.0, -8.0), (-6.0, -14.0), (4.0, 6.0)]
    result = optimize(points, EnergySettings(), return_to_start=True)
    assert result.naive_order == [0, 1, 2, 3]
    assert result.energy_optimized > 0
    assert result.distance_optimized > 0
    assert 0.0 <= result.energy_saving_percent <= 100.0
    assert result.energy_saving_percent == pytest.approx(
        100.0 * (result.energy_naive - result.energy_optimized)
        / result.energy_naive)


def test_optimize_rejects_bad_input():
    with pytest.raises(ValueError):
        optimize([], EnergySettings())
    with pytest.raises(ValueError):
        optimize([(1.0, float('inf'))], EnergySettings())


# ----------------------------------------------------------------------
# 巡航状态机
# ----------------------------------------------------------------------
def test_cruise_waits_for_odom():
    controller = WaypointCruiseController([(10.0, 0.0)])
    command, state, report = controller.step(1.0)
    assert command == (0.0, 0.0, 0.0)
    assert state == 'odom_stale'


def test_cruise_advances_through_waypoints_and_arrives():
    settings = CruiseSettings(cruise_altitude=-5.0, arrival_radius=0.5,
                              position_gain=2.0, max_speed=4.0)
    controller = WaypointCruiseController(
        [(10.0, 0.0), (10.0, 10.0), (0.0, 10.0)], settings)
    now = 0.0
    # 从出发点直接放到第一个航点附近，验证按序推进。
    controller.update_pose(9.8, 0.0, -5.0, 0.0, now)
    command, state, report = controller.step(now)
    assert state == 'cruising'
    assert report['waypoints_done'] == 1

    controller.update_pose(10.0, 9.7, -5.0, 0.0, now + 0.1)
    _, state, report = controller.step(now + 0.1)
    assert state == 'cruising'
    assert report['waypoints_done'] == 2

    controller.update_pose(0.4, 10.0, -5.0, 0.0, now + 0.2)
    command, state, report = controller.step(now + 0.2)
    assert state == 'arrived'
    assert report['waypoints_done'] == 3
    assert report['waypoints_left'] == 0
    assert report['progress_percent'] == 100.0


def test_cruise_arrived_hovers_over_last_waypoint():
    controller = WaypointCruiseController(
        [(5.0, 0.0), (5.0, 5.0)],
        CruiseSettings(position_gain=1.0, max_speed=2.0))
    now = 0.0
    # 依次到达两个航点。
    controller.update_pose(5.0, 0.1, -5.0, 0.0, now)
    controller.step(now)
    controller.update_pose(5.0, 4.9, -5.0, 0.0, now + 0.1)
    command, state, _ = controller.step(now + 0.1)
    assert state == 'arrived'
    assert command[1] == pytest.approx(0.6, abs=0.6)   # 在航点附近

    # 到达后从远处再给位姿：应输出指向最后航点的三轴速度。
    controller.update_pose(5.0, 3.0, -5.0, 0.0, now + 0.2)
    command, state, report = controller.step(now + 0.2)
    assert state == 'arrived'
    assert len(command) == 3, '速度指令必须是三轴'
    assert command[1] == pytest.approx(2.0)   # 指向 (5,5)，速度饱和
    assert command[0] == pytest.approx(0.0)
    assert command[2] == pytest.approx(0.0)   # 高度已达标


def test_cruise_rejects_bad_waypoints():
    with pytest.raises(ValueError):
        WaypointCruiseController([])
    with pytest.raises(ValueError):
        WaypointCruiseController([(float('nan'), 0.0)])


def test_cruise_report_contains_metrics():
    controller = WaypointCruiseController([(10.0, 0.0)])
    controller.update_pose(0.0, 0.0, -5.0, 0.0, 1.0)
    _, _, report = controller.step(1.0)
    for key in ('state', 'waypoints_done', 'waypoints_left',
                'distance_done_m', 'energy_done_j', 'energy_planned_j',
                'progress_percent'):
        assert key in report
    assert report['energy_planned_j'] > 0
