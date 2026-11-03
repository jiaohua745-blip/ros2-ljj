"""Unit tests for the energy model (pure logic, no ROS required)."""
import math

import pytest

from uav_waypoint_energy_optimizer.energy_model import (
    Settings, route_energy, segment_energy)


def test_settings_rejects_invalid_values():
    with pytest.raises(ValueError):
        Settings(hover_power=0.0)
    with pytest.raises(ValueError):
        Settings(drag_coeff=-1.0)
    with pytest.raises(ValueError):
        Settings(cruise_speed=float('nan'))


def test_optimal_speed_matches_physics():
    s = Settings(hover_power=120.0, drag_coeff=2.4)
    assert s.optimal_speed() == pytest.approx(math.sqrt(50.0))


def test_energy_curve_has_interior_optimum():
    """能耗-速度曲线应在 v* = sqrt(P0/c) 附近取得最小值。"""
    s = Settings(hover_power=120.0, drag_coeff=2.4)
    v_star = s.optimal_speed()
    distance = 100.0

    def energy_at(v):
        return (s.hover_power + s.drag_coeff * v * v) * distance / v

    assert energy_at(v_star) < energy_at(0.5 * v_star)
    assert energy_at(v_star) < energy_at(2.0 * v_star)


def test_segment_energy_positive_and_grows_with_distance():
    s = Settings()
    e1, t1 = segment_energy(10.0, 0.0, s)
    e2, t2 = segment_energy(20.0, 0.0, s)
    assert e1 > 0 and t1 > 0
    assert e2 == pytest.approx(2 * e1)
    assert t2 == pytest.approx(2 * t1)


def test_turn_penalty_accounts_for_hover_time():
    s = Settings(yaw_rate_max=1.5)
    e_straight, t_straight = segment_energy(0.0, 0.0, s)
    assert e_straight == 0.0 and t_straight == 0.0
    # 90° 转向悬停 1.5 s/2 ... Δψ=π/2 → 悬停时间 (π/2)/1.5。
    e_turn, t_turn = segment_energy(0.0, math.pi / 2, s)
    assert t_turn == pytest.approx((math.pi / 2) / 1.5)
    assert e_turn == pytest.approx(s.hover_power * t_turn)


def test_route_energy_rejects_bad_input():
    s = Settings()
    points = [(0.0, 0.0), (1.0, 1.0)]
    with pytest.raises(ValueError):
        route_energy([], [0], s)
    with pytest.raises(ValueError):
        route_energy(points, [0, 0], s)          # 不是排列
    with pytest.raises(ValueError):
        route_energy([(0.0, float('nan'))], [0], s)


def test_route_energy_sums_segments_and_closes_loop():
    s = Settings()
    points = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0)]
    total_open, time_open, segments_open = route_energy(
        points, [0, 1, 2], s)
    assert len(segments_open) == 2
    assert total_open == pytest.approx(sum(seg[4] for seg in segments_open))

    total_closed, time_closed, segments_closed = route_energy(
        points, [0, 1, 2], s, return_to_start=True)
    assert len(segments_closed) == 3
    assert total_closed > total_open
    assert time_closed > time_open
