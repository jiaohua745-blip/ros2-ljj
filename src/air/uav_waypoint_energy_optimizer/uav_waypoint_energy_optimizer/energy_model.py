"""Simplified multirotor energy model for route comparison.

功率模型采用教学常用的简化形式：悬停功率 P0 与随速度二次增长
的推进功率 c·v²，即 P(v) = P0 + c·v²。飞行一段距离 d 的能耗为

    E_cruise = (P0 + c·v²) · d / v

转向时减速悬停，转向角 Δψ 在最大偏航角速度 ω 下耗时 Δψ/ω，
期间按悬停功率计：

    E_turn = P0 · Δψ / ω

该模型能复现多旋翼的真实特性：能耗-速度曲线存在最优巡航速度
v* = sqrt(P0 / c)，并非越快越省电。
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Settings:
    hover_power: float = 120.0   # 悬停功率 P0，单位 W
    drag_coeff: float = 2.4      # 推进功率系数 c，单位 W/(m/s)^2
    cruise_speed: float = 2.0    # 巡航速度，单位 m/s
    yaw_rate_max: float = 1.5    # 最大偏航角速度，单位 rad/s

    def __post_init__(self):
        for name, value in vars(self).items():
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive')

    def optimal_speed(self):
        """能耗-速度曲线的最优巡航速度 sqrt(P0/c)。"""
        return math.sqrt(self.hover_power / self.drag_coeff)


def segment_energy(distance, heading_change, settings):
    """一段飞行的 (能耗 J, 时间 s)。

    distance 为水平距离，heading_change 为该段起始的转向角（rad）。
    """
    s = settings
    if not math.isfinite(distance) or distance < 0:
        raise ValueError('distance must be finite and nonnegative')
    if not math.isfinite(heading_change):
        raise ValueError('heading_change must be finite')
    turn_time = abs(heading_change) / s.yaw_rate_max
    cruise_time = distance / s.cruise_speed
    energy = ((s.hover_power + s.drag_coeff * s.cruise_speed ** 2) * cruise_time
              + s.hover_power * turn_time)
    return energy, turn_time + cruise_time


def route_energy(points, order, settings, return_to_start=False):
    """按 order 依次访问 points 的总能耗与分段明细。

    points: [(x, y), ...]，order: 索引列表（元素唯一且覆盖全部点）。
    return_to_start=True 时末尾飞回起点（order[0] 对应的位置）。
    返回 (total_energy, total_time, segments)，segments 为
    [(from_idx, to_idx, distance, heading_change, energy, time), ...]。
    """
    s = settings
    if not points:
        raise ValueError('points must not be empty')
    if sorted(order) != list(range(len(points))):
        raise ValueError('order must be a permutation of point indices')
    if not all(math.isfinite(c) for p in points for c in p):
        raise ValueError('points must be finite')

    path = list(order) + ([order[0]] if return_to_start else [])
    total_energy = 0.0
    total_time = 0.0
    segments = []
    previous_heading = 0.0
    for i in range(len(path) - 1):
        a = points[path[i]]
        b = points[path[i + 1]]
        distance = math.hypot(b[0] - a[0], b[1] - a[1])
        heading = math.atan2(b[1] - a[1], b[0] - a[0]) if distance > 1e-9 \
            else previous_heading
        heading_change = math.atan2(math.sin(heading - previous_heading),
                                    math.cos(heading - previous_heading))
        energy, time_s = segment_energy(distance, heading_change, s)
        segments.append((path[i], path[i + 1], distance,
                         heading_change, energy, time_s))
        total_energy += energy
        total_time += time_s
        previous_heading = heading
    return total_energy, total_time, segments
