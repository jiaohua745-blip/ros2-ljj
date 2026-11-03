"""Waypoint visit-order optimization: nearest neighbor + 2-opt.

问题形式化：无人机从当前位置出发，依次访问全部目标点
（可选返回起点），目标是最小化能耗模型给出的总能耗。
这是一个非对称开销的 TSP 变体（转向惩罚与进入方向有关），
采用经典两段式启发：

1. 最近邻（nearest neighbor）构造初始解；
2. 2-opt 迭代消除路径交叉，直到无改进或达到迭代上限。

启发式不保证全局最优，但确定、快速、能耗显著低于按输入顺序
直接飞行；文档中的离线实验给出量化对比。
"""
from dataclasses import dataclass
import math

from .energy_model import Settings, route_energy


@dataclass(frozen=True)
class OptimizeResult:
    order: list                 # 优化后的访问顺序（点索引）
    naive_order: list           # 输入顺序（即"不优化直接飞"）
    energy_optimized: float     # 优化顺序总能耗，J
    energy_naive: float         # 输入顺序总能耗，J
    distance_optimized: float   # 优化顺序总路程，m
    distance_naive: float       # 输入顺序总路程，m
    time_optimized: float       # 优化顺序总时间，s
    time_naive: float           # 输入顺序总时间，s

    @property
    def energy_saving_percent(self):
        if self.energy_naive <= 0:
            return 0.0
        return 100.0 * (self.energy_naive - self.energy_optimized) \
            / self.energy_naive


def _path_metrics(points, order, settings, return_to_start):
    energy, time_s, segments = route_energy(points, order, settings,
                                            return_to_start)
    distance = sum(seg[2] for seg in segments)
    return energy, time_s, distance


def nearest_neighbor_order(points, start_index=0):
    """最近邻构造初始访问顺序。"""
    n = len(points)
    if n == 0:
        raise ValueError('points must not be empty')
    if not 0 <= start_index < n:
        raise ValueError('start_index out of range')
    unvisited = set(range(n)) - {start_index}
    order = [start_index]
    current = points[start_index]
    while unvisited:
        nxt = min(unvisited, key=lambda i: math.hypot(
            points[i][0] - current[0], points[i][1] - current[1]))
        order.append(nxt)
        unvisited.remove(nxt)
        current = points[nxt]
    return order


def two_opt_improve(points, order, settings, return_to_start=False,
                    max_iterations=1000):
    """2-opt：反复反转路径子段，直到能耗不再下降。"""
    order = list(order)
    if len(order) < 4:
        return order
    best_energy, _, _ = _path_metrics(points, order, settings,
                                      return_to_start)
    for _ in range(max_iterations):
        improved = False
        for i in range(1, len(order) - 1):
            for j in range(i + 1, len(order)):
                candidate = order[:i] + order[i:j + 1][::-1] + order[j + 1:]
                energy, _, _ = _path_metrics(points, candidate, settings,
                                             return_to_start)
                if energy < best_energy - 1e-9:
                    order = candidate
                    best_energy = energy
                    improved = True
        if not improved:
            break
    return order


def optimize(points, settings=None, start_index=0, return_to_start=False):
    """主入口：输入点集，返回优化顺序与原始顺序的量化对比。

    points 为 [(x, y), ...]；输入顺序即 naive 顺序（第一个点视为
    出发点，start_index 指定最近邻从哪个点出发）。
    """
    s = settings or Settings()
    if not points:
        raise ValueError('points must not be empty')
    if not all(len(p) >= 2 and all(math.isfinite(c) for c in p)
               for p in points):
        raise ValueError('points must be finite (x, y) pairs')

    naive_order = list(range(len(points)))
    order = two_opt_improve(points,
                            nearest_neighbor_order(points, start_index),
                            s, return_to_start)
    e_opt, t_opt, d_opt = _path_metrics(points, order, s, return_to_start)
    e_naive, t_naive, d_naive = _path_metrics(points, naive_order, s,
                                              return_to_start)
    return OptimizeResult(order=order, naive_order=naive_order,
                          energy_optimized=e_opt, energy_naive=e_naive,
                          distance_optimized=d_opt,
                          distance_naive=d_naive,
                          time_optimized=t_opt, time_naive=t_naive)
