#!/usr/bin/env python3
"""Offline closed-loop comparison: optimized order vs input order.

用一阶惯性速度模型代替仿真器，让同一巡航控制器分别按"输入
顺序"和"能耗优化顺序"飞完全部航点，输出量化对比并绘制轨迹图。

用法：
    python3 tools/compare_routes.py --plot ../../docs/air/uav_waypoint_energy_optimizer/images/route_comparison.png
"""
import argparse
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import matplotlib
matplotlib.use('Agg')
matplotlib.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei',
                                          'Noto Sans CJK SC', 'DejaVu Sans']
matplotlib.rcParams['axes.unicode_minus'] = False
import matplotlib.pyplot as plt

from uav_waypoint_energy_optimizer.energy_model import Settings as EnergySettings
from uav_waypoint_energy_optimizer.route_optimizer import optimize
from uav_waypoint_energy_optimizer.waypoint_cruise import (
    Settings as CruiseSettings, WaypointCruiseController)


class SimpleDrone:
    """速度指令 -> 一阶惯性速度 -> 位置积分（世界 NED）。"""

    def __init__(self, x, y, z, tau=0.5):
        self.pos = [x, y, z]
        self.tau = tau
        self.vel = [0.0, 0.0, 0.0]

    def step(self, command, dt):
        for i in range(3):
            self.vel[i] += (command[i] - self.vel[i]) / self.tau * dt
            self.pos[i] += self.vel[i] * dt


def fly_route(points, settings, cruise_settings, dt=0.05, max_time=600.0):
    """按给定点序巡航，返回 (轨迹, 状态序列, 完成时间, 实际路程)。"""
    controller = WaypointCruiseController(points, cruise_settings, settings)
    drone = SimpleDrone(points[0][0], points[0][1], -1.0)
    now = 0.0
    trail = [(drone.pos[0], drone.pos[1])]
    states = []
    command = (0.0, 0.0, 0.0)
    while now <= max_time:
        drone.step(command, dt)
        now += dt
        controller.update_pose(drone.pos[0], drone.pos[1], drone.pos[2],
                               0.0, now)
        command, state, report = controller.step(now)
        trail.append((drone.pos[0], drone.pos[1]))
        states.append(state)
        if state == 'arrived':
            break
    distance = sum(math.hypot(trail[i][0] - trail[i - 1][0],
                              trail[i][1] - trail[i - 1][1])
                   for i in range(1, len(trail)))
    return trail, states, now, distance


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--points', type=float, nargs='+',
                        default=[12.0, -8.0, -6.0, -14.0, 4.0, 6.0,
                                 -12.0, 2.0, 10.0, -2.0],
                        help='展平的航点坐标 [x1, y1, x2, y2, ...]')
    parser.add_argument('--plot', type=str, default='',
                        help='对比图保存路径')
    args = parser.parse_args()
    if not args.points or len(args.points) % 2 != 0:
        raise SystemExit('points 必须是成对的 [x1, y1, ...] 列表')
    targets = [(args.points[i], args.points[i + 1])
               for i in range(0, len(args.points), 2)]

    energy_settings = EnergySettings()
    cruise_settings = CruiseSettings()
    start = (0.0, 0.0)
    plan = optimize([start] + targets, energy_settings,
                    start_index=0, return_to_start=False)

    optimized_route = [([start] + targets)[i] for i in plan.order][1:]
    naive_route = targets

    print('======== 规划结果（能耗模型估算）========')
    print(f'航点数        : {len(targets)}')
    print(f'输入顺序能耗  : {plan.energy_naive:8.0f} J '
          f'(路程 {plan.distance_naive:.1f} m)')
    print(f'优化顺序能耗  : {plan.energy_optimized:8.0f} J '
          f'(路程 {plan.distance_optimized:.1f} m)')
    print(f'能耗节省      : {plan.energy_saving_percent:.1f} %')
    print(f'优化访问顺序  : {plan.order[1:]}')

    trail_opt, _, t_opt, d_opt = fly_route(
        optimized_route, energy_settings, cruise_settings)
    trail_naive, _, t_naive, d_naive = fly_route(
        naive_route, energy_settings, cruise_settings)

    print('======== 闭环飞行对比（一阶惯性模型）========')
    print(f'输入顺序      : {t_naive:6.1f} s, 实际路程 {d_naive:6.1f} m')
    print(f'优化顺序      : {t_opt:6.1f} s, 实际路程 {d_opt:6.1f} m')
    print(f'路程节省      : {100.0 * (d_naive - d_opt) / d_naive:.1f} %')

    if args.plot:
        Path(args.plot).parent.mkdir(parents=True, exist_ok=True)
        fig, ax = plt.subplots(figsize=(7.2, 6.0))
        ax.plot([p[0] for p in trail_naive], [p[1] for p in trail_naive],
                '-', lw=1.4, color='tab:red', label=f'输入顺序 ({d_naive:.0f} m)')
        ax.plot([p[0] for p in trail_opt], [p[1] for p in trail_opt],
                '-', lw=1.4, color='tab:blue',
                label=f'优化顺序 ({d_opt:.0f} m)')
        ax.plot(start[0], start[1], 'k^', ms=10, label='出发点')
        for i, p in enumerate(targets):
            ax.plot(*p, 'ks', ms=7, mfc='white')
            ax.annotate(str(i + 1), p, textcoords='offset points',
                        xytext=(7, 5), fontsize=10)
        ax.set_xlabel('x / NED m')
        ax.set_ylabel('y / NED m')
        ax.set_aspect('equal', adjustable='datalim')
        ax.set_title(f'航点访问顺序对比（能耗节省 '
                     f'{plan.energy_saving_percent:.1f}%）')
        ax.legend(loc='best', fontsize=9)
        ax.grid(alpha=0.3)
        fig.tight_layout()
        fig.savefig(args.plot, dpi=150)
        print(f'对比图已保存  : {args.plot}')


if __name__ == '__main__':
    main()
