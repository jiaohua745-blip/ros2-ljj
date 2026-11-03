#!/usr/bin/env python3
"""Animate the energy-optimized cruise mission as a GIF (no AirSim).

左侧动画展示优化顺序的飞行过程（蓝色）并叠加输入顺序路径
（红色淡线）作对比；右侧实时显示任务指标。控制器为真实代码，
无人机用一阶惯性模型代替。

用法：
    python3 tools/make_demo_gif.py --output ../../docs/air/uav_waypoint_energy_optimizer/images/waypoint_energy_demo.gif
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
from matplotlib.animation import FuncAnimation, PillowWriter

from compare_routes import SimpleDrone, fly_route
from uav_waypoint_energy_optimizer.energy_model import Settings as EnergySettings
from uav_waypoint_energy_optimizer.route_optimizer import optimize
from uav_waypoint_energy_optimizer.waypoint_cruise import (
    Settings as CruiseSettings, WaypointCruiseController)

STATE_CN = {'waiting_odom': '等待里程计', 'cruising': '巡航中',
            'arrived': '全部航点已访问', 'odom_stale': '里程计中断'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--points', type=float, nargs='+',
                        default=[12.0, -8.0, -6.0, -14.0, 4.0, 6.0,
                                 -12.0, 2.0, 10.0, -2.0])
    parser.add_argument('--sample-step', type=int, default=6)
    parser.add_argument('--fps', type=int, default=6)
    parser.add_argument('--output', type=str, default='waypoint_energy_demo.gif')
    args = parser.parse_args()
    targets = [(args.points[i], args.points[i + 1])
               for i in range(0, len(args.points), 2)]

    energy_settings = EnergySettings()
    cruise_settings = CruiseSettings()
    start = (0.0, 0.0)
    plan = optimize([start] + targets, energy_settings, start_index=0,
                    return_to_start=False)
    optimized_route = [([start] + targets)[i] for i in plan.order][1:]
    naive_trail, _, _, d_naive = fly_route(
        targets, energy_settings, cruise_settings)

    # 重新飞一遍优化路线并记录逐步指标。
    controller = WaypointCruiseController(
        optimized_route, cruise_settings, energy_settings)
    drone = SimpleDrone(start[0], start[1], -1.0)
    log = {'t': [], 'pos': [], 'state': [], 'done': [], 'dist': [],
           'energy': []}
    distance = 0.0
    now = 0.0
    command = (0.0, 0.0, 0.0)
    prev = (start[0], start[1])
    while now <= 600.0:
        drone.step(command, 0.05)
        now += 0.05
        controller.update_pose(drone.pos[0], drone.pos[1], drone.pos[2],
                               0.0, now)
        command, state, report = controller.step(now)
        distance += math.hypot(drone.pos[0] - prev[0],
                               drone.pos[1] - prev[1])
        prev = (drone.pos[0], drone.pos[1])
        log['t'].append(now)
        log['pos'].append(prev)
        log['state'].append(state)
        log['done'].append(report['waypoints_done'])
        log['dist'].append(distance)
        log['energy'].append(report['energy_done_j'])
        if state == 'arrived':
            break

    indices = list(range(0, len(log['t']), args.sample_step))
    if indices[-1] != len(log['t']) - 1:
        indices.append(len(log['t']) - 1)

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(9.6, 4.8))
    ax1.plot([p[0] for p in naive_trail], [p[1] for p in naive_trail],
             '-', lw=1.0, color='tab:red', alpha=0.35,
             label=f'输入顺序 ({d_naive:.0f} m)')
    ax1.plot(start[0], start[1], 'k^', ms=9, label='出发点')
    for i, p in enumerate(targets):
        ax1.plot(*p, 'ks', ms=7, mfc='white')
        ax1.annotate(str(i + 1), p, textcoords='offset points',
                     xytext=(7, 5), fontsize=10)
    (trail_line,) = ax1.plot([], [], '-', lw=1.6, color='tab:blue',
                             label='优化顺序')
    (drone_dot,) = ax1.plot([], [], 'o', ms=8, color='tab:blue')
    ax1.set_xlabel('x / NED m')
    ax1.set_ylabel('y / NED m')
    ax1.set_aspect('equal', adjustable='datalim')
    ax1.legend(loc='upper left', fontsize=7)
    ax1.grid(alpha=0.3)
    title = ax1.set_title('energy-optimized cruise')

    ax2.axis('off')
    lines = {key: ax2.text(0.02, 0.92 - i * 0.13, '', fontsize=12,
                           transform=ax2.transAxes)
             for i, key in enumerate(['state', 't', 'done', 'dist',
                                      'energy', 'saving'])}
    ax2.text(0.02, 0.10, 'uav_waypoint_energy_optimizer 离线闭环仿真\n'
             '（一阶惯性模型代替 AirSim，控制器为真实代码）',
             fontsize=8, color='0.4', transform=ax2.transAxes)

    def update(i):
        k = indices[i]
        t = log['t'][k]
        trail_line.set_data([p[0] for p in log['pos'][:k + 1]],
                            [p[1] for p in log['pos'][:k + 1]])
        drone_dot.set_data([log['pos'][k][0]], [log['pos'][k][1]])
        title.set_text(f'energy-optimized cruise  t={t:.1f}s  '
                       f'state={log["state"][k]}')
        lines['state'].set_text(f'状态     : '
                                f'{STATE_CN.get(log["state"][k], log["state"][k])}')
        lines['t'].set_text(f'时间     : {t:6.1f} s')
        lines['done'].set_text(f'航点进度 : {log["done"][k]} / {len(targets)}')
        lines['dist'].set_text(f'已飞路程 : {log["dist"][k]:5.1f} m')
        lines['energy'].set_text(f'能耗估计 : {log["energy"][k]:7.0f} J')
        lines['saving'].set_text(f'规划节省 : '
                                 f'{plan.energy_saving_percent:.1f} %')
        return (trail_line, drone_dot, title, *lines.values())

    anim = FuncAnimation(fig, update, frames=len(indices),
                         interval=1000.0 / args.fps, blit=False)
    Path(args.output).parent.mkdir(parents=True, exist_ok=True)
    anim.save(args.output, writer=PillowWriter(fps=args.fps), dpi=75)
    size_mb = Path(args.output).stat().st_size / 1e6
    print(f'GIF 已保存: {args.output} ({size_mb:.2f} MB, '
          f'{len(indices)} 帧)')


if __name__ == '__main__':
    main()
