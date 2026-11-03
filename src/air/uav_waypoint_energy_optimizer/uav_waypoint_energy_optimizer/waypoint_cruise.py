"""Pure cruise controller: fly the optimized waypoint route in order.

状态机：waiting_odom -> cruising -> arrived。
每个控制周期朝当前航点输出世界系速度指令（水平 P 控制 + 高度
保持），进入到达半径后切下一个航点；同时用能耗模型累计估算
能耗，供任务报告使用。所有位置与速度均为 AirSim 世界 NED 坐标。
"""
from dataclasses import dataclass
import math

from .energy_model import Settings as EnergySettings, segment_energy


@dataclass(frozen=True)
class Settings:
    cruise_altitude: float = -5.0   # 巡航高度（NED，负值代表高度）
    altitude_tolerance: float = 0.3  # 到达巡航高度的判定误差，m
    arrival_radius: float = 0.8     # 航点到达半径，m
    position_gain: float = 1.2      # 位置误差 -> 水平速度的增益
    max_speed: float = 2.0          # 水平速度上限，m/s
    altitude_gain: float = 1.0      # 高度误差 -> 垂直速度的增益
    max_vertical_speed: float = 1.0  # 垂直速度上限，m/s
    odom_timeout: float = 0.5       # 里程计过期时间，s

    def __post_init__(self):
        for name, value in vars(self).items():
            if not math.isfinite(value):
                raise ValueError(f'{name} must be finite')
        for name in ('altitude_tolerance', 'arrival_radius', 'position_gain',
                     'max_speed', 'altitude_gain', 'max_vertical_speed',
                     'odom_timeout'):
            if getattr(self, name) <= 0:
                raise ValueError(f'{name} must be positive')
        if self.cruise_altitude >= 0:
            raise ValueError('cruise_altitude must be negative (NED height)')


class WaypointCruiseController:
    """按给定顺序巡航；waypoints 为 [(x, y), ...] 世界坐标（米）。"""

    def __init__(self, waypoints, settings=None, energy_settings=None):
        if not waypoints:
            raise ValueError('waypoints must not be empty')
        if not all(len(p) >= 2 and all(math.isfinite(c) for c in p)
                   for p in waypoints):
            raise ValueError('waypoints must be finite (x, y) pairs')
        self.settings = settings or Settings()
        self.energy_settings = energy_settings or EnergySettings()
        self.waypoints = [(float(p[0]), float(p[1])) for p in waypoints]
        self.pose = None
        self.pose_time = None
        self.last_step_time = None
        self.last_heading = 0.0
        self.index = 0                 # 当前目标航点序号
        self.state = 'waiting_odom'
        self.distance_done = 0.0       # 已飞行水平路程
        self.energy_done = 0.0         # 已完成段的估算能耗
        self.total_energy = 0.0        # 全航线估算能耗（规划值）
        previous_heading = 0.0
        for i in range(len(self.waypoints)):
            a = self.waypoints[i - 1] if i else (0.0, 0.0)
            b = self.waypoints[i]
            distance = math.hypot(b[0] - a[0], b[1] - a[1])
            heading = math.atan2(b[1] - a[1], b[0] - a[0]) if distance > 1e-9 \
                else previous_heading
            change = math.atan2(math.sin(heading - previous_heading),
                                math.cos(heading - previous_heading))
            energy, _ = segment_energy(distance, change, self.energy_settings)
            self.total_energy += energy
            previous_heading = heading

    # ------------------------------------------------------------------
    def update_pose(self, x, y, z, yaw, now):
        if not all(math.isfinite(v) for v in (x, y, z, yaw, now)):
            self.pose = None
            return
        self.pose = (x, y, z, yaw)
        self.pose_time = now

    # ------------------------------------------------------------------
    def step(self, now):
        """返回 (vx, vy, vz), state, report。"""
        s = self.settings
        dt = (0.0 if self.last_step_time is None
              else max(0.0, min(0.1, now - self.last_step_time)))
        self.last_step_time = now

        if self.pose is None or self.pose_time is None \
                or not 0 <= now - self.pose_time <= s.odom_timeout:
            return (0.0, 0.0, 0.0), 'odom_stale', self._report()

        x, y, z, yaw = self.pose
        if self.state == 'waiting_odom':
            self.state = 'cruising'

        if self.state == 'cruising':
            if self.index >= len(self.waypoints):
                self.state = 'arrived'
            else:
                tx, ty = self.waypoints[self.index]
                distance = math.hypot(tx - x, ty - y)
                if distance <= s.arrival_radius and \
                        abs(z - s.cruise_altitude) <= s.altitude_tolerance * 2:
                    heading = math.atan2(ty - y, tx - x) if distance > 1e-9 \
                        else self.last_heading
                    change = math.atan2(math.sin(heading - self.last_heading),
                                        math.cos(heading - self.last_heading))
                    energy, _ = segment_energy(distance, change,
                                               self.energy_settings)
                    self.energy_done += energy
                    self.distance_done += distance
                    self.index += 1
                else:
                    command = self._track(tx, ty, x, y, z)
                    moved = math.hypot(command[0], command[1]) * dt
                    self.distance_done += moved
                    vx, vy = command[0], command[1]
                    speed = math.hypot(vx, vy)
                    energy, _ = segment_energy(
                        moved, 0.0,
                        self.energy_settings) if speed > 1e-6 else (0.0, 0.0)
                    self.energy_done += energy
                    return command, self.state, self._report()

        if self.index >= len(self.waypoints):
            self.state = 'arrived'
        # arrived：在最后一个航点上方保持位置。
        last_x, last_y = self.waypoints[-1]
        return self._track(last_x, last_y, x, y, z), self.state, self._report()

    # ------------------------------------------------------------------
    def _track(self, tx, ty, x, y, z):
        s = self.settings
        vx = s.position_gain * (tx - x)
        vy = s.position_gain * (ty - y)
        horizontal = math.hypot(vx, vy)
        if horizontal > s.max_speed:
            scale = s.max_speed / horizontal
            vx *= scale
            vy *= scale
        vz = max(-s.max_vertical_speed,
                 min(s.max_vertical_speed,
                     s.altitude_gain * (s.cruise_altitude - z)))
        return (vx, vy, vz)

    def _report(self):
        remaining = len(self.waypoints) - min(self.index, len(self.waypoints))
        return {
            'state': self.state,
            'segment': min(self.index, len(self.waypoints) - 1)
                       if self.waypoints else None,
            'waypoints_done': self.index,
            'waypoints_left': remaining,
            'distance_done_m': round(self.distance_done, 2),
            'energy_done_j': round(self.energy_done, 1),
            'energy_planned_j': round(self.total_energy, 1),
            'progress_percent': round(100.0 * self.index
                                      / len(self.waypoints), 1),
        }
