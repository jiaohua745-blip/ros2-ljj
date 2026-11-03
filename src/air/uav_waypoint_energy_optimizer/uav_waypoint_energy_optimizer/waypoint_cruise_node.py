"""ROS 2 node: fly the energy-optimized waypoint route on /openhutb topics.

订阅 /openhutb/odom，发布速度指令；航点顺序由 route_optimizer
在启动时计算并固定。默认 dry_run 只发预览话题。
"""
import json
import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from .energy_model import Settings as EnergySettings
from .route_optimizer import optimize
from .waypoint_cruise import Settings as CruiseSettings, WaypointCruiseController


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class WaypointEnergyNode(Node):
    def __init__(self, **kwargs):
        super().__init__('waypoint_energy_optimizer', **kwargs)
        for name, value in vars(CruiseSettings()).items():
            self.declare_parameter(name, value)
        for name, value in vars(EnergySettings()).items():
            self.declare_parameter('energy_' + name, value)
        for name, value in {
            'waypoints': [12.0, -8.0, -6.0, -14.0, 4.0, 6.0, -12.0, 2.0],
            'start_x': 0.0, 'start_y': 0.0,
            'return_to_start': False,
            'dry_run': True,
            'control_dt': 0.05,
            'odom_topic': '/openhutb/odom',
            'cmd_vel_topic': '/openhutb/cmd_vel',
            'preview_topic': '/openhutb/waypoint_energy/preview_cmd_vel',
            'status_topic': '/openhutb/waypoint_energy/status',
        }.items():
            self.declare_parameter(name, value)

        cruise_settings = CruiseSettings(**{
            name: self.get_parameter(name).value
            for name in vars(CruiseSettings())})
        energy_settings = EnergySettings(**{
            name[len('energy_'):]: self.get_parameter('energy_' + name).value
            for name in vars(EnergySettings())})

        flat = [float(v) for v in self.get_parameter('waypoints').value]
        if not flat or len(flat) % 2 != 0:
            raise ValueError('waypoints must be a flat [x1, y1, x2, y2, ...] list')
        points = [(flat[i], flat[i + 1]) for i in range(0, len(flat), 2)]
        start = (float(self.get_parameter('start_x').value),
                 float(self.get_parameter('start_y').value))
        self.return_to_start = bool(self.get_parameter('return_to_start').value)

        # 排序：出发点在首位，其余点按能耗最优排序。
        plan_points = [start] + points
        self.plan = optimize(plan_points, energy_settings,
                             start_index=0,
                             return_to_start=self.return_to_start)
        route = [plan_points[i] for i in self.plan.order]
        cruise_points = route[1:]  # 出发点是当前位置，无需作为航点
        if self.return_to_start:
            cruise_points.append(route[0])
        self.controller = WaypointCruiseController(
            cruise_points, cruise_settings, energy_settings)

        self.dry_run = bool(self.get_parameter('dry_run').value)
        dt = float(self.get_parameter('control_dt').value)
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError('control_dt must be finite and positive')

        self.last_odom_stamp = -1
        self.last_status = None
        topic = lambda name: str(self.get_parameter(name).value)
        self.cmd_pub = self.create_publisher(
            Twist, topic('preview_topic') if self.dry_run
            else topic('cmd_vel_topic'), 1)
        self.status_pub = self.create_publisher(String, topic('status_topic'), 1)
        self.create_subscription(Odometry, topic('odom_topic'),
                                 self.odom_callback, qos_profile_sensor_data)
        self.timer = self.create_timer(dt, self.control_step)

        self.get_logger().info(
            f'Route planned: optimized order {self.plan.order} '
            f'(energy {self.plan.energy_optimized:.0f} J vs naive '
            f'{self.plan.energy_naive:.0f} J, saving '
            f'{self.plan.energy_saving_percent:.1f}%); '
            f'dry_run={self.dry_run}')

    # ------------------------------------------------------------------
    def odom_callback(self, msg):
        stamp = stamp_ns(msg.header.stamp)
        if stamp <= self.last_odom_stamp or msg.header.frame_id != 'airsim_ned':
            self.controller.pose = None
            return
        self.last_odom_stamp = stamp
        q = msg.pose.pose.orientation
        p = msg.pose.pose.position
        values = (q.x, q.y, q.z, q.w, p.x, p.y, p.z)
        norm = math.sqrt(sum(v * v for v in values[:4]))
        if not all(math.isfinite(v) for v in values) or abs(norm - 1) > 0.05:
            self.controller.pose = None
            return
        x, y, z, w = [v / norm for v in values[:4]]
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        age = (self.get_clock().now().nanoseconds - stamp) / 1e9
        self.controller.update_pose(p.x, p.y, p.z, yaw,
                                    time.monotonic() - age)

    # ------------------------------------------------------------------
    def control_step(self):
        command, state, report = self.controller.step(time.monotonic())
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.linear.z = command
        self.cmd_pub.publish(msg)
        report['dry_run'] = self.dry_run
        report['optimized_order'] = self.plan.order
        report['energy_saving_percent'] = round(
            self.plan.energy_saving_percent, 1)
        status = String()
        status.data = json.dumps(report)
        self.status_pub.publish(status)
        if state != self.last_status:
            if state == 'arrived':
                self.get_logger().warning(
                    f'All waypoints visited; distance '
                    f'{report["distance_done_m"]} m, energy estimate '
                    f'{report["energy_done_j"]} J. Hovering; land manually.')
            else:
                self.get_logger().info(f'Waypoint cruise: {state}')
            self.last_status = state

    def destroy_node(self):
        if rclpy.ok(context=self.context):
            self.cmd_pub.publish(Twist())
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = WaypointEnergyNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
