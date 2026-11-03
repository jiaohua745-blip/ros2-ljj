"""Archimedean spiral-free launch: bridge + energy-optimized cruise.

复用 openhutb_yolo_mlp_control 的 AirSim 桥接节点，叠加本包的
能耗最优航点巡航节点；任一进程退出时整体退出。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from uav_waypoint_energy_optimizer.energy_model import Settings as EnergySettings
from uav_waypoint_energy_optimizer.waypoint_cruise import Settings as CruiseSettings


def generate_launch_description():
    bridge_package = 'openhutb_yolo_mlp_control'
    arguments = {
        'airsim_ip': ('127.0.0.1', str),
        'airsim_port': (41451, int),
        'waypoints': ([12.0, -8.0, -6.0, -14.0, 4.0, 6.0, -12.0, 2.0], list),
        'start_x': (0.0, float),
        'start_y': (0.0, float),
        'return_to_start': (False, bool),
        'dry_run': (True, bool),
        'control_backend': ('auto', str),
        'control_dt': (0.05, float),
        'max_dispatch_distance': (0.2, float),
    }
    for name, value in vars(CruiseSettings()).items():
        arguments[name] = (value, type(value))
    for name, value in vars(EnergySettings()).items():
        arguments['energy_' + name] = (value, type(value))

    declarations = [DeclareLaunchArgument(name, default_value=str(value).lower()
                    if isinstance(value, bool) else str(value))
                    for name, (value, _) in arguments.items()]
    param = lambda name: ParameterValue(LaunchConfiguration(name),
                                        value_type=arguments[name][1])

    bridge = Node(package=bridge_package, executable='airsim_bridge',
                  name='airsim_bridge', output='screen', parameters=[{
                      'airsim_ip': param('airsim_ip'),
                      'airsim_port': param('airsim_port'),
                      'enable_api_control': ParameterValue(PythonExpression([
                          "'", LaunchConfiguration('dry_run'), "'.lower() == 'false'"
                      ]), value_type=bool),
                      'auto_takeoff': False,
                      'arm_on_start': False,
                      'hold_heading': True,
                      'command_smoothing': 1.0,
                      'async_image_capture': True,
                      'command_duration': param('control_dt'),
                      'command_timeout': 0.5,
                      'min_dispatch_distance': 0.0,
                      'max_dispatch_distance': param('max_dispatch_distance'),
                      'control_backend': param('control_backend'),
                      'image_hz': 1.0,
                      'state_hz': 20.0,
                  }])

    cruiser = Node(package='uav_waypoint_energy_optimizer',
                   executable='waypoint_energy_optimizer',
                   name='waypoint_energy_optimizer', output='screen',
                   parameters=[{
                       name: param(name) for name in
                       [*vars(CruiseSettings()),
                        *('energy_' + n for n in vars(EnergySettings())),
                        'waypoints', 'start_x', 'start_y',
                        'return_to_start', 'dry_run', 'control_dt']
                   }])

    exits = [RegisterEventHandler(OnProcessExit(target_action=node,
              on_exit=[EmitEvent(event=Shutdown(
                  reason='Waypoint energy process exited'))]))
             for node in (bridge, cruiser)]
    return LaunchDescription(declarations + exits + [bridge, cruiser])
