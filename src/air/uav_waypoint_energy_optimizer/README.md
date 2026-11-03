# uav_waypoint_energy_optimizer — 基于能耗模型的多航点访问顺序优化与巡航

## 运行环境

- Windows 10/11 + OpenHUTB（AIR）仿真器，RPC 端口 `41451` 可达
- Ubuntu 20.04（虚拟机/WSL2）+ ROS 2 Foxy/Galactic
- Python 依赖：`rclpy`（离线工具仅需 Python 3.10+ 标准库）
- 复用 `openhutb_yolo_mlp_control` 功能包的 `airsim_bridge` 桥接节点，需先构建该包

## 运行步骤

1. 在 Windows 端启动 OpenHUTB 仿真器，手动起飞并稳定悬停。

2. 在 ROS 2 端构建并加载工作空间：

   ```bash
   cd ~/ros2_ws
   colcon build --packages-select openhutb_yolo_mlp_control uav_waypoint_energy_optimizer
   source install/setup.bash
   ```

3. 预览模式（不发送飞行指令）。启动日志打印优化结果（访问顺序、能耗对比、节省百分比）：

   ```bash
   ros2 launch uav_waypoint_energy_optimizer waypoint_energy_optimizer.launch.py \
     airsim_ip:="$AIRSIM_IP" dry_run:=true
   ```

   另开终端查看任务状态（JSON，20 Hz）：

   ```bash
   ros2 topic echo /openhutb/waypoint_energy/status
   ```

4. 实际巡航（无人机自动按优化顺序飞越全部航点，完成后在最后航点上方悬停）：

   ```bash
   ros2 launch uav_waypoint_energy_optimizer waypoint_energy_optimizer.launch.py \
     airsim_ip:="$AIRSIM_IP" \
     waypoints:="[12,-8,-6,-14,4,6,-12,2,10,-2]" \
     dry_run:=false
   ```

   任务结束后由操作者在仿真器中接管降落。

5. （可选）运行单元测试与离线闭环对比（无需 ROS / AirSim）：

   ```bash
   python3 -m pytest test/                                # 17 passed
   python3 tools/compare_routes.py --plot compare.png     # 输入顺序 vs 优化顺序
   python3 tools/make_demo_gif.py --output demo.gif       # 巡航动画
   ```

详细参数与状态说明见 `docs/air/uav_waypoint_energy_optimizer/waypoint_energy_optimizer.md`。
