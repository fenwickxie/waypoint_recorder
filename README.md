# Waypoint Recorder & Smooth Auto-Publisher

一个解耦、跨项目可直接复用的 ROS 航点录制与顺滑自动发布功能包。

---

## 核心特性

1. **手柄遥控与键盘即时录制**：
   - 遥控机器人前往期望航点，在交互终端单键按下 `r` 即可即时录制当前机器人坐标。
2. **单点与多点自动连续发布**：
   - 按下 `p` 键自动从第一个录制点开始发布，机器人到达后自动流转到下一个目标点。
3. **提前发布与顺滑衔接机制（可动态开关）**：
   - 传统航点发布会在机器人精确到达目标（如 `0.5m`）后才切换，容易导致机器人频繁减速甚至停顿。
   - 本功能包内置**提前发布顺滑切换算法**：在接近当前目标点达到预设阈值（如 `1.2m`，大于到达阈值）时，立即提前发布下一个目标点，规划器生成前瞻平滑轨迹，消除顿挫。
   - 最后一个终点目标仍会以严格到达阈值执行，并支持在终端按 `t` 键随时开启/关闭此功能。
4. **双目标格式接口兼容与高度解耦**：
   - 同时支持发布 `geometry_msgs/PointStamped`（默认话题 `/way_point`，适配 `local_planner`）以及 `geometry_msgs/PoseStamped`（默认话题 `/move_base_simple/goal`，适配标准 ROS 导航系统）。
   - 仅依赖 ROS 标准消息类型，零私有依赖，修改 `config/params.yaml` 中的话题名即可直接移植到任何 ROS 1 机器人项目。
5. **持久化保存与读取**：
   - 按 `w` 后可选择保存为 YAML（`y`）、ASCII PLY（`p`）或同时保存两种格式（`a`），按 `x` 取消；读取仍根据文件扩展名自动识别。
   - PLY 文件保存为 `x y z` 顶点列表，可直接作为 `waypoint_example` 的航点文件；PLY 不包含航向角和时间戳，加载时这两项设为 `0`。
6. **RViz 状态反馈**：
   - 自动在 `/waypoint_recorder/markers` 话题发布航点标记（绿色圆球）、当前行驶目标高亮（黄色圆球）、航点编号悬浮文本及轨迹连线。

---

## 快速使用

### 1. 启动仿真或实车系统
例如启动车库仿真环境：
```bash
roslaunch vehicle_simulator system_garage.launch
```

### 2. 启动航点录制与发布模块
在新的终端中运行：
```bash
roslaunch waypoint_recorder waypoint_recorder.launch
```
或者单独在新终端启动交互控制台：
```bash
rosrun waypoint_recorder waypoint_teleop
```

### 3. 终端快捷键操作指南
| 按键 | 功能 | 说明 |
| :---: | :--- | :--- |
| **`r`** | **录制当前航点** | 记录当前位姿，RViz 会实时显示新航点和编号 |
| **`p`** | **开始连续发布** | 自动从第 1 个点开始按顺序发布导航目标 |
| **`s`** | **暂停/停止发布** | 暂停当前自动导航 |
| **`u`** | **撤销上一个点** | 删除最近一次录制的航点 |
| **`c`** | **清空全部航点** | 清空内存中所有航点及 RViz 显示 |
| **`t`** | **顺滑提前发布开关** | 动态切换是否提前发布下一个目标点 |
| **`w`** | **选择格式保存** | 再按 `y` 保存 YAML、`p` 保存 PLY、`a` 保存两种格式，或按 `x` 取消 |
| **`l`** | **从文件加载** | 从 `saved_waypoints/waypoints.yaml` 加载之前保存的路径 |
| **`i`** | **打印当前状态** | 显示当前航点数、进行中航点和开关状态 |
| **`h`** | **查看帮助** | 打印按键提示 |
| **`q`** | **退出控制台** | 退出终端交互工具 |

默认路径基名仍为 `saved_waypoints/waypoints`。保存时会在此基名上添加 `.yaml` 或 `.ply` 扩展名；选择全部时会同时生成两份文件。若需更改保存位置，可覆盖 launch 参数：

```bash
roslaunch waypoint_recorder waypoint_recorder.launch waypoint_file:=/path/to/my_waypoints.yaml
```

保存后可将该文件作为 `waypoint_example` 的 `waypoint_file_dir` 使用。加载时同样依据 `.yaml` 或 `.ply` 扩展名自动识别。

---

## 参数配置说明 (`config/params.yaml`)

```yaml
topics:
  odom_topic: "/state_estimation"        # 里程计话题
  point_goal_topic: "/way_point"         # PointStamped 目标话题
  publish_point_goal: true               # 是否发布 PointStamped
  pose_goal_topic: "/move_base_simple/goal" # PoseStamped 目标话题
  publish_pose_goal: true                # 是否发布 PoseStamped
  speed_topic: "/speed"                  # 自主运行速度触发话题
  publish_speed: true
  speed_value: 1.0
  frame_id: "map"                        # 坐标系名称

navigation:
  arrival_radius: 0.5                    # 最终到达判定半径 (m)
  z_bound: 2.0                           # 垂直容差 (m)
  enable_early_switch: true              # 是否开启提前发布顺滑切换
  early_switch_radius: 1.2               # 提前发布下一个航点的距离阈值 (m)
  wait_time: 0.0                         # 到达后停顿等待时间 (s)
  loop_navigation: false                 # 是否循环巡航
  control_rate: 20.0                     # 控制循环频率 (Hz)
```
