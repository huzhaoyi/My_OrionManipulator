# Orion Manipulator

ROS 2 机械臂描述与仿真包：包含 Orion 机器人的 URDF、MoveIt2 配置及基于 PyBullet 的轨迹执行仿真（无需 Gazebo，适合虚拟机/无头环境）。

## 功能概览

- **URDF**：Orion 机器人模型（8 关节：6 自由度机械臂 + 2 自由度夹爪）
- **MoveIt2**：OMPL 规划、KDL 运动学（manipulator 规划组）、轨迹执行通过 `orion_controller/follow_joint_trajectory` Action
- **PyBullet 仿真**：订阅上述 Action，在 PyBullet 中跟踪轨迹并发布 `/joint_states`，供 RViz 与 MoveIt 使用
- **MTC 缆绳抓取**：MoveIt Task Constructor 任务节点（`orion_mtc_node`），流程为 MoveTo pregrasp → MoveRelative 接近 → CloseGripper → Attach → Lift；规划场景中用简化圆柱表示可抓取缆绳

## 依赖

- ROS 2（Humble 或更高）
- MoveIt2：`moveit_ros_move_group`、`moveit_kinematics`、`moveit_planners_ompl`、`moveit_ros_visualization`、`moveit_configs_utils`
- 其他：`robot_state_publisher`、`rviz2`、`tf2_ros`、`xacro`、`rclpy`、`rclcpp`、`control_msgs`、`sensor_msgs`、`moveit_ros_planning_interface`
- **MoveIt Task Constructor**（MTC 抓取用）：Humble 需从源码构建，见下方「MTC 缆绳抓取」。
- PyBullet（仿真用）：`pip3 install pybullet`

## 编译

```bash
cd /path/to/My_OrionManipulator
colcon build --packages-select orion
source install/setup.bash
```

## 运行

启动 MoveIt2 规划 + PyBullet 执行 + RViz：

```bash
ros2 launch orion moveit_pybullet.launch.py
```

可选参数：

| 参数            | 默认值  | 说明                     |
|-----------------|---------|--------------------------|
| `pybullet_gui`  | `true`  | 是否显示 PyBullet 窗口   |

无 GUI（如 SSH/无头环境）：

```bash
ros2 launch orion moveit_pybullet.launch.py pybullet_gui:=false
```

在 RViz 中使用 **Plan** 或 **Plan & Execute** 规划轨迹，轨迹会由 PyBullet 节点执行并反馈到 `/joint_states`。

### MTC 缆绳抓取

需先在同一 colcon 工作空间中从源码构建 MoveIt Task Constructor（Humble 无二进制包）：

```bash
cd /path/to/workspace/src   # 与 orion 同级的 src
git clone -b humble https://github.com/ros-planning/moveit_task_constructor.git
cd ..
rosdep install --from-paths src --ignore-src --rosdistro humble
colcon build --mixin release
source install/setup.bash
```

然后编译并启动 MTC 抓取（含 move_group、PyBullet、RViz 与 `orion_mtc_node`）：

```bash
colcon build --packages-select orion
source install/setup.bash
ros2 launch orion mtc_grasp.launch.py
```

节点会在 Planning Scene 中添加代表缆绳的简化圆柱，并执行一次完整抓取：预抓取 → 直线接近 5–10 cm → 闭爪 → Attach → 抬起。可选参数 `pybullet_gui` 同 `moveit_pybullet.launch.py`。

## 目录结构

```
My_OrionManipulator/
├── config/          # MoveIt：SRDF、关节限位、运动学、OMPL、控制器等
├── launch/          # moveit_pybullet.launch.py、mtc_grasp.launch.py、moveit.rviz
├── meshes/          # 机器人网格资源
├── scripts/         # pybullet_follow_joint_trajectory_server.py
├── src/             # orion_mtc_node.cpp（MTC 缆绳抓取任务）
├── urdf/            # orion.urdf
├── CMakeLists.txt
├── package.xml
└── README.md
```

## 控制器与接口

- **Action**：`orion_controller/follow_joint_trajectory`（`control_msgs/FollowJointTrajectory`）
- **Topic**：节点发布 `/joint_states`（`sensor_msgs/JointState`）
- 关节顺序：`joint1`～`joint8`（与 `config/moveit_controllers.yaml`、`config/joint_names_orion.yaml` 一致）

## 许可

BSD（见 package.xml）。
