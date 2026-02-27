# Orion Manipulator

ROS 2 机械臂描述与仿真包：包含 Orion 机器人的 URDF、MoveIt2 配置及基于 PyBullet 的轨迹执行仿真（无需 Gazebo，适合虚拟机/无头环境）。

## 功能概览

- **URDF**：Orion 机器人模型（8 关节：6 自由度机械臂 + 2 自由度夹爪）
- **MoveIt2**：OMPL 规划、KDL 运动学（manipulator 规划组）、轨迹执行通过 `orion_controller/follow_joint_trajectory` Action
- **PyBullet 仿真**：订阅上述 Action，在 PyBullet 中跟踪轨迹并发布 `/joint_states`，供 RViz 与 MoveIt 使用

## 依赖

- ROS 2（Humble 或更高）
- MoveIt2：`moveit_ros_move_group`、`moveit_kinematics`、`moveit_planners_ompl`、`moveit_ros_visualization`、`moveit_configs_utils`
- 其他：`robot_state_publisher`、`rviz2`、`tf2_ros`、`xacro`、`rclpy`、`control_msgs`、`sensor_msgs`
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

## 目录结构

```
My_OrionManipulator/
├── config/          # MoveIt：SRDF、关节限位、运动学、OMPL、控制器等
├── launch/           # moveit_pybullet.launch.py、moveit.rviz
├── meshes/           # 机器人网格资源
├── scripts/          # pybullet_follow_joint_trajectory_server.py
├── urdf/             # orion.urdf
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
