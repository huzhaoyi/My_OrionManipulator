#!/usr/bin/env python3
"""
MoveIt2 规划 + PyBullet 执行（VM 友好，无 Gazebo）：
订阅 orion_controller/follow_joint_trajectory action，在 PyBullet 中加载 URDF，
用位置/PD 控制跟踪轨迹点，并发布 /joint_states 供 RViz 与 MoveIt 使用。
"""

import os
import tempfile

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
# PyBullet 可选：未安装时给出明确错误
try:
    import pybullet as p
    import pybullet_data
except ImportError:
    p = None

# 默认控制增益（位置 P + 速度 D）
DEFAULT_POSITION_GAIN = 2.0
DEFAULT_VELOCITY_GAIN = 0.1
# 仿真步长与发布频率
SIM_DT = 1.0 / 240.0
JOINT_STATE_HZ = 50


def _resolve_urdf_package_path(urdf_str: str, pkg_share: str) -> str:
    """将 URDF 中的 package://orion/ 替换为包 share 绝对路径。"""
    return urdf_str.replace("package://orion/", pkg_share + "/")


class PyBulletFollowJointTrajectoryServer(Node):
    def __init__(self):
        super().__init__("pybullet_follow_joint_trajectory_server")

        self.declare_parameter("urdf_path", "")
        self.declare_parameter("use_gui", False)
        self.declare_parameter("position_gain", DEFAULT_POSITION_GAIN)
        self.declare_parameter("velocity_gain", DEFAULT_VELOCITY_GAIN)

        if p is None:
            self.get_logger().error(
                "pybullet 未安装，请执行: pip3 install pybullet"
            )
            raise RuntimeError("pybullet 未安装")

        self._pkg_share = self._get_package_share()
        urdf_param = self.get_parameter("urdf_path").value
        if urdf_param:
            self._urdf_path = urdf_param
        else:
            self._urdf_path = os.path.join(
                self._pkg_share, "urdf", "orion.urdf"
            )

        use_gui_val = self.get_parameter("use_gui").value
        self._use_gui = (
            use_gui_val if isinstance(use_gui_val, bool) else use_gui_val == "true"
        )
        self._kp = self.get_parameter("position_gain").value
        self._kd = self.get_parameter("velocity_gain").value

        self._client = None
        self._robot_id = None
        self._joint_name_to_index = {}
        self._joint_order = []

        self._init_pybullet()
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            "orion_controller/follow_joint_trajectory",
            self._execute_callback,
        )
        self._joint_state_pub = self.create_publisher(
            JointState,
            "joint_states",
            10,
        )
        self._joint_state_timer = self.create_timer(
            1.0 / JOINT_STATE_HZ,
            self._publish_joint_states,
        )
        self.get_logger().info(
            "PyBullet FollowJointTrajectory: orion_controller/follow_joint_trajectory"
        )

    def _get_package_share(self):
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory("orion")

    def _init_pybullet(self):
        if self._use_gui:
            self._client = p.connect(p.GUI)
        else:
            self._client = p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0.0, 0.0, -9.81)

        with open(self._urdf_path, "r") as f:
            urdf_str = f.read()
        urdf_resolved = _resolve_urdf_package_path(urdf_str, self._pkg_share)
        with tempfile.NamedTemporaryFile(
            mode="w", suffix=".urdf", delete=False
        ) as tf:
            tf.write(urdf_resolved)
            temp_urdf = tf.name
        try:
            self._robot_id = p.loadURDF(
                temp_urdf,
                basePosition=[0.0, 0.0, 0.0],
                baseOrientation=[0.0, 0.0, 0.0, 1.0],
                useFixedBase=True,
                flags=p.URDF_USE_SELF_COLLISION,
            )
        finally:
            try:
                os.unlink(temp_urdf)
            except OSError:
                pass

        num_joints = p.getNumJoints(self._robot_id)
        for i in range(num_joints):
            info = p.getJointInfo(self._robot_id, i)
            jtype = info[2]
            if jtype in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC):
                jname = info[1].decode("utf-8")
                self._joint_name_to_index[jname] = i
                self._joint_order.append(jname)

        self.get_logger().info(
            "PyBullet 已加载 URDF，可控关节: %s" % self._joint_order
        )

    def _publish_joint_states(self):
        if self._robot_id is None:
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = []
        msg.position = []
        msg.velocity = []
        for jname in self._joint_order:
            idx = self._joint_name_to_index.get(jname)
            if idx is None:
                continue
            state = p.getJointState(self._robot_id, idx)
            msg.name.append(jname)
            msg.position.append(float(state[0]))
            msg.velocity.append(float(state[1]))
        if msg.name:
            self._joint_state_pub.publish(msg)

    def _execute_callback(self, goal_handle):
        trajectory = goal_handle.request.trajectory
        if not trajectory.points:
            goal_handle.succeed()
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            return result

        joint_names = list(trajectory.joint_names)
        joint_indices = []
        for jname in joint_names:
            if jname in self._joint_name_to_index:
                joint_indices.append(self._joint_name_to_index[jname])
            else:
                self.get_logger().warn("未知关节名: %s" % jname)
                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
                goal_handle.abort()
                return result

        t_prev_sec = 0.0
        for i, point in enumerate(trajectory.points):
            t_sec = (
                point.time_from_start.sec
                + point.time_from_start.nanosec * 1e-9
            )
            target_pos = list(point.positions)
            if len(point.velocities) >= len(target_pos):
                target_vel = list(point.velocities[: len(target_pos)])
            else:
                target_vel = [0.0] * len(target_pos)

            duration = t_sec - t_prev_sec
            num_steps = max(1, int(duration / SIM_DT))
            t_prev_sec = t_sec
            for _ in range(num_steps):
                for k, jidx in enumerate(joint_indices):
                    if k < len(target_pos):
                        p.setJointMotorControl2(
                            self._robot_id,
                            jidx,
                            p.POSITION_CONTROL,
                            targetPosition=target_pos[k],
                            targetVelocity=target_vel[k] if k < len(target_vel) else 0.0,
                            positionGain=self._kp,
                            velocityGain=self._kd,
                        )
                p.stepSimulation(physicsClientId=self._client)
                rclpy.spin_once(self, timeout_sec=0)

        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return result

    def destroy_node(self, *args, **kwargs):
        if p is not None and self._client is not None:
            p.disconnect(physicsClientId=self._client)
        super().destroy_node(*args, **kwargs)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = PyBulletFollowJointTrajectoryServer()
        rclpy.spin(node)
    except Exception as e:
        import traceback
        traceback.print_exc()
    finally:
        if rclpy.ok():
            try:
                node.destroy_node()
            except NameError:
                pass
            rclpy.shutdown()


if __name__ == "__main__":
    main()
