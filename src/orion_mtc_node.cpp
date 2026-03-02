/*
 * Orion MTC 缆绳抓取任务节点
 * 流程: CurrentState -> OpenGripper -> MoveTo pregrasp -> MoveRelative 接近 -> AllowCollision -> CloseGripper -> Attach -> Lift
 */

#include <chrono>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>

#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif

static const rclcpp::Logger LOGGER = rclcpp::get_logger("orion_mtc_node");
namespace mtc = moveit::task_constructor;

/* 缆绳 CollisionObject ID，与 setupPlanningScene 一致 */
const std::string CABLE_OBJECT_ID = "cable";

class OrionMTCTaskNode
{
public:
  explicit OrionMTCTaskNode(const rclcpp::NodeOptions& options);
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();
  void doTask();
  void setupPlanningScene();

private:
  mtc::Task createTask();
  rclcpp::Node::SharedPtr node_;
  mtc::Task task_;
};

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr OrionMTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

OrionMTCTaskNode::OrionMTCTaskNode(const rclcpp::NodeOptions& options)
  : node_(std::make_shared<rclcpp::Node>("orion_mtc_node", options))
{
}

void OrionMTCTaskNode::setupPlanningScene()
{
  moveit_msgs::msg::CollisionObject object;
  object.id = CABLE_OBJECT_ID;
  object.header.frame_id = "world";
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  /* 半径约缆绳粗细(m), 高度为可抓取段长度(m) */
  object.primitives[0].dimensions = { 0.02f, 0.08f };

  /* 缆绳圆柱位姿：放在机械臂正前方、远离臂体，避免与 home/ready 碰撞；从 ready 用 -Z 接近 */
  geometry_msgs::msg::Pose pose;
  pose.position.x = 1.5;
  pose.position.y = 0.3;
  pose.position.z = 0.58;
  pose.orientation.w = 1.0;
  object.pose = pose;

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(object);
  RCLCPP_INFO(LOGGER, "Planning scene: added cable collision object '%s'", CABLE_OBJECT_ID.c_str());
  /* 等待 planning scene 同步后再启动任务 */
  std::this_thread::sleep_for(std::chrono::seconds(1));
}

void OrionMTCTaskNode::doTask()
{
  task_ = createTask();

  try
  {
    task_.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }

  if (!task_.plan(10))
  {
    RCLCPP_ERROR(LOGGER, "MTC task planning failed");
    return;
  }

  task_.introspection().publishSolution(*task_.solutions().front());

  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR(LOGGER, "MTC task execution failed: %d", result.val);
    return;
  }
  RCLCPP_INFO(LOGGER, "MTC cable grasp task completed");
}

mtc::Task OrionMTCTaskNode::createTask()
{
  mtc::Task task;
  task.stages()->setName("cable grasp");
  task.loadRobotModel(node_);

  const std::string arm_group_name = "manipulator";
  const std::string hand_group_name = "gripper";
  const std::string ik_frame = "Link6";

  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", ik_frame);

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(0.01);

  /* 1. CurrentState */
  task.add(std::make_unique<mtc::stages::CurrentState>("current state"));

  /* 2. Open gripper */
  auto stage_open = std::make_unique<mtc::stages::MoveTo>("open gripper", interpolation_planner);
  stage_open->setGroup(hand_group_name);
  stage_open->setGoal("open");
  task.add(std::move(stage_open));

  /* 3. Connect：从 open 到抓取序列起点（规划臂到 ready 等） */
  auto connect_grasp = std::make_unique<mtc::stages::Connect>(
      "connect to grasp",
      mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
  connect_grasp->setTimeout(5.0);
  connect_grasp->properties().configureInitFrom(mtc::Stage::PARENT);
  task.add(std::move(connect_grasp));

  /* 5–11. 抓取序列：pregrasp -> approach -> allow -> close -> attach -> lift（SerialContainer） */
  auto grasp = std::make_unique<mtc::SerialContainer>("grasp sequence");
  task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
  grasp->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

  /* 5. MoveTo pregrasp (ready) */
  auto stage_pregrasp = std::make_unique<mtc::stages::MoveTo>("pregrasp", sampling_planner);
  stage_pregrasp->setGroup(arm_group_name);
  stage_pregrasp->setGoal("ready");
  grasp->insert(std::move(stage_pregrasp));

  /* 6. MoveRelative 直线接近 5–10 cm */
  auto stage_approach = std::make_unique<mtc::stages::MoveRelative>("approach cable", cartesian_planner);
  stage_approach->properties().set("marker_ns", "approach_cable");
  stage_approach->properties().set("link", ik_frame);
  stage_approach->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  stage_approach->setMinMaxDistance(0.05, 0.10);
  geometry_msgs::msg::Vector3Stamped vec;
  vec.header.frame_id = ik_frame;
  vec.vector.z = -1.0;
  stage_approach->setDirection(vec);
  grasp->insert(std::move(stage_approach));

  /* 7. Allow collision (gripper, cable) */
  auto stage_allow = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (gripper,cable)");
  stage_allow->allowCollisions(
      CABLE_OBJECT_ID,
      task.getRobotModel()->getJointModelGroup(hand_group_name)->getLinkModelNamesWithCollisionGeometry(),
      true);
  grasp->insert(std::move(stage_allow));

  /* 8. CloseGripper */
  auto stage_close = std::make_unique<mtc::stages::MoveTo>("close gripper", interpolation_planner);
  stage_close->setGroup(hand_group_name);
  stage_close->setGoal("close");
  grasp->insert(std::move(stage_close));

  /* 9. Attach cable to Link7。若出现 "Attached body 'cable' not found"，可暂时注释本段先验证运动流程 */
  auto stage_attach = std::make_unique<mtc::stages::ModifyPlanningScene>("attach cable");
  stage_attach->attachObject(CABLE_OBJECT_ID, "Link7");
  grasp->insert(std::move(stage_attach));

  /* 10. Lift：关节空间 MoveTo "up" 抬起（attach 后用关节空间避免笛卡尔附着体查找问题） */
  auto stage_lift = std::make_unique<mtc::stages::MoveTo>("lift", sampling_planner);
  stage_lift->setGroup(arm_group_name);
  stage_lift->setGoal("up");
  grasp->insert(std::move(stage_lift));

  task.add(std::move(grasp));

  /* 末尾加 Generator，使任务根看到 (→) 接口，满足 MTC 对最后一阶段的要求 */
  task.add(std::make_unique<mtc::stages::CurrentState>("final state"));

  return task;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<OrionMTCTaskNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &node]() {
    executor.add_node(node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(node->getNodeBaseInterface());
  });

  node->setupPlanningScene();
  node->doTask();

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}
