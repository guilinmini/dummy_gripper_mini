#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "hello_moveit",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  auto const logger = rclcpp::get_logger("hello_moveit");

  using moveit::planning_interface::MoveGroupInterface;
  // auto move_group_interface = MoveGroupInterface(node, "dummy2_arm");
  // auto hand_group = MoveGroupInterface(node, "dummy_hand");
  auto move_group_interface = MoveGroupInterface(node, "dummy2_arm");
  auto hand_group = MoveGroupInterface(node, "hand");

  // 获取规划组的基本信息
  RCLCPP_INFO(logger, "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());      // 参考坐标系
  RCLCPP_INFO(logger, "End effector link: %s", move_group_interface.getEndEffectorLink().c_str()); // 末端执行器位置

  // Set a target Pose
  auto const target_pose = []
  {
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = 1.0;
    msg.position.x = 0.5;
    msg.position.y = 0.0;
    msg.position.z = 0.0;
    return msg;
  }();
  move_group_interface.setPoseTarget(target_pose);

  // Create a plan to that target pose
  auto const [success, plan] = [&move_group_interface]
  {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if (success)
  {
    move_group_interface.execute(plan);
  }
  else
  {
    RCLCPP_ERROR(logger, "arm Planing failed!");
  }

  // 2. 控制夹爪
  hand_group.setNamedTarget("close"); // MoveIt配置中定义的"open"姿态

  // 或者直接设置关节值（示例值，需根据实际夹爪调整）
  // std::vector<double> hand_open_positions = {0.0, 0.0}; // 假设夹爪有2个关节
  // hand_group.setJointValueTarget(hand_open_positions);

  // 执行夹爪动作
  auto const [hand_success, hand_plan] = [&hand_group]
  {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    bool ok = static_cast<bool>(hand_group.plan(msg));
    return std::make_pair(ok, msg);
  }();

  if (hand_success)
  {
    hand_group.execute(hand_plan);
    RCLCPP_INFO(logger, "Gripper successfully");
  }
  else
  {
    RCLCPP_ERROR(logger, "Gripper failed");
  }
  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}