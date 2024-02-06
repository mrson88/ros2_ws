#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include "arduinobot_msgs/action/arduinobot_task.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit/macros/console_colors.h>


using namespace std::placeholders;

namespace arduinobot_remote
{
class TaskServer : public rclcpp::Node
{
public:
  explicit TaskServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("task_server", options)
  {
    RCLCPP_INFO(get_logger(), "Starting the Server");
    action_server_ = rclcpp_action::create_server<arduinobot_msgs::action::ArduinobotTask>(
        this, "task_server", std::bind(&TaskServer::goalCallback, this, _1, _2),
        std::bind(&TaskServer::cancelCallback, this, _1),
        std::bind(&TaskServer::acceptedCallback, this, _1));
  }

private:
  rclcpp_action::Server<arduinobot_msgs::action::ArduinobotTask>::SharedPtr action_server_;

  rclcpp_action::GoalResponse goalCallback(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const arduinobot_msgs::action::ArduinobotTask::Goal> goal)
  {
    RCLCPP_INFO(get_logger(), "Received goal request with id %d", goal->task_number);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse cancelCallback(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<arduinobot_msgs::action::ArduinobotTask>> goal_handle)
  {
    (void)goal_handle;
    RCLCPP_INFO(get_logger(), "Received request to cancel goal");
    auto arm_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "/robot_description");
    // auto gripper_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "gripper");
    arm_move_group.stop();
    // gripper_move_group.stop();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void acceptedCallback(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<arduinobot_msgs::action::ArduinobotTask>> goal_handle)
  {
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{ std::bind(&TaskServer::execute, this, _1), goal_handle }.detach();
  }

  void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<arduinobot_msgs::action::ArduinobotTask>> goal_handle)
  {

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = rclcpp::Node::make_shared("/robot_description", node_options);
    // For current state monitor
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);
    std::thread([&executor]() { executor.spin(); }).detach();

    static const std::string PLANNING_GROUP = "arm_robot";

    // moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);

    // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;


    // const moveit::core::JointModelGroup* joint_model_group =
    // move_group.getRobotModel()->getJointModelGroup(PLANNING_GROUP);

    // // We can print the name of the reference frame for this robot.
    // RCLCPP_INFO(get_logger(), "Planning frame: %s", move_group.getPlanningFrame().c_str());

    // // We can also print the name of the end-effector link for this group.
    // RCLCPP_INFO(get_logger(), "End effector link: %s", move_group.getEndEffectorLink().c_str());

    // // We can get a list of all the groups in the robot:
    // RCLCPP_INFO(get_logger(), "Available Planning Groups:");
    // std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
    //           std::ostream_iterator<std::string>(std::cout, ", "));

    // Planning to a Pose goal
    // geometry_msgs::msg::Pose target_pose1;
    // target_pose1.orientation.w = 1.0;
    // target_pose1.position.x = 0.28;
    // target_pose1.position.y = -0.2;
    // target_pose1.position.z = 0.5;
    // move_group.setPoseTarget(target_pose1);
    //Setup MoveIt interface for planning

    // RCLCPP_INFO(get_logger(), "Executing goal");
    auto result = std::make_shared<arduinobot_msgs::action::ArduinobotTask::Result>();
    RCLCPP_INFO(get_logger(), "Executing goal 1.6");
    // MoveIt 2 Interface

    RCLCPP_INFO(get_logger(), "Executing goal 1");
    auto arm_move_group = moveit::planning_interface::MoveGroupInterface(move_group_node, "/robot_description");
    // auto gripper_move_group = moveit::planning_interface::MoveGroupInterface(move_group_node, "gripper_controller");
    RCLCPP_INFO(get_logger(), "Executing goal 2");
    std::vector<double> arm_joint_goal;
    // std::vector<double> gripper_joint_goal;

    if (goal_handle->get_goal()->task_number == 0)
    {
      arm_joint_goal = {1.0, 0.6, 0.0};
      // gripper_joint_goal = {-0.7,0.7};
    }
    else if (goal_handle->get_goal()->task_number == 1)
    {
      arm_joint_goal = {-1.14, -0.6, -0.07};
      // gripper_joint_goal = {-0.7,0.7};
    }
    else if (goal_handle->get_goal()->task_number == 2)
    {
      arm_joint_goal = {-1.57,0.0,-0.9};
      // gripper_joint_goal = {-0.7,0.7};
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Invalid Task Number");
      return;
    }

    bool arm_within_bounds = arm_move_group.setJointValueTarget(arm_joint_goal);
    // bool gripper_within_bounds = gripper_move_group.setJointValueTarget(gripper_joint_goal);
    // if (!arm_within_bounds | !gripper_within_bounds)
    if (!arm_within_bounds )
    {
      RCLCPP_WARN(get_logger(),
                  "Target joint position(s) were outside of limits, but we will plan and clamp to the limits ");
      return;
    }

    moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
    moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
    bool arm_plan_success = (arm_move_group.plan(arm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    // bool gripper_plan_success = (gripper_move_group.plan(gripper_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    
    // if(arm_plan_success && gripper_plan_success)
    if(arm_plan_success )
    {
      RCLCPP_INFO(get_logger(), "Planner SUCCEED, moving the arme and the gripper");
      arm_move_group.move();
      // gripper_move_group.move();
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "One or more planners failed!");
      return;
    }
  
    result->success = true;
    goal_handle->succeed(result);
    RCLCPP_INFO(get_logger(), "Goal succeeded");
  }
};
}  // namespace arduinobot_remote

RCLCPP_COMPONENTS_REGISTER_NODE(arduinobot_remote::TaskServer)