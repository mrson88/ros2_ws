

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include "arduinobot_msgs/action/arduinobot_task.hpp"
#include <moveit/move_group_interface/move_group_interface.h>

#include <memory>


using namespace std::placeholders;

namespace articubot_remote
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
    auto arm_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "arm_robot");
    auto gripper_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "gripper");
    arm_move_group.stop();
    gripper_move_group.stop();
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  tf2::Transform poseMsgToTransform(const geometry_msgs::msg::Pose& pose_in){
    
    // transform pose message from geometry_msgs::msg::Pose to tf2::Transform
    
    tf2::Transform t_out; 
    tf2::Vector3 pos(pose_in.position.x, pose_in.position.y, pose_in.position.z); 
    t_out.setOrigin(pos);  
    tf2::Quaternion q;
    tf2::fromMsg(pose_in.orientation, q);
    t_out.setRotation(q);

    return t_out; 
  }
 
  void acceptedCallback(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<arduinobot_msgs::action::ArduinobotTask>> goal_handle)
  {
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{ std::bind(&TaskServer::execute, this, _1), goal_handle }.detach();
  }

  void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<arduinobot_msgs::action::ArduinobotTask>> goal_handle)
  {
    RCLCPP_INFO(get_logger(), "Executing goal");
    auto result = std::make_shared<arduinobot_msgs::action::ArduinobotTask::Result>();

    // MoveIt 2 Interface
    auto arm_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "arm_robot");
    auto gripper_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "gripper");

    std::vector<double> arm_joint_goal;
    std::vector<double> gripper_joint_goal;

    if (goal_handle->get_goal()->task_number == 0)
    {
      arm_joint_goal = {0.0, -1.2, -1.2,-1.2, 0.0};
      gripper_joint_goal = {-0.7, 0.7};
    }
    else if (goal_handle->get_goal()->task_number == 1) 
    {
      arm_joint_goal = {0.0, 0.0, 0.0, 0.0, 0.0};
      gripper_joint_goal = {-0.0, 0.0};
    }
    else if (goal_handle->get_goal()->task_number == 2)
    {
      arm_joint_goal = {0.5, -1.2, -1.2,-1.2, 0.5};
      gripper_joint_goal = {-0.5, 0.5};
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Invalid Task Number");
      return;
    }

    bool arm_within_bounds = arm_move_group.setJointValueTarget(arm_joint_goal);
    bool gripper_within_bounds = gripper_move_group.setJointValueTarget(gripper_joint_goal);
    if (!arm_within_bounds | !gripper_within_bounds)
    {
      RCLCPP_WARN(get_logger(),
                  "Target joint position(s) were outside of limits, but we will plan and clamp to the limits ");
      return;
    }

    moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
    moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
    bool arm_plan_success = (arm_move_group.plan(arm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    bool gripper_plan_success = (gripper_move_group.plan(gripper_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    
    if(arm_plan_success && gripper_plan_success)
    {
      RCLCPP_INFO(get_logger(), "Planner SUCCEED, moving the arme and the gripper");
      arm_move_group.move();
      gripper_move_group.move();
      geometry_msgs::msg::PoseStamped end_effector_pose = arm_move_group.getCurrentPose();
      geometry_msgs::msg::PoseStamped end_gripper_pose = gripper_move_group.getCurrentPose();
            RCLCPP_INFO(get_logger(), "End-arm position: [%f, %f, %f,%f, %f, %f,%f]", 
                  end_effector_pose.pose.position.x,
                  end_effector_pose.pose.position.y,
                  end_effector_pose.pose.position.z,
                  end_effector_pose.pose.orientation.x,
                  end_effector_pose.pose.orientation.y,
                  end_effector_pose.pose.orientation.z,
                  end_effector_pose.pose.orientation.w);
            RCLCPP_INFO(get_logger(), "End-gripper position: [%f, %f, %f,%f, %f, %f,%f]", 
                  end_gripper_pose.pose.position.x,
                  end_gripper_pose.pose.position.y,
                  end_gripper_pose.pose.position.z,
                  end_gripper_pose.pose.orientation.x,
                  end_gripper_pose.pose.orientation.y,
                  end_gripper_pose.pose.orientation.z,
                  end_gripper_pose.pose.orientation.w);
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
}  // namespace articubot_remote

RCLCPP_COMPONENTS_REGISTER_NODE(articubot_remote::TaskServer)




// #define BOOST_BIND_NO_PLACEHOLDERS
// #include <math.h>
// #include <inttypes.h>
// #include <functional>
// #include <memory>
// #include <thread>
// #include <iostream>



// #include "control_msgs/action/gripper_command.hpp"
// #include <geometry_msgs/msg/pose_stamped.hpp>
// #include <sensor_msgs/msg/joint_state.hpp>
// #include <geometry_msgs/msg/pose.hpp>
// #include <moveit/moveit_cpp/moveit_cpp.h>
// #include <moveit/moveit_cpp/planning_component.h>

// #include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit_msgs/msg/planning_scene.hpp>

// #include <moveit/planning_scene_monitor/planning_scene_monitor.h>
// #include <moveit/robot_model_loader/robot_model_loader.h>
// #include <moveit/robot_state/conversions.h>
// #include <moveit/kinematic_constraints/utils.h>

// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// #include "arduinobot_msgs/action/move.hpp"
// #include "arduinobot_msgs/action/pnp.hpp"




// #include <geometry_msgs/msg/pose.hpp>
// #include <rclcpp/rclcpp.hpp>
// #include <rclcpp_action/rclcpp_action.hpp>
// #include <rclcpp_components/register_node_macro.hpp>
// #include "arduinobot_msgs/action/arduinobot_task.hpp"
// #include <moveit/move_group_interface/move_group_interface.h>
// #include <Eigen/Geometry>
// #include <memory>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// using namespace std::placeholders;

// namespace articubot_remote
// {
// class TaskServer : public rclcpp::Node
// {
// public:
//   explicit TaskServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
//     : Node("task_server", options)
//   {
//     RCLCPP_INFO(get_logger(), "Starting the Server");
//     action_server_ = rclcpp_action::create_server<arduinobot_msgs::action::ArduinobotTask>(
//         this, "task_server", std::bind(&TaskServer::goalCallback, this, _1, _2),
//         std::bind(&TaskServer::cancelCallback, this, _1),
//         std::bind(&TaskServer::acceptedCallback, this, _1));
//   }

// private:
//   rclcpp_action::Server<arduinobot_msgs::action::ArduinobotTask>::SharedPtr action_server_;

//   rclcpp_action::GoalResponse goalCallback(
//       const rclcpp_action::GoalUUID& uuid,
//       std::shared_ptr<const arduinobot_msgs::action::ArduinobotTask::Goal> goal)
//   {
//     RCLCPP_INFO(get_logger(), "Received goal request with id %d", goal->task_number);
//     (void)uuid;
//     return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
//   }

//   rclcpp_action::CancelResponse cancelCallback(
//       const std::shared_ptr<rclcpp_action::ServerGoalHandle<arduinobot_msgs::action::ArduinobotTask>> goal_handle)
//   {
//     (void)goal_handle;
//     RCLCPP_INFO(get_logger(), "Received request to cancel goal");
//     auto arm_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "arm_robot");
//     auto gripper_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "gripper");
//     // auto effector_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "end_effector");
//     arm_move_group.stop();
//     gripper_move_group.stop();
//     // effector_move_group.stop();
//     return rclcpp_action::CancelResponse::ACCEPT;
//   }
//   // functions for pick and place application 

//   tf2::Transform poseMsgToTransform(const geometry_msgs::msg::Pose& pose_in){
    
//     // transform pose message from geometry_msgs::msg::Pose to tf2::Transform
    
//     tf2::Transform t_out; 
//     tf2::Vector3 pos(pose_in.position.x, pose_in.position.y, pose_in.position.z); 
//     t_out.setOrigin(pos);  
//     tf2::Quaternion q;
//     tf2::fromMsg(pose_in.orientation, q);
//     t_out.setRotation(q);

//     return t_out; 
//   }
//   void acceptedCallback(
//       const std::shared_ptr<rclcpp_action::ServerGoalHandle<arduinobot_msgs::action::ArduinobotTask>> goal_handle)
//   {
//     // this needs to return quickly to avoid blocking the executor, so spin up a new thread
//     std::thread{ std::bind(&TaskServer::execute, this, _1), goal_handle }.detach();
//   }

//   void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<arduinobot_msgs::action::ArduinobotTask>> goal_handle)
//   {
//     RCLCPP_INFO(get_logger(), "Executing goal");
//     auto result = std::make_shared<arduinobot_msgs::action::ArduinobotTask::Result>();

//     // // MoveIt 2 Interface
//     auto arm_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "arm_robot");
//     auto gripper_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "gripper");
//     // auto effector_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "end_effector");
//     std::vector<double> arm_joint_goal;
//     std::vector<double> gripper_joint_goal;
//     geometry_msgs::msg::PoseStamped arm_pose_target;
//     geometry_msgs::msg::PoseStamped gripper_pose_target;
//     RCLCPP_INFO(get_logger(), "Reference frame: %s", arm_move_group.getPlanningFrame().c_str());
//     RCLCPP_INFO(get_logger(), "End effector link: %s", arm_move_group.getEndEffectorLink().c_str());
//     // geometry_msgs::msg::PoseStamped effector_pose_target;
//     if (goal_handle->get_goal()->task_number == 0)
//     {
//       arm_pose_target.header.frame_id = "hand_3_link";
//       tf2::Quaternion orientation;
//       orientation.setRPY(0, 0, M_PI / 2);
//       arm_pose_target.pose.orientation = tf2::toMsg(orientation);
//       // effector_pose_target.pose.position.x = 0.5;
//       // effector_pose_target.pose.position.y = 0.0;
//       // effector_pose_target.pose.position.z = 0.5;
//       arm_pose_target.pose.position.x = 0.3;
//       arm_pose_target.pose.position.y = 0.4;
//       arm_pose_target.pose.position.z = 0.75;
//       // arm_pose_target.pose.orientation.w = 1.0;
//       // effector_pose_target.pose.orientation.x = 0.464625;
//       // effector_pose_target.pose.orientation.y = -0.000000;
//       // effector_pose_target.pose.orientation.z = 0.724309;
//       // effector_pose_target.pose.orientation.w = 1.0;


//     }
//     else if (goal_handle->get_goal()->task_number == 1) 
//     {
//       arm_joint_goal = {0.0, 0.0, 0.0, 0.0, 0.0};
//       gripper_joint_goal = {-0.0, 0.0};
//     }
//     else if (goal_handle->get_goal()->task_number == 2)
//     {
//       arm_joint_goal = {0.5, -1.2, -1.2,-1.2, 0.5};
//       gripper_joint_goal = {-0.5, 0.5};
//     }
//     else
//     {
//       RCLCPP_ERROR(get_logger(), "Invalid Task Number");
//       return;
//     }

//     // bool arm_within_bounds = arm_move_group.setJointValueTarget(arm_joint_goal);
//     // bool gripper_within_bounds = gripper_move_group.setJointValueTarget(gripper_joint_goal);
//     // bool effector_within_bounds = effector_move_group.setPoseTarget(effector_pose_target);
//     bool arm_within_bounds = arm_move_group.setPoseTarget(arm_pose_target);
//     if (!arm_within_bounds )
//     {
//       RCLCPP_WARN(get_logger(),
//                   "Target joint position(s) were outside of limits, but we will plan and clamp to the limits ");
//       return;
//     }
//     // moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
//     // moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
//     moveit::planning_interface::MoveGroupInterface::Plan effector_plan;
//     bool effector_plan_success = (arm_move_group.plan(effector_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//     RCLCPP_INFO(get_logger(), "Visualizing plan 1 (pose goal) %s", effector_plan_success ? "" : "FAILED");

//     // bool gripper_plan_success = (gripper_move_group.plan(gripper_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//     // bool arm_plan_success = (arm_move_group.plan(effector_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//     if(effector_plan_success )
//     {
//       RCLCPP_INFO(get_logger(), "Planner SUCCEED, moving the arme and the gripper");
//       arm_move_group.execute(effector_plan);
//       arm_move_group.move();
//       result->success = true;
//       goal_handle->succeed(result);
//       // gripper_move_group.move();
//       // arm_move_group.move();
//       geometry_msgs::msg::PoseStamped end_effector_pose = arm_move_group.getCurrentPose();
//       // geometry_msgs::msg::PoseStamped end_gripper_pose = gripper_move_group.getCurrentPose();
//             RCLCPP_INFO(get_logger(), "End-arm position: [%f, %f, %f,%f, %f, %f,%f]", 
//                   end_effector_pose.pose.position.x,
//                   end_effector_pose.pose.position.y,
//                   end_effector_pose.pose.position.z,
//                   end_effector_pose.pose.orientation.x,
//                   end_effector_pose.pose.orientation.y,
//                   end_effector_pose.pose.orientation.z,
//                   end_effector_pose.pose.orientation.w);
//     }
//     else
//     {
//       RCLCPP_ERROR(get_logger(), "One or more planners failed!");
//       return;
//     }
//     result->success = true;
//     goal_handle->succeed(result);
//     RCLCPP_INFO(get_logger(), "Goal succeeded");
//   }
// };
// }  // namespace articubot_remote

// RCLCPP_COMPONENTS_REGISTER_NODE(articubot_remote::TaskServer)
