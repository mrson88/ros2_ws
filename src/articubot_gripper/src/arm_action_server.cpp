#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "control_msgs/action/gripper_command.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "arduinobot_msgs/action/home.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#define PI 3.1415926535

namespace arm_robot{

class ArmActionServer : public rclcpp::Node
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandleGripperCommand = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

  using Home = arduinobot_msgs::action::Home;
  using GoalHandleHome = rclcpp_action::ServerGoalHandle<Home>;


  explicit ArmActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("gripper_controller", options)
  {
    using namespace std::placeholders;
    RCLCPP_INFO(get_logger(), "Starting the Gripper Server");
    this->action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
      this,
      "~/gripper_action",
      std::bind(&ArmActionServer::handle_goal, this, _1, _2),
      std::bind(&ArmActionServer::handle_cancel, this, _1),
      std::bind(&ArmActionServer::handle_accepted, this, _1));

    this->gripper_homing_server_ = rclcpp_action::create_server<Home>(
      this, 
      "~/home", 
      std::bind(&ArmActionServer::handle_home_goal, this, _1, _2),
      std::bind(&ArmActionServer::handle_home_cancel, this, _1),
      std::bind(&ArmActionServer::handle_home_accepted, this, _1));

    this->gripper_command_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/gripper_controller/commands", 10);

    
  }

private:
  rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;
  rclcpp_action::Server<Home>::SharedPtr gripper_homing_server_;
  float gripper_angle_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gripper_command_publisher_;
 

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const FollowJointTrajectory::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with secs ");
    (void)uuid;
    
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::GoalResponse handle_home_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Home::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with secs %f", goal->position);
    (void)uuid;
    
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleGripperCommand> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  
  rclcpp_action::CancelResponse handle_home_cancel(
    const std::shared_ptr<GoalHandleHome> home_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cangel home goal"); 
    (void)home_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleGripperCommand> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&ArmActionServer::execute_grasp, this, _1), goal_handle}.detach();
  }

  void handle_home_accepted(const std::shared_ptr<GoalHandleHome> home_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&ArmActionServer::execute_homing, this, _1), home_handle}.detach();
  }

  void gripperControl(float gripper_position, float &gripper_angle){

    float max_gripper_position = 0.08;
    float min_gripper_position = 0.01; 
    float pivot_length = 0.04; 
    int offset_angle = 8; 

    if(gripper_position > max_gripper_position){

      gripper_position = max_gripper_position;

    }

    if(gripper_position < min_gripper_position){
        
      gripper_position = min_gripper_position;
        
    }

    float gripper_position_half = gripper_position/2;

    gripper_angle = 90*(M_PI/180) - (offset_angle * (M_PI/180) + asin(gripper_position_half/pivot_length)); // * (180/M_PI);

  }


  void execute_grasp(const std::shared_ptr<GoalHandleGripperCommand> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing grasp goal");  
    
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<FollowJointTrajectory::Result>();

    // Check if there is a cancel request
    if (goal_handle->is_canceling()) {
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Grasp goal canceled");
      return;
    }

    // Move gripper 
    
    auto gripper_command_message = std_msgs::msg::Float64MultiArray();

    gripperControl(goal->command.position, gripper_angle_);

    if(std::isnan( gripper_angle_ )){
      
      gripper_angle_ = 0.0;

    }
    
    gripper_command_message.data.push_back(gripper_angle_);

    gripper_command_publisher_->publish(gripper_command_message);
    
    if (rclcpp::ok()) {
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Grasp goal succeeded");
    }
  }

  void execute_homing(const std::shared_ptr<GoalHandleHome> home_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing homing goal");  
    
    auto result = std::make_shared<Home::Result>();

    // Check if there is a cancel request
    if (home_handle->is_canceling()) {
      home_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Homing goal canceled");
      return;
    }

    // Home gripper
    
    auto gripper_command_message = std_msgs::msg::Float64MultiArray(); 
    gripper_command_message.data.push_back(0.0);

    gripper_command_publisher_->publish(gripper_command_message);
    
    if (rclcpp::ok()) {
      home_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Homing goal succeeded");
    }
  }

};  // class ArmActionServer

} // namespace innobot_gripper

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto action_server1 = std::make_shared<arm_robot::ArmActionServer>();

  rclcpp::spin(action_server1);

  rclcpp::shutdown();
  return 0;
}

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandle = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

class GripperActionServer : public rclcpp::Node {
public:
    explicit GripperActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("gripper_action_server", options),
      action_server_(rclcpp_action::create_server<FollowJointTrajectory>(
          this,
          "gripper_control",
          std::bind(&GripperActionServer::handle_goal, this, _1, _2),
          std::bind(&GripperActionServer::handle_cancel, this, _1),
          std::bind(&GripperActionServer::handle_accepted, this, _1))) {
    }

private:
    rclcpp_action::GoalResponse handle_accepted(const std::shared_ptr<GoalHandle> goal_handle) {
        RCLCPP_INFO(get_logger(), "Goal accepted");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    void handle_cancel(const std::shared_ptr<GoalHandle> goal_handle) {
        RCLCPP_INFO(get_logger(), "Goal canceled");
        action_server_->cancel_goal(goal_handle);
    }

    void handle_goal(const std::shared_ptr<GoalHandle> goal_handle, const std::shared_ptr<const FollowJointTrajectory::Goal> goal) {
        RCLCPP_INFO(get_logger(), "Received gripper trajectory goal");

        auto result = std::make_shared<FollowJointTrajectory::Result>();
        
        // Extracting trajectory
        const auto &trajectory = goal->trajectory;

        // Assuming one joint in the gripper
        if (trajectory.joint_names.size() != 1) {
            RCLCPP_ERROR(get_logger(), "Unexpected number of joints in trajectory");
            result->error_code = result->INVALID_GOAL;
            goal_handle->abort(result);
            return;
        }

        // Assuming a simple gripper with only one joint
        if (trajectory.points.size() != 1) {
            RCLCPP_ERROR(get_logger(), "Unexpected number of trajectory points");
            result->error_code = result->INVALID_GOAL;
            goal_handle->abort(result);
            return;
        }

        // Assuming linear interpolation between start and goal position
        auto point = trajectory.points.front();
        auto goal_position = point.positions.front();
        RCLCPP_INFO(get_logger(), "Executing gripper trajectory to position: %f", goal_position);

        // Execute the trajectory (you should implement this)
        execute_trajectory(goal_position);

        // Mark the goal as succeeded
        result->error_code = result->SUCCESSFUL;
        goal_handle->succeed(result);
    }

    void execute_trajectory(double position) {
        // Execute the gripper trajectory (replace this with your code)
        // You would typically send commands to your gripper hardware here
        RCLCPP_INFO(get_logger(), "Executing gripper trajectory to position: %f", position);
    }

    rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GripperActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
