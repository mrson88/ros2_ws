#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "arduinobot_msgs/action/move_to_point.hpp" // Adjust this include path
#include "moveit/move_group_interface/move_group_interface.h"

class MoveToPointActionServer : public rclcpp::Node
{
public:
  MoveToPointActionServer() : Node("move_to_point_action_server")
  {
    using namespace std::placeholders;
    this->action_server_ = rclcpp_action::create_server<arduinobot_msgs::action::MoveToPoint>(
        this,
        "move_to_point",
        std::bind(&MoveToPointActionServer::handle_goal, this, _1, _2),
        std::bind(&MoveToPointActionServer::handle_cancel, this, _1),
        std::bind(&MoveToPointActionServer::handle_accepted, this, _1));
  }

private:
  rclcpp_action::Server<arduinobot_msgs::action::MoveToPoint>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const arduinobot_msgs::action::MoveToPoint::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request");
    (void)uuid;
    // You can reject a goal here
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<arduinobot_msgs::action::MoveToPoint>> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<arduinobot_msgs::action::MoveToPoint>> goal_handle)
  {
    using namespace std::placeholders;
    // This needs to run in a new thread to avoid blocking the executor
    std::thread{std::bind(&MoveToPointActionServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<arduinobot_msgs::action::MoveToPoint>> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<arduinobot_msgs::action::MoveToPoint::Feedback>();
    auto &distance_to_goal = feedback->distance_to_goal;
    auto result = std::make_shared<arduinobot_msgs::action::MoveToPoint::Result>();

    // Initialize MoveIt! here and move the robot arm to the goal->target_point

    // For simplicity, we assume success
    result->success = true;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveToPointActionServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
