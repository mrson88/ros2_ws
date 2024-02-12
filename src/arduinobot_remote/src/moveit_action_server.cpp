#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit_msgs/action/move_group.hpp"

using namespace std::placeholders;

class MoveRobotActionServer : public rclcpp::Node
{
public:
    using MoveGroup = moveit::planning_interface::MoveGroupInterface;
    using GoalHandleMoveRobot = rclcpp_action::ServerGoalHandle<moveit_msgs::action::MoveGroup>;

    MoveRobotActionServer()
        : Node("move_robot_action_server"),
          action_server_(rclcpp_action::create_server<moveit_msgs::action::MoveGroup>(
              this, "move_robot", std::bind(&MoveRobotActionServer::handle_goal, this, _1, _2),
              std::bind(&MoveRobotActionServer::handle_cancel, this, _1),
              std::bind(&MoveRobotActionServer::handle_accepted, this, _1)))
    {
        move_group_ = std::make_shared<MoveGroup>("arm"); // Specify the planning group name
    }

private:
    rclcpp_action::Server<moveit_msgs::action::MoveGroup>::SharedPtr action_server_;
    std::shared_ptr<MoveGroup> move_group_;

    void handle_goal(const std::shared_ptr<GoalHandleMoveRobot> goal_handle)
    {
        auto goal = goal_handle->get_goal();

        move_group_->setPoseTarget(goal->target_pose);
        moveit::planning_interface::MoveItErrorCode error_code = move_group_->move();

        if (error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            goal_handle->succeed(std::make_shared<moveit_msgs::action::MoveGroup::Result>());
        }
        else
        {
            goal_handle->abort();
        }
    }

    void handle_cancel(const std::shared_ptr<GoalHandleMoveRobot> goal_handle)
    {
        RCLCPP_INFO(get_logger(), "Received request to cancel goal");
        goal_handle->canceled(std::make_shared<moveit_msgs::action::MoveGroup::Result>());
    }

    void handle_accepted(const std::shared_ptr<GoalHandleMoveRobot> goal_handle)
    {
        (void)goal_handle;
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveRobotActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
