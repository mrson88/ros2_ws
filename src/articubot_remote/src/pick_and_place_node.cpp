#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("pick_and_place_node");

    // Setup
    static const std::string PLANNING_GROUP = "arm_robot";
    moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);
    // Define target pose for pick operation
    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.pose.orientation.w = 1.0;
    target_pose.pose.position.x = 0.4;  // Example values, adjust according to your setup
    target_pose.pose.position.y = 0.0;
    target_pose.pose.position.z = 0.5;
    move_group.setPoseTarget(target_pose);

    // Plan and execute
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if(success) {
        move_group.move();
    }

    rclcpp::shutdown();
    return 0;
}
