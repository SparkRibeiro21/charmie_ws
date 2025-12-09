#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("test_moveit_node");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() { executor.spin(); });

    auto arm = moveit::planning_interface::MoveGroupInterface(node, "xarm6");
    arm.setMaxVelocityScalingFactor(0.5);
    arm.setMaxAccelerationScalingFactor(0.5);


    // Named goal
    arm.setStartStateToCurrentState();
    arm.setNamedTarget("home");

    moveit::planning_interface::MoveGroupInterface::Plan plan1;
    bool success1 = (arm.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);
    if (success1)
    {
        arm.execute(plan1);
    }

    arm.setStartStateToCurrentState();
    arm.setNamedTarget("pick_front");

    moveit::planning_interface::MoveGroupInterface::Plan plan2;
    bool success2 = (arm.plan(plan2) == moveit::core::MoveItErrorCode::SUCCESS);
    if (success2)
    {
        arm.execute(plan2);
    }

    rclcpp::shutdown();
    spinner.join();
    return 0;
}