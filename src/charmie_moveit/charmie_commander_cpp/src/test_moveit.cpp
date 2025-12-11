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
    arm.setEndEffectorLink("xarm_link6");

    // Named goal

    arm.setStartStateToCurrentState();
    arm.setNamedTarget("pick_front");

    moveit::planning_interface::MoveGroupInterface::Plan plan1;
    bool success1 = (arm.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);
    if (success1)
    {
        arm.execute(plan1);
    }

    // arm.setStartStateToCurrentState();
    // arm.setNamedTarget("home");

    // moveit::planning_interface::MoveGroupInterface::Plan plan2;
    // bool success2 = (arm.plan(plan2) == moveit::core::MoveItErrorCode::SUCCESS);
    // if (success2)
    // {
    //     arm.execute(plan2);
    // }

    // -------------------------------------------------------------------

    // //Joint goal

    // std::vector<double> joints = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // arm.setStartStateToCurrentState();
    // arm.setJointValueTarget(joints);

    // moveit::planning_interface::MoveGroupInterface::Plan plan3;
    // bool success3 = (arm.plan(plan3) == moveit::core::MoveItErrorCode::SUCCESS);
    // if (success3)
    // {
    //     arm.execute(plan3);
    // }

    // -------------------------------------------------------------------
    // Pose goal

    // tf2::Quaternion q;
    // q.setRPY(
    //     90.7 * M_PI / 180.0,
    //     0.3 * M_PI / 180.0,
    //     -90.1 * M_PI / 180.0
    // );
    // q.normalize();


    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.frame_id = arm.getPlanningFrame();
    target_pose.pose.position.x = 0.079; 
    target_pose.pose.position.y = 0.2304;
    target_pose.pose.position.z = 0.9157;

    target_pose.pose.orientation.x = 0.0257;
    target_pose.pose.orientation.y = -0.3905;
    target_pose.pose.orientation.z = 0.0078;
    target_pose.pose.orientation.w = 0.9202;

    arm.setStartStateToCurrentState();
    arm.setPoseTarget(target_pose);

    // RCLCPP_INFO(node->get_logger(), "End effector link: %s", arm.getEndEffectorLink().c_str());
    // RCLCPP_INFO(node->get_logger(), "Planning frame: %s", arm.getPlanningFrame().c_str());


    // arm.setPlanningTime(20.0);

    moveit::planning_interface::MoveGroupInterface::Plan plan4;
    bool success4 = (arm.plan(plan4) == moveit::core::MoveItErrorCode::SUCCESS);
    if (success4)
    {
        arm.execute(plan4);
    }

    // -------------------------------------------------------------------

    // // Cartesian path

    // std::vector<geometry_msgs::msg::Pose> waypoints;
    // geometry_msgs::msg::Pose pose5 = arm.getCurrentPose().pose;
    // pose5.position.z -= 0.1;
    // waypoints.push_back(pose5);
    // geometry_msgs::msg::Pose pose6 = pose5;
    // pose6.position.x += 0.2;
    // waypoints.push_back(pose6);
    // geometry_msgs::msg::Pose pose7 = pose6;
    // pose7.position.z += 0.1;
    // pose7.position.x -= 0.2;
    // waypoints.push_back(pose7);

    // moveit_msgs::msg::RobotTrajectory trajectory;
    // double resolution = 0.01; // 1cm
    // double jump_threshold = 0.0; // disable jump threshold checking

    // double fraction = arm.computeCartesianPath(waypoints, resolution, jump_threshold, trajectory);

    // if (fraction == 1){
    //     arm.execute(trajectory);
    // }

    // arm.setStartStateToCurrentState();
    // arm.setNamedTarget("home");

    // moveit::planning_interface::MoveGroupInterface::Plan plan2;
    // bool success2 = (arm.plan(plan2) == moveit::core::MoveItErrorCode::SUCCESS);

    // if (success2)
    // {
    //     arm.execute(plan2);
    // }


    rclcpp::shutdown();
    spinner.join();
    return 0;
}