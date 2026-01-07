#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("test_moveit_pillar_node");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() { executor.spin(); });

    auto arm = moveit::planning_interface::MoveGroupInterface(node, "xarm6");
    auto gripper = moveit::planning_interface::MoveGroupInterface(node, "xarm_gripper");
    auto planning_scene = moveit::planning_interface::PlanningSceneInterface();

    arm.setMaxVelocityScalingFactor(0.5);
    arm.setMaxAccelerationScalingFactor(0.5);
    arm.setNumPlanningAttempts(20);
    arm.setPlanningTime(3.0);
    arm.setEndEffectorLink("xarm_link6");

    // Create obstacles

    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;

    // Create a table obstacle
    moveit_msgs::msg::CollisionObject table1;
    table1.header.frame_id = arm.getPlanningFrame();
    table1.id = "table1";

    shape_msgs::msg::SolidPrimitive table1_primitive;
    table1_primitive.type = table1_primitive.BOX;
    table1_primitive.dimensions.resize(3);
    table1_primitive.dimensions[table1_primitive.BOX_X] = 0.8; // x size
    table1_primitive.dimensions[table1_primitive.BOX_Y] = 1.2; // y size
    table1_primitive.dimensions[table1_primitive.BOX_Z] = 0.8; // z size

    geometry_msgs::msg::Pose table1_pose;
    table1_pose.position.x = 0.75;
    table1_pose.position.y = 0.0;
    table1_pose.position.z = 0.0;
    table1_pose.orientation.w = 1.0;

    table1.primitives.push_back(table1_primitive);
    table1.primitive_poses.push_back(table1_pose);
    table1.operation = table1.ADD;

    collision_objects.push_back(table1);

    // Create a vertical pillar obstacle in the middle of the table
    moveit_msgs::msg::CollisionObject pillar;
    pillar.header.frame_id = arm.getPlanningFrame();
    pillar.id = "pillar";

    shape_msgs::msg::SolidPrimitive pillar_primitive;
    pillar_primitive.type = pillar_primitive.CYLINDER;
    pillar_primitive.dimensions.resize(2);
    pillar_primitive.dimensions[pillar_primitive.CYLINDER_HEIGHT] = 1.0; // height
    pillar_primitive.dimensions[pillar_primitive.CYLINDER_RADIUS] = 0.08; // radius

    geometry_msgs::msg::Pose pillar_pose;
    pillar_pose.position.x = 0.6;
    pillar_pose.position.y = 0.0;
    pillar_pose.position.z = table1_pose.position.z + table1_primitive.dimensions[table1_primitive.BOX_Z]/2 + pillar_primitive.dimensions[pillar_primitive.CYLINDER_HEIGHT]/2;
    pillar_pose.orientation.w = 1.0;

    pillar.primitives.push_back(pillar_primitive);
    pillar.primitive_poses.push_back(pillar_pose);
    pillar.operation = pillar.ADD;

    collision_objects.push_back(pillar);

    // Object to grasp (positioned to the left of the pillar)
    moveit_msgs::msg::CollisionObject object;
    object.header.frame_id = arm.getPlanningFrame();
    object.id = "target";

    shape_msgs::msg::SolidPrimitive object_primitive;
    object_primitive.type = object_primitive.BOX;
    object_primitive.dimensions.resize(3);
    object_primitive.dimensions[object_primitive.BOX_X] = 0.05; // x size
    object_primitive.dimensions[object_primitive.BOX_Y] = 0.05; // y size
    object_primitive.dimensions[object_primitive.BOX_Z] = 0.15; // z size

    geometry_msgs::msg::Pose object_pose;
    object_pose.position.x = 0.4;
    object_pose.position.y = 0.2; // Left side of table
    object_pose.position.z = table1_pose.position.z + table1_primitive.dimensions[table1_primitive.BOX_Z]/2 + object_primitive.dimensions[object_primitive.BOX_Z]/2;
    object_pose.orientation.w = 1.0;

    object.primitives.push_back(object_primitive);
    object.primitive_poses.push_back(object_pose);
    object.operation = object.ADD;
    collision_objects.push_back(object);

    // Add the collision objects to the planning scene
    planning_scene.addCollisionObjects(collision_objects);
    RCLCPP_INFO(node->get_logger(), "Added collision objects to the planning scene (table, pillar, and target object).");

    // Target planning - approach from above

    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.frame_id = arm.getPlanningFrame();
    target_pose.pose.position.x = object_pose.position.x;
    target_pose.pose.position.y = object_pose.position.y;
    target_pose.pose.position.z = object_pose.position.z + object_primitive.dimensions[object_primitive.BOX_Z] + 0.2; // Above the object

    RCLCPP_INFO(node->get_logger(), "Target pose set to x: %.2f, y: %.2f, z: %.2f",
                target_pose.pose.position.x,
                target_pose.pose.position.y,
                target_pose.pose.position.z);

    // Gripper pointing down
    target_pose.pose.orientation.x = 1.0;
    target_pose.pose.orientation.y = 0.0;
    target_pose.pose.orientation.z = 0.0;
    target_pose.pose.orientation.w = 0.0;

    arm.setStartStateToCurrentState();
    arm.setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (arm.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (success)
    {
        RCLCPP_INFO(node->get_logger(), "Planning to pre-grasp pose successful. Executing...");
        arm.execute(plan);
    }
    else
    {
        RCLCPP_WARN(node->get_logger(), "Planning to pre-grasp pose failed.");
    }

    // Move down to grasp the object
    std::vector<geometry_msgs::msg::Pose> waypoints;
    geometry_msgs::msg::Pose grasp_pose = arm.getCurrentPose().pose;
    grasp_pose.position.z -= 0.15;

    waypoints.push_back(grasp_pose);

    moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 2*M_PI;
    const double eef_step = 0.01;
    double fraction = arm.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    if (fraction > 0.8)
    {
        RCLCPP_INFO(node->get_logger(), "Computed Cartesian path to grasp pose. Executing...");
        arm.execute(trajectory);
    }
    else
    {
        RCLCPP_WARN(node->get_logger(), "Could not compute a valid Cartesian path to the grasp pose. Fraction: %.2f", fraction);
    }

    // Close gripper
    gripper.setStartStateToCurrentState();
    gripper.setNamedTarget("gripper_closed");

    if (gripper.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_INFO(node->get_logger(), "Gripper planning successful. Executing...");
        gripper.execute(plan);
    }
    else
    {
        RCLCPP_WARN(node->get_logger(), "Gripper planning failed.");
    }

    std::vector<std::string> touch_links = {
        "xarm_gripper_base_link",
        "xarm_gripper_left_finger",
        "xarm_gripper_right_finger",
        "xarm_gripper_left_inner_knuckle",
        "xarm_gripper_right_inner_knuckle",
        "xarm_gripper_left_outer_knuckle",
        "xarm_gripper_right_outer_knuckle"
    };

    arm.attachObject("target", "xarm_link6", touch_links);

    // Lift the object
    waypoints.clear();
    geometry_msgs::msg::Pose lift_pose = arm.getCurrentPose().pose;
    lift_pose.position.z += 0.2;
    waypoints.push_back(lift_pose);

    fraction = arm.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    if (fraction > 0.8)
    {
        RCLCPP_INFO(node->get_logger(), "Computed Cartesian path to lift object. Executing...");
        arm.execute(trajectory);
    }
    else
    {
        RCLCPP_WARN(node->get_logger(), "Could not compute a valid Cartesian path to lift the object. Fraction: %.2f", fraction);
    }

    // Place planning - move to the right side of the table (opposite side of pillar)
    geometry_msgs::msg::PoseStamped place_pose;
    place_pose.header.frame_id = arm.getPlanningFrame();
    place_pose.pose.position.x = 0.4;
    place_pose.pose.position.y = -0.2; // Left side of table (opposite from pick location)
    place_pose.pose.position.z = target_pose.pose.position.z; // Above final placement

    // Gripper pointing down
    place_pose.pose.orientation.x = 1.0;
    place_pose.pose.orientation.y = 0.0;
    place_pose.pose.orientation.z = 0.0;
    place_pose.pose.orientation.w = 0.0;

    RCLCPP_INFO(node->get_logger(), "Place pose set to x: %.2f, y: %.2f, z: %.2f",
                place_pose.pose.position.x,
                place_pose.pose.position.y,
                place_pose.pose.position.z);

    arm.setStartStateToCurrentState();
    arm.setPoseTarget(place_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan3;
    bool success3 = (arm.plan(plan3) == moveit::core::MoveItErrorCode::SUCCESS);
    if (success3)
    {
        RCLCPP_INFO(node->get_logger(), "Planning to place pose successful (avoiding pillar). Executing...");
        arm.execute(plan3);
    }
    else
    {
        RCLCPP_WARN(node->get_logger(), "Planning to place pose failed.");
    }

    // Lower the object
    waypoints.clear();
    geometry_msgs::msg::Pose lower_pose = arm.getCurrentPose().pose;
    lower_pose.position.z -= 0.4;
    waypoints.push_back(lower_pose);

    fraction = arm.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    if (fraction > 0.2)
    {
        RCLCPP_INFO(node->get_logger(), "Computed Cartesian path to lower object. Executing...");
        arm.execute(trajectory);
    }
    else
    {
        RCLCPP_WARN(node->get_logger(), "Could not compute a valid Cartesian path to lower the object. Fraction: %.2f", fraction);
    }

    // Open gripper
    gripper.setStartStateToCurrentState();
    gripper.setNamedTarget("gripper_open");

    if (gripper.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_INFO(node->get_logger(), "Gripper opening planning successful. Executing...");
        gripper.execute(plan);
    }
    else
    {
        RCLCPP_WARN(node->get_logger(), "Gripper opening planning failed.");
    }

    // Remove Attached Object
    arm.detachObject("target");

    // Lift the gripper after placing the object
    waypoints.clear();
    geometry_msgs::msg::Pose retreat_pose = arm.getCurrentPose().pose;
    retreat_pose.position.z += 0.15;
    waypoints.push_back(retreat_pose);
    fraction = arm.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    if (fraction > 0.8)
    {
        RCLCPP_INFO(node->get_logger(), "Computed Cartesian path to retreat after placing object. Executing...");
        arm.execute(trajectory);
    }
    else
    {
        RCLCPP_WARN(node->get_logger(), "Could not compute a valid Cartesian path to retreat. Fraction: %.2f", fraction);
    }

    // Return to home position
    arm.setStartStateToCurrentState();
    arm.setNamedTarget("home");

    moveit::planning_interface::MoveGroupInterface::Plan plan2;
    bool success2 = (arm.plan(plan2) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success2)
    {
        RCLCPP_INFO(node->get_logger(), "Retreat to home planning successful. Executing...");
        arm.execute(plan2);
    }
    else
    {
        RCLCPP_WARN(node->get_logger(), "Retreat to home planning failed.");
    }

    // Remove obstacles
    planning_scene.removeCollisionObjects({"table1", "pillar", "target"});

    rclcpp::shutdown();
    spinner.join();
    return 0;
}