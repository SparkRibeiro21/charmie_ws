#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <example_interfaces/msg/bool.hpp>

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using Bool = example_interfaces::msg::Bool;
using namespace std::placeholders;

class Commander
{
    public:
        Commander(std::shared_ptr<rclcpp::Node> node)
        {
            node_ = node;
            xarm_ = std::make_shared<MoveGroupInterface>(node_, "xarm6");
            xarm_->setMaxVelocityScalingFactor(0.5);
            xarm_ ->setMaxAccelerationScalingFactor(0.5);
            xarm_gripper_ = std::make_shared<MoveGroupInterface>(node_, "xarm_gripper");

            open_gripper_sub_ = node_ ->create_subscription<Bool>(
                "open_gripper", 10,
                std::bind(&Commander::OpenGripperCallback, this, _1)
            );
        }

        void NamedTarget(const std::string &target_name)
        {
            xarm_->setStartStateToCurrentState();
            xarm_->setNamedTarget(target_name);
            planAndExecute(xarm_);
        }

        void JointTarget(const std::vector<double> &joints)
        {
            xarm_->setStartStateToCurrentState();
            xarm_->setJointValueTarget(joints);
            planAndExecute(xarm_);
        }

        void PoseTarget(double x, double y, double z,
                        double roll, double pitch, double yaw,
                        bool cartesian=false)
        {

            tf2::Quaternion q;
            q.setRPY(roll, pitch, yaw);
            q.normalize();

            geometry_msgs::msg::PoseStamped target_pose;
            target_pose.header.frame_id = xarm_->getPlanningFrame();
            target_pose.pose.position.x = x;
            target_pose.pose.position.y = y;
            target_pose.pose.position.z = z;
            target_pose.pose.orientation.x = q.x();
            target_pose.pose.orientation.y = q.y();
            target_pose.pose.orientation.z = q.z();
            target_pose.pose.orientation.w = q.w();


            xarm_->setStartStateToCurrentState();
            if(!cartesian)
            {
                xarm_->setPoseTarget(target_pose);
                planAndExecute(xarm_);
            }
            else
            {
                std::vector<geometry_msgs::msg::Pose> waypoints;
                waypoints.push_back(target_pose.pose);

                moveit_msgs::msg::RobotTrajectory trajectory;
                double resolution = 0.01; // 1cm
                double jump_threshold = 0.0; // disable jump threshold checking

                double fraction = xarm_->computeCartesianPath(waypoints, resolution, jump_threshold, trajectory);

                if (fraction == 1){
                    xarm_->execute(trajectory);
                }
            }

        }

        void openGripper()
        {
            xarm_gripper_->setStartStateToCurrentState();
            xarm_gripper_->setNamedTarget("gripper_open");
            planAndExecute(xarm_gripper_);
        }

        void closeGripper()
        {
            xarm_gripper_->setStartStateToCurrentState();
            xarm_gripper_->setNamedTarget("gripper_closed");
            planAndExecute(xarm_gripper_);
        }


    
    private:

        void planAndExecute(const std::shared_ptr<MoveGroupInterface> &interface)
        {
            MoveGroupInterface::Plan plan;
            bool success = (interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
            if (success)
            {
                interface->execute(plan);
            }
        }

        void OpenGripperCallback(const Bool &msg)
        {
            if (msg.data)
            {
                openGripper();
            }
            else
            {
                closeGripper();
            }
                
        }

        std::shared_ptr<rclcpp::Node> node_;
        std::shared_ptr<MoveGroupInterface> xarm_;
        std::shared_ptr<MoveGroupInterface> xarm_gripper_;

        rclcpp::Subscription<Bool>::SharedPtr open_gripper_sub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("commander_node");
    auto commander = Commander(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}