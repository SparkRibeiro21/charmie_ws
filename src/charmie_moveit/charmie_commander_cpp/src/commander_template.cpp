#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <example_interfaces/msg/bool.hpp>
#include <charmie_interfaces/msg/detected_object.hpp>
#include <charmie_interfaces/msg/list_of_detected_object.hpp>

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using Bool = example_interfaces::msg::Bool;
using DetectedObject = charmie_interfaces::msg::DetectedObject;
using ListOfDetectedObjects = charmie_interfaces::msg::ListOfDetectedObject;
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
            planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

            open_gripper_sub_ = node_ ->create_subscription<Bool>(
                "open_gripper", 10,
                std::bind(&Commander::OpenGripperCallback, this, _1)
            );

            detected_objects_sub_ = node_ ->create_subscription<ListOfDetectedObjects>(
                "objects_all_detected_filtered", 10,
                std::bind(&Commander::DetectedObjectsCallback, this, _1)
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

        void addDetectedObjectsToScene(const ListOfDetectedObjects::SharedPtr msg)
        {
            clearDetectedObjects();
            
            std::vector<moveit_msgs::msg::CollisionObject> collision_objects;

            for (const auto &obj : msg->objects)
            {
                if (obj.index <= 0) continue;

                moveit_msgs::msg::CollisionObject detected_object;
                detected_object.header.frame_id = xarm_->getPlanningFrame();
                detected_object.id = "detected_" + std::to_string(obj.index);

                shape_msgs::msg::SolidPrimitive shape;
                shape.type = shape.CYLINDER;
                shape.dimensions.resize(2);
                shape.dimensions[0] = 0.15; //height
                shape.dimensions[1] = 0.05; //radius

                //Set pose
                geometry_msgs::msg::Pose pose;
                pose.position.x = obj.position_absolute.x;
                pose.position.y = obj.position_absolute.y;
                pose.position.z = obj.position_absolute.z;
                pose.orientation.w = 1.0;

                detected_object.primitives.push_back(shape);
                detected_object.primitive_poses.push_back(pose);
                detected_object.operation = detected_object.ADD;

                collision_objects.push_back(detected_object);
            }

            planning_scene_interface_ -> addCollisionObjects(collision_objects);
            RCLCPP_INFO(node_ ->get_logger(),"Added %ld objects to planning scene", collision_objects.size());
        }

        void clearDetectedObjects()
        {
            std::vector<std::string> all_objects =planning_scene_interface_->getKnownObjectNames();
            std::vector<std::string> detected_objects;

            for (const auto& name : all_objects)
            {
                if(name.find("detected_")==0)
                {
                    detected_objects.push_back(name);
                }
            }

            if (!detected_objects.empty())
            {
                planning_scene_interface_->removeCollisionObjects(detected_objects);
            }
        }

        void clearPlanningScene()
        {
            std::vector<std::string> object_ids = planning_scene_interface_->getKnownObjectNames();
            planning_scene_interface_ ->removeCollisionObjects(object_ids);
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

        void DetectedObjectsCallback(const ListOfDetectedObjects::SharedPtr msg)
        {
            if(msg->objects.empty())
            {
                RCLCPP_WARN(node_->get_logger(), "No objects detected");
                clearDetectedObjects();
                return;
            }

            RCLCPP_INFO(node_->get_logger(),"Detected %ld objects", msg->objects.size());

            addDetectedObjectsToScene(msg);

            for (const auto &obj : msg->objects)
            {
                RCLCPP_INFO(
                    node_->get_logger(),
                    "ID: %d | %s | conf: %.2f | rel:(%f %f %f) | cam:(%f %f %f)",
                    obj.index,
                    obj.object_name.c_str(),
                    obj.confidence,
                    obj.position_relative.x,
                    obj.position_relative.y,
                    obj.position_relative.z,
                    obj.position_cam.x,
                    obj.position_cam.y,
                    obj.position_cam.z
                );

                calculatePreGraspPose(obj, "top");
                calculatePreGraspPose(obj, "front");
            }
        }

        //Calculate pre-grasp pose and print it
        void calculatePreGraspPose(
            const DetectedObject &obj,
            const std::string &approach)
            {
                geometry_msgs::msg::PoseStamped pre_grasp_pose;
                pre_grasp_pose.header.frame_id = xarm_->getPlanningFrame();
                pre_grasp_pose.pose.position.x = obj.position_absolute.x;
                pre_grasp_pose.pose.position.y = obj.position_absolute.y;
                pre_grasp_pose.pose.position.z = obj.position_absolute.z;

                if (approach == "top")
                {
                    pre_grasp_pose.pose.position.z += 0.2;
                    pre_grasp_pose.pose.orientation.x = 1.0;
                    pre_grasp_pose.pose.orientation.y = 0.0;
                    pre_grasp_pose.pose.orientation.z = 0.0;
                    pre_grasp_pose.pose.orientation.w = 0.0;
                }
                else if (approach == "front")
                {
                    pre_grasp_pose.pose.position.x -= 0.2;
                    pre_grasp_pose.pose.orientation.x = 0.0;
                    pre_grasp_pose.pose.orientation.y = 0.7071;
                    pre_grasp_pose.pose.orientation.z = 0.0;
                    pre_grasp_pose.pose.orientation.w = 0.7071;
                }
                else
                {
                    RCLCPP_WARN(node_->get_logger(), "Unknown approach direction: %s", approach.c_str());
                }
                
                RCLCPP_INFO(
                    node_->get_logger(),
                    "\033[34mPre-grasp pose (%s) for %s: x=%f, y=%f, z=%f\033[0m",
                    approach.c_str(),
                    obj.object_name.c_str(),
                    pre_grasp_pose.pose.position.x,
                    pre_grasp_pose.pose.position.y,
                    pre_grasp_pose.pose.position.z
                );

                // Plan to the pre-grasp pose
                xarm_->setStartStateToCurrentState();
                xarm_->setPoseTarget(pre_grasp_pose);
                
                MoveGroupInterface::Plan plan;
                bool success = (xarm_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
                
                if (success)
                {
                    RCLCPP_INFO(
                        node_->get_logger(),
                        "\033[32mSuccessfully planned %s approach for %s (planning time: %.2f s)\033[0m",
                        approach.c_str(),
                        obj.object_name.c_str(),
                        plan.planning_time_
                    );
                }
                else
                {
                    RCLCPP_WARN(
                        node_->get_logger(),
                        "\033[31mFailed to plan %s approach for %s\033[0m",
                        approach.c_str(),
                        obj.object_name.c_str()
                    );
                }

                return ;
            };

        std::shared_ptr<rclcpp::Node> node_;
        std::shared_ptr<MoveGroupInterface> xarm_;
        std::shared_ptr<MoveGroupInterface> xarm_gripper_;
        std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;

        rclcpp::Subscription<Bool>::SharedPtr open_gripper_sub_;
        rclcpp::Subscription<ListOfDetectedObjects>::SharedPtr detected_objects_sub_;
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