#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <example_interfaces/msg/bool.hpp>
#include <charmie_interfaces/msg/detected_object.hpp>
#include <charmie_interfaces/msg/list_of_detected_object.hpp>

#include <charmie_interfaces/srv/named_target.hpp>
#include <charmie_interfaces/srv/joint_target.hpp>
#include <charmie_interfaces/srv/pose_target.hpp>


using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

using Bool = example_interfaces::msg::Bool;

using DetectedObject = charmie_interfaces::msg::DetectedObject;
using ListOfDetectedObjects = charmie_interfaces::msg::ListOfDetectedObject;

using namespace std::placeholders;

using NamedTargetSrv = charmie_interfaces::srv::NamedTarget;
using JointTargetSrv = charmie_interfaces::srv::JointTarget;
using PoseTargetSrv = charmie_interfaces::srv::PoseTarget;

class Commander
{
    public:
        Commander(std::shared_ptr<rclcpp::Node> node)
        {
            node_ = node;
            xarm_ = std::make_shared<MoveGroupInterface>(node_, "xarm6");
            xarm_ ->setMaxVelocityScalingFactor(0.3);
            xarm_ ->setMaxAccelerationScalingFactor(0.1);
            xarm_ ->setNumPlanningAttempts(20);
            xarm_ ->setPlanningTime(3.0);
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

            // Services
            named_target_srv_ = node_ ->create_service<NamedTargetSrv>(
                "set_named_target",
                std::bind(&Commander::NamedTargetService, this, _1, _2)
            );

            joint_target_srv_ = node_ ->create_service<JointTargetSrv>(
                "set_joint_target",
                std::bind(&Commander::JointTargetService, this, _1, _2)
            );

            pose_target_srv_ = node_ ->create_service<PoseTargetSrv>(
                "set_pose_target",
                std::bind(&Commander::PoseTargetService, this, _1, _2)
            );
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

                std::string shape_type = obj.cf_shape;

                if (shape_type == "cylinder")
                {
                    shape.type = shape.CYLINDER;
                    shape.dimensions.resize(2);
                    shape.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT] = obj.cf_height;
                    shape.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS] = obj.cf_width/2.0;
                }
                else if (shape_type == "cuboid")
                {
                    shape.type = shape.BOX;
                    shape.dimensions.resize(3);
                    shape.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = obj.cf_length;
                    shape.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = obj.cf_width;
                    shape.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = obj.cf_height;
                }
                else if (shape_type == "sphere")
                {
                    shape.type = shape.SPHERE;
                    shape.dimensions.resize(1);
                    shape.dimensions[shape_msgs::msg::SolidPrimitive::SPHERE_RADIUS] = obj.cf_width;
                }
                else
                {
                    RCLCPP_WARN(node_ ->get_logger(),"Unknown shape %s for object %s", shape_type.c_str(), obj.object_name.c_str());
                    shape.type = shape.CYLINDER;
                    shape.dimensions.resize(2);
                    shape.dimensions[0] = 0.15; //height
                    shape.dimensions[1] = 0.05; //radius
                }

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

        void NamedTargetService(
            const std::shared_ptr<NamedTargetSrv::Request> request,
            std::shared_ptr<NamedTargetSrv::Response> response)
        {
            RCLCPP_INFO(
                node_->get_logger(),
                "Received NamedTargetService request: %s",
                request->target_name.c_str()
            );

            xarm_->setStartStateToCurrentState();
            xarm_->setNamedTarget(request->target_name);

            MoveGroupInterface::Plan plan;
            bool success = (xarm_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

            if (success)
            {
                xarm_->execute(plan);
                response->success = true;
                response->message = "Named target executed successfully.";
            }
            else
            {
                response->success = false;
                response->message = "Failed to plan for the named target.";
            }
        }

        void JointTargetService(
            const std::shared_ptr<JointTargetSrv::Request> request,
            std::shared_ptr<JointTargetSrv::Response> response)
        {
            RCLCPP_INFO(
                node_->get_logger(),
                "Received JointTargetService request with %ld joint positions.",
                request->joint_positions.size()
            );

            xarm_->setStartStateToCurrentState();
            xarm_->setJointValueTarget(request->joint_positions);

            MoveGroupInterface::Plan plan;
            bool success = (xarm_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

            if (success)
            {
                xarm_->execute(plan);
                response->success = true;
                response->message = "Joint target executed successfully.";
            }
            else
            {
                response->success = false;
                response->message = "Failed to plan for the joint target.";
            }
        }

        void PoseTargetService(
            const std::shared_ptr<PoseTargetSrv::Request> request,
            std::shared_ptr<PoseTargetSrv::Response> response)
        {
            RCLCPP_INFO(
                node_->get_logger(),
                "Received PoseTargetService request: position(%f, %f, %f), orientation(%f, %f, %f), cartesian=%s",
                request->x, request->y, request->z,
                request->roll, request->pitch, request->yaw,
                request->cartesian
            );

            tf2::Quaternion q;
            q.setRPY(request->roll, request->pitch, request->yaw);
            q.normalize();

            geometry_msgs::msg::PoseStamped target_pose;
            target_pose.header.frame_id = xarm_->getPlanningFrame();
            target_pose.pose.position.x = request->x;
            target_pose.pose.position.y = request->y;
            target_pose.pose.position.z = request->z;
            target_pose.pose.orientation.x = q.x();
            target_pose.pose.orientation.y = q.y();
            target_pose.pose.orientation.z = q.z();
            target_pose.pose.orientation.w = q.w();

            xarm_->setStartStateToCurrentState();

            if(!request->cartesian)
            {
                xarm_->setPoseTarget(target_pose);
                
                MoveGroupInterface::Plan plan;
                bool success = (xarm_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

                if (success)
                {
                    xarm_->execute(plan);
                    response->success = true;
                    response->message = "Pose target executed successfully.";
                }
                else
                {
                    response->success = false;
                    response->message = "Failed to plan for the pose target.";
                }
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
                    response->success = true;
                    response->message = "Cartesian pose target executed successfully.";
                }
                else
                {
                    response->success = false;
                    response->message = "Failed to compute Cartesian path for the pose target.";
                }
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
        };

        std::shared_ptr<rclcpp::Node> node_;
        std::shared_ptr<MoveGroupInterface> xarm_;
        std::shared_ptr<MoveGroupInterface> xarm_gripper_;
        std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;

        rclcpp::Subscription<Bool>::SharedPtr open_gripper_sub_;
        rclcpp::Subscription<ListOfDetectedObjects>::SharedPtr detected_objects_sub_;

        rclcpp::Service<charmie_interfaces::srv::NamedTarget>::SharedPtr named_target_srv_;
        rclcpp::Service<charmie_interfaces::srv::JointTarget>::SharedPtr joint_target_srv_;
        rclcpp::Service<charmie_interfaces::srv::PoseTarget>::SharedPtr pose_target_srv_;
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