#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>
#include "planning/CameraMsg.h"

// #include <opencv2/core.hpp>
// #include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

// #include <geometry_msgs/Pose.h>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>

// #include <tf2_eigen/tf2_eigen.h>
// #include <tf2/convert.h>

// #include <tf2_ros/transform_listener.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// MoveIt
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <yaml-cpp/yaml.h>

namespace my_planning
{
    class MyPlanningClass
    {
        public:
            MyPlanningClass(): move_group(PLANNING_GROUP)
            {
                target_pose1.orientation.w = 1.0;
                target_pose1.position.x = 0.38;
                target_pose1.position.y = -0.2;
                target_pose1.position.z = 0.65;

                move_group.allowReplanning(true);
                move_group.setNumPlanningAttempts(10);
            }

            void goToPoseGoal();
            void goToPoseGoal(geometry_msgs::Pose &pose);
            void goToJointState();
            void cartesianPath();
            void resetValues();
            void addObjects();
            void makeBox(std::string blk_name, double *pose);
            void removeObjects();
            void moveCircular();
            void setRPYtoQuaternion();
            void setQuaternion();
            void captureImage();

            ros::ServiceClient client;

        private:
            const std::string PLANNING_GROUP = "manipulator";
            const std::string END_EFFECTOR_LINK = "tool0";
            const double RAD2DEG = M_PI / 180.0;
            double angle;
            double floor;
            bool floor_level = false;
            double height = 0.2;
            double width = 0.1;
            double length = 0.1;
            double center = 0.4;
            double offset = 0.1;
            double step = 15.0;

            moveit::planning_interface::MoveGroupInterface move_group;
            moveit::planning_interface::PlanningSceneInterface virtual_world;
            const robot_state::JointModelGroup* joint_model_group;
            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            std::string pose_text;

            Eigen::Quaterniond quaternion;
            tf2::Quaternion tfQuaternion;
            geometry_msgs::Quaternion msgQuaternion;

            geometry_msgs::Pose target_pose;
            geometry_msgs::Pose target_pose1;

            cv_bridge::CvImagePtr cv_ptr;
            planning::CameraMsg camera_srv;
    };
}