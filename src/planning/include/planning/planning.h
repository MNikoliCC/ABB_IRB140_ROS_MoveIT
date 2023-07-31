#ifndef PLANNING_H
#define PLANNING_H

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
// #include <moveit_cpp.h>

#include <yaml-cpp/yaml.h>
#include <XmlRpcValue.h>


namespace my_planning
{
    class MyPlanningClass
    {
        public:
            MyPlanningClass(ros::NodeHandle& nh)
                : move_group(PLANNING_GROUP), nh_(nh)
            {
                if (!nh_.getParam("param_width", width) ||
                    !nh_.getParam("param_length", length) ||
                    !nh_.getParam("param_height", height) ||
                    !nh_.getParam("param_center", center)) 
                {
                    throw std::runtime_error("Failed to get the required parameters, permission for planning denied!");
                }

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
            void objectPosition();
            void goToObject();

            ros::ServiceClient client;

        private:
            const std::string PLANNING_GROUP = "manipulator";
            const std::string END_EFFECTOR_LINK = "tool0";
            const double RAD2DEG = M_PI / 180.0;
            double angle;
            double floor;
            double radius;
            bool floor_level = false;
            bool smaller_radius = false;

            //robot
            // double height = 0.32;
            // double width = 0.25;
            // double length = 0.28;
            // double center = 0.5;
            // double offset = 0.1;
            // double step = 15.0;

            // double height = 0.265;
            // double width = 0.07;
            // double length = 0.23;

            double height;
            double width;
            double length;
            double center;
            double offset = 0.05; //udaljenost od predmeta na svakom nivou (radius)
            double offset_z = 0.1; //offset visine izmedju svakog nivoa
            double offset_z2 = 0.15; //offset visine na poslednjem nivou
            double offset_radius = 0.1; //udaljenost predmeta na poslednjem nivou
            double step = 10.0;

            ros::NodeHandle& nh_;

            // double height = 0.2;
            // double width = 0.1;
            // double length = 0.1;
            // double center = 0.5;
            // double offset = 0.1;
            // double step = 15.0;

            // double height = 0.1;
            // double width = 0.05;
            // double length = 0.05;
            // double center = 0.5;
            // double offset = 0.12;
            // double step = 15.0;
            int selfie = 0;

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

            // cv_bridge::CvImagePtr cv_ptr;
            planning::CameraMsg camera_srv;
    };
}
#endif // PLANNING_H