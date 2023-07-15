#include <planning.h>

namespace my_planning
{
        void MyPlanningClass::setQuaternion()
        {
            std::cout << "Setting the quaternion!" << std::endl;

            Eigen::Vector3d unitX(1.0, 0.0, 0.0);

            // Eigen::Vector3d point1(0.5, 0.0, 0.0);
            // Eigen::Vector3d point2(target_pose.position.x, target_pose.position.y, 0.5);
            Eigen::Vector3d point1(center, 0.0, height / 2);
            Eigen::Vector3d point2(target_pose.position.x, target_pose.position.y, 0.35); //floor

            Eigen::Vector3d vector = point1 - point2;

            vector.normalize();

            quaternion.setFromTwoVectors(unitX, vector);

            // msgQuaternion.w = quaternion.w();
            // msgQuaternion.x = quaternion.x();
            // msgQuaternion.y = quaternion.y();
            // msgQuaternion.z = quaternion.z();

            // msgQuaternion.w = round(quaternion.w() * 1000) / 1000;
            // msgQuaternion.x = round(quaternion.w() * 1000) / 1000;
            // msgQuaternion.y = round(quaternion.w() * 1000) / 1000;
            // msgQuaternion.z = round(quaternion.w() * 1000) / 1000;

            //round(a * 100.0) / 100.0;

            // tf2::convert(quaternion, tfQuaternion);
            // tf2::convert(tfQuaternion, msgQuaternion);

            // target_pose.orientation.w = msgQuaternion.w;
            // target_pose.orientation.x = msgQuaternion.x;
            // target_pose.orientation.y = msgQuaternion.y;
            // target_pose.orientation.z = msgQuaternion.z;

            target_pose.orientation.w = quaternion.w();
            target_pose.orientation.x = quaternion.x();
            target_pose.orientation.y = quaternion.y();
            target_pose.orientation.z = quaternion.z();
        }

        void MyPlanningClass::moveCircular()
        {
            // move_group.setEndEffector("tool0");
            // move_group.setEndEffectorLink(END_EFFECTOR_LINK);
            std::cout<<" MOVE CIRCULAR"<<std::endl;
            namespace rvt = rviz_visual_tools;
            moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
            visual_tools.deleteAllMarkers();
            // visual_tools.loadRemoteControl();
            // Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
            const moveit::core::JointModelGroup* joint_model_group = move_group.getCurrentState() -> getJointModelGroup(PLANNING_GROUP);

            std::ostringstream oss;

            for (floor = 0.1; floor <= height + offset; floor += offset)
            {
                if(!floor_level) //Odd floor
                {
                    angle = step;
                    floor_level = true;
                }
                else //Even floor
                {
                    angle = step / 2;
                    floor_level = false;
                }
                for (angle; angle <= 360.0; angle += step)
                {
                    std::cout << "ANGLE:" << angle << std::endl;

                    // target_pose.position.x = 0.5 + 0.21 * cos(angle * RAD2DEG);
                    // target_pose.position.y = 0.21 * sin(angle * RAD2DEG);
                    // target_pose.position.z = 0.5;

                    if(length >= width)
                    {
                        target_pose.position.x = center + (length + offset) * cos(angle * RAD2DEG);
                        target_pose.position.y = (length + offset) * sin(angle * RAD2DEG);
                    }
                    else
                    {
                        target_pose.position.x = center + (width + offset) * cos(angle * RAD2DEG);
                        target_pose.position.y = (width + offset) * sin(angle * RAD2DEG);
                    }
                    target_pose.position.z = floor;

                    setQuaternion();
                    
                    // geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose();

                    // std::cout << current_pose << std::endl;
                    // std::cout << std::endl;
                    // std::cout << target_pose << std::endl;

                    move_group.setPoseTarget(target_pose);
                    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
                    if(!success) //execute
                        throw std::runtime_error("No plan found");

                    ROS_INFO_NAMED("Circle", "Visualizing plan 1 as trajectory line");

                    oss.str(std::string());
                    oss << "Floor:" << floor * 10 << " Angle=" << angle << "°";
                    pose_text = oss.str();

                    visual_tools.publishAxisLabeled(target_pose, pose_text);
                    // visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
                    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
                    visual_tools.trigger();

                    auto error_code = move_group.execute(my_plan); //blocking
                    std::cout << "Error code " << error_code << std::endl;

                    captureImage();
                }
            }
        }

        void MyPlanningClass::captureImage()
        {
            camera_srv.request.angle = angle;
            std::cout << client.call(camera_srv) << std::endl;

            // Call the camera service
            if (client.call(camera_srv))
            {
                // Extract the image from the response
                sensor_msgs::Image image = camera_srv.response.image;

                // Convert the image message to an OpenCV image
                cv_bridge::CvImagePtr cv_ptr;
                try
                {
                    cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
                }
                catch (cv_bridge::Exception& e)
                {
                    ROS_ERROR("cv_bridge exception: %s", e.what());
                }

                // Save the image to a file
                std::string filename = "/home/pilaciv/opencv_images/" + pose_text + ".jpg";
                cv::imwrite(filename, cv_ptr->image);

                ROS_INFO("Image saved: %s", filename.c_str());
            }
            else
            {
                ROS_ERROR("Failed to call camera service.");
            }
        }

        void MyPlanningClass::goToPoseGoal()
        {
            move_group.setPoseTarget(target_pose1);
            bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            if(!success) //execute
                throw std::runtime_error("No plan found");

            move_group.move(); //blocking
        }
        
        void MyPlanningClass::goToPoseGoal(geometry_msgs::Pose &pose)
        {
            move_group.setPoseTarget(pose);
            ros::Duration(0.5).sleep();
            bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            /*while (!success) //keep trying until a plan is found
            {
                
                success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            }*/
            
            if(!success) //execute
                throw std::runtime_error("No plan found");

            move_group.move(); //blocking
        }

        void MyPlanningClass::goToJointState()
        {
            robot_state::RobotState current_state = *move_group.getCurrentState();
            //moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
            std::vector<double> joint_positions;
            joint_model_group = current_state.getJointModelGroup(PLANNING_GROUP);
            current_state.copyJointGroupPositions(joint_model_group, joint_positions);
            std::cout << current_state << std::endl;
            std::cout << joint_model_group << std::endl;
            //joint_positions = move_group.getCurrentJointValues();

            // joint_positions[0] = -1.0;
            // joint_positions[3] = 0.7;

            move_group.setJointValueTarget(joint_positions);
            bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            if(!success)
                throw std::runtime_error("No plan found");

            move_group.move(); //blocking
        }

        void MyPlanningClass::cartesianPath()
        {
            std::vector<geometry_msgs::Pose> waypoints;
            waypoints.push_back(target_pose1);

            geometry_msgs::Pose target_pose2 = target_pose1;

            target_pose2.position.z -= 0.2;
            waypoints.push_back(target_pose2);

            target_pose2.position.y -= 0.2;
            waypoints.push_back(target_pose2);

            target_pose2.position.z += 0.2;
            target_pose2.position.y += 0.2;
            target_pose2.position.x -= 0.2;
            waypoints.push_back(target_pose2);

            move_group.setMaxVelocityScalingFactor(0.1);

            // We want the Cartesian path to be interpolated at a resolution of 1 cm
            // which is why we will specify 0.01 as the max step in Cartesian
            // translation.  We will specify the jump threshold as 0.0, effectively disabling it.
            // Warning - disabling the jump threshold while operating real hardware can cause
            // large unpredictable motions of redundant joints and could be a safety issue
            moveit_msgs::RobotTrajectory trajectory;
            const double jump_threshold = 0.0;
            const double eef_step = 0.01;
            double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

            move_group.move();
            ROS_INFO_STREAM("Percentage of path followed: " << fraction * 100 << "%");
        }

        void MyPlanningClass::resetValues()
        {
            //set the start state and operational speed
            move_group.setStartStateToCurrentState();
            move_group.setMaxVelocityScalingFactor(1.0);
        }

         void MyPlanningClass::makeBox(std::string blk_name, double *pose)
         {
            moveit_msgs::CollisionObject box;
            //set the relative frame
            box.header.frame_id = move_group.getPlanningFrame();
            box.id = blk_name;

            shape_msgs::SolidPrimitive primitive;
            primitive.type = primitive.BOX;
            primitive.dimensions.resize(3);
            primitive.dimensions[0] = 0.2;
            primitive.dimensions[1] = 0.2;
            primitive.dimensions[2] = 1.0;

            geometry_msgs::Pose box_pose;
            box_pose.orientation.w = 1.0;
            box_pose.position.x = pose[0];
            box_pose.position.y = pose[1];
            box_pose.position.z = pose[2];

            box.primitives.push_back(primitive);
            box.primitive_poses.push_back(box_pose);
            box.operation = box.ADD;

            std::vector<moveit_msgs::CollisionObject> collisionObjects;
            collisionObjects.push_back(box);
            ros::Duration(2).sleep();
            virtual_world.addCollisionObjects(collisionObjects);
            ROS_INFO_STREAM("Added: " << blk_name);
         }

        void MyPlanningClass::addObjects()
        {
            double box_pose1[3] = {0.60, -0.67, 0.0,};
            makeBox("block_1", box_pose1);

            double box_pose2[3] = {0.0, 0.77, 0.0,};
            makeBox("block_2", box_pose2);
        }

        void MyPlanningClass::removeObjects()
        {
            std::vector<std::string> object_ids;
            object_ids.push_back("block_1");
            object_ids.push_back("block_2");
            virtual_world.removeCollisionObjects(object_ids);

        }
}
