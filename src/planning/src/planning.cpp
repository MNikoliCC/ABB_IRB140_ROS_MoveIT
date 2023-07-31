#include <planning.h>

namespace my_planning
{
        void MyPlanningClass::setQuaternion()
        {
            std::cout << "Setting the quaternion!" << std::endl;

            Eigen::Vector3d unitX(1.0, 0.0, 0.0);
            Eigen::Vector3d point1; // Declare point1 here to make it accessible in both branches

            if (!smaller_radius) {
                point1 = Eigen::Vector3d(center, 0.0, height / 2); //0.052
            } else {
                point1 = Eigen::Vector3d(center, 0.0, height / 2);
            }

            Eigen::Vector3d point2(target_pose.position.x, target_pose.position.y, floor); //floor
            Eigen::Vector3d vector = point1 - point2;
            vector.normalize();
            quaternion.setFromTwoVectors(unitX, vector);

            target_pose.orientation.w = quaternion.w();
            target_pose.orientation.x = quaternion.x();
            target_pose.orientation.y = quaternion.y();
            target_pose.orientation.z = quaternion.z();

            move_group.setPoseTarget(target_pose);
            move_group.setPlanningTime(0.1); // Set IK solver timeout to 0.1 seconds

            bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            if(!success) //execute
            {
                std::cout << ("No plan found, rotating the FRAME now") << std::endl;

                double angle2 = M_PI / 4.0;

                // Eigen::Vector3d unitX(1.0, 0.0, 0.0);

                // Eigen::Vector3d point1(center, 0.0, height/2); //0.052
                // Eigen::Vector3d point2(target_pose.position.x, target_pose.position.y, floor); //floor

                vector = point1 - point2;
                vector.normalize();

                quaternion.setFromTwoVectors(unitX, vector);

                for (angle2; angle2 < (M_PI * 2) && !success; angle2 += angle2)
                {
                    std::cout << "Angle in radians is: " << angle2 << std::endl;
                    // Get the X-axis vector of the current orientation (local X-axis)
                    vector = quaternion * Eigen::Vector3d::UnitX();

                    // Create an AngleAxis rotation around the local X-axis
                    Eigen::AngleAxisd rotation(angle2, vector);

                    // Apply the rotation to the existing quaternion
                    Eigen::Quaterniond rotated_quaternion = rotation * quaternion;

                    // std::cout << "target_pose._oldorientation.w_old: " << quaternion.w() << std::endl;
                    // std::cout << "target_pose._oldorientation.x_old: " << quaternion.x() << std::endl;
                    // std::cout << "target_pose._oldorientation.y_old: " << quaternion.y() << std::endl;
                    // std::cout << "target_pose._oldorientation.z_old: " << quaternion.z() << std::endl;

                    target_pose.orientation.w = rotated_quaternion.w();
                    target_pose.orientation.x = rotated_quaternion.x();
                    target_pose.orientation.y = rotated_quaternion.y();
                    target_pose.orientation.z = rotated_quaternion.z();

                    // std::cout << "target_pose._oldorientation.w_new: " << target_pose.orientation.w << std::endl;
                    // std::cout << "target_pose._oldorientation.x_new: " << target_pose.orientation.x << std::endl;
                    // std::cout << "target_pose._oldorientation.y_new: " << target_pose.orientation.y << std::endl;
                    // std::cout << "target_pose._oldorientation.z_new: " << target_pose.orientation.z << std::endl;

                    // Call setPoseTarget and plan the motion
                    move_group.setPoseTarget(target_pose);
                    move_group.setPlanningTime(0.2); // Set IK solver timeout to 0.2 seconds
                    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
                }

                // Check if a plan was found
                if (!success)
                {
                    // No plan found for any combination, print the values
                    // throw std::runtime_error("No plan found for any combination of orientation values!");
                    std::cout << "No plan found for any combination of orientation values!" << std::endl;
                }
                else
                {
                    std::cout << "Plan FOUND!" << std::endl;
                }
            }
        }

        void MyPlanningClass::objectPosition()
        {
            Eigen::Vector3d unitX(1.0, 0.0, 0.0);

            Eigen::Vector3d point1(center, 0.0, 0.0);
            Eigen::Vector3d point2(center, 0.0, 0.1);

            Eigen::Vector3d vector = point1 - point2;

            vector.normalize();

            quaternion.setFromTwoVectors(unitX, vector);

            target_pose.orientation.w = quaternion.w();
            target_pose.orientation.x = quaternion.x();
            target_pose.orientation.y = quaternion.y();
            target_pose.orientation.z = quaternion.z();
        }

        void MyPlanningClass::moveCircular()
        {
            // cv::VideoCapture capture(2);
            // move_group.setEndEffector("tool0");
            // move_group.setEndEffectorLink(END_EFFECTOR_LINK);
            std::cout << "MOVE CIRCULAR" << std::endl;
            namespace rvt = rviz_visual_tools;
            moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
            visual_tools.deleteAllMarkers();
            // visual_tools.loadRemoteControl();
            // Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
            // joint_model_group = move_group.getCurrentState() -> getJointModelGroup(PLANNING_GROUP);
            joint_model_group = move_group.getCurrentState() -> getJointModelGroup(PLANNING_GROUP);

            std::ostringstream oss;

            if(!smaller_radius)
            {
                for (floor = 0.1; floor <= height + offset_z; floor += offset_z)
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

                        setQuaternion(); //And plan
                        
                        // geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose();

                        // std::cout << current_pose << std::endl;
                        // std::cout << std::endl;
                        // std::cout << target_pose << std::endl;

                        // move_group.setPoseTarget(target_pose);
                        // move_group.setPlanningTime(0.1); // Set IK solver timeout to 5 seconds
                        // bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
                        // if(!success) //execute
                        // {
                        //     // throw std::runtime_error("No plan found");
                        //     std::cout << "NO PLAN FOUND!" << std::endl;
                        //     continue;
                        // }

                        ROS_INFO_NAMED("Circle", "Visualizing plan 1 as trajectory line");

                        oss.str(std::string());
                        oss << "Floor:" << floor * 10 << " Angle=" << angle << "°";
                        pose_text = oss.str();

                        visual_tools.publishAxisLabeled(target_pose, pose_text);
                        // visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
                        visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
                        visual_tools.trigger();

                        auto error_code = move_group.execute(my_plan); //blocking

                        // cv::VideoCapture capture(2);

                        // cv::Mat frame;
                        // capture.read(frame);

                        // // Save the image to a file
                        // std::string filename = "/home/pilaciv/opencv_images/" + pose_text + ".jpg";
                        // cv::imwrite(filename, frame);
                        // frame.release();
                        // capture.release();

                        camera_srv.request.angle = angle;
                        captureImage();

                        std::cout << "Error code " << error_code << std::endl;
                        // cv::VideoCapture capture(2);
                        // std::string filename = "/home/pilaciv/opencv_images/" + pose_text + ".jpg";
                        // cv::imwrite(filename, cv_ptr->image);
                        // captureImage();
                    }
                }

                smaller_radius = true;
                std::cout << "IM HERE 1 " << std::endl;
            }
            if(smaller_radius)
            {
                // visual_tools.deleteAllMarkers();
                std::cout << "IM HERE 2 " << std::endl;
                step = step * 2; //less points to take
                floor = height + offset_z2;

                if(length >= width)
                {
                    radius = length;
                }
                else
                {
                    radius = width;
                }

                label:

                for (radius; radius >= 0.0; radius -= offset_radius)
                {
                    std::cout << "RADIUS " << radius <<  std::endl;
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
                        std::cout << "ANGLE: " << angle << std::endl;

                        target_pose.position.x = center + radius * cos(angle * RAD2DEG);
                        target_pose.position.y = radius * sin(angle * RAD2DEG);
                        
                        target_pose.position.z = floor;

                        setQuaternion();

                        // move_group.setPoseTarget(target_pose);
                        // move_group.setPlanningTime(0.1); // Set IK solver timeout to 5 seconds
                        // bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
                        // if(!success) //execute
                        // {
                        //     // throw std::runtime_error("No plan found");
                        //     std::cout << "NO PLAN FOUND!" << std::endl;
                        //     continue;
                        // }

                        ROS_INFO_NAMED("Circle", "Visualizing plan 2 as trajectory line");

                        oss.str(std::string());
                        oss << "Radius:" << radius << " Angle=" << angle << "°";
                        pose_text = oss.str();

                        visual_tools.publishAxisLabeled(target_pose, pose_text);
                        // visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
                        visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
                        visual_tools.trigger();

                        auto error_code = move_group.execute(my_plan); //blocking
                        std::cout << "Error code " << error_code << std::endl;

                        if(radius >= 0.0)
                        {
                            captureImage();
                            // cv::VideoCapture capture(2);

                            if (radius == 0)
                            {
                                break;
                            }
                        }
                    }
                }

                if(radius < 0.0 && radius != -offset_radius)
                {
                    radius = 0.0;
                    goto label;
                }
                else
                {
                    smaller_radius = false;
                }
            }
        }

        void MyPlanningClass::goToObject()
        {
            target_pose.position.x = center;
            target_pose.position.y = 0.0;
            target_pose.position.z = 0.1;

            objectPosition();

            move_group.setPoseTarget(target_pose);
            bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            if(!success) //execute
                throw std::runtime_error("No plan found");

            auto error_code = move_group.execute(my_plan); //blocking
            std::cout << "Error code " << error_code << std::endl;
        }

        void MyPlanningClass::captureImage()
        {
            // camera_srv.request.angle = angle;
            client.call(camera_srv);
            client.call(camera_srv);

            // Call the camera service
            if(client.call(camera_srv))
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
                // std::string filename = "/home/pilaciv/opencv_images/" + pose_text + ".jpg";
                
                // selfie ++;

                std::string filename = "/home/pilaciv/opencv_images/selfie" + std::to_string(selfie) + ".jpg";
                cv::imwrite(filename, cv_ptr->image);

                // cv_ptr = nullptr;

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

            joint_positions[0] = 0.0;
            joint_positions[1] = 0.139626;
            joint_positions[2] = -2.79253;
            joint_positions[3] = 3.14159;
            joint_positions[4] = -1.5708;
            joint_positions[5] = -3.14159;

            move_group.setJointValueTarget(joint_positions);
            bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            if(!success)
                throw std::runtime_error("No plan found");

            move_group.move(); //blocking

            captureImage();
            for(int i = 0; i <= 5; i++)
            {
                if(i == 0 || i == 2 || i == 4)
                {
                    joint_positions[3] = 2.35619;
                    move_group.setJointValueTarget(joint_positions);
                    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
                    if(!success)
                        throw std::runtime_error("No plan found");

                    move_group.move(); //blocking
                    captureImage();
                }
                if(i == 1 || i == 3 || i == 5)
                {
                    joint_positions[3] = 3.92699;
                    move_group.setJointValueTarget(joint_positions);
                    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
                    if(!success)
                        throw std::runtime_error("No plan found");

                    move_group.move(); //blocking
                    captureImage();
                }
                if(i == 5)
                {
                    joint_positions[3] = 3.14159;
                    move_group.setJointValueTarget(joint_positions);
                    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
                    if(!success)
                        throw std::runtime_error("No plan found");

                    move_group.move(); //blocking
                    captureImage();
                }
            }
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
            primitive.dimensions[0] = length;
            primitive.dimensions[1] = width;
            primitive.dimensions[2] = height;

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
            ros::Duration(1).sleep();
            virtual_world.addCollisionObjects(collisionObjects);
            ROS_INFO_STREAM("Added: " << blk_name);
        }

        void MyPlanningClass::addObjects()
        {
            double box_pose1[3] = {center, 0, height/2};
            makeBox("box_1", box_pose1);

            // double box_pose2[3] = {0.0, 0.77, 0.0,};
            // makeBox("block_2", box_pose2);
        }

        void MyPlanningClass::removeObjects()
        {
            std::vector<std::string> object_ids;
            object_ids.push_back("box_1");
            ros::Duration(1).sleep();

            // object_ids.push_back("block_2");
            // ros::Duration(1).sleep();
            
            virtual_world.removeCollisionObjects(object_ids);
        }
}
