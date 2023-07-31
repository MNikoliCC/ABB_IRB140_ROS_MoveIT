#include <ros/ros.h>
#include <ros/package.h>
#include <urdf/model.h>
#include <string>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "parameter_setter");
    ros::NodeHandle nh;

    double height = 0.32;
    double width = 0.28;
    double length = 0.25;
    double center = 0.5;
    double offset = 0.1;
    double value = height / 2;

    // Set the parameters
    nh.setParam("/param_height", height);
    nh.setParam("/param_width", width);
    nh.setParam("/param_length", length);
    nh.setParam("/param_center", center);
    nh.setParam("/param_offset", offset);
    nh.setParam("/param_value", value);

    ROS_INFO("Parameters have been set.");

    // // Load the xacro file with updated parameters
    // std::string urdf_path = ros::package::getPath("your_package_name") + "/my_robot.urdf.xacro";
    // urdf::ModelInterfaceSharedPtr model_interface = urdf::parseURDFFile(urdf_path);

    // if (!model_interface)
    // {
    //     ROS_ERROR("Failed to load URDF from file: %s", urdf_path.c_str());
    //     return -1;
    // }

    // // Write the URDF model to a string
    // std::string urdf_string;
    // model_interface->writeModelToString(urdf_string);

    // // Load the URDF to the ROS parameter server
    // nh.setParam("/robot_description", urdf_string);

    // ROS_INFO("URDF has been loaded to the parameter server.");

    return 0;
}
