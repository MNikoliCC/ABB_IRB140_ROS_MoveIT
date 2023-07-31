#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

#include <planning.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "get_transform_example");
    ros::NodeHandle nh;

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);

    ros::Rate rate(10.0); // Rate at which you want to check the transform (e.g., 10 Hz)

    while (ros::ok())
    {
        try
        {
            // Get the transform from "base_link" to "tool0" at the current time
            geometry_msgs::TransformStamped transform = tf_buffer.lookupTransform("base_link", "tool0", ros::Time(0));

            // Extracting the translation and rotation from the transform
            geometry_msgs::Vector3 translation = transform.transform.translation;
            geometry_msgs::Quaternion rotation = transform.transform.rotation;

            ROS_INFO("Transform from base_link to tool0:");
            ROS_INFO("Translation: x=%.2f, y=%.2f, z=%.2f", translation.x, translation.y, translation.z);
            // ROS_INFO("Rotation: x=%.2f, y=%.2f, z=%.2f, w=%.2f", rotation.x, rotation.y, rotation.z, rotation.w);
        }
        catch (tf2::TransformException& ex)
        {
            ROS_WARN("Failed to get transform between base_link and tool0: %s", ex.what());
        }

        rate.sleep();
    }

    return 0;
}
