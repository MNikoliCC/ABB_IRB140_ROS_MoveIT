#include <planning.h>

sensor_msgs::Image::ConstPtr latest_image;

void cameraCallback(const sensor_msgs::Image::ConstPtr& image_msg)
{
    latest_image = image_msg;
}

bool getSimulatedCameraImage(planning::CameraMsg::Request &req,
                    planning::CameraMsg::Response &resp)
{
    if (!latest_image)
    {
        ROS_ERROR("No image available from the camera.");
        return false;
    }

    // Assign the latest image to the response
    resp.image = *latest_image;

    std::cout << req.angle << std::endl;

    return true;
}

bool getWebCameraImage(planning::CameraMsg::Request &req,
                    planning::CameraMsg::Response &resp)
{
    // int numCameras = 10;

    // for (int i = 0; i < numCameras; i++)
    // {
    //     cv::VideoCapture capture(i);
    //     if (capture.isOpened())
    //     {
    //         std::cout << "Camera " << i << " is available." << std::endl;
    //         capture.release(); // Release the camera for further use
    //     }
    // }

    // Capture image from the web camera
    cv::VideoCapture capture(2);  // 0 indicates the default camera device

    // capture.set(cv::CAP_PROP_FRAME_WIDTH, 4000);           // Set frame width
    // capture.set(cv::CAP_PROP_FRAME_HEIGHT, 3000);         // Set frame height
    // capture.set(cv::CAP_PROP_FPS, frame_rate);              // Set frame rate
    // capture.set(cv::CAP_PROP_BRIGHTNESS, brightness);       // Set brightness
    // capture.set(cv::CAP_PROP_CONTRAST, contrast);           // Set contrast
    // capture.set(cv::CAP_PROP_EXPOSURE, exposure);           // Set exposure
    // capture.set(cv::CAP_PROP_WHITE_BALANCE_BLUE_U, wb_blue);// Set white balance (blue channel)
    // capture.set(cv::CAP_PROP_WHITE_BALANCE_RED_V, wb_red);  // Set white balance (red channel)
    // capture.set(cv::CAP_PROP_GAIN, gain);                   // Set gain

    if (!capture.isOpened())
    {
        ROS_ERROR("Failed to open camera.");
        return false;
    }

    cv::Mat frame;
    capture.read(frame);

    // Convert OpenCV image to ROS image message
    sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();

    // Assign the image message to the response
    resp.image = *img_msg;

    std::cout << req.angle << std::endl;

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "capture_image_node");
    ros::NodeHandle nh;

    ros::Subscriber image_sub = nh.subscribe("/image_raw", 1, cameraCallback);

    // ros::ServiceServer service = nh.advertiseService("capture_image", getSimulatedCameraImage); // For simulated camera

    ros::ServiceServer service = nh.advertiseService("capture_image", getWebCameraImage); // For web camera

    ROS_INFO("Get Image Service is running...");

    // ros::AsyncSpinner spinner(2);
    // spinner.start();

    // spinner.stop();

    ros::spin();

    return 0;
}