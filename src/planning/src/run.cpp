#include <planning.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "planning_node");
    ros::NodeHandle nh;

    my_planning::MyPlanningClass my_planning_;

    my_planning_.client = nh.serviceClient<planning::CameraMsg>("capture_image");

    ros::AsyncSpinner spinner(2);
    spinner.start();

    if(argc != 2)
    {
        ROS_INFO(" ");
        ROS_INFO("\tUsage:");
        ROS_INFO(" ");
        ROS_INFO("\trosrun planning run  n");
        return 1;
    }

    int selection = atoi(argv[1]);
    switch(selection)
    {
        case 1:
        {
            my_planning_.goToPoseGoal();
        }
            break;
        case 2:
        {
            my_planning_.goToJointState();
        }
            break;
        case 3:
        {
            my_planning_.cartesianPath();
            my_planning_.resetValues();
        }
            break;
        case 4:  
            my_planning_.addObjects();
            break;
        case 5:
            my_planning_.removeObjects();
            break;
        case 6:
        {
            my_planning_.cartesianPath();
            my_planning_.resetValues();
            my_planning_.moveCircular();
        }
            break;
        case 7:
        {
            my_planning_.cartesianPath();
            my_planning_.resetValues();
            my_planning_.goToObject();
        }
            break;
    }


    spinner.stop();
    return 0;
}