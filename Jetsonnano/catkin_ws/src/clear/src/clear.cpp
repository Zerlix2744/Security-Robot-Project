#include <ros/ros.h>
#include <std_srvs/Empty.h>

void clearCostmaps()
{
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
    std_srvs::Empty srv;

    if (client.call(srv))
    {
        ROS_INFO("Costmaps cleared successfully.");
    }
    else
    {
        ROS_ERROR("Failed to call service clear_costmaps");
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "navigation_with_clear_costmaps_cpp");
    ros::NodeHandle nh;

    // Example navigation task
    while (ros::ok())
    {
        // Perform navigation tasks here
        // ...

        // Clear costmaps periodically
        clearCostmaps();
        ros::Duration(2).sleep(); // Adjust the sleep time as needed
    }

    return 0;
}

