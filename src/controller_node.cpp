#include "controller.h"
#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "controller_node");
    ros::NodeHandle nh("");

    Controller controller(nh, 100);

    ros::Rate r(100);
    while(ros::ok())
    {
        controller.run();
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
