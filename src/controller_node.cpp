#include "controller.h"
#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "controller_node");
    ros::NodeHandle nh(""), nhpr("~");

    int rate;
    if (!nhpr.hasParam("rate"))
    {
        rate = 100;
    }
    else
    {
        nhpr.getParam("rate", rate);
    }

    std::cout << "running rate at " << rate << " Hz" << std::endl;

    Controller controller(nh, rate);

    ros::Rate r(rate);
    while(ros::ok())
    {
        controller.run();
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
