#include "controller.h"
#include <signal.h>
#include <ros/ros.h>

std::function<void(int)> shutdown_handler;
void signal_handler(int signal) { shutdown_handler(signal); }

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

    std::vector<std::string> input_topics;
    std::string base_link;

    std::cout << "running rate at " << rate << " Hz" << std::endl;

    Controller controller(nh, rate);

    shutdown_handler = [&](int sig)
    {
        controller.reset();
        ros::shutdown();
    };

    signal(SIGINT, signal_handler);

    ros::Rate r(rate);
    while(ros::ok())
    {
        controller.run();
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
