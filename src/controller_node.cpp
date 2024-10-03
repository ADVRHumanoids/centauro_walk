#include "controller.h"
#include "perception.h"
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

    std::vector<std::string> input_topics;
    std::string base_link;

    std::shared_ptr<Perception> perception;
    if (nhpr.hasParam("input_topics"))
    {
        nhpr.getParam("input_topics", input_topics);
        perception = std::make_shared<Perception>(nh, input_topics);

        if (nhpr.hasParam("base_link"))
        {
            nhpr.getParam("base_link", base_link);
            perception->setBaseLink(base_link);
        }
    }

    std::cout << "running rate at " << rate << " Hz" << std::endl;

    Controller controller(nh, rate);

    ros::Rate r(rate);
    while(ros::ok())
    {
        controller.run();
        if(perception)
        {
            perception->update();
        }
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
