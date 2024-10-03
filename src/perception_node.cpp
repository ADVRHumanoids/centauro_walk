#include "controller.h"
#include "perception.h"
#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "controller_node");
    ros::NodeHandle nh(""), nhpr("~");

    std::vector<std::string> input_topics;
    std::string base_link;

    std::shared_ptr<Perception> perception;
    if (!nhpr.hasParam("input_topics"))
    {
        ROS_ERROR("Missing mandatory private parameter inpout_topics");
    }
    nhpr.getParam("input_topics", input_topics);
    perception = std::make_shared<Perception>(nh, input_topics);

    if (nhpr.hasParam("base_link"))
    {
        nhpr.getParam("base_link", base_link);
        perception->setBaseLink(base_link);
    }

    ros::Rate r(30);
    while(ros::ok())
    {
        perception->update();
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
