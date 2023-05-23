#ifndef FLUSH_ME_MAYBE_H
#define FLUSH_ME_MAYBE_H

#include <ros/ros.h>

#include <Eigen/Dense>
#include <kyon_controller/FlushMeMaybe.h>

namespace Eigen
{
    typedef Eigen::Matrix<double, 6, 1> Vector6d;
}

namespace XBot {

class FlushMeMaybe
{
public:
    typedef std::shared_ptr<FlushMeMaybe> Ptr;

    FlushMeMaybe(std::string topic_name = "flush_me_maybe_topic");
    ~FlushMeMaybe() = default;

    void add(std::string name, Eigen::Vector3d vec);
    void add(std::string name, Eigen::Affine3d T);
    void add(std::string name, Eigen::Vector6d twist);
    void add(std::string name, double value);

    void publish(std::string topic_name, Eigen::Vector3d vec);
    void publish(std::string topic_name, Eigen::Affine3d T);
    void publish(std::string topic_name, Eigen::Vector6d twist);
    void publish(std::string topic_name, double value);

    void flush();

private:
    ros::Publisher _pub;
    ros::NodeHandle _nh;

    kyon_controller::FlushMeMaybe _msg;

}; }

#endif
