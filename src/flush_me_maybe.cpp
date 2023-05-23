#include <flush_me_maybe.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <eigen_conversions/eigen_msg.h>

using namespace XBot;

FlushMeMaybe::FlushMeMaybe(std::string topic_name):
_nh("")
{
    _pub = _nh.advertise<kyon_controller::FlushMeMaybe>(topic_name, 10);
}

void FlushMeMaybe::add(std::string name, Eigen::Vector3d vec)
{
    auto it = std::find(_msg.vector_names.begin(), _msg.vector_names.end(), name);
    if (it == _msg.vector_names.end())
    {
        _msg.vector_names.push_back(name);

        geometry_msgs::Vector3 vec_msg;
        tf::vectorEigenToMsg(vec, vec_msg);
        _msg.vectors.push_back(vec_msg);
    }
    else
    {
        int index = it - _msg.vector_names.begin();
        geometry_msgs::Vector3 vec_msg;
        tf::vectorEigenToMsg(vec, vec_msg);
        _msg.vectors[index] = vec_msg;
    }
}

void FlushMeMaybe::add(std::string name, Eigen::Affine3d T)
{
    auto it = std::find(_msg.pose_names.begin(), _msg.pose_names.end(), name);
    if (it == _msg.pose_names.end())
    {
        _msg.pose_names.push_back(name);

        geometry_msgs::Pose T_msg;
        tf::poseEigenToMsg(T, T_msg);
        _msg.poses.push_back(T_msg);
    }
    else
    {
        int index = it - _msg.pose_names.begin();
        geometry_msgs::Pose T_msg;
        tf::poseEigenToMsg(T, T_msg);
        _msg.poses[index] = T_msg;
    }
}

void FlushMeMaybe::add(std::string name, Eigen::Vector6d twist)
{
    auto it = std::find(_msg.twist_names.begin(), _msg.twist_names.end(), name);
    if (it == _msg.twist_names.end())
    {
        _msg.twist_names.push_back(name);

        geometry_msgs::Twist twist_msg;
        tf::twistEigenToMsg(twist, twist_msg);
        _msg.twists.push_back(twist_msg);
    }
    else
    {
        int index = it - _msg.twist_names.begin();
        geometry_msgs::Twist twist_msg;
        tf::twistEigenToMsg(twist, twist_msg);
        _msg.twists[index] = twist_msg;
    }
}

void FlushMeMaybe::add(std::string name, double value)
{
    auto it = std::find(_msg.value_names.begin(), _msg.value_names.end(), name);
    if (it == _msg.value_names.end())
    {
        _msg.value_names.push_back(name);
        _msg.values.push_back(value);
    }
    else
    {
        int index = it - _msg.value_names.begin();
        _msg.values[index] = value;
    }
}


void FlushMeMaybe::publish(std::string topic_name, Eigen::Vector3d vec)
{}

void FlushMeMaybe::publish(std::string topic_name, Eigen::Affine3d T)
{}

void FlushMeMaybe::publish(std::string topic_name, Eigen::Vector6d twist)
{}

void FlushMeMaybe::publish(std::string topic_name, double value)
{}

void FlushMeMaybe::flush()
{
    _msg.header.stamp = ros::Time::now();
    _pub.publish(_msg);
}
