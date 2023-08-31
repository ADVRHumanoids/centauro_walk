#include <mpc_handler.h>

MPCHandler::MPCHandler(ros::NodeHandle nh):
    _nh(nh),
    _solution_index(0),
    _is_callback_done(false),
    _is_base_pose_received(false),
    _is_base_twist_received(false),
    _initialized(false)
{
    _gt_pose_sub = _nh.subscribe("/base_link/pose", 1, &MPCHandler::gt_pose_callback, this);
    _gt_twist_sub = _nh.subscribe("/base_link/twist", 1, &MPCHandler::gt_twist_callback, this);
}

void MPCHandler::gt_pose_callback(const geometry_msgs::PoseStampedConstPtr msg)
{
    if (!_is_base_pose_received)
    {
        _is_base_pose_received = true;
        _fb_pose.resize(6);
    }

    Eigen::Quaterniond quat(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    Eigen::Vector3d rpy = Eigen::Vector3d(quat.toRotationMatrix().eulerAngles(0, 1, 2)(0),
                                          quat.toRotationMatrix().eulerAngles(0, 1, 2)(1),
                                          quat.toRotationMatrix().eulerAngles(0, 1, 2)(2));

    _fb_pose << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, rpy;
}

void MPCHandler::gt_twist_callback(const geometry_msgs::TwistStampedConstPtr msg)
{
    if (!_is_base_twist_received)
    {
        _is_base_twist_received = true;
        _fb_twist.resize(6);
    }

    _fb_twist << msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z,
                 msg->twist.angular.x, msg->twist.angular.y, msg->twist.angular.z;
}

bool MPCHandler::isMsgReceived() const
{
    return _is_callback_done;
}

bool MPCHandler::isInitialized() const
{
    return _initialized;
}


