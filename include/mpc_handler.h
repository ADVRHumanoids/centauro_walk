#ifndef MPC_HANDLER_H
#define MPC_HANDLER_H

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

class MPCHandler
{
public:
    typedef std::shared_ptr<MPCHandler> Ptr;
    typedef Eigen::Matrix<double, 6, 1> Vector6d;

    MPCHandler(ros::NodeHandle nh);

    bool isMsgReceived() const;
    bool isInitialized() const;

    virtual bool update() = 0;

private:
    void gt_pose_callback(const geometry_msgs::PoseStampedConstPtr msg);
    void gt_twist_callback(const geometry_msgs::TwistStampedConstPtr msg);

    ros::Subscriber _gt_pose_sub, _gt_twist_sub;

protected:
    bool _is_callback_done, _is_base_pose_received, _is_base_twist_received;
    bool _initialized;
    unsigned int _solution_index;
    ros::NodeHandle _nh;
    ros::Subscriber _mpc_sub;

    Eigen::VectorXd _fb_pose;
    Vector6d _fb_pose_rpy;
    Eigen::VectorXd _fb_twist;
};

#endif // MPC_HANDLER_H
