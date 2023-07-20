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

    bool is_msg_received();
    bool is_initialized();
    bool init_create_tasks();

    virtual bool update() = 0;

private:
    void gt_pose_callback(const geometry_msgs::PoseStampedConstPtr msg);
    void gt_twist_callback(const geometry_msgs::TwistStampedConstPtr msg);

    ros::Subscriber _gt_pose_sub, _gt_twist_sub;

protected:
    bool _is_callback_done;
    bool _initialized;
    unsigned int _solution_index;
    ros::NodeHandle _nh;
    ros::Subscriber _mpc_sub;

    Vector6d _fb_pose;
    Vector6d _fb_twist;
};

#endif // MPC_HANDLER_H
