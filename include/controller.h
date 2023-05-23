#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <ros/ros.h>

#include <XBotInterface/ModelInterface.h>
#include <XBotInterface/RobotInterface.h>
#include <cartesian_interface/Context.h>
#include <cartesian_interface/utils/RobotStatePublisher.h>
#include <cartesian_interface/CartesianInterfaceImpl.h>
#include <cartesian_interface/ros/RosServerClass.h>
#include <OpenSoT/tasks/acceleration/CoM.h>
#include <RobotInterfaceROS/ConfigFromParam.h>

#include "mpc_joint_handler.h"

#include <sensor_msgs/Joy.h>
#include <flush_me_maybe.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

class Controller {
public:
    Controller(ros::NodeHandle nh, int rate);

    void run();

private:
    void init_load_model();
    void init_load_publishers_and_subscribers();

    void set_stiffness_damping_torque(double duration);

    // Callbacks
    void gt_pose_callback(const geometry_msgs::PoseStampedConstPtr msg);
    void gt_twist_callback(const geometry_msgs::TwistStampedConstPtr msg);

    ros::NodeHandle _nh, _nhpr;
    ros::Subscriber _gt_pose_sub, _gt_twist_sub;
    ros::Publisher _joint_state_pub;

    YAML::Node _cfg;

    XBot::ModelInterface::Ptr _model;
    XBot::RobotInterface::Ptr _robot;
    std::shared_ptr<XBot::Cartesian::Utils::RobotStatePublisher> _rspub;
    Eigen::VectorXd _tau_offset;
    Eigen::Affine3d _base_init;

    XBot::ImuSensor::ConstPtr _imu;

    MPCJointHandler::Ptr _mpc_handler;

    double _time;
    double _rate;
    bool _init;
};

#endif // CONTROLLER
