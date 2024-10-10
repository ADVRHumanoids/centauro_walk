#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "mpc_joint_handler.h"

#include <ros/ros.h>

#include <XBotInterface/ModelInterface.h>
#include <XBotInterface/RobotInterface.h>
#include <OpenSoT/tasks/acceleration/CoM.h>
#include <RobotInterfaceROS/ConfigFromParam.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

class Controller {
public:
    Controller(ros::NodeHandle nh, int rate);

    void reset();
    void run();

private:
    void init_load_config();
    void init_load_model();
    void init_load_publishers_and_subscribers();

    void set_stiffness_damping(double duration);
    void set_stiffness_damping_torque(double duration);

    void set_control_mode_map(XBot::ControlMode mode);

    double _horizon_duration;
    int _n_nodes;

    std::map<std::string, double> _fixed_joints_map;

    ros::NodeHandle _nh, _nhpr;
    ros::Publisher _joint_state_pub;
    YAML::Node _config;

    YAML::Node _cfg;

    XBot::ModelInterface::Ptr _model;
    XBot::RobotInterface::Ptr _robot;
    std::map<std::string, XBot::ControlMode> _init_ctrl_map;
    XBot::JointNameMap _init_stiffness, _init_damping;
    std::map<std::string, XBot::ControlMode> _ctrl_map;
    std::map<std::string, XBot::ControlMode> _zero_ctrl_map;

    std::unordered_map<std::string, double> _stiffness_map, _damping_map;
    XBot::JointNameMap _tau_offset;
    Eigen::Affine3d _base_init;

    XBot::ImuSensor::ConstPtr _imu;

    MPCJointHandler::Ptr _mpc_handler;

    double _time;
    double _rate;
    bool _init;
};

#endif // CONTROLLER
