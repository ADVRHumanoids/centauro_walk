#include "controller.h"
#include <chrono>
#include <thread>
#include <xbot_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>

#include <cartesio_acceleration_support/Force.h>
#include <cartesio_acceleration_support/ForceLimits.h>

#include <eigen_conversions/eigen_msg.h>

#include <kdl_conversions/kdl_msg.h>

Controller::Controller(ros::NodeHandle nh, int rate):
_nh(nh),
_nhpr("~"),
_time(0),
_rate(rate),
_init(false)
{
    init_load_config();
    init_load_model();
    init_load_publishers_and_subscribers();

    _mpc_handler = std::make_shared<MPCJointHandler>(_nh, _model, _rate, _robot);
}

void Controller::init_load_config()
{
    if(!_nhpr.hasParam("config"))
    {
        ColoredTextPrinter::print("Missing 'config' parameter, using default \n", ColoredTextPrinter::TextColor::Yellow);
    }
    else
    {
        std::string config_string;
        _nhpr.getParam("config", config_string);
        _config = YAML::Load(config_string);

        if (!_config["control_mode"])
        {
            ColoredTextPrinter::print("Missing 'control_mode', using default \n", ColoredTextPrinter::TextColor::Yellow);
        }
        else
        {
            ColoredTextPrinter::print("Setting ctrl mode: \n", ColoredTextPrinter::TextColor::Green);
            for (std::pair<std::string, int> pair : _config["control_mode"].as<std::map<std::string, int>>())
            {
                ColoredTextPrinter::print(pair.first + ": " + std::to_string(pair.second) + "\n", ColoredTextPrinter::TextColor::Green);
                _ctrl_map[pair.first] = XBot::ControlMode::FromBitset(pair.second);
            }
        }

        if (!_config["stiffness"])
        {
            ColoredTextPrinter::print("Missing 'stiffness', using default \n", ColoredTextPrinter::TextColor::Yellow);
        }
        else
        {
            for (std::pair<std::string, double> pair : _config["stiffness"].as<std::map<std::string, double>>())
            {
                _stiffness_map[pair.first] = pair.second;
            }
        }

        if (!_config["damping"])
        {
            ColoredTextPrinter::print("Missing 'damping', using default \n", ColoredTextPrinter::TextColor::Yellow);
        }
        else
        {
            for (std::pair<std::string, double> pair : _config["damping"].as<std::map<std::string, double>>())
            {
                _damping_map[pair.first] = pair.second;
            }
        }
    }
}

void Controller::init_load_model()
{
    // Create instance of ModelInferace and RobotInterface
    auto cfg = XBot::ConfigOptionsFromParamServer();
    _model = XBot::ModelInterface::getModel(cfg);
    Eigen::VectorXd qhome;
    _model->getRobotState("home", qhome);
    _model->setJointPosition(qhome);
    _model->update();

    // Add offset to move the world in the middle of the feet
//    Eigen::Affine3d lfoot;
//    _model->getPose("ball_1", lfoot);
//    lfoot.translation()(1) = 0;
//    lfoot.linear().setIdentity();
//    _model->setFloatingBasePose(lfoot.inverse());
//    _model->update();

    try
    {
        _robot = XBot::RobotInterface::getRobot(cfg);
//        _imu = _robot->getImu().begin()->second;
        _tau_offset.setZero(_robot->getJointNum());
        if (!_ctrl_map.empty())
        {
            _robot->setControlMode(_ctrl_map);
        }
        else
        {
            _robot->setControlMode(XBot::ControlMode::PosImpedance() + XBot::ControlMode::Effort());
        }
        _robot->setPositionReference(qhome.tail(_robot->getJointNum()));
        _robot->move();
    }
    catch(std::runtime_error& e)
    {
        ROS_WARN("RobotInterface not initialized");
    }

    _rspub = std::make_shared<XBot::Cartesian::Utils::RobotStatePublisher>(_model);
}

void Controller::set_stiffness_damping_torque(double duration)
{
    // initialize the cartesian interface with the current position of the robot
    _robot->sense(false);
    _model->syncFrom(*_robot);
    _model->update();

    // prepare to set stiffness and damping to zero
    XBot::JointNameMap K, D;
    _robot->getStiffness(K);
    _robot->getDamping(D);

    XBot::JointNameMap q, q_ref, qdot, qdot_ref;
    _robot->getJointPosition(q);
    _robot->getPositionReference(q_ref);

    _robot->getJointVelocity(qdot);
    _robot->getVelocityReference(qdot_ref);

    XBot::JointNameMap tau_start, tau_goal, tau;
    for (auto pair : K)
    {
        tau_start[pair.first] = K[pair.first] * (q_ref[pair.first] - q[pair.first]) + D[pair.first] * (qdot_ref[pair.first] - qdot[pair.first]);
    }

    _robot->setStiffness(_stiffness_map);
    _robot->setDamping(_damping_map);
    _robot->move();

    double T = _time + duration;
    double dt = 1./_rate;

    // set stiffness and damping to zero while setting pure torque control
    // continuously update ci to smooth transition
    _robot->sense(false);
    _model->syncFrom(*_robot);
    _model->update();
    _model->getJointEffort(tau_goal);


    while (_time < T)
    {
        for (auto pair : tau_start)
        {
            tau[pair.first] = tau_start[pair.first] + (tau_goal[pair.first] - tau_start[pair.first]) * _time / T;
        }


        _robot->setEffortReference(tau);
        _robot->move();

        std::this_thread::sleep_for(std::chrono::milliseconds(int(dt * 1000)));
        ros::spinOnce();
        _time += dt;
    }
}

void Controller::init_load_publishers_and_subscribers()
{
//    _gt_pose_sub = _nh.subscribe("/xbotcore/link_state/base_link/pose", 1, &Controller::gt_pose_callback, this);
//    _gt_twist_sub = _nh.subscribe("/xbotcore/link_state/base_link/twist", 1, &Controller::gt_twist_callback, this);
    _joint_state_pub = _nh.advertise<xbot_msgs::JointState>("joint_state", 10);
}

//void Controller::gt_pose_callback(const geometry_msgs::PoseStampedConstPtr msg)
//{
//    Eigen::Affine3d T;
//    tf::poseMsgToEigen(msg->pose, T);
//    _model->setFloatingBasePose(T);
//    _model->update();
//}

//void Controller::gt_twist_callback(const geometry_msgs::TwistStampedConstPtr msg)
//{
//    Eigen::Matrix<double, 6, 1> twist;
//    tf::twistMsgToEigen(msg->twist, twist);
//    _model->setFloatingBaseTwist(twist);
//    _model->update();
//}

void Controller::run()
{
    if(_mpc_handler->is_msg_received())
    {
        if (_init)
        {
            _mpc_handler->update(); // update model
        }
    }
    else
    {
        return;
    }

    if (!_init)
    {
        _init = true;
        set_stiffness_damping_torque(0.1);
    }
}

