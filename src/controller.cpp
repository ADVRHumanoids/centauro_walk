#include "controller.h"
#include "utils.h"
#include <chrono>
#include <thread>
#include <xbot_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>

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

    _mpc_handler = std::make_shared<MPCJointHandler>(_nh, _model, _rate, _config, _robot, _fixed_joints_map);
    _mpc_handler->setTorqueOffset(_tau_offset);
}

void Controller::reset()
{
    std::cout << "Setting initial stiffness and damping" << std::endl;

    _robot->setControlMode(XBot::ControlMode::Stiffness() + XBot::ControlMode::Damping());

    double duration = 0.1;
    double T = _time + duration;
    double dt = 1./_rate;

    XBot::JointNameMap K, D, K_start, D_start;
    _robot->getStiffness(K_start);
    _robot->getDamping(D_start);
    K = K_start;
    D = D_start;

    while (_time < T)
    {
        for (auto stiff : _init_stiffness)
        {
            K[stiff.first] = (K_start[stiff.first] + (stiff.second - K_start[stiff.first]) * _time / T);
        }

        for (auto damp : _init_damping)
        {
            D[damp.first] = (D_start[damp.first] + (damp.second - D_start[damp.first]) * _time / T);
        }

        _robot->setStiffness(K);
        _robot->setDamping(D);
        _robot->move();

        std::this_thread::sleep_for(std::chrono::milliseconds(int(dt * 1000)));
        ros::spinOnce();
        _time += dt;
    }
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

        // remove from controller the joints labeled as fixed
        if (_config["fixed_joints"])
        {
            _fixed_joints_map  = _config["fixed_joints"].as<std::map<std::string, double>>();

            for (auto [name, _] : _fixed_joints_map)
            {
                _zero_ctrl_map[name] = XBot::ControlMode::Idle();
            }

            ColoredTextPrinter::print("Fixing joints: ", ColoredTextPrinter::TextColor::Green);
            for (auto elem : _fixed_joints_map)
            {
                ColoredTextPrinter::print(elem.first + ": " + std::to_string(elem.second), ColoredTextPrinter::TextColor::Green);
            }
        }

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

    try
    {
        _robot = XBot::RobotInterface::getRobot(cfg);

        _robot->sense();
        _robot->getStiffness(_init_stiffness);
        _robot->getDamping(_init_damping);

        // set all joint modes to pos impedance + effort
        set_control_mode_map(XBot::ControlMode::PosImpedance() + XBot::ControlMode::Effort());

        // set initial control mode
        _robot->setControlMode(_init_ctrl_map);

        // prepare control map as pos + vel + effort
        set_control_mode_map(XBot::ControlMode::Position() + XBot::ControlMode::Velocity() + XBot::ControlMode::Effort());


        if(_nhpr.hasParam("torque_offset"))
        {
            std::string tau_offset_string;
            _nhpr.getParam("torque_offset", tau_offset_string);
            YAML::Node tau_offset_node = YAML::Load(tau_offset_string);
            std::map<std::string, double> tau_offset = tau_offset_node.as<std::map<std::string, double>>();
            ColoredTextPrinter::print("Using torque offset: \n", ColoredTextPrinter::TextColor::Green);
            std::cout << std::endl;
            for (auto pair : tau_offset)
            {
                _tau_offset[pair.first] = pair.second;
                ColoredTextPrinter::print(pair.first + " - " + std::to_string(pair.second), ColoredTextPrinter::TextColor::Green);
                std::cout << std::endl;
            }
        }
        else
        {
            ColoredTextPrinter::print("No torque offset provided. Setting zero. \n", ColoredTextPrinter::TextColor::Yellow);
            _robot->getJointPosition(_tau_offset);
            for (auto &pair : _tau_offset)
                pair.second = 0;
        }
    }
    catch(std::runtime_error& e)
    {
        ROS_WARN("RobotInterface not initialized");
    }
}

void Controller::set_stiffness_damping_torque(double duration)
{
    // initialize with the current position of the robot
    _robot->sense();

    // prepare to set stiffness and damping to zero (and only zero)
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

    _robot->getJointEffort(tau_goal);

    double T = _time + duration;
    double dt = 1./_rate;

    while (_time < T)
    {
        for (auto pair : tau_start)
        {
            tau[pair.first] = (tau_start[pair.first] + (tau_goal[pair.first] - tau_start[pair.first]) * _time / T) - _tau_offset[pair.first];
        }

        _robot->setStiffness(_stiffness_map);
        _robot->setDamping(_damping_map);
        _robot->setEffortReference(tau);
        _robot->move();

        std::this_thread::sleep_for(std::chrono::milliseconds(int(dt * 1000)));
        ros::spinOnce();
        _time += dt;
    }
}

void Controller::set_control_mode_map(XBot::ControlMode cm)
{

    for (auto elem : _robot->getEnabledJointNames())
    {
        _init_ctrl_map[elem] = cm;
    }

    if (!_ctrl_map.empty())
    {
        // overwrite with entries in _ctrl_map (custom from the user)
        for (const auto& pair : _ctrl_map)
        {
            _init_ctrl_map[pair.first] = pair.second;
        }
    }

    if (!_zero_ctrl_map.empty())
    {
        // overwrite with entries in _ctrl_map (custom from the user)
        for (const auto& pair : _zero_ctrl_map)
        {
                _init_ctrl_map[pair.first] = pair.second;
        }
    }

}

void Controller::set_stiffness_damping(double duration)
{
    // initialize with the current position of the robot
    _robot->sense();

    XBot::JointNameMap K, D, K_start, D_start;
    _robot->getStiffness(K_start);
    _robot->getDamping(D_start);
    K = K_start;
    D = D_start;

    double T = _time + duration;
    double dt = 1./_rate;

    while (_time < T)
    {
        for (auto stiff : _stiffness_map)
        {
            K[stiff.first] = (K_start[stiff.first] + (stiff.second - K_start[stiff.first]) * _time / T);
        }

        for (auto damp : _damping_map)
        {
            D[damp.first] = (D_start[damp.first] + (damp.second - D_start[damp.first]) * _time / T);
        }

        _robot->setStiffness(K);
        _robot->setDamping(D);
        _robot->move();

        std::this_thread::sleep_for(std::chrono::milliseconds(int(dt * 1000)));
        ros::spinOnce();
        _time += dt;
    }
}

void Controller::init_load_publishers_and_subscribers()
{
    _joint_state_pub = _nh.advertise<xbot_msgs::JointState>("joint_state", 10);
}

void Controller::run()
{
    if(_mpc_handler->isMsgReceived())
    {
        if (_init)
        {
            _mpc_handler->update();
            return;
        }
    }
    else
    {
        return;
    }

    if (!_init)
    {
        _init = true;
        // set_stiffness_damping_torque(0.01);

        if (!_stiffness_map.empty() || !_damping_map.empty())
        {
            set_stiffness_damping_torque(0.1);
        }

        _robot->setControlMode(_init_ctrl_map);
    }
}
