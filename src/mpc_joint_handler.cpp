#include "mpc_joint_handler.h"
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <eigen_conversions/eigen_msg.h>

#include "utils.h"

MPCJointHandler::MPCJointHandler(ros::NodeHandle nh,
                                 XBot::ModelInterface::Ptr model,
                                 int rate,
                                 YAML::Node config,
                                 XBot::RobotInterface::Ptr robot):
MPCHandler(nh),
_model(model),
_robot(robot),
_rate(rate),
_flag_id(true)
{
    init_publishers_and_subscribers();

    _model->getJointPosition(_q);
    _model->getJointVelocity(_qdot);
    _model->getJointAcceleration(_qddot);
    _model->getJointEffort(_tau);

    auto urdf_model = std::make_shared<urdf::ModelInterface>(_robot->getUrdf());


    if (config["fixed_joints"])
    {
        _fixed_joints_map  = config["fixed_joints"].as<std::map<std::string, double>>();

        for (auto pair : _fixed_joints_map) {
            _fixed_joints.push_back(pair.first);
        }

        ColoredTextPrinter::print("Fixing joints: ", ColoredTextPrinter::TextColor::Green);
        for (auto elem : _fixed_joints_map)
        {
            ColoredTextPrinter::print(elem.first + ": " + std::to_string(elem.second), ColoredTextPrinter::TextColor::Green);
        }
    }

    _resampler = std::make_unique<Resampler>(urdf_model, _fixed_joints_map);

    _resampler_pub = _nh.advertise<sensor_msgs::JointState>("/resampler_solution_position", 1, true);
}

void MPCJointHandler::mpc_joint_callback(const kyon_controller::WBTrajectoryConstPtr msg)
{
//    if (!_mpc_solution.q.empty())
//        _old_solution = _mpc_solution;
//    else
//        _old_solution = *msg;

    ColoredTextPrinter::print("MPC message received:", ColoredTextPrinter::TextColor::Green);

    _mpc_solution = *msg;

    if (!_is_callback_done)
    {
        _joint_names.insert(_joint_names.begin(), std::begin(_mpc_solution.joint_names), std::end(_mpc_solution.joint_names));
//        _joint_names.insert(_joint_names.begin(), {"VIRTUALJOINT_1", "VIRTUALJOINT_2", "VIRTUALJOINT_3", "VIRTUALJOINT_4", "VIRTUALJOINT_5", "VIRTUALJOINT_6"});

        _x.resize(_mpc_solution.q.size() + _mpc_solution.v.size());
        _u.resize(_mpc_solution.a.size() + _mpc_solution.force_names.size() * 6);

        _p.resize(_resampler->nq());
        _v.resize(_resampler->nv());
        _a.resize(_resampler->nv());
        _f.resize(_mpc_solution.force_names.size() * 6);

        std::vector<std::string> frames(_mpc_solution.force_names.data(), _mpc_solution.force_names.data() + _mpc_solution.force_names.size());
        _resampler->setFrames(frames);

        if (frames.empty())
        {
            _flag_id = false;
        }
    }



    // set state and input to Resampler (?)
    _robot->sense();

    // getting the state from the robot (position and velocity)
    // q map for joint positions from the robot
    XBot::JointNameMap q_map;
    // qdot vector for velocities from the robot
    Eigen::VectorXd qdot(_robot->getJointNum());

//    _robot->getJointPosition(q_map);
//    _robot->getJointVelocity(qdot);

//    Eigen::VectorXd q_pinocchio = _resampler->mapToQ(q_map);

    // from eigen to quaternion
    // (FOR CLOSED LOOP)
//    _p << _fb_pose, q_pinocchio.tail(_resampler->nq() - 7);
//    _v << _fb_twist, qdot;

    // getting the input from the MPC solution (FOR OPEN LOOP)
    _p = Eigen::VectorXd::Map(_mpc_solution.q.data(), _mpc_solution.q.size());
    _v = Eigen::VectorXd::Map(_mpc_solution.v.data(), _mpc_solution.v.size());
    _a = Eigen::VectorXd::Map(_mpc_solution.a.data(), _mpc_solution.a.size());

//    if (!_mpc_solution.j.empty())
//        _j = Eigen::VectorXd::Map(_mpc_solution.j.data(), _mpc_solution.j.size());

    for (int i = 0; i < _mpc_solution.force_names.size(); i++)
    {
        _f.block<6, 1>(i * 6, 0) << _mpc_solution.f[i].x, _mpc_solution.f[i].y, _mpc_solution.f[i].z, 0, 0, 0;
    }

    _x << _p, _v;
    _u << _a, _f;

    if(!_resampler->setState(_x))
        throw std::runtime_error("wrong dimension of the state vector! " + std::to_string(_x.size()) + " != ");
    if(!_resampler->setInput(_u))
        throw std::runtime_error("wrong dimension of the input vector! " + std::to_string(_u.size()) + " != ");


    _is_callback_done = true;
    _solution_index = 1;
}

void MPCJointHandler::init_publishers_and_subscribers()
{
    _mpc_sub = _nh.subscribe("/mpc_solution", 1, &MPCJointHandler::mpc_joint_callback, this);
}

void MPCJointHandler::setTorqueOffset(XBot::JointNameMap tau_offset)
{
    _tau_offset = tau_offset;
}

void MPCJointHandler::smooth(const Eigen::VectorXd state, const Eigen::VectorXd input, Eigen::VectorXd& out)
{
    double alpha = 0.1;
    out = alpha * input + (1-alpha) * state;
}

bool MPCJointHandler::update()
{
    _robot->sense();

    // resample
    _resampler->resample(1./_rate);

    // get resampled state and set it to the robot
    Eigen::VectorXd tau;
    _resampler->getState(_x);

    if (_flag_id)
    {
        _resampler->getTau(tau);
    }

    _p = _x.head(_p.size());
    _v = _x.segment(_p.size(), _v.size());
    _a = _u.head(_a.size());

    msg_pub.position.clear();
    msg_pub.velocity.clear();
    msg_pub.position.assign(_p.data(), _p.data() + _p.size());
    msg_pub.velocity.assign(_v.data(), _v.data() + _v.size());
    
    if (_flag_id)
    {
        msg_pub.effort.assign(tau.data(), tau.data() + tau.size());
    }

    _resampler_pub.publish(msg_pub);


    // q_euler does not have the fixed joints inside
    Eigen::VectorXd q_euler(_model->getJointNum());
    q_euler = _resampler->getMinimalQ(_x.head(_resampler->nq()));

    // create variables:
    // reduced to account for fixed joints,
    // smooth and current_robot to filter the MPC reference
    Eigen::VectorXd q_reduced(_robot->getJointNum() - _fixed_joints.size()), qdot_reduced(_robot->getJointNum() - _fixed_joints.size());
    Eigen::VectorXd q_smooth(_robot->getJointNum() - _fixed_joints.size()), qdot_smooth(_robot->getJointNum() - _fixed_joints.size());
    XBot::JointNameMap q_current_robot, qdot_current_robot;

    // get current state from robot for smoothing
    _robot->getPositionReference(q_current_robot);
    _robot->getVelocityReference(qdot_current_robot);

    // if fixed joints, remove them from the state
    for (auto name : _fixed_joints)
    {
        auto q_it = q_current_robot.find(name);
        if (q_it != q_current_robot.end())
        {
            q_current_robot.erase(q_it);
        }

        auto qdot_it = qdot_current_robot.find(name);
        if (qdot_it != qdot_current_robot.end())
        {
            qdot_current_robot.erase(qdot_it);
        }
    }

    // transform from JointNameMap to Eigen for smoothing, considering the order of q_euler and _v (= order of joint_names)
    int q_index = 0;
    for (auto el_name : _joint_names)
    {
        q_reduced(q_index) = q_current_robot[el_name];
        q_index++;
    }

    int qdot_index = 0;
    for (auto el_name : _joint_names)
    {
        qdot_reduced(qdot_index) = qdot_current_robot[el_name];
        qdot_index++;
    }

//    q_current << _fb_pose_rpy, q_current_robot;
//    qdot_current << _fb_twist, q_current_robot;

    // smooth mpc reference with current reference (q_euler and _v comes from the MPC, so they have the floating base joints)
    // careful floating base assumed to be rpy and removed
    smooth(q_reduced, q_euler.tail(q_euler.size() - 6), q_smooth);
    smooth(qdot_reduced, _v.tail(_v.size() - 6), qdot_smooth);

    // zip togheter joint names and relative values (joint names comes from MPC message, it does not contain fixed joint strings)
    vectors_to_map<std::string, double>(_joint_names, q_smooth, _q);
    vectors_to_map<std::string, double>(_joint_names, qdot_smooth, _qdot);


    if (_flag_id)
    {
        vectors_to_map<std::string, double>(_joint_names, tau.tail(tau.size() - 6), _tau);


        for (auto &pair : _tau)
            pair.second -= _tau_offset[pair.first];
    }

    _robot->setPositionReference(_q);
    _robot->setVelocityReference(_qdot);

    if (_flag_id)
    { 
        _robot->setEffortReference(_tau);
    }
    
    _robot->move();

    return true;
}
