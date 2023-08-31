#include "mpc_joint_handler.h"
#include <trajectory_msgs/JointTrajectoryPoint.h>

MPCJointHandler::MPCJointHandler(ros::NodeHandle nh,
                                 XBot::ModelInterface::Ptr model,
                                 int rate,
                                 XBot::RobotInterface::Ptr robot):
MPCHandler(nh),
_model(model),
_robot(robot),
_rate(rate)
{
    init_publishers_and_subscribers();

    _model->getJointPosition(_q);
    _model->getJointVelocity(_qdot);
    _model->getJointAcceleration(_qddot);
    _model->getJointEffort(_tau);

//    _flusher = std::make_shared<XBot::FlushMeMaybe>();
    auto urdf_model = std::make_shared<urdf::ModelInterface>(_model->getUrdf());
    _resampler = std::make_unique<Resampler>(urdf_model);
}

void MPCJointHandler::mpc_joint_callback(const kyon_controller::WBTrajectoryConstPtr msg)
{
    if (!_mpc_solution.q.empty())
        _old_solution = _mpc_solution;
    else
        _old_solution = *msg;

    _mpc_solution = *msg;

    if (!_is_callback_done)
    {
        _joint_names.insert(_joint_names.begin(), std::begin(_mpc_solution.joint_names), std::end(_mpc_solution.joint_names));
        _joint_names.insert(_joint_names.begin(), {"VIRTUALJOINT_1", "VIRTUALJOINT_2", "VIRTUALJOINT_3", "VIRTUALJOINT_4", "VIRTUALJOINT_5", "VIRTUALJOINT_6"});     

        // second order system
        _x.resize(_old_solution.q.size() + _old_solution.v.size());
        _u.resize(_old_solution.a.size() + _old_solution.force_names.size() * 6);

        // third order system
//        _x.resize(_old_solution.q.size() + _old_solution.v.size() + _old_solution.a.size() + (_old_solution.force_names.size() * 6));
//        _u.resize(_old_solution.j.size() + _old_solution.force_names.size() * 6);

        _f.resize(_old_solution.force_names.size() * 6);
        _fdot.resize(_old_solution.force_names.size() * 6);

        _p.resize(_model->getJointNum() + 1);
        _v.resize(_model->getJointNum());
        _a.resize(_model->getJointNum());

        std::vector<std::string> frames(_old_solution.force_names.data(), _old_solution.force_names.data() + _old_solution.force_names.size());
        _resampler->setFrames(frames);
    }

    // set state and input to Resampler (?)
    Eigen::VectorXd q(_robot->getJointNum()), qdot(_robot->getJointNum());
    _robot->getJointPosition(q);
    _robot->getJointVelocity(qdot);

    // from eigen to quaternion
    Eigen::Quaterniond quat;
    std::cout << "fb_pose: " << _fb_pose.transpose() << std::endl;
    quat = Eigen::AngleAxisd(_fb_pose(3), Eigen::Vector3d::UnitX()) *
           Eigen::AngleAxisd(_fb_pose(4), Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxisd(_fb_pose(5), Eigen::Vector3d::UnitZ());

    _p << _fb_pose.head(3), quat.coeffs(), q;
    _v << _fb_twist, qdot;

//    _p = Eigen::VectorXd::Map(_old_solution.q.data(), _old_solution.q.size());
//    _v = Eigen::VectorXd::Map(_old_solution.v.data(), _old_solution.v.size());
    _a = Eigen::VectorXd::Map(_old_solution.a.data(), _old_solution.a.size());
    if (!_old_solution.j.empty())
        _j = Eigen::VectorXd::Map(_old_solution.j.data(), _old_solution.j.size());

    for (int i = 0; i < _old_solution.force_names.size(); i++)
    {
        _f.block<6, 1>(i * 6, 0) << _old_solution.f[i].x, _old_solution.f[i].y, _old_solution.f[i].z, 0, 0, 0;
    }

    if (!_old_solution.fdot.empty())
    {
        for (int i = 0; i < _old_solution.force_names.size(); i++)
        {
            _fdot.block<6, 1>(i * 6, 0) << _old_solution.fdot[i].x, _old_solution.fdot[i].y, _old_solution.fdot[i].z, 0, 0, 0;
        }
    }

    // second order system
    _x << _p, _v;
    _u << _a, _f;

    // third order system
//    _x << _p, _v, _a, _f;
//    _u << _j, _fdot;

    if(!_resampler->setState(_x))
        throw std::runtime_error("wrong dimension of the state vector! " + std::to_string(_x.size()) + " != ");
    if(!_resampler->setInput(_u))
        throw std::runtime_error("wrong dimension of the input vector! " + std::to_string(_u.size()) + " != ");


    _is_callback_done = true;
    _solution_index = 1;
}

void MPCJointHandler::init_publishers_and_subscribers()
{
    _mpc_sub = _nh.subscribe("/mpc_solution", 10, &MPCJointHandler::mpc_joint_callback, this);
}

bool MPCJointHandler::update()
{
    _robot->sense();
    _model->syncFrom(*_robot);
    _model->update();

    // resample
    // TODO: add guard to check when we exceed the dt_MPC
    _resampler->resample(1./_rate);

    // get resampled state and set it to the robot
    std::vector<std::string> joint_names(_mpc_solution.joint_names.data(), _mpc_solution.joint_names.data() + _mpc_solution.joint_names.size());
    Eigen::VectorXd tau;
    _resampler->getState(_x);
    _resampler->getTau(tau);
    _p = _x.head(_p.size());
    _v = _x.segment(_p.size(), _v.size());
    _a = _x.segment(_p.size() + _v.size(), _a.size());

//    std::cout << "P: " << _p.transpose() << std::endl;

    // From quaternion to RPY
    Eigen::Quaterniond quat(_p(6), _p(3), _p(4), _p(5));
    Eigen::Vector3d rpy = quat.toRotationMatrix().eulerAngles(0, 1, 2);
    Eigen::VectorXd q_euler(_model->getJointNum());
    q_euler.head(6) << _p(0), _p(1), _p(2), rpy;
    q_euler.tail(_model->getJointNum() - 6) = _p.tail(_p.size() - 7);


//    std::cout << "JOINT_NAMES: " << std::endl;
//    for (auto name : _joint_names)
//        std::cout << name << std::endl;

//    std::cout << "Q_EULER: " << q_euler.transpose() << std::endl;

    vectors_to_map<std::string, double>(_joint_names, q_euler, _q);
    vectors_to_map<std::string, double>(_joint_names, _v, _qdot);
    vectors_to_map<std::string, double>(_joint_names, _a, _qddot);
    vectors_to_map<std::string, double>(_joint_names, tau, _tau);

//    std::cout << "Q: " << std::endl;
//    for (auto pair : _q)
//        std::cout << pair.first << ": " << pair.second << std::endl;

//    std::cout << "TAU: " << std::endl;
//    for(auto pair : _tau)
//        std::cout << pair.first << ": " << pair.second << std::endl;

//    _flusher->add(joint_names,
//                  q_euler,
//                  Eigen::VectorXd::Map(trj_point.velocities.data(), trj_point.velocities.size()),
//                  Eigen::VectorXd::Map(trj_point.effort.data(), trj_point.effort.size()));

    _robot->setPositionReference(_q);
    _robot->setVelocityReference(_qdot);
    _robot->setEffortReference(_tau);
    _robot->move();

//    if (_solution_index == _mpc_solution.points.size() - 1)
//    {
//        _solution_index = _mpc_solution.points.size() - 1;
//    }
//    else
//        _solution_index++;

//    _flusher->flush();

    return true;
}
