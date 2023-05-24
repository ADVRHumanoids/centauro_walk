#include "mpc_joint_handler.h"
#include <trajectory_msgs/JointTrajectoryPoint.h>

MPCJointHandler::MPCJointHandler(ros::NodeHandle nh,
                                 XBot::ModelInterface::Ptr model,
                                 XBot::RobotInterface::Ptr robot):
MPCHandler(nh),
_model(model),
_robot(robot)
{
    init_publishers_and_subscribers();

    _model->getJointPosition(_q);
    _model->getJointVelocity(_qdot);
    _model->getJointAcceleration(_qddot);
    _model->getJointEffort(_tau);
}

void MPCJointHandler::mpc_joint_callback(const trajectory_msgs::JointTrajectoryConstPtr msg)
{
    _mpc_solution = *msg;
    if (!_is_callback_done)
    {
        _joint_names.insert(_joint_names.begin(), std::begin(_mpc_solution.joint_names), std::end(_mpc_solution.joint_names));
        _joint_names.insert(_joint_names.begin(), {"VIRTUALJOINT_1", "VIRTUALJOINT_2", "VIRTUALJOINT_3", "VIRTUALJOINT_4", "VIRTUALJOINT_5", "VIRTUALJOINT_6"});
    }
    _is_callback_done = true;
    _solution_index = 1;
}

void MPCJointHandler::init_publishers_and_subscribers()
{
    _mpc_sub = _nh.subscribe("/mpc_solution", 10, &MPCJointHandler::mpc_joint_callback, this);
}

bool MPCJointHandler::update()
{
    // Read the mpc solution
    trajectory_msgs::JointTrajectoryPoint trj_point;
    trj_point = _mpc_solution.points[_solution_index];
    std::vector<std::string> joint_names(_mpc_solution.joint_names.data(), _mpc_solution.joint_names.data() + _mpc_solution.joint_names.size());

    // From quaternion to RPY
    Eigen::Quaterniond quat(trj_point.positions[6], trj_point.positions[3], trj_point.positions[4], trj_point.positions[5]);
    auto rpy = quat.toRotationMatrix().eulerAngles(0, 1, 2);
    Eigen::VectorXd q_euler;
    q_euler.head(6) << trj_point.positions[0], trj_point.positions[1], trj_point.positions[2], rpy[0], rpy[1], rpy[2];
    q_euler.tail(_model->getJointNum() - 6) = Eigen::VectorXd::Map(trj_point.positions.data() + 7, trj_point.positions.size() - 7);

    vectors_to_map<std::string, double>(_joint_names, q_euler, _q);
    vectors_to_map<std::string, double>(_joint_names, Eigen::VectorXd::Map(trj_point.velocities.data(), trj_point.velocities.size()), _qdot);
    vectors_to_map<std::string, double>(_joint_names, Eigen::VectorXd::Map(trj_point.accelerations.data(), trj_point.accelerations.size()), _qddot);
    vectors_to_map<std::string, double>(_joint_names, Eigen::VectorXd::Map(trj_point.effort.data(), trj_point.effort.size()), _qdot);

    if (_robot)
    {
        _robot->sense();
        _robot->setPositionReference(_q);
        _robot->setVelocityReference(_qdot);
        _robot->setEffortReference(_tau);
        _robot->move();
    }
    else
    {
        _model->setJointPosition(_q);
        _model->setJointVelocity(_qdot);
        _model->setJointAcceleration(_qddot);
        _model->update();
    }

    if (_solution_index == _mpc_solution.points.size() - 1)
    {
        _solution_index = _mpc_solution.points.size() - 1;
    }
    else
        _solution_index++;

    return true;
}
