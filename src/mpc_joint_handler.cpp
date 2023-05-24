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

    _q.resize(_model->getJointNum());
    _qdot.resize(_model->getJointNum());
    _qddot.resize(_model->getJointNum());
    _tau.resize(_model->getJointNum());
}

void MPCJointHandler::mpc_joint_callback(const trajectory_msgs::JointTrajectoryConstPtr msg)
{
    _mpc_solution = *msg;
    _is_callback_done = true;
    _solution_index = 1;
    std::cout << "francescoruscelli" << std::endl;
}

void MPCJointHandler::init_publishers_and_subscribers()
{
    _mpc_sub = _nh.subscribe("/mpc_solution", 10, &MPCJointHandler::mpc_joint_callback, this);
}

bool MPCJointHandler::update()
{
    std::cout << "camadonna" << std::endl;
    // Read the mpc solution
    trajectory_msgs::JointTrajectoryPoint trj_point;
    trj_point = _mpc_solution.points[_solution_index];

    // From quaternion to RPY
    Eigen::Quaterniond q(trj_point.positions[6], trj_point.positions[3], trj_point.positions[4], trj_point.positions[5]);
    auto rpy = q.toRotationMatrix().eulerAngles(0, 1, 2);

    _q.head(3) << trj_point.positions[0], trj_point.positions[1], trj_point.positions[2];
    _q.block<3, 1>(3, 1) = rpy;
    _q.tail(_model->getJointNum() - 6) = Eigen::VectorXd::Map(trj_point.positions.data() + 6, trj_point.positions.size() - 6);
    _qdot = Eigen::VectorXd::Map(trj_point.velocities.data(), trj_point.velocities.size());
    _qddot = Eigen::VectorXd::Map(trj_point.accelerations.data(), trj_point.accelerations.size());
    _tau = Eigen::VectorXd::Map(trj_point.effort.data(), trj_point.effort.size());

    if (_robot)
    {
        _robot->sense();
        _robot->setPositionReference(_q.tail(_model->getJointNum() - 6));
        _robot->setVelocityReference(_qdot.tail(_model->getJointNum() - 6));
        _robot->setEffortReference(_tau.tail(_model->getJointNum() - 6));
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
