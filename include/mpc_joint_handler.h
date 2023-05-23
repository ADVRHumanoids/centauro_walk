#ifndef MPC_JOINT_HANDLER_H
#define MPC_JOINT_HANDLER_H

#include "mpc_handler.h"
#include <trajectory_msgs/JointTrajectory.h>
#include <Eigen/Dense>

#include <XBotInterface/ModelInterface.h>
#include <XBotInterface/RobotInterface.h>
#include <cartesian_interface/utils/RobotStatePublisher.h>

class MPCJointHandler : public MPCHandler {
public:
    MPCJointHandler(ros::NodeHandle nh,
                    XBot::ModelInterface::Ptr model,
                    XBot::RobotInterface::Ptr robot = nullptr);

    bool update() override;

private:
    void init_publishers_and_subscribers();

    void mpc_joint_callback(const trajectory_msgs::JointTrajectoryConstPtr msg);

    Eigen::VectorXd _q;
    Eigen::VectorXd _qdot;
    Eigen::VectorXd _qddot;
    Eigen::VectorXd _tau;

    trajectory_msgs::JointTrajectory _mpc_solution;

    XBot::ModelInterface::Ptr _model;
    XBot::RobotInterface::Ptr _robot;
};

#endif // MPC_JOINT_HANDLER_H
