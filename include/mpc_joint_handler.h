#ifndef MPC_JOINT_HANDLER_H
#define MPC_JOINT_HANDLER_H

#include "resampler.h"
#include "mpc_handler.h"
#include "flush_me_maybe.h"
#include <kyon_controller/WBTrajectory.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <Eigen/Dense>

#include <XBotInterface/ModelInterface.h>
#include <XBotInterface/RobotInterface.h>
#include <cartesian_interface/utils/RobotStatePublisher.h>

#include <algorithm>

class MPCJointHandler : public MPCHandler {
public:
    MPCJointHandler(ros::NodeHandle nh,
                    XBot::ModelInterface::Ptr model,
                    int rate,
                    XBot::RobotInterface::Ptr robot = nullptr);

    bool update() override;

private:
    void init_publishers_and_subscribers();

    void mpc_joint_callback(const kyon_controller::WBTrajectoryConstPtr msg);

    template<typename key, typename value>
    void vectors_to_map(const std::vector<key> vec1, const Eigen::Matrix<value, 1, -1> vec2, std::unordered_map<key, value>& map)
    {
        if (vec1.size() != vec2.size())
            throw std::runtime_error("you are trying to merge two vectors of different size in the same map!");

        for (int i = 0; i < vec1.size(); i++)
        {
            map[vec1[i]] = vec2[i];
        }
    }

    XBot::JointNameMap _q, _qdot, _qddot, _tau;

    Eigen::VectorXd _p, _v, _a, _f;
    Eigen::VectorXd _j, _fdot;

    std::vector<std::string> _joint_names;

    Eigen::VectorXd _x, _u;
    kyon_controller::WBTrajectory _mpc_solution, _old_solution;
    Resampler::UniquePtr _resampler;

    XBot::FlushMeMaybe::Ptr _flusher;

    XBot::ModelInterface::Ptr _model;
    XBot::RobotInterface::Ptr _robot;
    int _rate;
};

#endif // MPC_JOINT_HANDLER_H
