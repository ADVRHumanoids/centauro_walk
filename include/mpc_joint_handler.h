#ifndef MPC_JOINT_HANDLER_H
#define MPC_JOINT_HANDLER_H

#include "mpc_handler.h"
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
                    XBot::RobotInterface::Ptr robot = nullptr);

    bool update() override;

private:
    void init_publishers_and_subscribers();

    void mpc_joint_callback(const trajectory_msgs::JointTrajectoryConstPtr msg);

    template<typename key, typename value>
    void vectors_to_map(const std::vector<key> vec1, const Eigen::Matrix<value, 1, -1> vec2, std::unordered_map<key, value>& map)
    {
//        std::vector<value> stl_vec2(vec2.data(), vec2.data() + vec2.size());
//        std::transform(vec1.begin(), vec1.end(), stl_vec2.begin(), std::inserter(map, map.end()), [](key a, value b)
//        {
//            return std::make_pair(a, b);
//        });
        if (vec1.size() != vec2.size())
            throw std::runtime_error("you are trying to merge two vectors of different size in the same map!");

        for (int i = 0; i < vec1.size(); i++)
        {
            map[vec1[i]] = vec2[i];
        }
    }

    XBot::JointNameMap _q;
    XBot::JointNameMap _qdot;
    XBot::JointNameMap _qddot;
    XBot::JointNameMap _tau;
    std::vector<std::string> _joint_names;

    trajectory_msgs::JointTrajectory _mpc_solution;

    XBot::ModelInterface::Ptr _model;
    XBot::RobotInterface::Ptr _robot;
};

#endif // MPC_JOINT_HANDLER_H
