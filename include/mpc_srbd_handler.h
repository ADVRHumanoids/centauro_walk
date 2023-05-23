#ifndef MPC_SRBD_HANDLER_H
#define MPC_SRBD_HANDLER_H

#include "mpc_handler.h"
#include <kyon_controller/SRBDCoordinate.h>
#include <kyon_controller/SRBDTrajectory.h>
#include <ros/ros.h>

#include <cartesian_interface/CartesianInterfaceImpl.h>

#include <cartesio_acceleration_support/Force.h>
#include <cartesio_acceleration_support/ForceLimits.h>



class MPCSRBDHandler : public MPCHandler {
public:
    MPCSRBDHandler(XBot::Cartesian::CartesianInterfaceImpl::Ptr ci,
                   ros::NodeHandle nh);

private:   
    void init_publishers_and_subscribers();
    bool init_create_tasks();

    bool update_com(kyon_controller::SRBDCoordinate msg);
    bool update_base(kyon_controller::SRBDCoordinate msg);
    bool update_contacts(kyon_controller::SRBDCoordinate msg);
    bool update_forces(kyon_controller::SRBDCoordinate msg);
    bool send_references();
    bool update() override;

    void force_to_wrench(const double x_size, const double y_size);

    void mpc_srbd_callback(const kyon_controller::SRBDTrajectoryConstPtr msg);

    ros::NodeHandle _nh;

    XBot::Cartesian::CartesianInterfaceImpl::Ptr _ci;

    XBot::Cartesian::ComTask::Ptr _com_task;
    XBot::Cartesian::CartesianTask::Ptr _base_ori_task;

    std::map<std::string, XBot::Cartesian::CartesianTask::Ptr> _contact_task_map;

    std::map<std::string, XBot::Cartesian::acceleration::ForceLimits::Ptr> _flim_task_map;
    std::map<std::string, XBot::Cartesian::acceleration::ForceTask::Ptr> _force_task_map;

    std::vector<std::string> _contact_names;
    std::vector<std::string> _force_names;

    Eigen::Affine3d _com_pos_ref, _base_pos_ref;
    Eigen::Vector6d _com_vel_ref, _com_acc_ref;
    Eigen::Vector6d _base_vel_ref, _base_acc_ref;
    std::map<std::string, Eigen::Affine3d> _contact_pos_ref_map;
    std::map<std::string, Eigen::Vector6d> _contact_vel_ref_map, _contact_acc_ref_map;
    std::map<std::string, Eigen::Vector6d> _force_ref_map;
    std::map<std::string, Eigen::Vector6d> _wrench_ref_map;
    std::map<std::string, Eigen::Vector6d> _f_min_map, _f_max_map;

    kyon_controller::SRBDTrajectory _mpc_solution;
};

#endif // MPC_SRBD_HANDLER_H
