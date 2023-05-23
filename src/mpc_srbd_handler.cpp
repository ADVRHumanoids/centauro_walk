#include "mpc_srbd_handler.h"

MPCSRBDHandler::MPCSRBDHandler(XBot::Cartesian::CartesianInterfaceImpl::Ptr ci,
                               ros::NodeHandle nh):
MPCHandler(nh),
_ci(ci)
{
    init_publishers_and_subscribers();
}

void MPCSRBDHandler::mpc_srbd_callback(const kyon_controller::SRBDTrajectoryConstPtr msg)
{
    _is_callback_done = true;
    _mpc_solution = *msg;
    _solution_index = 1;
}

void MPCSRBDHandler::init_publishers_and_subscribers()
{
    _mpc_sub = _nh.subscribe("/mpc_solution", 10, &MPCSRBDHandler::mpc_srbd_callback, this);
}

void MPCSRBDHandler::force_to_wrench(const double x_size, const double y_size)
{
    _wrench_ref_map["l_sole"] << 0., 0., 0., 0., 0., 0.;
    _wrench_ref_map["l_sole"].head(3) += _force_ref_map["l_foot_lower_left_link"].head(3);
    _wrench_ref_map["l_sole"].head(3) += _force_ref_map["l_foot_lower_right_link"].head(3);
    _wrench_ref_map["l_sole"].head(3) += _force_ref_map["l_foot_upper_left_link"].head(3);
    _wrench_ref_map["l_sole"].head(3) += _force_ref_map["l_foot_upper_right_link"].head(3);

    _wrench_ref_map["l_sole"].tail(3) += Eigen::Vector3d(-x_size/2, y_size/2, 0).cross(Eigen::Vector3d(_force_ref_map["l_foot_lower_left_link"][0],
                                                                                                              _force_ref_map["l_foot_lower_left_link"][1],
                                                                                                              _force_ref_map["l_foot_lower_left_link"][2]));
    _wrench_ref_map["l_sole"].tail(3) += Eigen::Vector3d(-x_size/2, -y_size/2, 0).cross(Eigen::Vector3d(_force_ref_map["l_foot_lower_right_link"][0],
                                                                                                               _force_ref_map["l_foot_lower_right_link"][1],
                                                                                                               _force_ref_map["l_foot_lower_right_link"][2]));
    _wrench_ref_map["l_sole"].tail(3) += Eigen::Vector3d(x_size/2, y_size/2, 0).cross(Eigen::Vector3d(_force_ref_map["l_foot_upper_left_link"][0],
                                                                                                             _force_ref_map["l_foot_upper_left_link"][1],
                                                                                                             _force_ref_map["l_foot_upper_left_link"][2]));
    _wrench_ref_map["l_sole"].tail(3) += Eigen::Vector3d(x_size/2, -y_size/2, 0).cross(Eigen::Vector3d(_force_ref_map["l_foot_upper_right_link"][0],
                                                                                                              _force_ref_map["l_foot_upper_right_link"][1],
                                                                                                              _force_ref_map["l_foot_upper_right_link"][2]));

    _wrench_ref_map["r_sole"] << 0., 0., 0., 0., 0., 0.;
    _wrench_ref_map["r_sole"].head(3) += _force_ref_map["r_foot_lower_left_link"].head(3);
    _wrench_ref_map["r_sole"].head(3) += _force_ref_map["r_foot_lower_right_link"].head(3);
    _wrench_ref_map["r_sole"].head(3) += _force_ref_map["r_foot_upper_left_link"].head(3);
    _wrench_ref_map["r_sole"].head(3) += _force_ref_map["r_foot_upper_right_link"].head(3);

    _wrench_ref_map["r_sole"].tail(3) += Eigen::Vector3d(-x_size/2, y_size/2, 0).cross(Eigen::Vector3d(_force_ref_map["r_foot_lower_left_link"][0],
                                                                                                              _force_ref_map["r_foot_lower_left_link"][1],
                                                                                                              _force_ref_map["r_foot_lower_left_link"][2]));
    _wrench_ref_map["r_sole"].tail(3) += Eigen::Vector3d(-x_size/2, -y_size/2, 0).cross(Eigen::Vector3d(_force_ref_map["r_foot_lower_right_link"][0],
                                                                                                               _force_ref_map["r_foot_lower_right_link"][1],
                                                                                                               _force_ref_map["r_foot_lower_right_link"][2]));
    _wrench_ref_map["r_sole"].tail(3) += Eigen::Vector3d(x_size/2, y_size/2, 0).cross(Eigen::Vector3d(_force_ref_map["r_foot_upper_left_link"][0],
                                                                                                             _force_ref_map["r_foot_upper_left_link"][1],
                                                                                                             _force_ref_map["r_foot_upper_left_link"][2]));
    _wrench_ref_map["r_sole"].tail(3) += Eigen::Vector3d(x_size/2, -y_size/2, 0).cross(Eigen::Vector3d(_force_ref_map["r_foot_upper_right_link"][0],
                                                                                                              _force_ref_map["r_foot_upper_right_link"][1],
                                                                                                              _force_ref_map["r_foot_upper_right_link"][2]));
}

bool MPCSRBDHandler::update_com(kyon_controller::SRBDCoordinate msg)
{
    // update com
        // pos:
            // positon of the BASE from the SRBD model
        // vel:
            // velocity of the BASE from the SRBD model
        // acc:
            // acceleration of the BASE from the SRBD model

    _com_pos_ref.translation() << msg.com.x, msg.com.y, msg.com.z;
    _com_vel_ref << msg.com_dot.x, msg.com_dot.y, msg.com_dot.z, 0., 0., 0.;
    _com_acc_ref << msg.com_ddot.x, msg.com_ddot.y, msg.com_ddot.z, 0., 0., 0.;

    return true;
}

bool MPCSRBDHandler::update_base(kyon_controller::SRBDCoordinate msg)
{
    // update base
        // pos (not sent):
            // linear: orientation of the BASE from the SRBD model
            // translation: position of the BASE from the SRBD model
        // vel:
            // linear: -- (not sent)
            // angular: velocity of the BASE from the SRBD model
        // acc:
            // linear: -- (not sent)
            // angular: acceleration of the BASE from the SRBD model

    Eigen::Affine3d w_T_b;
    Eigen::Quaterniond q_ori(msg.base_ori.w, msg.base_ori.x, msg.base_ori.y, msg.base_ori.z);

    w_T_b.linear() = q_ori.toRotationMatrix();
//    w_T_b.linear().setIdentity();
    w_T_b.translation() = _com_pos_ref.translation();


    _base_pos_ref = w_T_b;
    _base_vel_ref << 0., 0., 0., msg.base_vel.x, msg.base_vel.y, msg.base_vel.z;
//    _base_vel_ref << 0., 0., 0., 0., 0., 0.;
    _base_acc_ref << 0., 0., 0., msg.base_acc.x, msg.base_acc.y, msg.base_acc.z;

    return true;
}

bool MPCSRBDHandler::update_contacts(kyon_controller::SRBDCoordinate msg)
{
    // update contacts
        // pos:
            // linear: orientation of the contact from the SRBD model
            // translation: position of the contact from the SRBD model
        // vel:
            // linear: linear velocity of the contact from the SRBD model
            // angular: angular velocity of the contact from the SRBD model
        // acc:
            // linear: ??
            // angular: ??

    for (int i = 0; i < msg.contact_names.size(); i++)
    {
        std::string contact_name = msg.contact_names[i];

        _contact_pos_ref_map[contact_name].translation() << msg.c[i].position.x, msg.c[i].position.y, msg.c[i].position.z;
        Eigen::Quaterniond q(msg.c[i].orientation.w, msg.c[i].orientation.x, msg.c[i].orientation.y, msg.c[i].orientation.z);

        // TODO: restore angular position and velocity references from the MPC
//        _contact_pos_ref_map[contact_name].linear() = q.toRotationMatrix();
        _contact_pos_ref_map[contact_name].linear() = Eigen::Matrix3d::Identity();

//        _contact_vel_ref_map[contact_name] << msg.c_dot[i].linear.x, msg.c_dot[i].linear.y, msg.c_dot[i].linear.z, msg.c_dot[i].angular.x, msg.c_dot[i].angular.y, msg.c_dot[i].angular.z;
         _contact_vel_ref_map[contact_name] << msg.c_dot[i].linear.x, msg.c_dot[i].linear.y, msg.c_dot[i].linear.z, 0, 0, 0;

    }

    return true;
}

bool MPCSRBDHandler::update_forces(kyon_controller::SRBDCoordinate msg)
{
    // update forces
        // linear: forces from the SRBD model
        // torques: -- (not sent)

        // limits: threshold for contact switching

    // Force Version
//    for (int i = 0; i < msg.force_names.size(); i++)
//    {
//        std::string force_name = msg.force_names[i];
//        _force_ref_map[force_name] << msg.forces[i].x, msg.forces[i].y, msg.forces[i].z, 0., 0., 0.;

//        if (_force_ref_map[force_name].norm() < 0.1)
//        {
//            _f_min_map[force_name] << 0., 0., 0., 0., 0., 0.;
//            _f_max_map[force_name] << _f_min_map[force_name];

//        }
//        else
//        {
//            _f_min_map[force_name] << -1000., -1000., -1000., -1000., -1000., -1000.;
//            _f_max_map[force_name] = -1 * _f_min_map[force_name];
//        }
//    }

    // Wrench Version
    for (int j = 0; j < msg.force_names.size(); j++)
    {
        _force_ref_map[msg.force_names[j]] << msg.forces[j].x, msg.forces[j].y, msg.forces[j].z, 0., 0., 0.;
    }
    force_to_wrench(0.2, 0.1);

    for (auto wrench_name : msg.contact_names)
    {
        if (_wrench_ref_map[wrench_name].norm() < 0.1)
        {
            _f_min_map[wrench_name] << 0., 0., 0., 0., 0., 0.;
            _f_max_map[wrench_name] << _f_min_map[wrench_name];

        }
        else
        {
            _f_min_map[wrench_name] << -1000., -1000., -1000., -15., -10., -10.;
            _f_max_map[wrench_name] = -1 * _f_min_map[wrench_name];
        }
    }

    return true;
}

bool MPCSRBDHandler::send_references()
{
    _com_task->setPoseReference(_com_pos_ref);
    _com_task->setVelocityReference(_com_vel_ref);
    _com_task->setAccelerationReference(_com_acc_ref);

    _base_ori_task->setPoseReference(_base_pos_ref);
    _base_ori_task->setVelocityReference(_base_vel_ref);
    _base_ori_task->setAccelerationReference(_base_acc_ref);


    for (auto contact_name : _contact_names)
    {
        _contact_task_map[contact_name]->setPoseReference(_contact_pos_ref_map[contact_name]);
        _contact_task_map[contact_name]->setVelocityReference(_contact_vel_ref_map[contact_name]);
    }


    for (auto force_name : _force_names)
    {
        if (_solution_index < _mpc_solution.srbd_trajectory.size() - 1)
        {
            _flim_task_map[force_name]->setLimits(_f_min_map[force_name], _f_max_map[force_name]);
            // Force Version
//            _force_task_map[force_name]->setForceReference(_force_ref_map[force_name]);
            // Wrench Version
            _force_task_map[force_name]->setForceReference(_wrench_ref_map[force_name]);
        }
    }

    return true;

}

bool MPCSRBDHandler::init_create_tasks()
{
    kyon_controller::SRBDCoordinate msg;
    msg = _mpc_solution.srbd_trajectory[_solution_index];


    auto com_t = _ci->getTask("com");
    _com_task = std::dynamic_pointer_cast<XBot::Cartesian::ComTask>(com_t);
    if(_com_task == nullptr)
        return false;

    auto base_t = _ci->getTask("DWYTorso");
    _base_ori_task = std::dynamic_pointer_cast<XBot::Cartesian::CartesianTask>(base_t);
    if (_base_ori_task == nullptr)
        return false;

    for (int i = 0; i < msg.contact_names.size(); i++)
    {
        _contact_names.push_back(msg.contact_names[i]);

        auto contact_t = _ci->getTask(msg.contact_names[i]);
        auto contact_task = std::dynamic_pointer_cast<XBot::Cartesian::CartesianTask>(contact_t);
        if (contact_task == nullptr)
            return false;

        _contact_task_map[msg.contact_names[i]] = contact_task;

        // Wrench version
        _force_names.push_back(msg.contact_names[i]);
        auto force_lim_task = _ci->getTask("force_lims_" + _force_names[i]);
        auto flim_task = std::dynamic_pointer_cast<XBot::Cartesian::acceleration::ForceLimits>(force_lim_task);
        if (flim_task == nullptr)
            return false;
        _flim_task_map[_force_names[i]] = flim_task;

        auto force_t = _ci->getTask("force_" + _force_names[i]);
        auto force_task = std::dynamic_pointer_cast<XBot::Cartesian::acceleration::ForceTask>(force_t);
        if (force_task == nullptr)
            return false;
        _force_task_map[_force_names[i]] = force_task;
    }

    // Force Version
//    for (int i = 0; i < msg.force_names.size(); i++)
//    {

//        _force_names.push_back(msg.force_names[i]);
//        auto force_lim_task = _ci->getTask("force_lims_" + msg.force_names[i]);
//        auto flim_task = std::dynamic_pointer_cast<XBot::Cartesian::acceleration::ForceLimits>(force_lim_task);
//        if (flim_task == nullptr)
//            return false;

//        _flim_task_map[msg.force_names[i]] = flim_task;

//        auto force_t = _ci->getTask("force_" + msg.force_names[i]);
//        auto force_task = std::dynamic_pointer_cast<XBot::Cartesian::acceleration::ForceTask>(force_t);
//        if (force_task == nullptr)
//            return false;

//        _force_task_map[msg.force_names[i]] = force_task;
//    }

    _initialized = true;
    _solution_index = 0;

    return true;

}


bool MPCSRBDHandler::update()
{
    kyon_controller::SRBDCoordinate msg;
    msg = _mpc_solution.srbd_trajectory[_solution_index];

    update_com(msg);
    update_base(msg);
    update_contacts(msg);
    update_forces(msg);

    send_references();

    if (_solution_index == _mpc_solution.srbd_trajectory.size() - 1)
    {
        _solution_index = _mpc_solution.srbd_trajectory.size() - 1;
    }
    else
        _solution_index++;

    return true;
}



