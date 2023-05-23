#ifndef MPC_HANDLER_H
#define MPC_HANDLER_H

#include <ros/ros.h>

class MPCHandler
{
public:
    typedef std::shared_ptr<MPCHandler> Ptr;

    MPCHandler(ros::NodeHandle nh);

    bool is_msg_received();
    bool is_initialized();
    bool init_create_tasks();

    virtual bool update() = 0;

protected:
    bool _is_callback_done;
    bool _initialized;
    unsigned int _solution_index;
    ros::NodeHandle _nh;
    ros::Subscriber _mpc_sub;
};

#endif // MPC_HANDLER_H
