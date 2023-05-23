#include <mpc_handler.h>


MPCHandler::MPCHandler(ros::NodeHandle nh):
    _nh(nh),
    _solution_index(0),
    _is_callback_done(false),
    _initialized(false)
{}

bool MPCHandler::is_msg_received()
{
    return _is_callback_done;
}

bool MPCHandler::is_initialized()
{
    return _initialized;
}
