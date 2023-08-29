#include "resampler.h"

Resampler::Resampler(urdf::ModelInterfaceSharedPtr urdf_model, std::vector<std::string> frames, int sys_order):
_urdf(urdf_model),
_frames(frames),
_sys_order(sys_order),
_time(0)
{
    // parse pinocchio model from urdf
    pinocchio::urdf::buildModel(urdf_model, _model);
    _data = std::make_unique<pinocchio::Data>(_model);

    if (_sys_order != 2  && _sys_order != 3)
    {
        throw std::runtime_error("Resampler works with systems of order 2 and 3 only!");
    }

    if (!_frames.empty())
    {
        resize();
    }
}

void Resampler::resize()
{
    // resize state and input vectors
    if (_sys_order == 2)
    {
        _x.resize(_model.nq + _model.nv);
        _u.resize(_model.nv + 6 * _frames.size());
    }
    else if (_sys_order == 3)
    {
        _x.resize(_model.nq + _model.nv + _model.nv + 6 * _frames.size());
        _u.resize(_model.nv + 6 * _frames.size());
    }

    // resize xdot and double integrator vectors
    _xdot.resize(_x.size());
    _k1.resize(_xdot.size());
    _k2.resize(_xdot.size());
    _k3.resize(_xdot.size());
    _k4.resize(_xdot.size());

    // resize velocity (quaternion compatible) and torque vectors
    _v.resize(_model.nq);
    _tau.resize(_model.nv);

    // resize Jacobian
    _J.setZero(6, _model.nv);
}

void Resampler::guard_function()
{
    if (_frames.empty())
        throw std::runtime_error("You forgot to assign contact frames. You can do that from Resampler Constructor or using setFrames() function.");
}

bool Resampler::setState(const Eigen::VectorXd x)
{
    guard_function();
    if (x.size() != _x.size())
    {
        return false;
    }

    _x = x;
    _time = 0;
    return true;
}

bool Resampler::setInput(const Eigen::VectorXd u)
{
    guard_function();
    if (u.size() != _u.size())
    {
        return false;
    }

    _u = u;
    return true;
}

void Resampler::setFrames(const std::vector<std::string> frames)
{
    _frames = frames;
    resize();
}

void Resampler::getState(Eigen::VectorXd &x) const
{
    x = _x;
}

void Resampler::getInput(Eigen::VectorXd &u) const
{
    u = _u;
}

void Resampler::getTau(Eigen::VectorXd & tau) const
{
    tau = _tau;
}

void Resampler::getFrames(std::vector<std::string> &frames) const
{
    frames = _frames;
}

void Resampler::id()
{
    guard_function();
    if (_sys_order == 2)
    {
        pinocchio::rnea(_model, *_data, _x.segment(0, _model.nq), _x.segment(_model.nq, _model.nv), _u.segment(0, _model.nv));
    }
    else if (_sys_order == 3)
    {
        pinocchio::rnea(_model, *_data, _x.segment(0, _model.nq), _x.segment(_model.nq, _model.nv), _x.segment(_model.nq + _model.nv, _model.nv));
    }
    _tau = _data->tau;

//    std::cout << "tau: \n" << _data->tau.transpose() << std::endl;

//    std::cout << "x: " << _x.transpose() << std::endl;
    for (int i = 0; i < _frames.size(); i++)
    {
//        std::cout << "f_" << _frames[i] << ": " << _x.segment(_model.nq + _model.nv + _model.nv + i*6, 6).transpose() << std::endl;
        Eigen::MatrixXd J;
        J.setZero(6, _model.nv);
        auto frame_id = _model.getFrameId(_frames[i]);
        pinocchio::computeJointJacobians(_model, *_data, _x.head(_model.nq));
        pinocchio::framesForwardKinematics(_model, *_data, _x.head(_model.nq));
        pinocchio::getFrameJacobian(_model, *_data, frame_id, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J);
        if (_sys_order == 2)
        {
            _tau -= J.transpose() * _u.segment(_model.nv + i*6, 6);
        }
        else if (_sys_order == 3)
        {
            _tau -= J.transpose() * _x.segment(_model.nq + _model.nv + _model.nv + i*6, 6);
        }
    }
}

void Resampler::double_integrator(const Eigen::VectorXd &x, const Eigen::VectorXd &u, Eigen::VectorXd &xdot)
{
    qdot(x.segment(0, _model.nq), x.segment(_model.nq, _model.nv), _v);

    if (_sys_order == 2)
    {
        xdot << _v, _u.segment(0, _model.nv);
    }
    else if (_sys_order == 3)
    {
        xdot << _v, x.segment(_model.nq + _model.nv, _model.nv), _u;
    }
}

void Resampler::rk4(double dt_res)
{
    double_integrator(_x, _u, _k1);
    double_integrator(_x + dt_res / 2 * _k1, _u, _k2);
    double_integrator(_x + dt_res / 2 * _k2, _u, _k3);
    double_integrator(_x + dt_res * _k3, _u, _k4);

    _x = _x + dt_res / 6. * (_k1 + 2 * _k2 + 2 * _k3 + _k4);
    _time += dt_res;
}

void Resampler::qdot(Eigen::Ref<const Eigen::VectorXd> qeig,
                     Eigen::Ref<const Eigen::VectorXd> veig,
                     Eigen::Ref<Eigen::VectorXd> qdot)
{
    for(int i = 0; i < _model.njoints; i++)
    {
        int nq = _model.nqs[i];

        if(nq == 0)
        {
            continue;
        }

        auto jname = _model.names[i];
        auto uj = _urdf->getJoint(jname);
        int iv = _model.idx_vs[i];
        int iq = _model.idx_qs[i];

        if(nq == 7)
        {
            Eigen::Quaterniond quat(
                        qeig[iq+6],
                        qeig[iq+3],
                        qeig[iq+4],
                        qeig[iq+5]);

            Eigen::Quaterniond qomega(
                        0,
                        veig[iv+3],
                        veig[iv+4],
                        veig[iv+5]);

            Eigen::Vector3d pdot = quat * veig.segment<3>(iv);

            Eigen::Vector4d quat_dot = 0.5 * (quat * qomega).coeffs();

            qdot.segment<3>(iq) = pdot;

            qdot.segment<4>(iq+3) = quat_dot;

        }
        else if(nq == 2)
        {
            qdot[iq] = -veig[iv]*qeig[iq+1];
            qdot[iq+1] = veig[iv]*qeig[iq];
        }
        else if(nq == 1)
        {
            qdot[iq] = veig[iv];
        }
        else
        {
            throw std::runtime_error("invalid nq: " + std::to_string(nq));
        }
    }
}

void Resampler::resample(double dt_res)
{
    rk4(dt_res);
    id();
}
