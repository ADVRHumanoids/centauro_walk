#include "resampler.h"

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/frames.hpp>

Resampler::Resampler(urdf::ModelInterfaceSharedPtr urdf_model, std::vector<std::string> frames):
_urdf(urdf_model),
_frames(frames)
{
    // parse pinocchio model from urdf
    pinocchio::urdf::buildModel(urdf_model, _model);
    _data = std::make_unique<pinocchio::Data>(_model);

    _x.resize(_model.nq + _model.nv + _model.nv + 3*_frames.size());
    _u.resize(_model.nv + 3*_frames.size());
    _xdot.resize(_x.size() - 1);
    _k1.resize(_xdot.size());
    _k2.resize(_xdot.size());
    _k3.resize(_xdot.size());
    _k4.resize(_xdot.size());

    _v.resize(_model.nv);
    _tau.resize(_model.nv);

    // resize Jacobian
    _J.setZero(6, _model.nv);
}

bool Resampler::setState(const Eigen::VectorXd x)
{
    if (x.size() != _model.nq + _model.nv + _model.nv + 3*_frames.size())
    {
        return false;
    }

    _x = x;
    return true;
}

bool Resampler::setInput(const Eigen::VectorXd u)
{
    if (u.size() != _model.nv + _model.nv + _model.nv + 3*_frames.size())
    {
        return false;
    }

    _u = u;
    return true;
}

void Resampler::getState(Eigen::VectorXd &x)
{
    x = _x;
}

void Resampler::getInput(Eigen::VectorXd &u)
{
    u = _u;
}

void Resampler::id()
{
    pinocchio::rnea(_model, *_data, _x.segment(0, _model.nq), _x.segment(_model.nq, _model.nv), _x.segment(_model.nq + _model.nv, _model.nv));

    for (int i = 0; i < _frames.size(); i++)
    {
        auto frame_id = _model.getFrameId(_frames[i]);
        pinocchio::getFrameJacobian(_model, *_data, frame_id, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, _J);
        _data->tau += _J.transpose() * _x.segment(_model.nq + _model.nv + _model.nv + i*3, 6);
    }

    _tau = _data->tau;
}

void Resampler::double_integrator(const Eigen::VectorXd &x, const Eigen::VectorXd &u, Eigen::VectorXd &xdot)
{
    qdot(x.segment(0, _model.nq), x.segment(_model.nq, _model.nv), _v);

    xdot << _v, x.segment(_model.nq + _model.nv, _model.nv), _u;
}

void Resampler::rk4(double dt_res)
{
    double_integrator(_x, _u, _k1);
    double_integrator(_x + dt_res / 2 * _k1, _u, _k2);
    double_integrator(_x + dt_res / 2 * _k2, _u, _k3);
    double_integrator(_x + dt_res * _k3, _u, _k4);

    _x = _x + dt_res / 6. * (_k1 + 2 * _k2 + 2 * _k3 + _k4);
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
