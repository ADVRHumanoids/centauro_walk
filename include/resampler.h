#ifndef RESAMPLER_H
#define RESAMPLER_H

#include <urdf_parser/urdf_parser.h>

#include <pinocchio/algorithm/model.hpp>

typedef Eigen::Matrix<double, 6, 1> Vector6d;

class Resampler {
public:
    Resampler(urdf::ModelInterfaceSharedPtr urdf_model,
              std::vector<std::string> frames);

    bool setState(const Eigen::VectorXd x);
    bool setInput(const Eigen::VectorXd u);

    void getState(Eigen::VectorXd &x);
    void getInput(Eigen::VectorXd &u);

    void resample(double dt_res);

private:
    void double_integrator(const Eigen::VectorXd &x,
                           const Eigen::VectorXd &u,
                           Eigen::VectorXd &xdot);

    void qdot(Eigen::Ref<const Eigen::VectorXd> qeig,
              Eigen::Ref<const Eigen::VectorXd> veig,
              Eigen::Ref<Eigen::VectorXd> qdot);

    void rk4(double dt_res);

    void id();

    Eigen::VectorXd _x, _u, _xdot, _v, _tau;
    Eigen::VectorXd _k1, _k2, _k3, _k4;

    urdf::ModelInterfaceSharedPtr _urdf;
    pinocchio::Model _model;
    std::unique_ptr<pinocchio::Data> _data;

    Eigen::Matrix<double, 6, -1> _J;
    std::vector<std::string> _frames;
};

#endif // RESAMPLER_H
