#include <casadi/casadi.hpp>
#include <yaml-cpp/yaml.h>
#include "ModelDIPC.h"

using casadi::DM;
using casadi::MX;
using casadi::Slice;

namespace nmpc
{

    template <typename T>
    ModelDIPC<T>::ModelDIPC(const std::string &model_file) : H_(3, 1)
    {
        ReadParams(model_file);

        // Matrix entries (for more details see the paper: Optimal Control of a Double Inverted Pendulum on a Cart)
        model_params_.d_1 = model_params_.m_0 + model_params_.m_1 + model_params_.m_2;
        model_params_.d_2 = (model_params_.m_1 / 2 + model_params_.m_2) * model_params_.L_1;
        model_params_.d_3 = model_params_.m_2 * model_params_.L_2 / 2;
        model_params_.d_4 = (model_params_.m_1 / 3 + model_params_.m_2) * pow(model_params_.L_1, 2);
        model_params_.d_5 = model_params_.m_2 * model_params_.L_1 * model_params_.L_2 / 2;
        model_params_.d_6 = model_params_.m_2 * pow(model_params_.L_2, 2) / 3;
        model_params_.f_1 = (model_params_.m_1 / 2 + model_params_.m_2) * model_params_.L_1 * model_params_.g;
        model_params_.f_2 = model_params_.m_2 * model_params_.L_2 * model_params_.g / 2;

        H_(0) = 1;
        H_(1) = 0;
        H_(2) = 0;
    }

    template <typename T>
    auto ModelDIPC<T>::BuildSystemMatrices(const T &x1, const T &x2) const -> SystemMatrices
    {
        // Matrices D, C, G and H (for more details see the paper: Optimal Control of a Double Inverted Pendulum on a Cart)
        T D_(3, 3);
        D_(0, 0) = model_params_.d_1;
        D_(0, 1) = model_params_.d_2 * cos(x1(1));
        D_(0, 2) = model_params_.d_3 * cos(x1(2));
        D_(1, 0) = model_params_.d_2 * cos(x1(1));
        D_(1, 1) = model_params_.d_4;
        D_(1, 2) = model_params_.d_5 * cos(x1(1) - x1(2));
        D_(2, 0) = model_params_.d_3 * cos(x1(2));
        D_(2, 1) = model_params_.d_5 * cos(x1(1) - x1(2));
        D_(2, 2) = model_params_.d_6;

        T C_(3, 3);
        C_(0, 0) = 0;
        C_(0, 1) = -model_params_.d_2 * sin(x1(1)) * x2(1);
        C_(0, 2) = -model_params_.d_3 * sin(x1(2)) * x2(2);
        C_(1, 0) = 0;
        C_(1, 1) = 0;
        C_(1, 2) = model_params_.d_5 * sin(x1(1) - x1(2)) * x2(2);
        C_(2, 0) = 0;
        C_(2, 1) = -model_params_.d_5 * sin(x1(1) - x1(2)) * x2(1);
        C_(2, 2) = 0;

        T G_(3, 1);
        G_(0) = 0;
        G_(1) = -model_params_.f_1 * sin(x1(1));
        G_(2) = -model_params_.f_2 * sin(x1(2));

        return {D_, C_, G_};
    }

    template <typename T>
    T ModelDIPC<T>::operator()(const T &x_k, const T &u_k) const
    {
        const T &x1 = x_k(Slice(0, 3));
        const T &x2 = x_k(Slice(3, 6));
        auto sys_mat{BuildSystemMatrices(x1, x2)};
        // Nonlinear system equations in compact matrix form derived from the Lagrange equations
        T dx_(6, 1);
        dx_(Slice(0, 3)) = x2;
        dx_(Slice(3, 6)) = mtimes(inv(sys_mat[0]), (H_ * u_k - mtimes(sys_mat[1], x2) - sys_mat[2]));
        return dx_;
    }

    template <typename T>
    void ModelDIPC<T>::ReadParams(const std::string &model_file)
    {
        // Cart and pendulum parameters
        YAML::Node config = YAML::LoadFile(model_file);
        model_params_.g = config["model.g"].as<double>();
        model_params_.m_0 = config["model.m_0"].as<double>();
        model_params_.m_1 = config["model.m_1"].as<double>();
        model_params_.m_2 = config["model.m_2"].as<double>();
        model_params_.L_1 = config["model.L_1"].as<double>();
        model_params_.L_2 = config["model.L_2"].as<double>();
    }

    template class ModelDIPC<DM>;
    template class ModelDIPC<MX>;

} // namespace nmpc
