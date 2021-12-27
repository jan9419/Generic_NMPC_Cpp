#include <math.h>
#include <casadi/casadi.hpp>
#include <yaml-cpp/yaml.h>
#include "ModelCSTR.h"

using casadi::DM;
using casadi::MX;
using casadi::Slice;

namespace nmpc
{

    template <typename T>
    ModelCSTR<T>::ModelCSTR(const std::string &model_file)
    {
        ReadParams(model_file);
        // Steady state parameters at the optimal operating point
        model_params_.c_A0 = 5.1;
        model_params_.theta_0 = 104.9;
    }

    template <typename T>
    T ModelCSTR<T>::operator()(const T &x_k, const T &u_k) const
    {
        // Reaction velocities k_i depend on the temperature via the arrhenius law
        const float temperature{273.15};
        T k1 = model_params_.k_10 * exp(model_params_.E_1 / (x_k(2) + temperature));
        T k2 = model_params_.k_20 * exp(model_params_.E_2 / (x_k(2) + temperature));
        T k3 = model_params_.k_30 * exp(model_params_.E_3 / (x_k(2) + temperature));

        // The dynamics of the reactor are derived from component balances for substances A and B and from energy balances for the reactor and cooling jacket
        T dx_(4, 1);
        dx_(0) = u_k(0) * (model_params_.c_A0 - x_k(0)) - k1 * x_k(0) - k3 * pow(x_k(0), 2);
        dx_(1) = -u_k(0) * x_k(1) + k1 * x_k(0) - k2 * x_k(1);
        dx_(2) = u_k(0) * (model_params_.theta_0 - x_k(2)) - 1 / (model_params_.rho * model_params_.C_p) * (k1 * x_k(0) * model_params_.dH_AB + k2 * x_k(1) * model_params_.dH_BC + k3 * pow(x_k(0), 2) * model_params_.dH_AD) + model_params_.k_w * model_params_.A_R / (model_params_.rho * model_params_.C_p * model_params_.V_R) * (x_k(3) - x_k(2));
        dx_(3) = 1 / (model_params_.m_K * model_params_.C_PK) * (u_k(1) + model_params_.k_w * model_params_.A_R * (x_k(2) - x_k(3)));

        return dx_;
    }

    template <typename T>
    void ModelCSTR<T>::ReadParams(const std::string &model_file)
    {
        // Physico-chemical parameters for the CSTR (most parameters are only known within bounds)
        YAML::Node config = YAML::LoadFile(model_file);
        model_params_.k_10 = config["model.k_10"].as<double>();
        model_params_.k_20 = config["model.k_20"].as<double>();
        model_params_.k_30 = config["model.k_30"].as<double>();
        model_params_.E_1 = config["model.E_1"].as<double>();
        model_params_.E_2 = config["model.E_2"].as<double>();
        model_params_.E_3 = config["model.E_3"].as<double>();
        model_params_.dH_AB = config["model.dH_AB"].as<double>();
        model_params_.dH_BC = config["model.dH_BC"].as<double>();
        model_params_.dH_AD = config["model.dH_AD"].as<double>();
        model_params_.rho = config["model.rho"].as<double>();
        model_params_.C_p = config["model.C_p"].as<double>();
        model_params_.k_w = config["model.k_w"].as<double>();
        model_params_.A_R = config["model.A_R"].as<double>();
        model_params_.V_R = config["model.V_R"].as<double>();
        model_params_.m_K = config["model.m_K"].as<double>();
        model_params_.C_PK = config["model.C_PK"].as<double>();
    }

    template class ModelCSTR<DM>;
    template class ModelCSTR<MX>;

} // namespace nmpc
