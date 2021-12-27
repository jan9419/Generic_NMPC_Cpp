#pragma once

#include <string>
#include "ModelBase.h"

namespace nmpc
{
    // CSTR model parameters from the model file
    template <typename T>
    struct ModelCSTRParams
    {
        T k_10;    // 1/h
        T k_20;    // 1/h
        T k_30;    // 1/(mol A*h)
        T E_1;     // K
        T E_2;     // K
        T E_3;     // K
        T dH_AB;   // kJ/(mol A)
        T dH_BC;   // kJ/(mol B)
        T dH_AD;   // kJ/(mol A)
        T rho;     // kg/l
        T C_p;     // kJ/(kg*K)
        T k_w;     // kJ/(h*m^2*K)
        T A_R;     // m^2
        T V_R;     // l
        T m_K;     // kg
        T C_PK;    // kJ/(kg*K)
        T c_A0;    // mol/l
        T theta_0; // °C
    };

    // Continuous Stirred Tank Reactor (CSTR) model class based on the paper: Nonlinear Predictive Control of a Benchmark CSTR
    // Inherits from abstract model base class and implements the nonlinear system equations for the CSTR
    template <typename T>
    class ModelCSTR : public ModelBase<T>
    {
    public:
        // Custom constructor: read CSTR parameters from the model file
        ModelCSTR(const std::string &model_file);

        // States: x_0: c_A (concentration substance A) [mol/l], x_1: c_B (concentration substance B) [mol/l], x_2: theta (temperature in the reactor) [°C], x_3: theta_k (temperature in the cooling jacket) [°C]
        // Controls: u_0: V_dot/V_R (feed flow) [1/h], u_1: Q_dot_k (heat removal) [kJ/h]
        T operator()(const T &x_k, const T &u_k) const override;

    private:
        // Read parameters from yaml file
        void ReadParams(const std::string &model_file);

        // CSTR model parameters needed for the nonlinear system equations
        ModelCSTRParams<T> model_params_;
    };

} // namespace nmpc
