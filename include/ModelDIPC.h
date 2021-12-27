#pragma once

#include <array>
#include <string>
#include "ModelBase.h"

namespace nmpc
{
    // DIPC model parameters from the model file
    template <typename T>
    struct ModelParams
    {
        T g;   // m/(s^2)
        T m_0; // kg
        T m_1; // kg
        T m_2; // kg
        T L_1; // m
        T L_2; // m
        T d_1; // kg
        T d_2; // kg*m
        T d_3; // kg*m
        T d_4; // kg*m^2
        T d_5; // kg*m^2
        T d_6; // kg*m^2
        T f_1; // kg*m^2/(s^2)
        T f_2; // kg*m^2/(s^2)
    };

    // Double Inverted Pendulum on a Cart (DIPC) model class based on the paper: Optimal Control of a Double Inverted Pendulum on a Cart
    // Inherits from abstract model base class and implements the nonlinear system equations for the DIPC
    template <typename T>
    class ModelDIPC : public ModelBase<T>
    {
        using SystemMatrices = std::array<T, 3>;

    public:
        // Custom constructor: read DIPC parameters from the model file
        ModelDIPC(const std::string &model_file);

        // States: x_0: cart position [m], x_1: bottom pendulum angles [rad], x_2: top pendulum angles [rad], x_3: cart velocity [m/s], x_4: bottom pendulum velocity [rad/s], x_5: top pendulum velocity [rad/s]
        // Controls: u: control force [N]
        T operator()(const T &x_k, const T &u_k) const override;

    private:
        // Read parameters from yaml file
        void ReadParams(const std::string &model_file);

        // Build system matrices needed for the nonlinear system equations in a compact matrix form
        SystemMatrices BuildSystemMatrices(const T &x1, const T &x2) const;

        // DIPC model parameters needed for the nonlinear system equations
        ModelParams<T> model_params_;

        // Constant matrix related to the control vector u in the system equations
        T H_;
    };

} // namespace nmpc
