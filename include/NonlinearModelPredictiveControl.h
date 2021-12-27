#pragma once

#include <string>
#include <casadi/casadi.hpp>
#include "OptimalControlProblem.h"

namespace nmpc
{
    // NMPC parameters from the config file
    struct NMPCParams
    {
        // Required terminal state
        casadi::DM x_e;
        // Initial state
        casadi::DM x_0;
        // Number of dimensions of the state vector
        int nx;
        // Number of dimensions of the control vector
        int nu;
    };

    // Generic NMPC class
    class NonlinearModelPredictiveControl
    {
    public:
        // Custom constructor: read the NMPC parameters from the config file and initialize the OCP
        NonlinearModelPredictiveControl(const std::string &config_file, const ModelBase<casadi::MX> &model, const Integrator<casadi::MX> &integrator);

        // Solve the OCP and take the first value of the computed control trajectory
        inline casadi::DM ComputeControlInput()
        {
            casadi::Slice all;
            u_k_ = ocp_.Solve()(all, 0);

            return u_k_;
        }

        // Initialize the next OCP with the measured/simulated state vector
        inline void SetInitialCondition(const casadi::DM &x_meas)
        {
            ocp_.Init(x_meas);
        }

        // Get the initial state
        inline casadi::DM x_0() const
        {
            return nmpc_params_.x_0;
        }

        // Get the number of dimensions of the state vector
        inline int nx() const
        {
            return nmpc_params_.nx;
        }

        // Get the number of dimensions of the control vector
        inline int nu() const
        {
            return nmpc_params_.nu;
        }

    private:
        // Read parameters from yaml file
        void ReadParams(const std::string &config_file);

        // NMPC config parameters
        NMPCParams nmpc_params_;
        // OCP which is solved iteratively
        OptimalControlProblem ocp_;
        // Computed control input to apply to the plant
        casadi::DM u_k_;
    };

} // namespace nmpc
