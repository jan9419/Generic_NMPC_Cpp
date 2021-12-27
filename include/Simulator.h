#pragma once

#include <string>
#include <casadi/casadi.hpp>
#include "Integrator.h"
#include "ModelBase.h"

namespace nmpc
{
    // Simulation parameters from the config file
    struct SimParams
    {
        // Simulation start time
        double t0;
        // Simulation end time
        double tf;
        // Simulation step size
        double dt;
    };

    // Simulation class simulates the real plant and supplies the nmpc controller with measurements
    class Simulator
    {
    public:
        // Custom constructor: read the simulation parameters from the config file and initialize the model and integrator
        Simulator(const std::string &config_file, const ModelBase<casadi::DM> &model, const Integrator<casadi::DM> &integrator);

        // Simulate the model with the computed control input from the nmpc controller for the specified timestep
        inline casadi::DM ApplyControlForTimeStep(const casadi::DM &x_k, const casadi::DM &u_k) const
        {
            return integrator_(model_, sim_params_.dt, x_k, u_k);
        }

        // Get the simulation start time
        inline double t0() const
        {
            return sim_params_.t0;
        }

        // Get the simulation end time
        inline double tf() const
        {
            return sim_params_.tf;
        }

        // Get the simulation step size
        inline double dt() const
        {
            return sim_params_.dt;
        }

    private:
        // Read parameters from yaml file
        void ReadParams(const std::string &config_file);

        // Simulator config parameters
        SimParams sim_params_;
        // Specified model, which inherits from the abstract model base class
        const ModelBase<casadi::DM> &model_;
        // Implemented integrator
        const Integrator<casadi::DM> &integrator_;
    };

} // namespace nmpc
