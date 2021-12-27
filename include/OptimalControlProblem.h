#pragma once

#include <string>
#include <vector>
#include <casadi/casadi.hpp>
#include "Integrator.h"
#include "ModelBase.h"

namespace nmpc
{
    // OCP parameters from the config file
    // Optionally, scaling factors for the states and controls can be specified to avoid numerical problems when solving the NLP
    struct OCPParams
    {
        // Numerical solver for the NLP, e.g. IPOPT
        std::string solver;
        // Number of shooting intervals
        int n_shoot;
        // Number of dimensions of the state vector
        int nx;
        // Number of dimensions of the control vector
        int nu;
        // Discretization step size
        double dt;
        // Required terminal state
        casadi::MX x_e;
        // Indices of the required terminal state
        std::vector<int> x_e_index;
        // Initial state
        casadi::DM x_0;
        // Weighting matrix for the control trajectory
        casadi::MX R;
        // Weighting matrix for the state trajectory
        casadi::MX Q;
        // Weighting matrix for the terminal state
        casadi::MX P;
        // Scaling factors for the state vector
        casadi::DM sc_x;
        // Scaling factors for the control vector
        casadi::DM sc_u;
        // Required state constraints
        casadi::DMDict x_const;
        // Indices of the required state constraints
        std::vector<int> x_const_index;
        // Required control constraints
        casadi::DMDict u_const;
        // Indices of the required control constraints
        std::vector<int> u_const_index;
    };

    // Generic OCP class
    // This class needs the system dynamics, the control and state constraints, the initial and terminal states and the weighting matrices for the cost functional
    class OptimalControlProblem
    {
    public:
        // Custom constructor: read the OCP parameters from the config file and initialize the model and integrator
        OptimalControlProblem(const std::string &config_file, const ModelBase<casadi::MX> &model, const Integrator<casadi::MX> &integrator);

        // Build the OCP
        void BuildOCP();

        // Solve the OCP with direct multiple shooting
        inline casadi::DM Solve()
        {
            const casadi::OptiSol sol = nlp_.solve();
            X_sol_ = sol.value(X_);
            U_sol_ = sol.value(U_);
            return U_sol_ / ocp_params_.sc_u;
        }

        // Initialize OCP for next time step with measured state vector
        inline void Init(const casadi::DM &x_0)
        {
            nlp_.set_value(X_0_, ocp_params_.sc_x * x_0);
            nlp_.set_initial(X_, X_sol_);
            nlp_.set_initial(U_, U_sol_);
        }

    private:
        // Read parameters from yaml file
        void ReadParams(const std::string &config_file);

        // OCP config parameters
        OCPParams ocp_params_;
        // Specified model, which inherits from the abstract model base class
        const ModelBase<casadi::MX> &model_;
        // Implemented integrator
        const Integrator<casadi::MX> &integrator_;
        // Constructed NLP using CasADi
        casadi::Opti nlp_;
        // Solution trajectory of the state vector, which includes the scaling factors
        casadi::DM X_sol_;
        // Solution trajectory of the control vector, which includes the scaling factors
        casadi::DM U_sol_;
        // Cost functional
        casadi::MX J_;
        // Discretized state (NLP state parameters)
        casadi::MX X_;
        // Discretized control (NLP control parameters)
        casadi::MX U_;
        // Initial state variable
        casadi::MX X_0_;
    };

} // namespace nmpc
