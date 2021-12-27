#include <assert.h>
#include <math.h>
#include <yaml-cpp/yaml.h>
#include "OptimalControlProblem.h"

using casadi::DM;
using casadi::MX;
using casadi::OptiSol;
using casadi::Slice;
using std::string;
using std::vector;

namespace nmpc
{

    OptimalControlProblem::OptimalControlProblem(const std::string &config_file, const ModelBase<MX> &model, const Integrator<casadi::MX> &integrator) : model_{model}, integrator_{integrator}
    {
        ReadParams(config_file);

        BuildOCP();
    }

    void OptimalControlProblem::BuildOCP()
    {
        nlp_ = casadi::Opti();
        // Initial guess
        X_sol_ = repmat(ocp_params_.sc_x * ocp_params_.x_0, 1, ocp_params_.n_shoot + 1);
        U_sol_ = repmat(0.5 * ocp_params_.sc_u * (ocp_params_.u_const["max"] - ocp_params_.u_const["min"]), 1, ocp_params_.n_shoot);
        // Initial condition
        X_0_ = nlp_.parameter(ocp_params_.nx, 1);
        // Discretized state and control trajectory (NLP parameters)
        X_ = nlp_.variable(ocp_params_.nx, ocp_params_.n_shoot + 1);
        U_ = nlp_.variable(ocp_params_.nu, ocp_params_.n_shoot);
        // Cost functional
        J_ = 0;
        Slice all;
        MX X_next;
        for (int i = 0; i < ocp_params_.n_shoot; i++)
        {
            X_next = ocp_params_.sc_x * integrator_(model_, ocp_params_.dt, X_(all, i) / ocp_params_.sc_x, U_(all, i) / ocp_params_.sc_u);
            nlp_.subject_to(X_(all, i + 1) == X_next);
            // Set input constraints
            nlp_.subject_to(ocp_params_.sc_u(ocp_params_.u_const_index) * ocp_params_.u_const["min"] <= U_((ocp_params_.u_const_index), i) <= ocp_params_.sc_u(ocp_params_.u_const_index) * ocp_params_.u_const["max"]);
            // Set path constraints
            nlp_.subject_to(ocp_params_.sc_x(ocp_params_.x_const_index) * ocp_params_.x_const["min"] <= X_(ocp_params_.x_const_index, i + 1) <= ocp_params_.sc_x(ocp_params_.x_const_index) * ocp_params_.x_const["max"]);
            // Cost functional (setpoint stabilization)
            const MX &dx = X_(ocp_params_.x_e_index, i + 1) - ocp_params_.x_e;
            const MX &du = U_(all, i);
            J_ = J_ + mtimes(dx.T(), mtimes(ocp_params_.Q, dx));
            J_ = J_ + mtimes(du.T(), mtimes(ocp_params_.R, du));
        }
        // Set terminal cost
        J_ = J_ + mtimes((X_(ocp_params_.x_e_index, ocp_params_.n_shoot) - ocp_params_.x_e).T(), mtimes(ocp_params_.P, X_(ocp_params_.x_e_index, ocp_params_.n_shoot) - ocp_params_.x_e));
        // Terminal condition
        // nlp_.subject_to(X_(all,ocp_params_.n_shoot) == ocp_params_.sc_x*ocp_params_.x_e);
        // Set initial condition
        nlp_.subject_to(X_(all, 0) == X_0_);
        nlp_.set_value(X_0_, ocp_params_.sc_x * ocp_params_.x_0);
        // Set initial guess
        nlp_.set_initial(X_, X_sol_);
        nlp_.set_initial(U_, U_sol_);
        // Set solver
        nlp_.solver(ocp_params_.solver);
        // Set objective
        nlp_.minimize(J_);
    }

    void OptimalControlProblem::ReadParams(const std::string &config_file)
    {
        YAML::Node config = YAML::LoadFile(config_file);
        ocp_params_.nx = config["nmpc.nx"].as<int>();
        ocp_params_.nu = config["nmpc.nu"].as<int>();
        ocp_params_.n_shoot = config["ocp.n_shoot"].as<int>();
        ocp_params_.dt = config["ocp.dt"].as<double>();
        ocp_params_.solver = config["ocp.solver"].as<string>();
        ocp_params_.x_0 = config["nmpc.x_0"].as<vector<double>>();
        ocp_params_.x_e = config["nmpc.x_e"].as<vector<double>>();
        ocp_params_.x_e_index = config["nmpc.x_e_index"].as<vector<int>>();
        ocp_params_.R = MX::eye(ocp_params_.nu) * config["ocp.r"].as<vector<double>>();
        ocp_params_.Q = MX::eye(ocp_params_.x_e_index.size()) * config["ocp.q"].as<vector<double>>();
        ocp_params_.P = MX::eye(ocp_params_.x_e_index.size()) * config["ocp.p"].as<vector<double>>();
        ocp_params_.x_const["min"] = config["ocp.con.x_min"].as<vector<double>>();
        ocp_params_.x_const["max"] = config["ocp.con.x_max"].as<vector<double>>();
        ocp_params_.x_const_index = config["ocp.con.x_index"].as<vector<int>>();
        ocp_params_.u_const["min"] = config["ocp.con.u_min"].as<vector<double>>();
        ocp_params_.u_const["max"] = config["ocp.con.u_max"].as<vector<double>>();
        ocp_params_.u_const_index = config["ocp.con.u_index"].as<vector<int>>();
        ocp_params_.sc_x = config["ocp.scale.x"].as<vector<double>>();
        ocp_params_.sc_u = config["ocp.scale.u"].as<vector<double>>();
    }

} // namespace nmpc
