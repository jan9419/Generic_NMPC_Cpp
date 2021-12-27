#include <math.h>
#include <vector>
#include <yaml-cpp/yaml.h>
#include "NonlinearModelPredictiveControl.h"

using casadi::DM;
using casadi::MX;
using casadi::Slice;
using std::vector;

namespace nmpc
{

    NonlinearModelPredictiveControl::NonlinearModelPredictiveControl(const std::string &config_file, const ModelBase<MX> &model, const Integrator<casadi::MX> &integrator) : ocp_{config_file, model, integrator}
    {
        ReadParams(config_file);
    }

    void NonlinearModelPredictiveControl::ReadParams(const std::string &config_file)
    {
        YAML::Node config = YAML::LoadFile(config_file);
        nmpc_params_.x_0 = config["nmpc.x_0"].as<std::vector<double>>();
        nmpc_params_.x_e = config["nmpc.x_e"].as<std::vector<double>>();
        nmpc_params_.nx = config["nmpc.nx"].as<int>();
        nmpc_params_.nu = config["nmpc.nu"].as<int>();
    }

} // namespace nmpc
