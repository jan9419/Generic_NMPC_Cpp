#include <yaml-cpp/yaml.h>
#include "Simulator.h"

using casadi::DM;

namespace nmpc
{

    Simulator::Simulator(const std::string &config_file, const ModelBase<DM> &model, const Integrator<casadi::DM> &integrator) : model_{model}, integrator_(integrator)
    {
        ReadParams(config_file);
    }

    void Simulator::ReadParams(const std::string &config_file)
    {
        YAML::Node config = YAML::LoadFile(config_file);
        sim_params_.t0 = config["sim.t0"].as<double>();
        sim_params_.dt = config["sim.dt"].as<double>();
        sim_params_.tf = config["sim.tf"].as<double>();
    }

} // namespace nmpc