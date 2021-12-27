#include <array>
#include <iostream>
#include "IntegratorRK4.h"
#include "ModelDIPC.h"
#include "NonlinearModelPredictiveControl.h"
#include "Plot.h"
#include "Simulator.h"

using namespace std;
using namespace casadi;
using namespace nmpc;

int main(int argc, char **argv)
{

    if (argc != 4)
    {
        cerr << "Usage: ./nmpc_main path_to_config path_to_nmpc_model path_to_sim_model " << endl;
        return EXIT_FAILURE;
    }

    // Parse yaml files
    const string config_file{argv[1]};
    const string nmpc_model_file{argv[2]};
    const string sim_model_file{argv[3]};

    // Initialize NMPC and simulator
    const ModelDIPC<MX> nmpc_model{nmpc_model_file};
    const ModelDIPC<DM> sim_model{sim_model_file};
    const IntegratorRK4<MX> nmpc_integrator;
    const IntegratorRK4<DM> sim_integrator;
    NonlinearModelPredictiveControl nmpc{config_file, nmpc_model, nmpc_integrator};
    const Simulator sim{config_file, sim_model, sim_integrator};

    // Start simulation
    const int N{static_cast<int>((sim.tf() - sim.t0()) / sim.dt())};
    DM t{DM::zeros(1, N + 1)};
    DM x{DM::zeros(nmpc.nx(), N + 1)};
    DM u{DM::zeros(nmpc.nu(), N)};
    Slice all;
    t(0) = sim.t0();
    x(all, 0) = nmpc.x_0();
    for (int k = 0; k < N; k++)
    {
        t(k + 1) = (k + 1) * sim.dt();
        // Get control input for time step k from NMPC
        u(all, k) = nmpc.ComputeControlInput();
        // Simulate time step (apply control input for timestep k)
        x(all, k + 1) = sim.ApplyControlForTimeStep(x(all, k), u(all, k));
        // Reinitialize NMPC with measured state from simulator
        nmpc.SetInitialCondition(x(all, k + 1));
    }

    // Plot simulated state results
    for (int c = 0; c < x.size(1); ++c)
    {
        const string state_name{"x_" + to_string(c)};
        const string file_name{"DIPC_" + state_name + ".png"};
        const string title{"Simulated state " + state_name + " over time"};
        Plot(file_name, title, "t", vector<double>(t(0, all)), state_name, vector<double>(x(c, all)));
    }

    // Plot simulated control results
    for (int c = 0; c < u.size(1); ++c)
    {
        const string control_name{"u_" + to_string(c)};
        const string file_name{"DIPC_" + control_name + ".png"};
        const string title{"Simulated control " + control_name + " over time"};
        Plot(file_name, title, "t", vector<double>(t(Slice(0, t.size(2) - 1))), control_name, vector<double>(u(c, all)));
    }

    return EXIT_SUCCESS;
}
