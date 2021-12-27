# Generic NMPC Controller for Setpoint Tracking in C++

In this project, a Nonlinear Model Predictive Controller (NMPC) with finite time horizon is implemented to stabilize a given setpoint.   
There are several methods that guarantee the stability of the closed loop system. In this project the stability of the closed loop system can be guaranteed via a large weighting of the final cost term.   
Since it is a NMPC with fixed end time, the time horizon can be constant or receding. Here, the classical NMPC Controller with a time horizon of constant length is implemented, which typically approaches the given setpoint asymptotically.

The NMPC Controller iteratively solves an Optimal Control Problem (OCP). To solve the OCP numerically, the open source software CasADi is used. CasADi is based on algorithmic differentiation and can therefore solve such problems efficiently.   
In the OCP implementation here, optistack, a collection of helper classes provided by CasADi, is used to transform the OCP into a Nonlinear Program (NLP) with a solution method called direct multiple shooting.   
The resulting NLP is then solved using e.g. the IPOPT (primal-dual interior point method) solver.

Several numerical integration methods are implemented to solve the optimal control problem, namely the explicit Euler method and the 4th order Runge-Kutta method. Hereby, the control signals are approximated as piecewise constant functions over equidistant ranges and allow a variation of the sampling rate.

Different model parameters for the NMPC and Simulator can be configured to study the robustness of the NMPC Controller to parameter uncertainties (model-plant mismatch).

The modular written code makes it easy to extend the NMPC controller for your own models.

# Prerequisites
The code is tested with **Ubuntu 20.04**.

## CasADi
[CasADi](https://github.com/casadi/casadi) is used to build and solve the Optimal Control Problem. Install instructions can be found at: https://github.com/casadi/casadi.

Joel A. E. Andersson, Joris Gillis, Greg Horn, James B. Rawlings and Moritz Diehl, **CasADi: a software framework for nonlinear optimization and optimal control**

## yaml-cpp
[yaml-cpp](https://github.com/jbeder/yaml-cpp) is required to read the parameter files. Install instructions can be found at: https://github.com/jbeder/yaml-cpp

## matplotlib-cpp (and Python3)
[matplotlib-cpp](https://github.com/lava/matplotlib-cpp) is required to plot the simulated state and control trajectories. Install instructions can be found at: https://github.com/lava/matplotlib-cpp

# Building the code
Clone the repository:
```
git clone https://github.com/jan9419/Generic_NMPC_C++
```

Please make sure you have installed all required dependencies and set the PYTHONHOME and PYTHONPATH environment variable for matplotlib-cpp. Execute:
```
cd Generic_NMPC_C++
mkdir build
cd build
cmake -DCMAKE_MODULE_PATH=path_to_casadi_repo/cmake ..
make 
```

This will create the CSTR and DIPC executable in the *Examples* folder.

# Continuous Stirred Tank Reactor (CSTR) Example  
Related publication:   
H. Chen, A. Kremling and F. Allgöwer, **Nonlinear Predictive Control of a Benchmark CSTR**

The system equations for the multi-input control problem from the paper are implemented. It is additionally assumed that all states are measurable so that no extended Kalman filter is required for state estimation.    
The config file in the *Exampes/CSTR* folder simulates the closed-loop response of the cstr to a step change in the setpoint `c_B` from maximum to minimum as described in the paper.   
Execute the CTSR example (Please note two seperate model files can be used to simulate a model-plant mismatch):
```
cd Generic_NMPC_C++/Examples
./CSTR/nmpc_cstr CSTR/config.yaml CSTR/model_nmpc.yaml CSTR/model_sim.yaml
```

# Double Inverted Pendulum on a Cart (DIPC) Example
Related publication:   
A. Bogdanov, **Optimal Control of a Double Inverted Pendulum on a Cart**

The config file in the *Exampes/DIPC* folder tries to stabilize the DIPC when both pendulums were initially deflected from their vertical position up to e.g. 30 degrees as described in the paper.   
Execute the DIPC example (Please note two seperate model files can be used to simulate a model-plant mismatch):
```
cd Generic_NMPC_C++/Examples
./DIPC/nmpc_dipc DIPC/config.yaml DIPC/model_nmpc.yaml DIPC/model_sim.yaml
```

# Use your own model
To apply the NMPC to your own model, you must inherit from the abstract model base class and implement the nonlinear system equations for the pure virtual function. Please note that for the application of numerical integration methods it may be necessary to transform the higher order system into a first order system.      
In addition, it is also possible to inherit from the abstract integrator base class and implement a custom numeric integrator for this NMPC project. Currently, the explicit Euler method and the 4th order Runge Kutta method are implemented. Keep in mind that different integrators can be used for the NMPC controller and the simulator.
