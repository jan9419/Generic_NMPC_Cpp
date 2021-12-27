#pragma once

#include "ModelBase.h"

namespace nmpc
{
    // Abstract integrator base class
    template <typename T>
    class Integrator
    {
    public:
        virtual ~Integrator() = default;

        // Inputs: nonlinear first order differential equations, dt (timestep), x_k (state at time k), u_k (control input at time k)
        // Output: x_k+1 (state at time k+1)
        virtual T operator()(const ModelBase<T> &model, double dt, const T &x, const T &u) const = 0;
    };

} // namespace nmpc
