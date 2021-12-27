#pragma once

#include "Integrator.h"

namespace nmpc
{
    // Runge kutta 4th order integrator class
    template <typename T>
    class IntegratorRK4 : public Integrator<T>
    {
    public:
        T operator()(const ModelBase<T> &model, double dt, const T &x, const T &u) const override
        {
            // Runge Kutta 4 integration scheme
            T k1 = model(x, u);
            T k2 = model(x + dt / 2 * k1, u);
            T k3 = model(x + dt / 2 * k2, u);
            T k4 = model(x + dt * k3, u);
            T x_next = x + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4);
            return x_next;
        }
    };

} // namespace nmpc
