#pragma once

#include "Integrator.h"

namespace nmpc
{
    // Explicit euler integrator class
    template <typename T>
    class IntegratorEulerF : public Integrator<T>
    {
    public:
        T operator()(const ModelBase<T> &model, double dt, const T &x, const T &u) const override
        {
            // Explicit euler integration scheme
            T x_next = x + dt * model(x, u);
            return x_next;
        }
    };

} // namespace nmpc
