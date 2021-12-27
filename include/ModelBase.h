#pragma once

namespace nmpc
{
    // Abstract model base class (interface for the first order nonlinear system equations)
    template <typename T>
    class ModelBase
    {
    public:
        virtual ~ModelBase() = default;

        // Input: x_k (state at time k), u_k (control input at time k)
        // Output: x_dot_k (differential state at time k)
        virtual T operator()(const T &x_k, const T &u_k) const = 0;
    };

} // namespace nmpc
