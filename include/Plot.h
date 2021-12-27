#pragma once

#include <string>
#include <vector>

namespace nmpc
{
    // Plot the state or control trajectory over time
    void Plot(const std::string &file_name, const std::string &title, const std::string &x_name, const std::vector<double> &x_value,
              const std::string &y_name, const std::vector<double> &y_value);

} // namespace nmpc
