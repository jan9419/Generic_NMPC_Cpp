#include <matplotlibcpp.h>
#include "Plot.h"

using std::string;
using std::vector;

namespace plt = matplotlibcpp;

namespace nmpc
{

    void Plot(const string &file_name, const string &title, const string &x_name, const vector<double> &x_value,
              const string &y_name, const vector<double> &y_value)
    {
        plt::figure();
        plt::title(title);
        plt::plot(x_value, y_value, {{"label", y_name}});
        plt::xlabel(x_name);
        plt::ylabel(y_name);
        plt::legend();
        plt::grid(true);
        plt::save(file_name);
    }

} // namespace nmpc
