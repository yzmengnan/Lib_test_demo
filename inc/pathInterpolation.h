//
// Created by 91418 on 2023/11/13.
//

#ifndef LIB_DEMO_TRAJ_HPP
#define LIB_DEMO_TRAJ_HPP

#include <libInterpolate/Interpolate.hpp>
#include "vector"
using namespace std;
class _interpolation {
public:
    static auto _1_Order_Linear_no_time_adjustment(const vector<vector<double>> &pathPointsData, const int &costTime) -> vector<vector<double>>;
    static vector<vector<double>> _3_Order_CubicSpline_is_time_adjustment(vector<vector<double>> yData, size_t totalTime);
    //    _1D::CubicSplineInterpolator<double> interp;
};
#endif//LIB_DEMO_TRAJ_HPP
