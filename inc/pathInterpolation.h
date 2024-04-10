//
// Created by 91418 on 2023/11/13.
//

#ifndef LIB_DEMO_TRAJ_HPP
#define LIB_DEMO_TRAJ_HPP

#include "vector"
#include <libInterpolate/Interpolate.hpp>
using namespace std;
class _interpolation
{
public:
	static auto _1_Order_Linear_no_time_adjustment(const vector<vector<double>>& pathPointsData,
	                                               const int& costTime) -> vector<vector<double>>;
	//  请勿使用3阶B样条插补，部分轨迹会超出
	static vector<vector<double>>
	_3_Order_CubicSpline_is_time_adjustment(vector<vector<double>> yData, const int& totalTime);
	//    _1D::CubicSplineInterpolator<double> interp;
};
#endif// LIB_DEMO_TRAJ_HPP
