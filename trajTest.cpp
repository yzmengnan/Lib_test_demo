//
// Created by 91418 on 2023/11/13.
//

#include "pathInterpolation.h"
#include <synchapi.h>

#include "iostream"
#include "vector"
using namespace std;
int main(){
    int t = 10;
    vector<vector<double>> data = {{0,0,0,0,0,0,0,0,0},{-8.35,23.83,7.67,0,0,0,0,0,0},{-8.36,44.89,21.19,0,0,-30.41,0,0,0},
                                   {-8.35,60.20,27.19,0,-33.23,-64.72,0,0,0},{-4.15,62.76,25.17,18.01,-33.74,-98.95,0.2008,0,0},
                                   {-11.78,37.42,30.48,25.63,-24.88,-83.26,0.2294,0,0},{-11.78,37.42,30.48,25.63,-24.88,-83.26,0.4254,0,0}};
    auto traj= _interpolation::_3_Order_CubicSpline_is_time_adjustment(data, t);
    for(auto it=traj.begin();it!=traj.end();it++){
        for(auto it2 = (*it).begin();it2!=(*it).end();it2++){
            cout<<*it2<<",";
        }
        cout<<endl;
//        Sleep(10);
    }
    return 0;
}