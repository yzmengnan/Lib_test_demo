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
    vector<vector<double>> data = {{0,0,0,0,0,0},{2,3,4,5,6,7},{3,4,5,6,7,8}};
    auto traj= _interpolation::_1_Order_Linear_no_time_adjustment(data, t);
    for(auto it=traj.begin();it!=traj.end();it++){
        for(auto it2 = (*it).begin();it2!=(*it).end();it2++){
            cout<<*it2<<",";
        }
        cout<<endl;
        Sleep(10);
    }
    return 0;
}