//
// Created by 91418 on 2023/11/13.
//

#ifndef LIB_DEMO_TRAJ_HPP
#define LIB_DEMO_TRAJ_HPP

#include "vector"
using namespace std;

class my_traj {
public:
    static auto _jtraj_Linear(const vector<vector<float>>& pathPointsData,const int& costTime) -> vector<vector<float>>{
        float single_stepTime = static_cast<float>(costTime)/(pathPointsData.size()-1);
        vector<vector<float>> res(costTime*100+1,vector<float>{});
        res[0] = pathPointsData[0];
        int No_step{};
        for(auto it=pathPointsData.begin();it!=pathPointsData.end()-1;it++){
            auto p0 = *it,pf=*(it+1);
            for(int j{};j<p0.size();j++){
                float delta = (pf[j]-p0[j])/single_stepTime/100;
                int i{};
                while(i++<single_stepTime*100){
                    res[i+No_step*single_stepTime*100].push_back(delta +res[i+No_step*single_stepTime*100-1][j]);
                }
            }
            No_step++;
        }
        return res;
    }

};


#endif//LIB_DEMO_TRAJ_HPP
