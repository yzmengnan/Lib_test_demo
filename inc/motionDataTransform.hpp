//
// Created by 91418 on 2023/7/10.
//

#ifndef SEAL_DEMO_MOTIONDATATRANSFORM_HPP
#define SEAL_DEMO_MOTIONDATATRANSFORM_HPP

#include "DATA_STRUCT.h"
#include "Driver.h"
#include "windows.h"
#include <vector>
using namespace std;
using MDT = class motionDataTransform {
public:
public:

    static vector<float> getAngles(const Driver &d, const vector<DFS> &getData) {
        vector<float> result{};
        int i{};
        for (auto g: getData) {
            result.push_back((g.Actual_Pos + pulse_offset[i]) / d._driver_gearRatioScalar[i]);
            i++;
        }
        return result;
    }
    static void fromAnglesToPulses(const Driver &d, const vector<float> &angles, vector<DTS> &SendData) {
        int i{};
        for (auto &s: SendData) {
            if(i>=angles.size()){
                break;
            }
            s.Target_Pos = (angles[i]) * d._driver_gearRatioScalar[i] - pulse_offset[i];
            i++;
        }
    }
    static vector<float> getMoments(const Driver &d, const vector<DFS> &getData) {
        vector<float> result{};
        for (auto g: getData) {
            result.push_back(g.Actual_Torque * 1.21 / 1000);
        }
        return result;
    }
    static vector<float> getVecs(const Driver &d, const vector<DFS> &getData) {
        vector<float> result{};
        int i{};
        for (auto g: getData) {
            result.push_back(g.Actual_Vec / d._driver_gearRatioScalar[i]);
            i++;
        }
        return result;
    }
};

#endif//SEAL_DEMO_MOTIONDATATRANSFORM_HPP
