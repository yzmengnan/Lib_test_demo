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

    static auto getAngles(const Driver &d, const vector<DFS> &getData) -> vector<double> {
        vector<double> result{};
        int i{};
        for (auto g: getData) {
            result.push_back((g.Actual_Pos + pulse_offset[i]) / d._driver_gearRatioScalar[i]);
            i++;
        }
#ifdef UP_NEEDED
        while (result.size() != 9) {
            result.push_back(0.0F);
        }
#endif
        return result;
    }
    static void fromAnglesToPulses(const Driver &d, const vector<float> &angles, vector<DTS> &SendData) {
        int i{};
        for (auto &s: SendData) {
            if (i >= angles.size()) {
                break;
            }
            s.Target_Pos = (angles[i]) * d._driver_gearRatioScalar[i] - pulse_offset[i];
            i++;
        }
    }
    static void fromAnglesToPulses(const Driver &d, const vector<double> &angles, vector<DTS> &SendData) {
        int i{};
        for (auto &s: SendData) {
            if (i >= angles.size()) {
                break;
            }
            s.Target_Pos = (angles[i]) * d._driver_gearRatioScalar[i] - pulse_offset[i];
            i++;
        }
    }
    static vector<double> getMoments(const Driver &d, const vector<DFS> &getData) {
        vector<double> result{};
        for (auto g: getData) {
            result.push_back(g.Actual_Torque * 1.21 / 1000);
        }
#ifdef UP_NEEDED
        while (result.size() != 9)
            result.push_back(0.0f);
#endif
        return result;
    }
    static vector<double> getVecs(const Driver &d, const vector<DFS> &getData) {
        vector<double> result{};
        int i{};
        for (auto g: getData) {
            result.push_back(g.Actual_Vec / d._driver_gearRatioScalar[i]);
            i++;
        }
#ifdef UP_NEEDED
        while(result.size()!=9)
            result.push_back(0.0f);
#endif
        return result;
    }
};

#endif//SEAL_DEMO_MOTIONDATATRANSFORM_HPP
