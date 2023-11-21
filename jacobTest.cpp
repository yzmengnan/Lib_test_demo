#include "Transform.hpp"
#include "mat.hpp"
#define TESTCOUNTS 5000
clock_t invCount_start, invCount_end;
int main() {
    vector<float> q0(6, 0.0f);
    vector<float> q1(6, 3.1415927 / 6);
    //forward kinematic
    auto res = joint2Position(q0);
    //jacob0
    auto jacob0_1_3_pi = jacob0(q1);
    auto jacob0_1_3_pi_e = jacobe(q1);
    cout<<"jacob0:"<<endl;
        jacob0_1_3_pi.disp();
    cout<<"jacobe:"<<endl;
    jacob0_1_3_pi_e.disp();
    invCount_start = clock();
    for (int i = 0; i < TESTCOUNTS; i++)
        jacob0_1_3_pi.inv();
    invCount_end = clock();
//    jacob0_1_3_pi.inv().disp();
//    auto E = jacob0_1_3_pi*(jacob0_1_3_pi.inv());
//    E.disp();
    auto timeCount = (invCount_end - invCount_start) / CLOCKS_PER_SEC;
    cout << "test total time: " << timeCount * 1000 << endl;
    cout << "test for singlet time consumption: " << (double) timeCount / TESTCOUNTS << endl;
    cout << "frequency: " << (double) TESTCOUNTS / timeCount << endl;
    //matrix 3*3 for time cost at 2us
    //matrix 6*6 for time cost at 8us
    vector<float>c_dot(6,0.2f);
    auto c_dot_m = mat(c_dot);
    auto q_dot = jacob0_1_3_pi.inv()*mat(c_dot);
    q_dot.disp();
    return 0;
}
