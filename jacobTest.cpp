#include "Transform.hpp"
#include <armadillo>
using namespace arma;
#define TESTCOUNTS 5000000
clock_t invCount_start, invCount_end;
int main() {
#ifdef testCOST
    invCount_start = clock();
    mat data{};
    for (int i = 0; i < TESTCOUNTS; i++)
        data = inv(j0_1_3PI);
    invCount_end = clock();
    auto timeCount = (invCount_end - invCount_start) / CLOCKS_PER_SEC;
    cout << "test total time: " << timeCount * 1000 << endl;
    cout << "test for singlet time consumption: " << (double) timeCount / TESTCOUNTS << endl;
    cout << "frequency: " << (double) TESTCOUNTS / timeCount << endl;
#endif
//    vector<double> c_target{1.399, 1.461, 0.278, -146.922, 49.863, -15.719};
    vector<double> q10{30,90,80,40,40,40};
    auto q10_C =vec(fkine(q10));
    q10_C.print("q10_c");
    auto qd_f = ikine(vector<double>{q10_C.begin(),q10_C.end()},vector<double>(6,0));
    auto qdouble =vector<double>{qd_f.begin(),qd_f.end()};
    vec qd(qdouble);
    qd.print("qd: ");
    vector<double>a {qd.begin(),qd.end()};
    auto qd_Check = vec(fkine(a));
    qd_Check.print("qd check:");
    system("pause");
    return 0;

}
