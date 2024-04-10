#include "Transform.hpp"
#include <armadillo>
using namespace arma;
#define TESTCOUNTS 5000000
clock_t invCount_start, invCount_end;
int main()
{
#ifdef testCOST
	invCount_start = clock();
	mat data {};
	for (int i = 0; i < TESTCOUNTS; i++)
		data = inv(j0_1_3PI);
	invCount_end   = clock();
	auto timeCount = (invCount_end - invCount_start) / CLOCKS_PER_SEC;
	cout << "test total time: " << timeCount * 1000 << endl;
	cout << "test for singlet time consumption: " << (double)timeCount / TESTCOUNTS << endl;
	cout << "frequency: " << (double)TESTCOUNTS / timeCount << endl;
#endif
	//    vector<double> c_target{1.399, 1.461, 0.278, -146.922, 49.863, -15.719};
	vector<double> q10 {10, 10, -20, 0, 10, -10};
	auto q10_C = vec(fkine(q10));
	q10_C.print("q10_c");
	vec delta     = {0, 0, 0.08, 0, 0, 0};
	vec q10_C_end = q10_C + delta;
	vec qd        = ikine(vector<double> {q10_C_end.begin(), q10_C_end.end()}, q10);
	qd.print("qd: ");
	vector<double> a {qd.begin(), qd.end()};
	auto qd_Check = vec(fkine(a));
	qd_Check.print("qd check:");
	system("pause");
	return 0;
}
