//
// Created by 91418 on 2023/11/6.
//

#ifndef JACOBDEMO_MAT_HPP
#define JACOBDEMO_MAT_HPP
#include "vector"
#include <iostream>
using namespace std;
class mat {
public:
    mat() {
        m = 0;
        n = 0;
    };
    ~mat() = default;
    mat(const vector<vector<float>> &a) : data{a} {
        m = data.size();
        if (m >= 1) {
            n = data.front().size();
        } else
            n = 0;
    };
    mat(const int &m, const int &n) : m{m}, n{n} {
        vector<float> a(n, 0);
        vector<vector<float>> b(m, a);
        data = b;
    }
    void disp() {
        string s;
        for (auto it = this->data.begin(); it != this->data.end(); it++) {
            for (auto it2 = (*it).begin(); it2 != (*it).end(); it2++) {
                s += to_string(*it2) + " , ";
            }
            s += "\n";
        }
        cout << s << endl;
    }
    vector<vector<float>> data{};
    mat T() {
        mat B(this->n, this->m);
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < m; j++) {
                B.data[i][j] = this->data[j][i];
            }
        }
        return B;
    }
    mat inv() {
        auto _Zero = [](float &a) {
            if (abs(a) <= 0.00005) {
                a = 0.0f;
                return true;
            }
            return false;
        };
        if (m != n) {
            return mat();
        }
        mat res(this->m, this->n);
        //expand origin matrix
        mat A_Expand(this->m, this->n * 2);
        for (int i{}; i < m; i++) {
            for (int j{}; j < A_Expand.n; j++) {
                if (j < this->n)
                    A_Expand.data[i][j] = this->data[i][j];
                else {
                    A_Expand.data[i][j] = j - this->n == i ? 1 : 0;
                }
            }
        }
        for (int i{}; i < A_Expand.m; i++) {
            //add from i+1 to m rows to the i
            for (int j = i + 1; j < A_Expand.m; j++) {
                for (int k = i; k < A_Expand.n; k++) {
                    A_Expand.data[i][k] += A_Expand.data[j][k];
                }
            }
            //make the first element be 1
            auto Aii = A_Expand.data[i][i];
            //make sure the ii element is not zero
            if (!_Zero(Aii)) {
                for (int k = i; k < A_Expand.n; k++) {
                    A_Expand.data[i][k] = static_cast<float>(A_Expand.data[i][k] / Aii);
                }
                for (int j = i + 1; j < A_Expand.m; j++) {
                    auto Aii2 = A_Expand.data[j][i];
                    for (int k = i; k < A_Expand.n; k++) {
                        A_Expand.data[j][k] -= A_Expand.data[i][k] * Aii2;
                    }
                }
            }
        }
        //make left part all be 1
        for(int j = A_Expand.m-2;j>=0;j--){
            for(int i{};i<=j;i++) {
                auto Ajj1 = A_Expand.data[j-i][j + 1];
                for (int k = j + 1; k < A_Expand.n; k++) {
                    A_Expand.data[j-i][k] -= Ajj1 * A_Expand.data[j + 1][k];
                }
            }
        }
        for(int i=0;i<res.m;i++){
            for(int j=0;j<res.n;j++){
               res.data[i][j] = A_Expand.data[i][res.n+j];
            }
        }
        return res;
    }
    int m{}, n{};
};
mat operator*(const mat &a, const mat &b) {
    // 第一步，判断是否满足矩阵相乘法则
    if (a.n != b.m) {
        mat res{};
        return res;
    }
    mat res(a.m, b.n);
    for (int i{}; i < a.m; i++) {
        for (int j{}; j < b.n; j++) {
            float temp{};
            for (int k{}; k < b.n; k++) {
                temp += a.data[i][k] * b.data[k][j];
            }
            res.data[i][j] = temp;
        }
    }
    return res;
}

#endif//JACOBDEMO_MAT_HPP
