//
// Created by 91418 on 2024/1/11.
//
#include "pathInterpolation.h"
auto _interpolation::_1_Order_Linear_no_time_adjustment(
    const vector<vector<double>>& pathPointsData, const int& costTime) -> vector<vector<double>>
{
	float single_stepTime = static_cast<float>(costTime) / (pathPointsData.size() - 1);
	vector<vector<double>> res(costTime * 100 + 1, vector<double> {});
	res[0] = pathPointsData[0];
	int No_step {};
	for (auto it = pathPointsData.begin(); it != pathPointsData.end() - 1; it++)
	{
		auto p0 = *it, pf = *(it + 1);
		for (int j {}; j < p0.size(); j++)
		{
			float delta = (pf[j] - p0[j]) / single_stepTime / 100;
			int i {};
			while (i++ < single_stepTime * 100)
			{
				res[i + No_step * single_stepTime * 100].push_back(
				    delta + res[i + No_step * single_stepTime * 100 - 1][j]);
			}
		}
		No_step++;
	}
	return res;
}
vector<vector<double>>
_interpolation::_3_Order_CubicSpline_is_time_adjustment(vector<vector<double>> yData,
                                                        const int& totalTime)
{
	Eigen::MatrixXd yDataMat(yData.size(), yData[0].size());
	for (int i {}; i < yData.size(); i++)
	{
		yDataMat.row(i) = Eigen::VectorXd::Map(yData[i].data(), yData[i].size());
	}
	//    cout<<yDataMat<<endl;
	auto trajectoryPointsCount {100 * totalTime};
	size_t pathPointsCount {yData.size()};
	size_t jointsCount {yData[0].size()};
	Eigen::VectorXd deltaBuffer(pathPointsCount - 1);
	if (jointsCount != 0)
	{
		for (int i {}; i < pathPointsCount - 1; i++)
		{
			Eigen::VectorXd buffer = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
			    yData[i].data(), (long long)jointsCount);
			Eigen::VectorXd bufferNext = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
			    yData[i + 1].data(), (long long)jointsCount);
			Eigen::VectorXd delta = (bufferNext - buffer);
			//            cout<<delta<<endl;
			deltaBuffer[i] = delta.norm();
		}
	}
	//    cout<<"deltaBuffer"<<deltaBuffer<<endl;
	// normalize
	deltaBuffer.normalize();
	vector<double> timeArray;
	timeArray.push_back(0);
	//    cout<<"derltaBuffer"<<endl<<deltaBuffer<<endl;
	double base = deltaBuffer.lpNorm<1>();
	for (int i {}; i < deltaBuffer.size(); i++)
	{
		if (i >= 1)
			timeArray.push_back(
			    (int)(deltaBuffer[i] / base * trajectoryPointsCount + (int)timeArray[i]) + 1);
		else
			timeArray.push_back((int)(deltaBuffer[i] / base * trajectoryPointsCount));
	}
	yDataMat.transposeInPlace();
	cout << yDataMat << endl;
	vector<_1D::CubicSplineInterpolator<double>> it(yDataMat.rows());
	for (int i {}; i < jointsCount; i++)
	{
		Eigen::VectorXd x = yDataMat.row(i);
		it[i].setData(timeArray, x);
	}
	vector<vector<double>> res;
	for (int i {}; i < trajectoryPointsCount; i++)
	{
		vector<double> temp;
		for (int j {}; j < jointsCount; j++)
			temp.push_back(it[j](i));
		res.push_back(temp);
	}
	return res;
}
