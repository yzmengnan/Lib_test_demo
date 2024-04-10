#include "windows.h"
#include <conio.h>
#include <iostream>
#include <math.h>
#include <thread>

#include <Dbt.h>
#include <WS2tcpip.h>
#include <stdio.h>
#include <string>
using namespace std;

#include "DVPCamera.h"
#include <opencv2/core/utils/logger.hpp>
#include <opencv2/opencv.hpp>
#pragma warning(disable : 4996)

#define GRABCOUNT 50

bool Continuous_Running = TRUE;

bool NeedToStoreDesiredPose = FALSE;
bool AlreadyStore           = FALSE;
bool AlreadyShow            = FALSE;

char keyboardInput;
bool startGrasping = FALSE;

FILE* fileStoreImageTraj;
FILE* fileStoreImageTraj_Pose;

bool fileAreadyOpen = FALSE;

cv::Point desiredFeatures_Image[7];
cv::Point currentFeatures_Image[7];

double desiredFeatures_Pose[6];

cv::Mat desiredC_rvec_O = cv::Mat::zeros(3, 1, CV_64FC1);
cv::Mat desiredC_tvec_O = cv::Mat::zeros(3, 1, CV_64FC1);

cv::Mat currentE_HomogenousT_currentC = cv::Mat::eye(
    4, 4, CV_64FC1);//(cv::Mat_<double>(4, 4) << 1544.970675627578, 0, 1312.414149621184, 0,
                    // 1544.76986234472, 965.0566324183379, 0, 0, 1);
cv::Mat initialC_HomogenousT_O = cv::Mat::zeros(4, 4, CV_64FC1);

bool bFirstStage_Pose = TRUE;

bool successfulDetect   = FALSE;
bool successfulTracking = FALSE;

vector<cv::Point> featurePoints_Sending;

typedef struct {
	cv::Point featurePoints_Snd[7];
} Package_Feature_Send;

typedef struct {
	double translVel[3];
	double rotVel[3];
} Robot_Command, Package_Command_Send;

Robot_Command commands_Sending;

bool bsendingFeature = FALSE;

double max_translVel = 5;
double max_rotVel    = (double)20 / (double)180 * 3.1415926;

bool canSend = FALSE;

bool firstConnect = TRUE;

bool bTCP = TRUE;

bool successConnect = FALSE;

typedef struct {
	int upIndex;
	int bottom_LeftIndex;
	int bottom_CenterIndex;
	int bottom_RightIndex;
} GroupIndexAlloc;

typedef struct {
	cv::Vec4f VertiLinePara;
	cv::Vec4f HorizLinePara;
} GroupLinesParameters;

bool need2StoreDesiredInfo = FALSE;

vector<vector<cv::Point>> featurePoints_Filtered_Queue;
vector<cv::Point> featurePoints_Filtered_Sum;
int featureQueueSize = 4;

// RGB to BGR
bool RGB2BGR(unsigned char* pRgbData, unsigned int nWidth, unsigned int nHeight)
{
	if (NULL == pRgbData)
	{
		return false;
	}

	for (unsigned int j = 0; j < nHeight; j++)
	{
		for (unsigned int i = 0; i < nWidth; i++)
		{
			unsigned char red                      = pRgbData[j * (nWidth * 3) + i * 3];
			pRgbData[j * (nWidth * 3) + i * 3]     = pRgbData[j * (nWidth * 3) + i * 3 + 2];
			pRgbData[j * (nWidth * 3) + i * 3 + 2] = red;
		}
	}
	return true;
}

// �ѻ�ȡ����bufferת��Mat��ʽ
bool Convert2Mat(dvpFrame* pFrameInfo, unsigned char* pData, cv::Mat& srcImage)
{
	if (pFrameInfo->format == FORMAT_MONO)
	{
		srcImage = cv::Mat(pFrameInfo->iHeight, pFrameInfo->iWidth, CV_8UC1, pData);
		// printf("MONO convert to cv::Mat OK.\n");
	}
	else if (pFrameInfo->format == FORMAT_BGR24)
	{
		srcImage = cv::Mat(pFrameInfo->iHeight, pFrameInfo->iWidth, CV_8UC3, pData);
		// printf("BGR24 convert to cv::Mat OK.\n");
	}
	else
	{
		printf("unsupported pixel format\n");
		return false;
	}

	if (NULL == srcImage.data)
	{
		printf("Invalid data!\n");
		return false;
	}

	/* ����ͼƬ */
	try
	{
		cv::imwrite("MatImage.bmp", srcImage);
	}
	catch (cv::Exception& ex)
	{
		fprintf(stderr, "Exception saving image to bmp format: %s\n", ex.what());
	}

	return true;
}

float GetPoint2PointDist(cv::Point point1, cv::Point point2)
{
	float dist = 0;

	dist = sqrtf(powf(point1.x - point2.x, 2) + powf(point1.y - point2.y, 2));

	return dist;
}

float GetPoint2LineDist(cv::Point point, cv::Vec4f LinePara)
{
	float dist = 0;
	float k    = 0;
	float b    = 0;

	k = LinePara[1] / LinePara[0];
	b = LinePara[3] - k * LinePara[2];

	dist = fabs(k * point.x - point.y + b) / (sqrtf(k * k + 1));

	return dist;
}

cv::Point GetIntersectionPoint(cv::Vec4f lineParaA, cv::Vec4f lineParaB)
{
	cv::Point2f intersectionPoint;
	float kA = 0;
	float bA = 0;
	float kB = 0;
	float bB = 0;

	kA = lineParaA[1] / lineParaA[0];
	bA = lineParaA[3] - kA * lineParaA[2];

	kB = lineParaB[1] / lineParaB[0];
	bB = lineParaB[3] - kB * lineParaB[2];

	intersectionPoint.x = (bB - bA) / (kA - kB);
	intersectionPoint.y = kA * intersectionPoint.x + bA;

	return (cv::Point)intersectionPoint;
}

void GetMoreAccurateFeaturePoints(vector<cv::Point>& featurePoints, vector<cv::Point> contourPoints,
                                  float epsilonAccuracy)
{
	cv::Vec4f one_linePara_coarse;
	cv::Vec4f one_linePara_fine;
	vector<cv::Vec4f> all_lineParas_fine;

	vector<cv::Point> one_linePoints;
	vector<vector<cv::Point>> all_linePoints;

	vector<cv::Point> featurePoints_MoreAccurate;

	cv::Point linePointA;
	cv::Point linePointB;

	vector<cv::Point> segmentPoints;
	cv::Point checkPoint;

	float distCheckPoint2Line;

	int segmentNum = featurePoints.size();

	cv::Point one_intersectionPoint_fine;
	vector<cv::Point> all_intersectionPoints_fine;
	cv::Vec4f lineParaA_fine;
	cv::Vec4f lineParaB_fine;

	vector<cv::Point> contourPoints_temp;
	contourPoints_temp = contourPoints;

	// vector<cv::Point>::iterator itr;

	vector<bool> alreadyConsidered;
	for (int i = 0; i < contourPoints.size(); i++)
	{
		alreadyConsidered.push_back(FALSE);
	}

	for (int i = 0; i < segmentNum; i++)
	{
		linePointA = featurePoints.at(i);
		segmentPoints.push_back(linePointA);
		linePointB = featurePoints.at((i + 1) % featurePoints.size());
		segmentPoints.push_back(linePointB);

		cv::fitLine(segmentPoints, one_linePara_coarse, cv::DIST_L2, 0, 0.01, 0.01);

		// itr = contourPoints_temp.begin();
		for (int j = 0; j < contourPoints_temp.size(); j++)
		{
			checkPoint          = contourPoints_temp.at(j);
			distCheckPoint2Line = GetPoint2LineDist(checkPoint, one_linePara_coarse);

			if (distCheckPoint2Line < epsilonAccuracy)// �����������ڵ�ǰ���߶�
			{
				one_linePoints.push_back(checkPoint);
				// itr = contourPoints_temp.erase(itr);  //�Ƴ��Ѿ����ֵ�������
				alreadyConsidered.at(j) = TRUE;// ����ѿ��Ǹ�������
			}
		}

		segmentPoints.clear();

		// �������ڵ�ǰ�߶ε�������
		all_linePoints.push_back(one_linePoints);
		one_linePoints.clear();
	}

	// ��ȡ�����߶θ���ȷ��ֱ�߲���
	for (int p = 0; p < segmentNum; p++)
	{
		one_linePoints = all_linePoints.at(p);// �������ڵ�ǰ�߶ε�����������
		cv::fitLine(one_linePoints, one_linePara_fine, cv::DIST_L2, 0, 0.01, 0.01);
		all_lineParas_fine.push_back(one_linePara_fine);
	}

	// ���»���δ�����������ֱ�߹���
	float minDistP2L;
	float distP2L;
	int nearestLineIndex;
	for (int i = 0; i < contourPoints.size(); i++)
	{
		if (!alreadyConsidered.at(i))
		{
			printf("��������δ����\r\n\n");
			checkPoint = contourPoints.at(i);
			minDistP2L = 1000;
			for (int j = 0; j < segmentNum; j++)
			{
				one_linePara_fine = all_lineParas_fine.at(j);
				distP2L           = GetPoint2LineDist(checkPoint, one_linePara_fine);
				if (distP2L < minDistP2L)
				{
					minDistP2L       = distP2L;
					nearestLineIndex = j;
				}
			}
			all_linePoints.at(nearestLineIndex).push_back(checkPoint);
		}
	}

	all_lineParas_fine.clear();
	// ��ȡ������������������ȷ��ֱ�߲���
	for (int p = 0; p < segmentNum; p++)
	{
		one_linePoints = all_linePoints.at(p);// �������ڵ�ǰ�߶ε�����������
		cv::fitLine(one_linePoints, one_linePara_fine, cv::DIST_L2, 0, 0.01, 0.01);
		all_lineParas_fine.push_back(one_linePara_fine);
	}

	// ��ȡ����ȷֱ�ߵĽ���
	for (int p = 0; p < segmentNum; p++)
	{
		lineParaA_fine = all_lineParas_fine.at(p);
		lineParaB_fine = all_lineParas_fine.at((p + 1) % segmentNum);

		one_intersectionPoint_fine = GetIntersectionPoint(lineParaA_fine, lineParaB_fine);
		all_intersectionPoints_fine.push_back(one_intersectionPoint_fine);
	}

	// Ѱ��ƥ��Ľ���
	int NearestIndex;
	float minDist;
	float distA2B;
	cv::Point consideredPointA;
	cv::Point consideredPointB;

	for (int i = 0; i < segmentNum; i++)
	{
		consideredPointA = featurePoints.at(i);
		minDist          = 1000;

		for (int j = 0; j < segmentNum; j++)
		{
			consideredPointB = all_intersectionPoints_fine.at(j);

			distA2B = GetPoint2PointDist(consideredPointA, consideredPointB);
			if (distA2B < minDist)
			{
				minDist      = distA2B;
				NearestIndex = j;
			}
		}

		featurePoints_MoreAccurate.push_back(all_intersectionPoints_fine.at(NearestIndex));
	}

	featurePoints = featurePoints_MoreAccurate;
}

bool Square_Triangle_Matching(cv::Point2f center_of_square, float average_side_square,
                              float area_square, cv::Point2f center_of_triangle,
                              float average_side_triangle, float area_triangle)
{
	float distance_square_2_triangle   = sqrtf(powf(center_of_square.x - center_of_triangle.x, 2)
	                                           + powf(center_of_square.y - center_of_triangle.y, 2));
	float side_ratio_square_2_triangle = average_side_square / average_side_triangle;
	float area_ratio_square_2_triangle = area_square / area_triangle;
	float average_side                 = (average_side_square + average_side_triangle) / 2;

	if (distance_square_2_triangle > 1 / 3 * average_side
	    && distance_square_2_triangle < 3 * average_side && side_ratio_square_2_triangle > 1 / 2
	    && side_ratio_square_2_triangle < 2 && area_ratio_square_2_triangle > 1 / 3
	    && area_ratio_square_2_triangle < 3)
		return TRUE;
	else
		return FALSE;
}

void FeaturePoint_Extraction(int matched_num, vector<vector<cv::Point>> matched_squares,
                             vector<vector<cv::Point>> matched_triangles,
                             vector<vector<cv::Point>>& FeaturePoints_AllMatching)
{
	vector<cv::Point> FeaturePoints_OneMatching;
	int Nearest_SquareIndex;
	int Nearest_TriangleIndex;
	float Minimum_Distance;
	float distance_2Points;

	for (int i = 0; i < matched_num; i++)
	{
		distance_2Points =
		    sqrtf(powf(matched_squares.at(i).at(0).x - matched_triangles.at(i).at(0).x, 2)
		          + powf(matched_squares.at(i).at(0).y - matched_triangles.at(i).at(0).y, 2));
		Minimum_Distance      = distance_2Points;
		Nearest_SquareIndex   = 0;
		Nearest_TriangleIndex = 0;

		for (int j = 0; j < 4; j++)
		{
			for (int k = 0; k < 3; k++)
			{
				distance_2Points = sqrtf(
				    powf(matched_squares.at(i).at(j).x - matched_triangles.at(i).at(k).x, 2)
				    + powf(matched_squares.at(i).at(j).y - matched_triangles.at(i).at(k).y, 2));
				if (distance_2Points < Minimum_Distance)
				{
					Minimum_Distance      = distance_2Points;
					Nearest_SquareIndex   = j;
					Nearest_TriangleIndex = k;
				}
			}
		}

		for (int j = 0; j < 4; j++)
		{
			FeaturePoints_OneMatching.push_back(
			    matched_squares.at(i).at((j + Nearest_SquareIndex) % 4));
		}

		for (int j = 0; j < 3; j++)
		{
			FeaturePoints_OneMatching.push_back(
			    matched_triangles.at(i).at((j + Nearest_TriangleIndex) % 3));
		}

		FeaturePoints_AllMatching.push_back(FeaturePoints_OneMatching);
	}
}

void Feature_Detection(int ctrl, cv::Mat& imgsrc, vector<vector<cv::Point>>& featurePoints)
{
	cv::Mat imgHSV, imgBGR;
	cv::Mat imgThresholded;
	cv::Mat imgThresholded_green;
	cv::Mat imgThresholded_indigo;
	// if (0)
	//{
	//	vector<cv::Mat> hsvSplit;
	//	cv::cvtColor(imgsrc,imgHSV,cv::COLOR_BGR2HSV);
	//	cv::split(imgHSV,hsvSplit);
	//	cv::equalizeHist(hsvSplit[2],hsvSplit[2]);
	//	cv::merge(hsvSplit,imgHSV);
	//	cv::cvtColor(imgHSV,imgBGR,cv::COLOR_HSV2BGR);
	// }
	// imgBGR = imgsrc.clone();
	cv::cvtColor(imgsrc, imgHSV, cv::COLOR_BGR2HSV);
	// else
	//{
	//	imgBGR = imgsrc.clone();
	// }

	switch (ctrl)
	{
		case 0:
		{
			// cv::inRange(imgHSV,cv::Scalar(128,0,0),cv::Scalar(255,127,127),imgThresholded);
			// //��ɫBGR
			cv::inRange(imgHSV, cv::Scalar(100, 43, 46), cv::Scalar(124, 255, 255),
			            imgThresholded);// ��ɫHSV
			break;
		}
		case 1:
		{
			// cv::inRange(imgHSV,cv::Scalar(128,128,128),cv::Scalar(255,255,255),imgThresholded);//��ɫBGR
			cv::inRange(imgHSV, cv::Scalar(0, 0, 221), cv::Scalar(180, 30, 255),
			            imgThresholded);// ��ɫHSV
			break;
		}
		case 2:
		{
			// cv::inRange(imgHSV,cv::Scalar(128,128,0),cv::Scalar(255,255,127),imgThresholded);//��ɫBGR
			// cv::inRange(imgHSV, cv::Scalar(72, 40, 30), cv::Scalar(102, 255, 255),
			// imgThresholded_indigo);//��ɫHSV cv::inRange(imgHSV, cv::Scalar(90, 40, 40),
			// cv::Scalar(150, 255, 255), imgThresholded_green);//��ɫ cv::inRange(imgHSV,
			// cv::Scalar(48, 48, 5), cv::Scalar(88, 240, 255), imgThresholded_green);//��ɫ
			// cv::add(imgThresholded_indigo,imgThresholded_green,imgThresholded);

			// cv::inRange(imgHSV, cv::Scalar(48, 48, 5), cv::Scalar(88, 240, 240),
			// imgThresholded);//��ɫHSV
			cv::inRange(imgHSV, cv::Scalar(63, 40, 40), cv::Scalar(140, 255, 255),
			            imgThresholded);// ��ɫHSV
			break;
		}
		case 3:
		{
			// cv::inRange(imgHSV,cv::Scalar(128,0,128),cv::Scalar(255,127,255),imgThresholded);//��ɫBGR
			cv::inRange(imgHSV, cv::Scalar(125, 43, 46), cv::Scalar(155, 255, 255),
			            imgThresholded);// ��ɫHSV
			break;
		}
		case 4:
		{
			// cv::inRange(imgHSV,cv::Scalar(0,128,128),cv::Scalar(127,255,255),imgThresholded);//��ɫBGR
			// cv::inRange(imgHSV, cv::Scalar(26, 43, 46), cv::Scalar(34, 255, 255),
			// imgThresholded);//��ɫHSV
			cv::inRange(imgHSV, cv::Scalar(27, 45, 30), cv::Scalar(39, 255, 255),
			            imgThresholded);// ��ɫHSV
			break;
		}
		case 5:
		{
			// cv::inRange(imgHSV,cv::Scalar(0,128,0),cv::Scalar(127,255,127),imgThresholded);//��ɫBGR
			// cv::inRange(imgHSV, cv::Scalar(35, 43, 46), cv::Scalar(77, 255, 255),
			// imgThresholded);//��ɫHSV
			cv::inRange(imgHSV, cv::Scalar(45, 48, 48), cv::Scalar(88, 240, 240),
			            imgThresholded_green);// ��ɫHSV
			cv::inRange(imgHSV, cv::Scalar(72, 40, 30), cv::Scalar(102, 255, 255),
			            imgThresholded_indigo);// ��ɫHSV
			cv::add(imgThresholded_indigo, imgThresholded_green, imgThresholded);
			break;
		}
		case 6:
		{
			// cv::inRange(imgHSV,cv::Scalar(0,0,128),cv::Scalar(127,127,255),imgThresholded);//��ɫBGR
			// cv::inRange(imgHSV, cv::Scalar(0, 43, 46), cv::Scalar(10, 255, 255),
			// imgThresholded);//��ɫHSV
			cv::inRange(imgHSV, cv::Scalar(0, 43, 30), cv::Scalar(15, 255, 255),
			            imgThresholded);// ��ɫHSV
			break;
		}
		case 7:
		{
			// cv::inRange(imgHSV,cv::Scalar(0,0,0),cv::Scalar(127,127,127),imgThresholded);//��ɫBGR
			cv::inRange(imgHSV, cv::Scalar(0, 0, 0), cv::Scalar(180, 255, 46),
			            imgThresholded);// ��ɫHSV
			break;
		}
	}

	cv::namedWindow("ThresholdedImage", 0);
	cv::resizeWindow("ThresholdedImage", 640, 480);
	cv::imshow("ThresholdedImage", imgThresholded);

	cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));

	cv::morphologyEx(imgThresholded, imgThresholded, cv::MORPH_CLOSE, element);
	cv::morphologyEx(imgThresholded, imgThresholded, cv::MORPH_OPEN, element);

	cv::namedWindow("ThresholdedImageAfterMorph", 0);
	cv::resizeWindow("ThresholdedImageAfterMorph", 640, 480);
	cv::imshow("ThresholdedImageAfterMorph", imgThresholded);

	vector<vector<cv::Point>> contours;
	vector<cv::Vec4i> hierarchy;

	cv::findContours(imgThresholded, contours, hierarchy, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);

	vector<cv::Point> triangle;
	vector<vector<cv::Point>> Triangles;
	vector<vector<cv::Point>> MatchedTriangles;

	vector<cv::Point> approx;

	vector<cv::Point> square;
	vector<vector<cv::Point>> Squares;
	vector<vector<cv::Point>> MatchedSquares;

	int MatchedNum = 0;

	cv::RotatedRect rotatedrect;
	cv::Point2f rectPoints[4];

	vector<cv::Point> square_To_triangle;

	vector<float> AreaOfSquares;
	vector<float> AreaOfTriangles;

	float area_square;
	float area_triangle;

	vector<float> AverageSide_Squares;
	vector<float> AverageSide_Triangles;

	float average_side_square;
	float average_side_triangle;

	vector<cv::Point2f> Center_Of_Squares;
	vector<cv::Point2f> Center_Of_Triangles;

	cv::Point2f center_of_square;
	cv::Point2f center_of_triangle;

	int squares_num   = 0;
	int triangles_num = 0;

	float square_side_length[4]   = {0, 0, 0, 0};
	float triangle_side_length[3] = {0, 0, 0};

	float side_length_min = 0;
	float side_length_max = 0;

	float side_ratio              = 0;
	float opposite_side_ratios[2] = {0, 0};

	int contourSize_Square = 150;// ԭ������600
	int contourSize_Triang = 100;// ԭ������400

	for (size_t i = 0; i < contours.size(); i++)
	{
		cv::approxPolyDP(contours.at(i), approx,
		                 cv::arcLength(cv::Mat(contours.at(i)), TRUE) * 0.02, TRUE);

		if (approx.size() == 4 && fabs(cv::contourArea(cv::Mat(approx))) > contourSize_Square
		    && cv::isContourConvex(cv::Mat(approx)))
		{
			for (int j = 0; j < 4; j++)
			{
				square.push_back(cv::Point(approx.at(j).x, approx.at(j).y));
				// ctrl+K��ctrl+C//ctrl+K��ctrl+U
			}
			cv::drawContours(imgsrc, contours, i, cv::Scalar(255, 0, 0), 3);

			square_side_length[0] = sqrtf(powf(approx.at(0).x - approx.at(1).x, 2)
			                              + powf(approx.at(0).y - approx.at(1).y, 2));
			side_length_min       = square_side_length[0];
			side_length_max       = square_side_length[0];

			square_side_length[1] = sqrtf(powf(approx.at(1).x - approx.at(2).x, 2)
			                              + powf(approx.at(1).y - approx.at(2).y, 2));

			if (square_side_length[1] < side_length_min)
				side_length_min = square_side_length[1];
			else if (square_side_length[1] > side_length_max)
				side_length_max = square_side_length[1];

			square_side_length[2] = sqrtf(powf(approx.at(2).x - approx.at(3).x, 2)
			                              + powf(approx.at(2).y - approx.at(3).y, 2));

			if (square_side_length[2] < side_length_min)
				side_length_min = square_side_length[2];
			else if (square_side_length[2] > side_length_max)
				side_length_max = square_side_length[2];

			square_side_length[3] = sqrtf(powf(approx.at(3).x - approx.at(0).x, 2)
			                              + powf(approx.at(3).y - approx.at(0).y, 2));

			if (square_side_length[3] < side_length_min)
				side_length_min = square_side_length[3];
			else if (square_side_length[3] > side_length_max)
				side_length_max = square_side_length[3];

			side_ratio              = side_length_min / side_length_max;
			opposite_side_ratios[0] = square_side_length[0] / square_side_length[2];

			if (opposite_side_ratios[0] > 1)
				opposite_side_ratios[0] = square_side_length[2] / square_side_length[0];

			opposite_side_ratios[1] = square_side_length[1] / square_side_length[3];

			if (opposite_side_ratios[1] > 1)
				opposite_side_ratios[1] = square_side_length[3] / square_side_length[1];

			float lengRatio_square = cv::arcLength(cv::Mat(contours.at(i)), TRUE)
			                       / (square_side_length[0] + square_side_length[1]
			                          + square_side_length[2] + square_side_length[3]);

			if (side_ratio > 0.3 && opposite_side_ratios[0] > 0.8 && opposite_side_ratios[1] > 0.8
			    && lengRatio_square < 1.5 && lengRatio_square > 2 / 3)
			{
				// rotatedrect = cv::minAreaRect(contours.at(i));

				// rotatedrect.points(rectPoints);
				// center_of_square = rotatedrect.center;

				center_of_square.x  = (square[0].x + square[2].x) / 2;
				center_of_square.y  = (square[0].y + square[2].y) / 2;
				area_square         = square_side_length[0] * square_side_length[1];
				average_side_square = (square_side_length[0] + square_side_length[1]) / 2;

				// square.push_back((cv::Point)rectPoints[0]);
				// square.push_back((cv::Point)rectPoints[1]);
				// square.push_back((cv::Point)rectPoints[2]);
				// square.push_back((cv::Point)rectPoints[3]);

				squares_num++;

				// ��ȡ����ȷ��������
				vector<cv::Point> featurePoints_moreAccurate;
				featurePoints_moreAccurate.push_back(square[0]);
				featurePoints_moreAccurate.push_back(square[1]);
				featurePoints_moreAccurate.push_back(square[2]);
				featurePoints_moreAccurate.push_back(square[3]);

				GetMoreAccurateFeaturePoints(featurePoints_moreAccurate, contours.at(i),
				                             side_length_min / 12);
				for (int j = 0; j < 4; j++)
				{
					square[j] = featurePoints_moreAccurate.at(j);
				}
				featurePoints_moreAccurate.clear();
				// ��ȡ����ȷ�����������

				Squares.push_back(square);
				Center_Of_Squares.push_back(center_of_square);
				AreaOfSquares.push_back(area_square);
				AverageSide_Squares.push_back(average_side_square);

				// cv::line(imgsrc, cv::Point(square.at(0).x, square.at(0).y),
				// cv::Point(square.at(1).x, square.at(1).y), cv::Scalar(0, 0, 255), 3, 8, 0); //BGR
				// cv::line(imgsrc, cv::Point(square.at(1).x, square.at(1).y),
				// cv::Point(square.at(2).x, square.at(2).y), cv::Scalar(0, 255, 0), 3, 8, 0);
				// cv::line(imgsrc, cv::Point(square.at(2).x, square.at(2).y),
				// cv::Point(square.at(3).x, square.at(3).y), cv::Scalar(255, 0, 0), 3, 8, 0);
				// cv::line(imgsrc, cv::Point(square.at(3).x, square.at(3).y),
				// cv::Point(square.at(0).x, square.at(0).y), cv::Scalar(0, 0, 0), 3, 8, 0);
				// cv::circle(imgsrc,(cv::Point)center_of_square,15,cv::Scalar(0,0,255),2,8,0);
			}
			else if (side_ratio < 0.2)
			{
				area_triangle = cv::minEnclosingTriangle(
				    contours.at(i),
				    square_To_triangle);// �п��ܻ��Ƿ���������㣡������������������Խ����ʣ�������������

				if (square_To_triangle.size() == 3)
				{
					triangle_side_length[0] =
					    sqrtf(powf(square_To_triangle.at(0).x - square_To_triangle.at(1).x, 2)
					          + powf(square_To_triangle.at(0).y - square_To_triangle.at(1).y, 2));
					side_length_min = triangle_side_length[0];
					side_length_max = triangle_side_length[0];

					triangle_side_length[1] =
					    sqrtf(powf(square_To_triangle.at(1).x - square_To_triangle.at(2).x, 2)
					          + powf(square_To_triangle.at(1).y - square_To_triangle.at(2).y, 2));

					if (triangle_side_length[1] < side_length_min)
						side_length_min = triangle_side_length[1];
					else if (triangle_side_length[1] > side_length_max)
						side_length_max = triangle_side_length[1];

					triangle_side_length[2] =
					    sqrtf(powf(square_To_triangle.at(2).x - square_To_triangle.at(0).x, 2)
					          + powf(square_To_triangle.at(2).y - square_To_triangle.at(0).y, 2));

					if (triangle_side_length[2] < side_length_min)
						side_length_min = triangle_side_length[2];
					else if (triangle_side_length[2] > side_length_max)
						side_length_max = triangle_side_length[2];

					side_ratio = side_length_min / side_length_max;

					float lengRatio_triangle = cv::arcLength(cv::Mat(contours.at(i)), TRUE)
					                         / (triangle_side_length[0] + triangle_side_length[1]
					                            + triangle_side_length[2]);

					if (side_ratio > 0.5 && lengRatio_triangle < 1.5 && lengRatio_triangle > 2 / 3)
					{
						triangles_num++;

						average_side_triangle = (triangle_side_length[0] + triangle_side_length[1]
						                         + triangle_side_length[2])
						                      / 3;
						center_of_triangle.x =
						    (square_To_triangle.at(0).x + square_To_triangle.at(1).x
						     + square_To_triangle.at(2).x)
						    / 3;
						center_of_triangle.y =
						    (square_To_triangle.at(0).y + square_To_triangle.at(1).y
						     + square_To_triangle.at(2).y)
						    / 3;

						// ��ȡ����ȷ��������
						vector<cv::Point> featurePoints_moreAccurate;
						featurePoints_moreAccurate.push_back(square_To_triangle[0]);
						featurePoints_moreAccurate.push_back(square_To_triangle[1]);
						featurePoints_moreAccurate.push_back(square_To_triangle[2]);

						GetMoreAccurateFeaturePoints(featurePoints_moreAccurate, contours.at(i),
						                             side_length_min / 12);
						for (int j = 0; j < 3; j++)
						{
							square_To_triangle[j] = featurePoints_moreAccurate.at(j);
						}
						featurePoints_moreAccurate.clear();
						// ��ȡ����ȷ�����������

						Triangles.push_back(square_To_triangle);
						Center_Of_Triangles.push_back(center_of_triangle);
						AreaOfTriangles.push_back(area_triangle);
						AverageSide_Triangles.push_back(average_side_triangle);

						// cv::line(imgsrc, cv::Point(square_To_triangle.at(0).x,
						// square_To_triangle.at(0).y), cv::Point(square_To_triangle.at(1).x,
						// square_To_triangle.at(1).y), cv::Scalar(0, 255, 0), 3, 8, 0);
						// cv::line(imgsrc, cv::Point(square_To_triangle.at(1).x,
						// square_To_triangle.at(1).y), cv::Point(square_To_triangle.at(2).x,
						// square_To_triangle.at(2).y), cv::Scalar(0, 255, 0), 3, 8, 0);
						// cv::line(imgsrc, cv::Point(square_To_triangle.at(2).x,
						// square_To_triangle.at(2).y), cv::Point(square_To_triangle.at(0).x,
						// square_To_triangle.at(0).y), cv::Scalar(0, 255, 0), 3, 8, 0);
						// cv::circle(imgsrc,(cv::Point)center_of_triangle,15,cv::Scalar(0,0,255),2,8,0);

						printf("�ɡ��ı��Ρ�����Ϊ�������Ρ�\r\n");
					}
				}
				square_To_triangle.clear();
			}
			square.clear();
		}
		else if (approx.size() == 3 && fabs(cv::contourArea(cv::Mat(approx))) > contourSize_Triang
		         && cv::isContourConvex(cv::Mat(approx)))
		{
			cv::drawContours(imgsrc, contours, i, cv::Scalar(255, 0, 0), 3);
			for (int j = 0; j < 3; j++)
			{
				triangle.push_back(cv::Point(approx.at(j).x, approx.at(j).y));
			}

			area_triangle = fabs(cv::contourArea(cv::Mat(approx)));

			// cv::minEnclosingTriangle(contours.at(i),triangle);  //��ø���ϸ��������

			if (triangle.size() == 3)
			{
				triangle_side_length[0] = sqrtf(powf(triangle.at(0).x - triangle.at(1).x, 2)
				                                + powf(triangle.at(0).y - triangle.at(1).y, 2));
				side_length_min         = triangle_side_length[0];
				side_length_max         = triangle_side_length[0];

				triangle_side_length[1] = sqrtf(powf(triangle.at(1).x - triangle.at(2).x, 2)
				                                + powf(triangle.at(1).y - triangle.at(2).y, 2));

				if (triangle_side_length[1] < side_length_min)
					side_length_min = triangle_side_length[1];
				else if (triangle_side_length[1] > side_length_max)
					side_length_max = triangle_side_length[1];

				triangle_side_length[2] = sqrtf(powf(triangle.at(2).x - triangle.at(0).x, 2)
				                                + powf(triangle.at(2).y - triangle.at(0).y, 2));

				if (triangle_side_length[2] < side_length_min)
					side_length_min = triangle_side_length[2];
				else if (triangle_side_length[2] > side_length_max)
					side_length_max = triangle_side_length[2];

				side_ratio = side_length_min / side_length_max;

				float lengRatio_triangle =
				    cv::arcLength(cv::Mat(contours.at(i)), TRUE)
				    / (triangle_side_length[0] + triangle_side_length[1] + triangle_side_length[2]);

				if (side_ratio > 0.5 && lengRatio_triangle < 1.5 && lengRatio_triangle > 2 / 3)
				{
					triangles_num++;
					average_side_triangle = (triangle_side_length[0] + triangle_side_length[1]
					                         + triangle_side_length[2])
					                      / 3;
					center_of_triangle.x =
					    (triangle.at(0).x + triangle.at(1).x + triangle.at(2).x) / 3;
					center_of_triangle.y =
					    (triangle.at(0).y + triangle.at(1).y + triangle.at(2).y) / 3;

					// ��ȡ����ȷ��������
					vector<cv::Point> featurePoints_moreAccurate;
					featurePoints_moreAccurate.push_back(triangle[0]);
					featurePoints_moreAccurate.push_back(triangle[1]);
					featurePoints_moreAccurate.push_back(triangle[2]);

					GetMoreAccurateFeaturePoints(featurePoints_moreAccurate, contours.at(i),
					                             side_length_min / 12);
					for (int j = 0; j < 3; j++)
					{
						triangle[j] = featurePoints_moreAccurate.at(j);
					}
					featurePoints_moreAccurate.clear();
					// ��ȡ����ȷ�����������

					Triangles.push_back(triangle);
					Center_Of_Triangles.push_back(center_of_triangle);
					AreaOfTriangles.push_back(area_triangle);
					AverageSide_Triangles.push_back(average_side_triangle);

					// cv::line(imgsrc, cv::Point(triangle.at(0).x, triangle.at(0).y),
					// cv::Point(triangle.at(1).x, triangle.at(1).y), cv::Scalar(0, 0, 255), 3, 8,
					// 0); //BGR cv::line(imgsrc, cv::Point(triangle.at(1).x, triangle.at(1).y),
					// cv::Point(triangle.at(2).x, triangle.at(2).y), cv::Scalar(0, 255, 0), 3, 8,
					// 0); cv::line(imgsrc, cv::Point(triangle.at(2).x, triangle.at(2).y),
					// cv::Point(triangle.at(0).x, triangle.at(0).y), cv::Scalar(255, 0, 0), 3, 8,
					// 0); cv::circle(imgsrc, (cv::Point)center_of_triangle, 15, cv::Scalar(0, 0,
					// 255), 2, 8, 0);
				}
			}
			triangle.clear();
		}
		approx.clear();
	}

	for (int i = 0; i < squares_num; i++)
		for (int j = 0; j < triangles_num; j++)
		{
			if (Square_Triangle_Matching(Center_Of_Squares.at(i), AverageSide_Squares.at(i),
			                             AreaOfSquares.at(i), Center_Of_Triangles.at(j),
			                             AverageSide_Triangles.at(j), AreaOfTriangles.at(j)))
			{
				MatchedSquares.push_back(Squares.at(i));
				MatchedTriangles.push_back(Triangles.at(j));
				MatchedNum++;
			}
		}

	if (MatchedNum == 0)
	{
		successfulDetect   = FALSE;
		successfulTracking = FALSE;
		printf("û��⵽.....����ͼ������������\r\n\n");
	}
	else
	{
		successfulDetect   = TRUE;
		successfulTracking = TRUE;
		printf("�Ѽ�⵽.....����ͼ������-------------%d ��\r\n\n", MatchedNum);
		FeaturePoint_Extraction(MatchedNum, MatchedSquares, MatchedTriangles, featurePoints);
	}

	printf("���-------The numbers of squares and triangles are respectively: "
	       "(�ı���)%d{%d}����(������)%d{%d}��\r\n\n",
	       squares_num, (int)Squares.size(), triangles_num, (int)Triangles.size());
	// cv::namedWindow("ContoursImage", 0);
	// cv::resizeWindow("ContoursImage", 640, 480);
	// cv::imshow("ContoursImage", imgsrc);
	// cv::waitKey(1);
}

void Feature_Tracking(int ctrl_ROI, cv::Mat& imgsrc, vector<cv::Point>& featurePoints_Previous,
                      vector<cv::Point>& featurePoints_Current, float WindowRatio)
{
	cv::Mat imgHSV_ROI;
	cv::Mat imgThresholded_ROI;
	cv::Mat imgThresholded_green_ROI;
	cv::Mat imgThresholded_indigo_ROI;

	cv::Point ROI_center(0, 0);
	cv::Point ROI_LeftTop;
	cv::Point ROI_RightBottom;

	int left_top_x;
	int left_top_y;
	int right_bottom_x;
	int right_bottom_y;

	int min_x = featurePoints_Previous.at(0).x;
	int max_x = featurePoints_Previous.at(0).x;
	int min_y = featurePoints_Previous.at(0).y;
	int max_y = featurePoints_Previous.at(0).y;

	int height;
	int width;

	int height_ROI;
	int width_ROI;

	for (int i = 0; i < featurePoints_Previous.size(); i++)
	{
		ROI_center.x = ROI_center.x + featurePoints_Previous.at(i).x;
		ROI_center.y = ROI_center.y + featurePoints_Previous.at(i).y;

		if (featurePoints_Previous.at(i).x < min_x)
			min_x = featurePoints_Previous.at(i).x;
		else if (featurePoints_Previous.at(i).x > max_x)
			max_x = featurePoints_Previous.at(i).x;

		if (featurePoints_Previous.at(i).y < min_y)
			min_y = featurePoints_Previous.at(i).y;
		else if (featurePoints_Previous.at(i).y > max_y)
			max_y = featurePoints_Previous.at(i).y;
	}

	ROI_center.x = (int)ROI_center.x / featurePoints_Previous.size();
	ROI_center.y = (int)ROI_center.y / featurePoints_Previous.size();

	height = max_y - min_y;
	width  = max_x - min_x;

	height_ROI = (int)height * WindowRatio;
	width_ROI  = (int)width * WindowRatio;

	left_top_x = (int)(ROI_center.x - width_ROI / 2);
	left_top_y = (int)(ROI_center.y - height_ROI / 2);

	right_bottom_x = (int)(ROI_center.x + width_ROI / 2);
	right_bottom_y = (int)(ROI_center.y + height_ROI / 2);

	if (left_top_x < 0)
		ROI_LeftTop.x = 0;
	else
		ROI_LeftTop.x = left_top_x;

	if (left_top_y < 0)
		ROI_LeftTop.y = 0;
	else
		ROI_LeftTop.y = left_top_y;

	if (right_bottom_x > imgsrc.cols)
		ROI_RightBottom.x = imgsrc.cols;
	else
		ROI_RightBottom.x = right_bottom_x;

	if (right_bottom_y > imgsrc.rows)
		ROI_RightBottom.y = imgsrc.rows;
	else
		ROI_RightBottom.y = right_bottom_y;

	cv::Mat imgBGR_ROI(imgsrc, cv::Rect(ROI_LeftTop, ROI_RightBottom));

	cv::Mat imgHSV_ROI_Blur;
	cv::Mat imgBGR_ROI_Blur(imgsrc, cv::Rect(ROI_LeftTop, ROI_RightBottom));
	cv::GaussianBlur(imgBGR_ROI, imgBGR_ROI_Blur, cv::Size(7, 7), 7, 7);
	cv::cvtColor(imgBGR_ROI_Blur, imgHSV_ROI_Blur, cv::COLOR_BGR2HSV);
	cv::GaussianBlur(imgHSV_ROI_Blur, imgHSV_ROI, cv::Size(7, 7), 7, 7);

	// cv::cvtColor(imgBGR_ROI, imgHSV_ROI, cv::COLOR_BGR2HSV);

	cv::namedWindow("TrackingWindow", 0);
	cv::resizeWindow("TrackingWindow", 640, 480);

	// cv::imshow("TrackingWindow",imgBGR_ROI);

	cv::imshow("TrackingWindow", imgBGR_ROI_Blur);

	switch (ctrl_ROI)
	{
		case 0:
		{
			// cv::inRange(imgHSV,cv::Scalar(128,0,0),cv::Scalar(255,127,127),imgThresholded);
			// //��ɫBGR
			cv::inRange(imgHSV_ROI, cv::Scalar(100, 43, 46), cv::Scalar(124, 255, 255),
			            imgThresholded_ROI);// ��ɫHSV
			break;
		}
		case 1:
		{
			// cv::inRange(imgHSV,cv::Scalar(128,128,128),cv::Scalar(255,255,255),imgThresholded);//��ɫBGR
			cv::inRange(imgHSV_ROI, cv::Scalar(0, 0, 221), cv::Scalar(180, 30, 255),
			            imgThresholded_ROI);// ��ɫHSV
			break;
		}
		case 2:
		{
			// cv::inRange(imgHSV,cv::Scalar(128,128,0),cv::Scalar(255,255,127),imgThresholded);//��ɫBGR
			// cv::inRange(imgHSV_ROI, cv::Scalar(72, 40, 30), cv::Scalar(102, 255, 255),
			// imgThresholded_indigo_ROI);//��ɫHSV cv::inRange(imgHSV_ROI, cv::Scalar(90, 40, 40),
			// cv::Scalar(150, 255, 255), imgThresholded_green_ROI);//��ɫ cv::inRange(imgHSV_ROI,
			// cv::Scalar(48, 48, 5), cv::Scalar(88, 240, 255), imgThresholded_green_ROI);//��ɫHSV
			// cv::add(imgThresholded_indigo_ROI, imgThresholded_green_ROI, imgThresholded_ROI);
			// cv::inRange(imgHSV_ROI, cv::Scalar(48, 48, 5), cv::Scalar(88, 240, 240),
			// imgThresholded_ROI);//��ɫHSV
			cv::inRange(imgHSV_ROI, cv::Scalar(63, 40, 40), cv::Scalar(140, 255, 255),
			            imgThresholded_ROI);// ��ɫHSV
			break;
		}
		case 3:
		{
			// cv::inRange(imgHSV,cv::Scalar(128,0,128),cv::Scalar(255,127,255),imgThresholded);//��ɫBGR
			cv::inRange(imgHSV_ROI, cv::Scalar(125, 43, 46), cv::Scalar(155, 255, 255),
			            imgThresholded_ROI);// ��ɫHSV
			break;
		}
		case 4:
		{
			// cv::inRange(imgHSV,cv::Scalar(0,128,128),cv::Scalar(127,255,255),imgThresholded);//��ɫBGR
			// cv::inRange(imgHSV, cv::Scalar(26, 43, 46), cv::Scalar(34, 255, 255),
			// imgThresholded);//��ɫHSV
			cv::inRange(imgHSV_ROI, cv::Scalar(27, 45, 30), cv::Scalar(39, 255, 255),
			            imgThresholded_ROI);// ��ɫHSV
			break;
		}
		case 5:
		{
			// cv::inRange(imgHSV,cv::Scalar(0,128,0),cv::Scalar(127,255,127),imgThresholded);//��ɫBGR
			// cv::inRange(imgHSV, cv::Scalar(35, 43, 46), cv::Scalar(77, 255, 255),
			// imgThresholded);//��ɫHSV
			cv::inRange(imgHSV_ROI, cv::Scalar(45, 48, 48), cv::Scalar(88, 240, 240),
			            imgThresholded_green_ROI);// ��ɫHSV
			cv::inRange(imgHSV_ROI, cv::Scalar(72, 40, 30), cv::Scalar(102, 255, 255),
			            imgThresholded_indigo_ROI);// ��ɫHSV
			cv::add(imgThresholded_indigo_ROI, imgThresholded_green_ROI, imgThresholded_ROI);
			break;
		}
		case 6:
		{
			// cv::inRange(imgHSV,cv::Scalar(0,0,128),cv::Scalar(127,127,255),imgThresholded);//��ɫBGR
			// cv::inRange(imgHSV, cv::Scalar(0, 43, 46), cv::Scalar(10, 255, 255),
			// imgThresholded);//��ɫHSV
			cv::inRange(imgHSV_ROI, cv::Scalar(0, 43, 30), cv::Scalar(15, 255, 255),
			            imgThresholded_ROI);// ��ɫHSV
			break;
		}
		case 7:
		{
			// cv::inRange(imgHSV,cv::Scalar(0,0,0),cv::Scalar(127,127,127),imgThresholded);//��ɫBGR
			cv::inRange(imgHSV_ROI, cv::Scalar(0, 0, 0), cv::Scalar(180, 255, 46),
			            imgThresholded_ROI);// ��ɫHSV
			break;
		}
	}

	cv::namedWindow("ThresholdedImage_ROI", 0);
	// cv::resizeWindow("ThresholdedImage", 640, 480);
	cv::imshow("ThresholdedImage_ROI", imgThresholded_ROI);

	cv::Mat element_ROI = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));

	cv::morphologyEx(imgThresholded_ROI, imgThresholded_ROI, cv::MORPH_CLOSE, element_ROI);
	cv::morphologyEx(imgThresholded_ROI, imgThresholded_ROI, cv::MORPH_OPEN, element_ROI);

	cv::namedWindow("ThresholdedImageAfterMorph_ROI", 0);
	// cv::resizeWindow("ThresholdedImageAfterMorph", 640, 480);
	cv::imshow("ThresholdedImageAfterMorph_ROI", imgThresholded_ROI);

	vector<vector<cv::Point>> contours_ROI;
	vector<cv::Vec4i> hierarchy_ROI;

	cv::findContours(imgThresholded_ROI, contours_ROI, hierarchy_ROI, cv::RETR_LIST,
	                 cv::CHAIN_APPROX_NONE);

	vector<vector<cv::Point>> featurePoints_ROI;
	vector<cv::Point> featurePoints_Current_ROI;

	vector<cv::Point> triangle_ROI;
	vector<vector<cv::Point>> Triangles_ROI;
	vector<vector<cv::Point>> MatchedTriangles_ROI;

	vector<cv::Point> approx_ROI;

	vector<cv::Point> square_ROI;
	vector<vector<cv::Point>> Squares_ROI;
	vector<vector<cv::Point>> MatchedSquares_ROI;

	int MatchedNum_ROI = 0;

	cv::RotatedRect rotatedrect_ROI;
	cv::Point2f rectPoints_ROI[4];

	vector<cv::Point> square_To_triangle_ROI;

	vector<float> AreaOfSquares_ROI;
	vector<float> AreaOfTriangles_ROI;

	float area_square_ROI;
	float area_triangle_ROI;

	vector<float> AverageSide_Squares_ROI;
	vector<float> AverageSide_Triangles_ROI;

	float average_side_square_ROI;
	float average_side_triangle_ROI;

	vector<cv::Point2f> Center_Of_Squares_ROI;
	vector<cv::Point2f> Center_Of_Triangles_ROI;

	cv::Point2f center_of_square_ROI;
	cv::Point2f center_of_triangle_ROI;

	int squares_num_ROI   = 0;
	int triangles_num_ROI = 0;

	float square_side_length_ROI[4]   = {0, 0, 0, 0};
	float triangle_side_length_ROI[3] = {0, 0, 0};

	float side_length_min_ROI = 0;
	float side_length_max_ROI = 0;

	float side_ratio_ROI              = 0;
	float opposite_side_ratios_ROI[2] = {0, 0};

	int contourSize_Square = 150;// ԭ������600
	int contourSize_Triang = 100;// ԭ������400

	for (size_t i = 0; i < contours_ROI.size(); i++)
	{
		cv::approxPolyDP(contours_ROI.at(i), approx_ROI,
		                 cv::arcLength(cv::Mat(contours_ROI.at(i)), TRUE) * 0.02, TRUE);

		if (approx_ROI.size() == 4
		    && fabs(cv::contourArea(cv::Mat(approx_ROI))) > contourSize_Square
		    && cv::isContourConvex(cv::Mat(approx_ROI)))
		{
			for (int j = 0; j < 4; j++)
			{
				square_ROI.push_back(cv::Point(approx_ROI.at(j).x, approx_ROI.at(j).y));
				// ctrl+K��ctrl+C//ctrl+K��ctrl+U
			}
			cv::drawContours(imgBGR_ROI, contours_ROI, i, cv::Scalar(255, 0, 0), 3);

			square_side_length_ROI[0] = sqrtf(powf(approx_ROI.at(0).x - approx_ROI.at(1).x, 2)
			                                  + powf(approx_ROI.at(0).y - approx_ROI.at(1).y, 2));
			side_length_min_ROI       = square_side_length_ROI[0];
			side_length_max_ROI       = square_side_length_ROI[0];

			square_side_length_ROI[1] = sqrtf(powf(approx_ROI.at(1).x - approx_ROI.at(2).x, 2)
			                                  + powf(approx_ROI.at(1).y - approx_ROI.at(2).y, 2));

			if (square_side_length_ROI[1] < side_length_min_ROI)
				side_length_min_ROI = square_side_length_ROI[1];
			else if (square_side_length_ROI[1] > side_length_max_ROI)
				side_length_max_ROI = square_side_length_ROI[1];

			square_side_length_ROI[2] = sqrtf(powf(approx_ROI.at(2).x - approx_ROI.at(3).x, 2)
			                                  + powf(approx_ROI.at(2).y - approx_ROI.at(3).y, 2));

			if (square_side_length_ROI[2] < side_length_min_ROI)
				side_length_min_ROI = square_side_length_ROI[2];
			else if (square_side_length_ROI[2] > side_length_max_ROI)
				side_length_max_ROI = square_side_length_ROI[2];

			square_side_length_ROI[3] = sqrtf(powf(approx_ROI.at(3).x - approx_ROI.at(0).x, 2)
			                                  + powf(approx_ROI.at(3).y - approx_ROI.at(0).y, 2));

			if (square_side_length_ROI[3] < side_length_min_ROI)
				side_length_min_ROI = square_side_length_ROI[3];
			else if (square_side_length_ROI[3] > side_length_max_ROI)
				side_length_max_ROI = square_side_length_ROI[3];

			side_ratio_ROI              = side_length_min_ROI / side_length_max_ROI;
			opposite_side_ratios_ROI[0] = square_side_length_ROI[0] / square_side_length_ROI[2];

			if (opposite_side_ratios_ROI[0] > 1)
				opposite_side_ratios_ROI[0] = square_side_length_ROI[2] / square_side_length_ROI[0];

			opposite_side_ratios_ROI[1] = square_side_length_ROI[1] / square_side_length_ROI[3];

			if (opposite_side_ratios_ROI[1] > 1)
				opposite_side_ratios_ROI[1] = square_side_length_ROI[3] / square_side_length_ROI[1];

			if (side_ratio_ROI > 0.3 && opposite_side_ratios_ROI[0] > 0.8
			    && opposite_side_ratios_ROI[1] > 0.8)
			{
				// rotatedrect_ROI = cv::minAreaRect(contours_ROI.at(i));

				// rotatedrect_ROI.points(rectPoints_ROI);
				// center_of_square_ROI = rotatedrect_ROI.center;

				center_of_square_ROI.x = (square_ROI[0].x + square_ROI[2].x) / 2;
				center_of_square_ROI.y = (square_ROI[0].y + square_ROI[2].y) / 2;
				area_square_ROI        = square_side_length_ROI[0] * square_side_length_ROI[1];
				average_side_square_ROI =
				    (square_side_length_ROI[0] + square_side_length_ROI[1]) / 2;

				// square_ROI.push_back((cv::Point)rectPoints_ROI[0]);
				// square_ROI.push_back((cv::Point)rectPoints_ROI[1]);
				// square_ROI.push_back((cv::Point)rectPoints_ROI[2]);
				// square_ROI.push_back((cv::Point)rectPoints_ROI[3]);

				squares_num_ROI++;

				// ��ȡ����ȷ��������
				vector<cv::Point> featurePoints_moreAccurate;
				featurePoints_moreAccurate.push_back(square_ROI[0]);
				featurePoints_moreAccurate.push_back(square_ROI[1]);
				featurePoints_moreAccurate.push_back(square_ROI[2]);
				featurePoints_moreAccurate.push_back(square_ROI[3]);

				GetMoreAccurateFeaturePoints(featurePoints_moreAccurate, contours_ROI.at(i),
				                             side_length_min_ROI / 12);
				for (int j = 0; j < 4; j++)
				{
					square_ROI[j] = featurePoints_moreAccurate.at(j);
				}
				featurePoints_moreAccurate.clear();
				// ��ȡ����ȷ�����������

				Squares_ROI.push_back(square_ROI);
				Center_Of_Squares_ROI.push_back(center_of_square_ROI);
				AreaOfSquares_ROI.push_back(area_square_ROI);
				AverageSide_Squares_ROI.push_back(average_side_square_ROI);

				// cv::line(imgsrc, cv::Point(square_ROI.at(0).x, square_ROI.at(0).y),
				// cv::Point(square_ROI.at(1).x, square_ROI.at(1).y), cv::Scalar(0, 0, 255), 3, 8,
				// 0);  //BGR cv::line(imgsrc, cv::Point(square_ROI.at(1).x, square_ROI.at(1).y),
				// cv::Point(square_ROI.at(2).x, square_ROI.at(2).y), cv::Scalar(0, 255, 0), 3, 8,
				// 0); cv::line(imgsrc, cv::Point(square_ROI.at(2).x, square_ROI.at(2).y),
				// cv::Point(square_ROI.at(3).x, square_ROI.at(3).y), cv::Scalar(255, 0, 0), 3, 8,
				// 0); cv::line(imgsrc, cv::Point(square_ROI.at(3).x, square_ROI.at(3).y),
				// cv::Point(square_ROI.at(0).x, square_ROI.at(0).y), cv::Scalar(0, 0, 0), 3, 8, 0);
				// cv::circle(imgsrc,(cv::Point)center_of_square_ROI,15,cv::Scalar(0,0,255),2,8,0);
			}
			else if (side_ratio_ROI < 0.2)
			{
				area_triangle_ROI = cv::minEnclosingTriangle(
				    contours_ROI.at(i),
				    square_To_triangle_ROI);// �п��ܻ��Ƿ���������㣡������������������Խ����ʣ�������������

				if (square_To_triangle_ROI.size() == 3)
				{
					triangle_side_length_ROI[0] = sqrtf(
					    powf(square_To_triangle_ROI.at(0).x - square_To_triangle_ROI.at(1).x, 2)
					    + powf(square_To_triangle_ROI.at(0).y - square_To_triangle_ROI.at(1).y, 2));
					side_length_min_ROI = triangle_side_length_ROI[0];
					side_length_max_ROI = triangle_side_length_ROI[0];

					triangle_side_length_ROI[1] = sqrtf(
					    powf(square_To_triangle_ROI.at(1).x - square_To_triangle_ROI.at(2).x, 2)
					    + powf(square_To_triangle_ROI.at(1).y - square_To_triangle_ROI.at(2).y, 2));

					if (triangle_side_length_ROI[1] < side_length_min_ROI)
						side_length_min_ROI = triangle_side_length_ROI[1];
					else if (triangle_side_length_ROI[1] > side_length_max_ROI)
						side_length_max_ROI = triangle_side_length_ROI[1];

					triangle_side_length_ROI[2] = sqrtf(
					    powf(square_To_triangle_ROI.at(2).x - square_To_triangle_ROI.at(0).x, 2)
					    + powf(square_To_triangle_ROI.at(2).y - square_To_triangle_ROI.at(0).y, 2));

					if (triangle_side_length_ROI[2] < side_length_min_ROI)
						side_length_min_ROI = triangle_side_length_ROI[2];
					else if (triangle_side_length_ROI[2] > side_length_max_ROI)
						side_length_max_ROI = triangle_side_length_ROI[2];

					side_ratio_ROI = side_length_min_ROI / side_length_max_ROI;

					if (side_ratio_ROI > 0.5)
					{
						triangles_num_ROI++;

						average_side_triangle_ROI =
						    (triangle_side_length_ROI[0] + triangle_side_length_ROI[1]
						     + triangle_side_length_ROI[2])
						    / 3;
						center_of_triangle_ROI.x =
						    (square_To_triangle_ROI.at(0).x + square_To_triangle_ROI.at(1).x
						     + square_To_triangle_ROI.at(2).x)
						    / 3;
						center_of_triangle_ROI.y =
						    (square_To_triangle_ROI.at(0).y + square_To_triangle_ROI.at(1).y
						     + square_To_triangle_ROI.at(2).y)
						    / 3;

						// ��ȡ����ȷ��������
						vector<cv::Point> featurePoints_moreAccurate;
						featurePoints_moreAccurate.push_back(square_To_triangle_ROI[0]);
						featurePoints_moreAccurate.push_back(square_To_triangle_ROI[1]);
						featurePoints_moreAccurate.push_back(square_To_triangle_ROI[2]);

						GetMoreAccurateFeaturePoints(featurePoints_moreAccurate, contours_ROI.at(i),
						                             side_length_min_ROI / 12);
						for (int j = 0; j < 3; j++)
						{
							square_To_triangle_ROI[j] = featurePoints_moreAccurate.at(j);
						}
						featurePoints_moreAccurate.clear();
						// ��ȡ����ȷ�����������

						Triangles_ROI.push_back(square_To_triangle_ROI);
						Center_Of_Triangles_ROI.push_back(center_of_triangle_ROI);
						AreaOfTriangles_ROI.push_back(area_triangle_ROI);
						AverageSide_Triangles_ROI.push_back(average_side_triangle_ROI);

						// cv::line(imgsrc, cv::Point(square_To_triangle_ROI.at(0).x,
						// square_To_triangle_ROI.at(0).y),
						// cv::Point(square_To_triangle_ROI.at(1).x,
						// square_To_triangle_ROI.at(1).y), cv::Scalar(0, 255, 0), 3, 8, 0);
						// cv::line(imgsrc, cv::Point(square_To_triangle_ROI.at(1).x,
						// square_To_triangle_ROI.at(1).y),
						// cv::Point(square_To_triangle_ROI.at(2).x,
						// square_To_triangle_ROI.at(2).y), cv::Scalar(0, 255, 0), 3, 8, 0);
						// cv::line(imgsrc, cv::Point(square_To_triangle_ROI.at(2).x,
						// square_To_triangle_ROI.at(2).y),
						// cv::Point(square_To_triangle_ROI.at(0).x,
						// square_To_triangle_ROI.at(0).y), cv::Scalar(0, 255, 0), 3, 8, 0);
						// cv::circle(imgsrc,(cv::Point)center_of_triangle_ROI,15,cv::Scalar(0,0,255),2,8,0);

						printf("�ɡ��ı��Ρ�����Ϊ�������Ρ�\r\n");
					}
				}
				square_To_triangle_ROI.clear();
			}
			square_ROI.clear();
		}
		else if (approx_ROI.size() == 3
		         && fabs(cv::contourArea(cv::Mat(approx_ROI))) > contourSize_Triang
		         && cv::isContourConvex(cv::Mat(approx_ROI)))
		{
			cv::drawContours(imgBGR_ROI, contours_ROI, i, cv::Scalar(255, 0, 0), 3);
			for (int j = 0; j < 3; j++)
			{
				triangle_ROI.push_back(cv::Point(approx_ROI.at(j).x, approx_ROI.at(j).y));
			}

			area_triangle_ROI = fabs(cv::contourArea(cv::Mat(approx_ROI)));

			// cv::minEnclosingTriangle(contours_ROI.at(i),triangle_ROI);  //��ø���ϸ��������

			if (triangle_ROI.size() == 3)
			{
				triangle_side_length_ROI[0] =
				    sqrtf(powf(triangle_ROI.at(0).x - triangle_ROI.at(1).x, 2)
				          + powf(triangle_ROI.at(0).y - triangle_ROI.at(1).y, 2));
				side_length_min_ROI = triangle_side_length_ROI[0];
				side_length_max_ROI = triangle_side_length_ROI[0];

				triangle_side_length_ROI[1] =
				    sqrtf(powf(triangle_ROI.at(1).x - triangle_ROI.at(2).x, 2)
				          + powf(triangle_ROI.at(1).y - triangle_ROI.at(2).y, 2));

				if (triangle_side_length_ROI[1] < side_length_min_ROI)
					side_length_min_ROI = triangle_side_length_ROI[1];
				else if (triangle_side_length_ROI[1] > side_length_max_ROI)
					side_length_max_ROI = triangle_side_length_ROI[1];

				triangle_side_length_ROI[2] =
				    sqrtf(powf(triangle_ROI.at(2).x - triangle_ROI.at(0).x, 2)
				          + powf(triangle_ROI.at(2).y - triangle_ROI.at(0).y, 2));

				if (triangle_side_length_ROI[2] < side_length_min_ROI)
					side_length_min_ROI = triangle_side_length_ROI[2];
				else if (triangle_side_length_ROI[2] > side_length_max_ROI)
					side_length_max_ROI = triangle_side_length_ROI[2];

				side_ratio_ROI = side_length_min_ROI / side_length_max_ROI;

				if (side_ratio_ROI > 0.5)
				{
					triangles_num_ROI++;
					average_side_triangle_ROI =
					    (triangle_side_length_ROI[0] + triangle_side_length_ROI[1]
					     + triangle_side_length_ROI[2])
					    / 3;
					center_of_triangle_ROI.x =
					    (triangle_ROI.at(0).x + triangle_ROI.at(1).x + triangle_ROI.at(2).x) / 3;
					center_of_triangle_ROI.y =
					    (triangle_ROI.at(0).y + triangle_ROI.at(1).y + triangle_ROI.at(2).y) / 3;

					// ��ȡ����ȷ��������
					vector<cv::Point> featurePoints_moreAccurate;
					featurePoints_moreAccurate.push_back(triangle_ROI[0]);
					featurePoints_moreAccurate.push_back(triangle_ROI[1]);
					featurePoints_moreAccurate.push_back(triangle_ROI[2]);

					GetMoreAccurateFeaturePoints(featurePoints_moreAccurate, contours_ROI.at(i),
					                             side_length_min_ROI / 12);
					for (int j = 0; j < 3; j++)
					{
						triangle_ROI[j] = featurePoints_moreAccurate.at(j);
					}
					featurePoints_moreAccurate.clear();
					// ��ȡ����ȷ�����������

					Triangles_ROI.push_back(triangle_ROI);
					Center_Of_Triangles_ROI.push_back(center_of_triangle_ROI);
					AreaOfTriangles_ROI.push_back(area_triangle_ROI);
					AverageSide_Triangles_ROI.push_back(average_side_triangle_ROI);

					// cv::line(imgsrc, cv::Point(triangle_ROI.at(0).x, triangle_ROI.at(0).y),
					// cv::Point(triangle_ROI.at(1).x, triangle_ROI.at(1).y), cv::Scalar(0, 0, 255),
					// 3, 8, 0); //BGR cv::line(imgsrc, cv::Point(triangle_ROI.at(1).x,
					// triangle_ROI.at(1).y), cv::Point(triangle_ROI.at(2).x, triangle_ROI.at(2).y),
					// cv::Scalar(0, 255, 0), 3, 8, 0); cv::line(imgsrc,
					// cv::Point(triangle_ROI.at(2).x, triangle_ROI.at(2).y),
					// cv::Point(triangle_ROI.at(0).x, triangle_ROI.at(0).y), cv::Scalar(255, 0, 0),
					// 3, 8, 0); cv::circle(imgsrc, (cv::Point)center_of_triangle_ROI, 15,
					// cv::Scalar(0, 0, 255), 2, 8, 0);
				}
			}
			triangle_ROI.clear();
		}
		approx_ROI.clear();
	}

	for (int i = 0; i < squares_num_ROI; i++)
		for (int j = 0; j < triangles_num_ROI; j++)
		{
			if (Square_Triangle_Matching(Center_Of_Squares_ROI.at(i), AverageSide_Squares_ROI.at(i),
			                             AreaOfSquares_ROI.at(i), Center_Of_Triangles_ROI.at(j),
			                             AverageSide_Triangles_ROI.at(j),
			                             AreaOfTriangles_ROI.at(j)))
			{
				MatchedSquares_ROI.push_back(Squares_ROI.at(i));
				MatchedTriangles_ROI.push_back(Triangles_ROI.at(j));
				MatchedNum_ROI++;
			}
		}

	if (MatchedNum_ROI == 0)
	{
		successfulTracking = FALSE;
		featurePoints_Filtered_Queue.clear();
		printf("�޷����ٵ�.....����ͼ������������\r\n\n");
	}
	else
	{
		successfulTracking = TRUE;
		printf("�Ѿ����ٵ�.....����ͼ������-------------%d ��\r\n\n", MatchedNum_ROI);
		FeaturePoint_Extraction(MatchedNum_ROI, MatchedSquares_ROI, MatchedTriangles_ROI,
		                        featurePoints_ROI);
	}

	vector<cv::Point> featurePoints_Filtered;

	double alpha = 0.9;

	if (featurePoints_ROI.size() > 0)
	{
		featurePoints_Current.clear();
		featurePoints_Current_ROI = featurePoints_ROI.at(0);
		for (int i = 0; i < featurePoints_Current_ROI.size(); i++)
		{
			featurePoints_Current.push_back(
			    cv::Point(featurePoints_Current_ROI.at(i).x + ROI_LeftTop.x,
			              featurePoints_Current_ROI.at(i).y + ROI_LeftTop.y));
		}

		// filtering
		featurePoints_Filtered = featurePoints_Current;
		for (int i = 0; i < featurePoints_Current_ROI.size(); i++)
		{
			featurePoints_Filtered.at(i).x = alpha * featurePoints_Current.at(i).x
			                               + (1 - alpha) * featurePoints_Previous.at(i).x;
			featurePoints_Filtered.at(i).y = alpha * featurePoints_Current.at(i).y
			                               + (1 - alpha) * featurePoints_Previous.at(i).y;
		}
		featurePoints_Current = featurePoints_Filtered;

		featurePoints_Filtered_Queue.push_back(featurePoints_Filtered);

		if (featurePoints_Filtered_Queue.size() > featureQueueSize)
		{
			featurePoints_Filtered_Queue.erase(featurePoints_Filtered_Queue.cbegin());
			for (int i = 0; i < featurePoints_ROI.size(); i++)
			{
				featurePoints_Filtered_Sum.push_back(cv::Point(0, 0));
			}
			for (int i = 0; i < featurePoints_ROI.size(); i++)
			{
				for (int j = 0; j < featureQueueSize; j++)
				{
					featurePoints_Filtered_Sum.at(i) =
					    featurePoints_Filtered_Sum.at(i) + featurePoints_Filtered_Queue.at(j).at(i);
				}
				featurePoints_Current.at(i) = featurePoints_Filtered_Sum.at(i) / featureQueueSize;
			}

			featurePoints_Filtered_Sum.clear();
		}
		else
		{
			for (int i = 0; i < featurePoints_ROI.size(); i++)
			{
				featurePoints_Filtered_Sum.push_back(cv::Point(0, 0));
			}

			int queueSize = featurePoints_Filtered_Queue.size();
			for (int i = 0; i < featurePoints_ROI.size(); i++)
			{
				for (int j = 0; j < queueSize; j++)
				{
					featurePoints_Filtered_Sum.at(i) =
					    featurePoints_Filtered_Sum.at(i) + featurePoints_Filtered_Queue.at(j).at(i);
				}
				featurePoints_Current.at(i) = featurePoints_Filtered_Sum.at(i) / queueSize;
			}

			featurePoints_Filtered_Sum.clear();
		}

		featurePoints_Previous = featurePoints_Current;
	}

	if (featurePoints_Current.size() == 7)
	{
		cv::line(imgsrc, cv::Point(featurePoints_Current.at(0).x, featurePoints_Current.at(0).y),
		         cv::Point(featurePoints_Current.at(1).x, featurePoints_Current.at(1).y),
		         cv::Scalar(0, 0, 255), 3, cv::LINE_AA, 0);// BGR
		cv::line(imgsrc, cv::Point(featurePoints_Current.at(1).x, featurePoints_Current.at(1).y),
		         cv::Point(featurePoints_Current.at(2).x, featurePoints_Current.at(2).y),
		         cv::Scalar(0, 255, 0), 3, cv::LINE_AA, 0);
		cv::line(imgsrc, cv::Point(featurePoints_Current.at(2).x, featurePoints_Current.at(2).y),
		         cv::Point(featurePoints_Current.at(3).x, featurePoints_Current.at(3).y),
		         cv::Scalar(255, 0, 0), 3, cv::LINE_AA, 0);
		cv::line(imgsrc, cv::Point(featurePoints_Current.at(3).x, featurePoints_Current.at(3).y),
		         cv::Point(featurePoints_Current.at(0).x, featurePoints_Current.at(0).y),
		         cv::Scalar(0, 0, 0), 3, cv::LINE_AA, 0);

		// �������α���
		cv::line(imgsrc, cv::Point(featurePoints_Current.at(4).x, featurePoints_Current.at(4).y),
		         cv::Point(featurePoints_Current.at(5).x, featurePoints_Current.at(5).y),
		         cv::Scalar(0, 0, 255), 3, cv::LINE_AA, 0);// BGR
		cv::line(imgsrc, cv::Point(featurePoints_Current.at(5).x, featurePoints_Current.at(5).y),
		         cv::Point(featurePoints_Current.at(6).x, featurePoints_Current.at(6).y),
		         cv::Scalar(0, 255, 0), 3, cv::LINE_AA, 0);
		cv::line(imgsrc, cv::Point(featurePoints_Current.at(6).x, featurePoints_Current.at(6).y),
		         cv::Point(featurePoints_Current.at(4).x, featurePoints_Current.at(4).y),
		         cv::Scalar(255, 0, 0), 3, cv::LINE_AA, 0);
	}

	printf("����-------The tracking numbers of squares and triangles are respectively: "
	       "(�ı���)%d{%d}����(������)%d{%d}��\r\n",
	       squares_num_ROI, (int)Squares_ROI.size(), triangles_num_ROI, (int)Triangles_ROI.size());
	// cv::namedWindow("ContoursImage_Tracking", 0);
	// cv::resizeWindow("ContoursImage_Tracking", 640, 480);
	// cv::imshow("ContoursImage_Tracking", imgsrc);
	// cv::waitKey(1);
}

void Open_Socket_UDP()
{
	WORD sockVersion = MAKEWORD(2, 2);
	WSADATA wsadata_servoing;

	int nRet = SOCKET_ERROR;
	SOCKET hServer_servoing;

	while (nRet == SOCKET_ERROR)
	{
		if (WSAStartup(sockVersion, &wsadata_servoing))
		{
			printf("WSAStartup failed\r\n");
			// return 0;
		}

		hServer_servoing = socket(AF_INET, SOCK_DGRAM, 0);

		bool bReuseaddr = TRUE;
		setsockopt(hServer_servoing, SOL_SOCKET, SO_REUSEADDR, (const char*)&bReuseaddr,
		           sizeof(BOOL));

		const char* addr_ip = "127.0.0.1";

		if (hServer_servoing == INVALID_SOCKET)
		{
			printf("socket failed \r\n");
			// return 0;
		}

		sockaddr_in addrServer_servoing;
		addrServer_servoing.sin_family = AF_INET;
		addrServer_servoing.sin_port   = htons(8899);
		// addrServer_servoing.sin_addr.S_un.S_addr = INADDR_ANY;

		inet_pton(AF_INET, addr_ip, &addrServer_servoing.sin_addr);

		nRet =
		    ::bind(hServer_servoing, (sockaddr*)&addrServer_servoing, sizeof(addrServer_servoing));

		if (nRet == SOCKET_ERROR)
		{
			printf("socket bind failed %d \r\n", WSAGetLastError());

			closesocket(hServer_servoing);
			WSACleanup();
			// return 0;
		}
	}
	sockaddr_in addrClient_servoing;
	int nlen = sizeof(addrClient_servoing);

	bool alreadyConnect = TRUE;
	// std::string str;

	char buffer_servoing[1024];

	memset(buffer_servoing, 0, sizeof(buffer_servoing));

	int irecv;
	int isend;

	// ostringstream oos_sending;

	while (TRUE)
	{
		while (!alreadyConnect)
		{
			irecv = recvfrom(hServer_servoing, buffer_servoing, sizeof(buffer_servoing), 0,
			                 (SOCKADDR*)&addrClient_servoing, &nlen);
			if (irecv > 0)
			{
				if (!(strcmp(buffer_servoing, "byebye")))
				{
					std::cout << "ClientA:" << buffer_servoing << std::endl;
					std::cout << "close connection..." << std::endl;
					closesocket(hServer_servoing);
					WSACleanup();
					std::cout << "5s��رտ���̨" << std::endl;
					Sleep(5000);
					// return 0;
				}
				else
				{
					std::cout << "ClientA:" << buffer_servoing << std::endl;
				}

				::MessageBox(NULL, TEXT("�Ѿ��յ���Ϣ������"), TEXT("ͨ�ſ�"), MB_OKCANCEL);
				// alreadyConnect = TRUE;
			}
			else
			{
				::MessageBox(NULL, TEXT("û���յ��κ���Ϣ������"), TEXT("ͨ�ſ�"), MB_OKCANCEL);
				// std::cout << "û���յ��κ���Ϣ������" << std::endl;
				// closesocket(hServer_servoing);
				// WSACleanup();
				// std::cout << "5s��رտ���̨" << std::endl;
				// Sleep(5000);
				// return 0;
			}
		}
		memset(buffer_servoing, 0, sizeof(buffer_servoing));

		while (alreadyConnect)
		{
			if (firstConnect)
			{
				while (irecv = recvfrom(hServer_servoing, buffer_servoing, sizeof(buffer_servoing),
				                        0, (SOCKADDR*)&addrClient_servoing, &nlen)
				             < 0)
					;
				std::cout << "ClientA:" << buffer_servoing << std::endl;
				firstConnect = FALSE;
			}
			// std::cout << "Server:" << std::endl;
			// std::cin >> buffer_servoing;
			// buffer_servoing = ;
			// getline(cin,str);
			// const int len = sizeof(str);
			// char senddata[len];
			// strcpy_s(senddata,str.c_str());
			if (canSend)
			{
				if (bsendingFeature)
				{
					Package_Feature_Send Package_featurePoints;
					memset(&Package_featurePoints, 0, sizeof(Package_Feature_Send));
					if (featurePoints_Sending.size() == 7)
					{
						for (int i = 0; i < 7; i++)
							Package_featurePoints.featurePoints_Snd[i] =
							    featurePoints_Sending.at(i);
						isend = sendto(hServer_servoing, (char*)&Package_featurePoints,
						               sizeof(Package_featurePoints), 0,
						               (SOCKADDR*)&addrClient_servoing, nlen);
						if (isend == SOCKET_ERROR)
						{
							std::cout << "sendto failed" << std::endl;
							closesocket(hServer_servoing);
							WSACleanup();
							std::cout << "5s��رտ���̨" << std::endl;
							Sleep(5000);
							// return 0;
						}
						canSend = FALSE;
					}
				}
				else
				{
					Package_Command_Send Package_commands;
					memset(&Package_commands, 0, sizeof(Package_Command_Send));
					for (int i = 0; i < 3; i++)
						Package_commands.translVel[i] = commands_Sending.translVel[i];
					for (int i = 0; i < 3; i++)
						Package_commands.rotVel[i] = commands_Sending.rotVel[i];

					isend =
					    sendto(hServer_servoing, (char*)&Package_commands, sizeof(Package_commands),
					           0, (SOCKADDR*)&addrClient_servoing, nlen);
					if (isend == SOCKET_ERROR)
					{
						std::cout << "sendto failed" << std::endl;
						closesocket(hServer_servoing);
						WSACleanup();
						std::cout << "5s��رտ���̨" << std::endl;
						Sleep(5000);
						// return 0;
					}
					canSend = FALSE;
				}
			}
		}
		// str = "";
		memset(buffer_servoing, 0, sizeof(buffer_servoing));
	}

	closesocket(hServer_servoing);
	WSACleanup();
	// return 0;
}

void Open_Socket_TCP()
{
	WSADATA wsaData;
	SOCKET sockServer;
	SOCKADDR_IN addrServer;
	SOCKADDR_IN addrClient;

	int len = sizeof(SOCKADDR);
	int err;
	WORD wVersionRequested;

	wVersionRequested = MAKEWORD(2, 2);
	err               = WSAStartup(wVersionRequested, &wsaData);

	if (err != 0)
	{
		cout << "����WinSock DLLʧ��" << endl;
	}

	if (LOBYTE(wsaData.wVersion) != 2 || HIBYTE(wsaData.wVersion) != 2)
	{
		WSACleanup();
		cout << "����WinSock DLLʧ��" << endl;
	}

	sockServer = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

	addrServer.sin_family = AF_INET;
	addrServer.sin_port   = htons(8899);
	// addrServer.sin_addr.S_un.S_addr = htonl(INADDR_ANY);

	const char* addr_ip = "127.0.0.1";

	inet_pton(AF_INET, addr_ip, &addrServer.sin_addr);

	bool bReuseaddr = TRUE;
	setsockopt(sockServer, SOL_SOCKET, SO_REUSEADDR, (const char*)&bReuseaddr, sizeof(BOOL));

	::bind(sockServer, (SOCKADDR*)&addrServer, sizeof(SOCKADDR));

	listen(sockServer, 5);

	SOCKET sockConnect = accept(sockServer, (SOCKADDR*)&addrClient, &len);

	successConnect = TRUE;

	cout << "ͨ���ѽ���" << endl;
	featurePoints_Sending.clear();
	// send(sockConnect,"�����Ѿ�����ͨ��",20,0);

	while (TRUE)
	{
		// char sendBuf[256];
		// char recvBuf[256];

		// recv(sockConnect,recvBuf,strlen(recvBuf),0);
		// Sleep(1000);
		// if (strlen(recvBuf) > 0)
		//{
		//	cout << recvBuf << endl;
		//	cout << "������Ҫ���͵���Ϣ��";
		//	cin >> sendBuf;

		//	send(sockConnect,sendBuf,strlen(sendBuf)+1,0);
		//}
		if (canSend)
		{
			if (bsendingFeature)
			{
				Package_Feature_Send Package_featurePoints;
				memset(&Package_featurePoints, 0, sizeof(Package_Feature_Send));
				if (featurePoints_Sending.size() == 7)
				{
					for (int i = 0; i < 7; i++)
						Package_featurePoints.featurePoints_Snd[i] = featurePoints_Sending.at(i);
					send(sockConnect, (char*)&Package_featurePoints, sizeof(Package_featurePoints),
					     0);
					canSend = FALSE;
				}
			}
			else
			{
				Package_Command_Send Package_commands;
				memset(&Package_commands, 0, sizeof(Package_Command_Send));
				for (int i = 0; i < 3; i++)
					Package_commands.translVel[i] = commands_Sending.translVel[i];
				for (int i = 0; i < 3; i++)
					Package_commands.rotVel[i] = commands_Sending.rotVel[i];
				send(sockConnect, (char*)&Package_commands, sizeof(Package_commands), 0);
				canSend = FALSE;
			}
		}
	}
	closesocket(sockConnect);
	closesocket(sockServer);
	WSACleanup();
}

cv::Mat R_t2HomogeneousT(const cv::Mat tvec, const cv::Mat R)
{
	cv::Mat HomogenousT;
	cv::Mat_<double> R1 =
	    (cv::Mat_<double>(4, 3) << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
	     R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), R.at<double>(2, 0),
	     R.at<double>(2, 1), R.at<double>(2, 2), 0, 0, 0);
	cv::Mat_<double> t1 = (cv::Mat_<double>(4, 1) << tvec.at<double>(0, 0), tvec.at<double>(1, 0),
	                       tvec.at<double>(2, 0), 1);
	cv::hconcat(R1, t1, HomogenousT);
	return HomogenousT;
}

void HomogeneousT2Rt(cv::Mat HomogenousT, cv::Mat& R, cv::Mat& tvec)
{
	cv::Rect R_rect(0, 0, 3, 3);
	cv::Rect t_rect(3, 0, 1, 3);
	R    = HomogenousT(R_rect);
	tvec = HomogenousT(t_rect);
}

cv::Mat Vec2SkewMat(cv::Mat vec)
{
	cv::Mat_<double> skewMat =
	    (cv::Mat_<double>(3, 3) << 0, -vec.at<double>(2, 0), vec.at<double>(1, 0),
	     vec.at<double>(2, 0), 0, -vec.at<double>(0, 0), -vec.at<double>(1, 0),
	     vec.at<double>(0, 0), 0);

	return skewMat;
}

Robot_Command ControlVel_Pose_inCameraFrame(cv::Mat desiredC_tvec_O, cv::Mat desiredC_rvec_O,
                                            cv::Mat currentC_tvec_O, cv::Mat currentC_rvec_O,
                                            bool bRelative2ObjectFrame)
{
	cv::Mat desiredC_R_O(3, 3, CV_64FC1);
	cv::Mat currentC_R_O(3, 3, CV_64FC1);

	cv::Mat desiredC_HomogenousT_O(4, 4, CV_64FC1);
	cv::Mat currentC_HomogenousT_O(4, 4, CV_64FC1);

	cv::Rodrigues(desiredC_rvec_O, desiredC_R_O);
	cv::Rodrigues(currentC_rvec_O, currentC_R_O);

	desiredC_HomogenousT_O = R_t2HomogeneousT(desiredC_tvec_O, desiredC_R_O);
	currentC_HomogenousT_O = R_t2HomogeneousT(currentC_tvec_O, currentC_R_O);

	cv::Mat desiredC_HomogenousT_currentC(4, 4, CV_64FC1);

	cv::Mat O_HomogenousT_currentC(4, 4, CV_64FC1);

	cv::invert(currentC_HomogenousT_O, O_HomogenousT_currentC, cv::DECOMP_LU);

	desiredC_HomogenousT_currentC = desiredC_HomogenousT_O * O_HomogenousT_currentC;

	Robot_Command vRobot_Command = {0};

	cv::Mat desiredC_R_currentC(3, 3, CV_64FC1);
	cv::Mat desiredC_tvec_currentC(3, 1, CV_64FC1);

	HomogeneousT2Rt(desiredC_HomogenousT_currentC, desiredC_R_currentC, desiredC_tvec_currentC);

	cv::Mat desiredC_angleAxis_currentC(3, 1, CV_64FC1);
	cv::Rodrigues(desiredC_R_currentC, desiredC_angleAxis_currentC);

	cv::Mat currentC_transVel_Camera(3, 1, CV_64FC1);
	cv::Mat currentC_rotVel_Camera(3, 1, CV_64FC1);

	double controlGain = 10;

	if (bRelative2ObjectFrame)
	{
		currentC_transVel_Camera = -controlGain
		                         * ((desiredC_tvec_O - currentC_tvec_O)
		                            + Vec2SkewMat(currentC_tvec_O) * desiredC_angleAxis_currentC);
		currentC_rotVel_Camera = -controlGain * desiredC_angleAxis_currentC;
	}
	else
	{
		currentC_transVel_Camera = -controlGain * desiredC_R_currentC.t() * desiredC_tvec_currentC;
		currentC_rotVel_Camera   = -controlGain * desiredC_angleAxis_currentC;
	}

	cv::Mat currentE_R_currentC(3, 3, CV_64FC1);
	cv::Mat currentE_tvec_currentC(3, 1, CV_64FC1);

	HomogeneousT2Rt(currentE_HomogenousT_currentC, currentE_R_currentC, currentE_tvec_currentC);

	cv::Mat currentE_transVel_Effector(3, 1, CV_64FC1);
	cv::Mat currentE_rotVel_Effector(3, 1, CV_64FC1);

	currentE_transVel_Effector =
	    currentE_R_currentC * currentC_transVel_Camera
	    + Vec2SkewMat(currentE_tvec_currentC) * currentE_R_currentC * currentC_rotVel_Camera;
	currentE_rotVel_Effector = currentE_R_currentC * currentC_rotVel_Camera;

	double max_Trans = 0;
	double max_Rot   = 0;

	double ratio_Trans;
	double ratio_Rot;
	double ratio_Vel;

	for (int i = 0; i < 3; i++)
	{
		vRobot_Command.translVel[i] = currentE_transVel_Effector.at<double>(i, 0);
		if (fabs(currentE_transVel_Effector.at<double>(i, 0)) > max_Trans)
		{
			max_Trans = fabs(currentE_transVel_Effector.at<double>(i, 0));
		}
	}

	if (max_Trans < max_translVel)
	{
		ratio_Trans = 1;
	}
	else
	{
		ratio_Trans = max_translVel / max_Trans;
	}

	for (int i = 0; i < 3; i++)
	{
		vRobot_Command.rotVel[i] = currentE_rotVel_Effector.at<double>(i, 0);
		if (fabs(currentE_rotVel_Effector.at<double>(i, 0)) > max_Rot)
		{
			max_Rot = fabs(currentE_rotVel_Effector.at<double>(i, 0));
		}
	}

	if (max_Rot < max_rotVel)
	{
		ratio_Rot = 1;
	}
	else
	{
		ratio_Rot = max_rotVel / max_Rot;
	}

	if (ratio_Rot < ratio_Trans)
	{
		ratio_Vel = ratio_Rot;
	}
	else
	{
		ratio_Vel = ratio_Trans;
	}

	for (int i = 0; i < 3; i++)
	{
		vRobot_Command.translVel[i] = ratio_Vel * vRobot_Command.translVel[i];
	}

	for (int i = 0; i < 3; i++)
	{
		vRobot_Command.rotVel[i] = ratio_Vel * vRobot_Command.rotVel[i];
	}

	return vRobot_Command;
}

Robot_Command ControlVel_Pose_inEffectorFrame(cv::Mat desiredC_tvec_O, cv::Mat desiredC_rvec_O,
                                              cv::Mat currentC_tvec_O, cv::Mat currentC_rvec_O,
                                              bool bRelative2ObjectFrame)
{
	cv::Mat desiredC_R_O(3, 3, CV_64FC1);
	cv::Mat currentC_R_O(3, 3, CV_64FC1);

	cv::Mat desiredC_HomogenousT_O(4, 4, CV_64FC1);
	cv::Mat currentC_HomogenousT_O(4, 4, CV_64FC1);

	cv::Rodrigues(desiredC_rvec_O, desiredC_R_O);
	cv::Rodrigues(currentC_rvec_O, currentC_R_O);

	desiredC_HomogenousT_O = R_t2HomogeneousT(desiredC_tvec_O, desiredC_R_O);
	currentC_HomogenousT_O = R_t2HomogeneousT(currentC_tvec_O, currentC_R_O);

	/*Effector frame related information*/
	cv::Mat desiredE_HomogenousT_desiredC = currentE_HomogenousT_currentC;

	cv::Mat desiredE_HomogenousT_O(4, 4, CV_64FC1);
	cv::Mat currentE_HomogenousT_O(4, 4, CV_64FC1);

	desiredE_HomogenousT_O = desiredE_HomogenousT_desiredC * desiredC_HomogenousT_O;
	currentE_HomogenousT_O = currentE_HomogenousT_currentC * currentC_HomogenousT_O;

	cv::Mat desiredE_HomogenousT_currentE(4, 4, CV_64FC1);

	cv::Mat O_HomogenousT_currentE(4, 4, CV_64FC1);

	cv::invert(currentE_HomogenousT_O, O_HomogenousT_currentE, cv::DECOMP_LU);

	desiredE_HomogenousT_currentE = desiredE_HomogenousT_O * O_HomogenousT_currentE;

	Robot_Command vRobot_Command = {0};

	cv::Mat desiredE_R_currentE(3, 3, CV_64FC1);
	cv::Mat desiredE_tvec_currentE(3, 1, CV_64FC1);

	cv::Mat desiredE_R_O(3, 3, CV_64FC1);
	cv::Mat desiredE_tvec_O(3, 1, CV_64FC1);

	cv::Mat currentE_R_O(3, 3, CV_64FC1);
	cv::Mat currentE_tvec_O(3, 1, CV_64FC1);

	HomogeneousT2Rt(desiredE_HomogenousT_O, desiredE_R_O, desiredE_tvec_O);
	HomogeneousT2Rt(currentE_HomogenousT_O, currentE_R_O, currentE_tvec_O);

	HomogeneousT2Rt(desiredE_HomogenousT_currentE, desiredE_R_currentE, desiredE_tvec_currentE);

	cv::Mat desiredE_angleAxis_currentE(3, 1, CV_64FC1);
	cv::Rodrigues(desiredE_R_currentE, desiredE_angleAxis_currentE);

	cv::Mat currentE_transVel_Effector(3, 1, CV_64FC1);
	cv::Mat currentE_rotVel_Effector(3, 1, CV_64FC1);

	double controlGain = 0.1;

	if (bRelative2ObjectFrame)
	{
		currentE_transVel_Effector = -controlGain
		                           * ((desiredE_tvec_O - currentE_tvec_O)
		                              + Vec2SkewMat(currentE_tvec_O) * desiredE_angleAxis_currentE);
		currentE_rotVel_Effector = -controlGain * desiredE_angleAxis_currentE;
	}
	else
	{
		currentE_transVel_Effector =
		    -controlGain * desiredE_R_currentE.t() * desiredE_tvec_currentE;
		currentE_rotVel_Effector = -controlGain * desiredE_angleAxis_currentE;
	}

	// ����
	// printf("���������ٶȣ�%lf,%lf,%lf��\r\n", currentE_transVel_Effector.at<double>(0, 0),
	// currentE_transVel_Effector.at<double>(1, 0), currentE_transVel_Effector.at<double>(2, 0));

	double max_Trans = 0;
	double max_Rot   = 0;

	double ratio_Trans;
	double ratio_Rot;
	double ratio_Vel;

	for (int i = 0; i < 3; i++)
	{
		vRobot_Command.translVel[i] = currentE_transVel_Effector.at<double>(i, 0);
		if (fabs(currentE_transVel_Effector.at<double>(i, 0)) > max_Trans)
		{
			max_Trans = fabs(currentE_transVel_Effector.at<double>(i, 0));
		}
	}

	// printf("���������ٶ�--���ź�%lf,%lf,%lf��\r\n", vRobot_Command.translVel[0],
	// vRobot_Command.translVel[1], vRobot_Command.translVel[2]);

	if (max_Trans < max_translVel)
	{
		ratio_Trans = 1;
	}
	else
	{
		ratio_Trans = max_translVel / max_Trans;
	}

	for (int i = 0; i < 3; i++)
	{
		vRobot_Command.rotVel[i] = currentE_rotVel_Effector.at<double>(i, 0);
		if (fabs(currentE_rotVel_Effector.at<double>(i, 0)) > max_Rot)
		{
			max_Rot = fabs(currentE_rotVel_Effector.at<double>(i, 0));
		}
	}

	// printf("��󡰼��㡱���ٶȣ�%lf��\r\n", max_Rot);

	if (max_Rot < max_rotVel)
	{
		ratio_Rot = 1;
	}
	else
	{
		ratio_Rot = max_rotVel / max_Rot;
	}

	// printf("���ٶ��������ӣ�%lf��\r\n", ratio_Rot);

	if (ratio_Rot < ratio_Trans)
	{
		ratio_Vel = ratio_Rot;
	}
	else
	{
		ratio_Vel = ratio_Trans;
	}

	// printf("�ٶ��������ӣ�%lf,%lf,%lf��\r\n", ratio_Vel,ratio_Trans,ratio_Rot);

	for (int i = 0; i < 3; i++)
	{
		vRobot_Command.translVel[i] = ratio_Vel * vRobot_Command.translVel[i];
	}

	for (int i = 0; i < 3; i++)
	{
		vRobot_Command.rotVel[i] = ratio_Vel * vRobot_Command.rotVel[i];
	}

	return vRobot_Command;
}

Robot_Command
ControlVel_Pose_inEffectorFrame_useViaPoint(cv::Mat desiredC_tvec_O, cv::Mat desiredC_rvec_O,
                                            cv::Mat currentC_tvec_O, cv::Mat currentC_rvec_O,
                                            bool bRelative2ObjectFrame, bool bConstantViaPoint,
                                            unsigned long& vtimeStep_Pose)
{
	vtimeStep_Pose++;

	cv::Mat desiredC_R_O(3, 3, CV_64FC1);
	cv::Mat currentC_R_O(3, 3, CV_64FC1);

	cv::Mat desiredC_HomogenousT_O(4, 4, CV_64FC1);
	cv::Mat currentC_HomogenousT_O(4, 4, CV_64FC1);

	cv::Rodrigues(desiredC_rvec_O, desiredC_R_O);
	cv::Rodrigues(currentC_rvec_O, currentC_R_O);

	desiredC_HomogenousT_O = R_t2HomogeneousT(desiredC_tvec_O, desiredC_R_O);
	currentC_HomogenousT_O = R_t2HomogeneousT(currentC_tvec_O, currentC_R_O);

	// ��ʼʱ��λ��
	if (vtimeStep_Pose == 1)
	{
		initialC_HomogenousT_O = currentC_HomogenousT_O;
	}

	printf("��ǰ������%u\r\n\n", vtimeStep_Pose);

	/*Effector frame related information*/
	cv::Mat desiredE_HomogenousT_desiredC = currentE_HomogenousT_currentC;

	// ��ʼλ�����
	cv::Mat initialE_HomogenousT_initialC = currentE_HomogenousT_currentC;

	cv::Mat desiredE_HomogenousT_O(4, 4, CV_64FC1);
	cv::Mat currentE_HomogenousT_O(4, 4, CV_64FC1);

	// ��ʼλ�����
	cv::Mat initialE_HomogenousT_O(4, 4, CV_64FC1);

	desiredE_HomogenousT_O = desiredE_HomogenousT_desiredC * desiredC_HomogenousT_O;
	currentE_HomogenousT_O = currentE_HomogenousT_currentC * currentC_HomogenousT_O;

	// ��ʼλ�����
	initialE_HomogenousT_O = initialE_HomogenousT_initialC * initialC_HomogenousT_O;

	cv::Mat desiredE_HomogenousT_currentE(4, 4, CV_64FC1);

	// ��ʼλ�����
	cv::Mat desiredE_HomogenousT_initialE(4, 4, CV_64FC1);

	cv::Mat O_HomogenousT_currentE(4, 4, CV_64FC1);

	// ��ʼλ�����
	cv::Mat O_HomogenousT_initialE(4, 4, CV_64FC1);

	cv::invert(currentE_HomogenousT_O, O_HomogenousT_currentE, cv::DECOMP_LU);

	// ��ʼλ�����
	cv::invert(initialE_HomogenousT_O, O_HomogenousT_initialE, cv::DECOMP_LU);

	desiredE_HomogenousT_currentE = desiredE_HomogenousT_O * O_HomogenousT_currentE;

	// ��ʼλ�����
	desiredE_HomogenousT_initialE = desiredE_HomogenousT_O * O_HomogenousT_initialE;

	Robot_Command vRobot_Command = {0};

	cv::Mat desiredE_R_currentE(3, 3, CV_64FC1);
	cv::Mat desiredE_tvec_currentE(3, 1, CV_64FC1);
	HomogeneousT2Rt(desiredE_HomogenousT_currentE, desiredE_R_currentE, desiredE_tvec_currentE);

	// ��ʼλ�����
	cv::Mat desiredE_R_initialE(3, 3, CV_64FC1);
	cv::Mat desiredE_tvec_initialE(3, 1, CV_64FC1);
	HomogeneousT2Rt(desiredE_HomogenousT_initialE, desiredE_R_initialE, desiredE_tvec_initialE);

	// viaPoint���**********************
	cv::Mat desiredE_HomogenousT_viaE(4, 4, CV_64FC1);
	cv::Mat desiredE_R_viaE(3, 3, CV_64FC1);
	cv::Mat desiredE_tvec_viaE(3, 1, CV_64FC1);

	if (bConstantViaPoint)
	{
		desiredE_R_viaE                     = cv::Mat::eye(3, 3, CV_64FC1);
		desiredE_tvec_viaE.at<double>(0, 0) = 0;
		desiredE_tvec_viaE.at<double>(1, 0) = 0;
		desiredE_tvec_viaE.at<double>(2, 0) = desiredE_tvec_initialE.at<double>(2, 0);
	}
	else
	{
		desiredE_R_viaE                     = cv::Mat::eye(3, 3, CV_64FC1);
		desiredE_tvec_viaE.at<double>(0, 0) = 0;
		desiredE_tvec_viaE.at<double>(1, 0) = 0;
		desiredE_tvec_viaE.at<double>(2, 0) = desiredE_tvec_currentE.at<double>(2, 0);
	}

	desiredE_HomogenousT_viaE = R_t2HomogeneousT(desiredE_tvec_viaE, desiredE_R_viaE);
	// viaPoint���**********************

	cv::Mat desiredE_R_O(3, 3, CV_64FC1);
	cv::Mat desiredE_tvec_O(3, 1, CV_64FC1);
	HomogeneousT2Rt(desiredE_HomogenousT_O, desiredE_R_O, desiredE_tvec_O);

	// viaPoint���********************
	cv::Mat viaE_HomogenousT_O(4, 4, CV_64FC1);
	cv::Mat viaE_R_O(3, 3, CV_64FC1);
	cv::Mat viaE_tvec_O(3, 1, CV_64FC1);
	cv::Mat viaE_HomogenousT_deisredE(4, 4, CV_64FC1);
	cv::invert(desiredE_HomogenousT_viaE, viaE_HomogenousT_deisredE, cv::DECOMP_LU);
	viaE_HomogenousT_O = viaE_HomogenousT_deisredE * desiredE_HomogenousT_O;
	HomogeneousT2Rt(viaE_HomogenousT_O, viaE_R_O, viaE_tvec_O);
	// viaPoint���********************

	cv::Mat currentE_R_O(3, 3, CV_64FC1);
	cv::Mat currentE_tvec_O(3, 1, CV_64FC1);
	HomogeneousT2Rt(currentE_HomogenousT_O, currentE_R_O, currentE_tvec_O);

	cv::Mat desiredE_angleAxis_currentE(3, 1, CV_64FC1);
	cv::Rodrigues(desiredE_R_currentE, desiredE_angleAxis_currentE);

	// viaPoint���********************
	cv::Mat viaE_HomogenousT_currentE(4, 4, CV_64FC1);
	cv::Mat viaE_R_currentE(3, 3, CV_64FC1);
	cv::Mat viaE_tvec_currentE(3, 1, CV_64FC1);
	viaE_HomogenousT_currentE = viaE_HomogenousT_O * O_HomogenousT_currentE;
	HomogeneousT2Rt(viaE_HomogenousT_currentE, viaE_R_currentE, viaE_tvec_currentE);
	cv::Mat viaE_angleAxis_currentE(3, 1, CV_64FC1);
	cv::Rodrigues(viaE_R_currentE, viaE_angleAxis_currentE);
	// viaPoint���********************

	cv::Mat currentE_transVel_Effector(3, 1, CV_64FC1);
	cv::Mat currentE_rotVel_Effector(3, 1, CV_64FC1);

	double controlGain   = 0.1;
	double stageAccuracy = 3;

	if (bFirstStage_Pose)
	{
		if (bRelative2ObjectFrame)
		{
			currentE_transVel_Effector = -controlGain
			                           * ((viaE_tvec_O - currentE_tvec_O)
			                              + Vec2SkewMat(currentE_tvec_O) * viaE_angleAxis_currentE);
			currentE_rotVel_Effector = -controlGain * viaE_angleAxis_currentE;
		}
		else
		{
			currentE_transVel_Effector = -controlGain * viaE_R_currentE.t() * viaE_tvec_currentE;
			currentE_rotVel_Effector   = -controlGain * viaE_angleAxis_currentE;
		}

		if (norm(viaE_tvec_currentE) < stageAccuracy)
		{
			bFirstStage_Pose = FALSE;
		}
		printf("��һ�׶�\r\n\n");
	}
	else
	{
		if (bRelative2ObjectFrame)
		{
			currentE_transVel_Effector =
			    -controlGain
			    * ((desiredE_tvec_O - currentE_tvec_O)
			       + Vec2SkewMat(currentE_tvec_O) * desiredE_angleAxis_currentE);
			currentE_rotVel_Effector = -controlGain * desiredE_angleAxis_currentE;
		}
		else
		{
			currentE_transVel_Effector =
			    -controlGain * desiredE_R_currentE.t() * desiredE_tvec_currentE;
			currentE_rotVel_Effector = -controlGain * desiredE_angleAxis_currentE;
		}

		if (norm(viaE_tvec_currentE) > stageAccuracy)
		{
			bFirstStage_Pose = TRUE;
			printf("��һ�׶�\r\n\n");
		}
		printf("�ڶ��׶�\r\n\n");
	}

	double max_Trans = 0;
	double max_Rot   = 0;

	double ratio_Trans;
	double ratio_Rot;
	double ratio_Vel;

	for (int i = 0; i < 3; i++)
	{
		vRobot_Command.translVel[i] = currentE_transVel_Effector.at<double>(i, 0);
		if (fabs(currentE_transVel_Effector.at<double>(i, 0)) > max_Trans)
		{
			max_Trans = fabs(currentE_transVel_Effector.at<double>(i, 0));
		}
	}

	if (max_Trans < max_translVel)
	{
		ratio_Trans = 1;
	}
	else
	{
		ratio_Trans = max_translVel / max_Trans;
	}

	for (int i = 0; i < 3; i++)
	{
		vRobot_Command.rotVel[i] = currentE_rotVel_Effector.at<double>(i, 0);
		if (fabs(currentE_rotVel_Effector.at<double>(i, 0)) > max_Rot)
		{
			max_Rot = fabs(currentE_rotVel_Effector.at<double>(i, 0));
		}
	}

	if (max_Rot < max_rotVel)
	{
		ratio_Rot = 1;
	}
	else
	{
		ratio_Rot = max_rotVel / max_Rot;
	}

	if (ratio_Rot < ratio_Trans)
	{
		ratio_Vel = ratio_Rot;
	}
	else
	{
		ratio_Vel = ratio_Trans;
	}

	for (int i = 0; i < 3; i++)
	{
		vRobot_Command.translVel[i] = ratio_Vel * vRobot_Command.translVel[i];
	}

	for (int i = 0; i < 3; i++)
	{
		vRobot_Command.rotVel[i] = ratio_Vel * vRobot_Command.rotVel[i];
	}

	return vRobot_Command;
}

Robot_Command ControlVel_Image(cv::Point featurePoints_Desired[7],
                               cv::Point featurePoints_Current[7], cv::Mat currentC_tvec_O,
                               cv::Mat currentC_rvec_O, int num_FeaturePointsForControl)
{
	cv::Point3d featurePoints_Desired_Normalized[7];
	cv::Point3d featurePoints_Current_Normalized[7];

	cv::Point3d featurePoints_Desired_Extended[7];
	cv::Point3d featurePoints_Current_Extended[7];

	for (int i = 0; i < 7; i++)
	{
		featurePoints_Desired_Extended[i].x = (double)featurePoints_Desired[i].x;
		featurePoints_Desired_Extended[i].y = (double)featurePoints_Desired[i].y;
		featurePoints_Desired_Extended[i].z = (double)1;

		featurePoints_Current_Extended[i].x = (double)featurePoints_Current[i].x;
		featurePoints_Current_Extended[i].y = (double)featurePoints_Current[i].y;
		featurePoints_Current_Extended[i].z = (double)1;
	}

	cv::Matx33d camera_matrix(1544.970675627578, 0, 1312.414149621184, 0, 1544.76986234472,
	                          965.0566324183379, 0, 0, 1);

	for (int i = 0; i < 7; i++)
	{
		featurePoints_Desired_Normalized[i] =
		    camera_matrix.inv() * featurePoints_Desired_Extended[i];

		featurePoints_Current_Normalized[i] =
		    camera_matrix.inv() * featurePoints_Current_Extended[i];
	}

	cv::Point3d O_Points3D[7];
	O_Points3D[0] = cv::Point3d(16, 16, 0) / 1000.00;
	O_Points3D[1] = cv::Point3d(16, 0, 0) / 1000.00;
	O_Points3D[2] = cv::Point3d(0, 0, 0) / 1000.00;
	O_Points3D[3] = cv::Point3d(0, 16, 0) / 1000.00;

	O_Points3D[4] = cv::Point3d(13, 20, 0) / 1000.00;
	O_Points3D[5] = cv::Point3d(4, 36, 0) / 1000.00;
	O_Points3D[6] = cv::Point3d(22.4, 36, 0) / 1000.00;

	cv::Matx41d O_Points3D_Extended[7];
	cv::Matx41d currentC_Points3D_Extended[7];

	cv::Mat currentC_R_O(3, 3, CV_64FC1);
	cv::Mat currentC_HomogenousT_O_temp(4, 4, CV_64FC1);
	cv::Rodrigues(currentC_rvec_O, currentC_R_O);
	currentC_HomogenousT_O_temp = R_t2HomogeneousT(currentC_tvec_O / 1000.00, currentC_R_O);

	cv::Matx44d currentC_HomogenousT_O;

	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			currentC_HomogenousT_O(i, j) = currentC_HomogenousT_O_temp.at<double>(i, j);
		}
	}

	for (int i = 0; i < 7; i++)
	{
		O_Points3D_Extended[i] = cv::Matx41d(O_Points3D[i].x, O_Points3D[i].y, O_Points3D[i].z, 1);
	}

	for (int i = 0; i < 7; i++)
	{
		currentC_Points3D_Extended[i] = currentC_HomogenousT_O * O_Points3D_Extended[i];
	}

	double currentC_featurePoints_Z[7];

	for (int i = 0; i < 7; i++)
	{
		currentC_featurePoints_Z[i] = currentC_Points3D_Extended[i](2);
	}

	cv::Mat Jacobian(2 * num_FeaturePointsForControl, 6, CV_64FC1);

	cv::Mat Jacobian_Inv(6, 2 * num_FeaturePointsForControl, CV_64FC1);

	cv::Mat ImageErrs(2 * num_FeaturePointsForControl, 1, CV_64FC1);

	for (int i = 0; i < num_FeaturePointsForControl; i++)
	{
		ImageErrs.at<double>(2 * i, 0) =
		    featurePoints_Current_Normalized[i].x - featurePoints_Desired_Normalized[i].x;
		ImageErrs.at<double>(2 * i + 1, 0) =
		    featurePoints_Current_Normalized[i].y - featurePoints_Desired_Normalized[i].y;
	}

	for (int i = 0; i < num_FeaturePointsForControl; i++)
	{
		Jacobian.at<double>(2 * i, 0) = -1 / currentC_featurePoints_Z[i];
		Jacobian.at<double>(2 * i, 1) = 0;
		Jacobian.at<double>(2 * i, 2) =
		    featurePoints_Current_Normalized[i].x / currentC_featurePoints_Z[i];
		Jacobian.at<double>(2 * i, 3) =
		    featurePoints_Current_Normalized[i].x * featurePoints_Current_Normalized[i].y;
		Jacobian.at<double>(2 * i, 4) =
		    -(1 + featurePoints_Current_Normalized[i].x * featurePoints_Current_Normalized[i].x);
		Jacobian.at<double>(2 * i, 5) = featurePoints_Current_Normalized[i].y;

		Jacobian.at<double>(2 * i + 1, 0) = 0;
		Jacobian.at<double>(2 * i + 1, 1) = -1 / currentC_featurePoints_Z[i];
		Jacobian.at<double>(2 * i + 1, 2) =
		    featurePoints_Current_Normalized[i].y / currentC_featurePoints_Z[i];
		Jacobian.at<double>(2 * i + 1, 3) =
		    1 + featurePoints_Current_Normalized[i].y * featurePoints_Current_Normalized[i].y;
		Jacobian.at<double>(2 * i + 1, 4) =
		    -featurePoints_Current_Normalized[i].x * featurePoints_Current_Normalized[i].y;
		Jacobian.at<double>(2 * i + 1, 5) = -featurePoints_Current_Normalized[i].x;
	}

	cv::Mat currentC_Vel_Camera(6, 1, CV_64FC1);

	cv::Mat currentC_transVel_Camera(3, 1, CV_64FC1);
	cv::Mat currentC_rotVel_Camera(3, 1, CV_64FC1);

	double controlGain = 10.10;
	cv::invert(Jacobian, Jacobian_Inv, cv::DECOMP_LU);

	currentC_Vel_Camera = -controlGain * Jacobian_Inv * ImageErrs;
	// currentC_Vel_Camera = -controlGain * Jacobian.inv() * ImageErrs;
	// currentC_Vel_Camera = ImageErrs;

	// printf("ͼ������%lf,%lf��\r\n\n", ImageErrs.at<double>(0, 0), ImageErrs.at<double>(1,
	// 0)); printf("���������꣺��%lf,%lf,%lf��\r\n\n", O_Points3D[6].x, O_Points3D[6].y,
	// O_Points3D[6].z); printf("�ſɱ�����ʽ����%lf��\r\n\n", cv::determinant(Jacobian));

	// printf("�ſɱȸ�Ԫ��--��һ�У���%lf,%lf,%lf,%lf,%lf,%lf��\r\n\n", Jacobian.at<double>(0, 0),
	// Jacobian.at<double>(0, 1), Jacobian.at<double>(0, 2), Jacobian.at<double>(0, 3),
	// Jacobian.at<double>(0, 4), Jacobian.at<double>(0, 5));
	// printf("�ſɱȸ�Ԫ��--�ڶ��У���%lf,%lf,%lf,%lf,%lf,%lf��\r\n\n", Jacobian.at<double>(1, 0),
	// Jacobian.at<double>(1, 1), Jacobian.at<double>(1, 2), Jacobian.at<double>(1, 3),
	// Jacobian.at<double>(1, 4), Jacobian.at<double>(1, 5));
	// printf("�ſɱȸ�Ԫ��--�����У���%lf,%lf,%lf,%lf,%lf,%lf��\r\n\n", Jacobian.at<double>(2, 0),
	// Jacobian.at<double>(2, 1), Jacobian.at<double>(2, 2), Jacobian.at<double>(2, 3),
	// Jacobian.at<double>(2, 4), Jacobian.at<double>(2, 5));
	// printf("�ſɱȸ�Ԫ��--�����У���%lf,%lf,%lf,%lf,%lf,%lf��\r\n\n", Jacobian.at<double>(3, 0),
	// Jacobian.at<double>(3, 1), Jacobian.at<double>(3, 2), Jacobian.at<double>(3, 3),
	// Jacobian.at<double>(3, 4), Jacobian.at<double>(3, 5));
	// printf("�ſɱȸ�Ԫ��--�����У���%lf,%lf,%lf,%lf,%lf,%lf��\r\n\n", Jacobian.at<double>(4, 0),
	// Jacobian.at<double>(4, 1), Jacobian.at<double>(4, 2), Jacobian.at<double>(4, 3),
	// Jacobian.at<double>(4, 4), Jacobian.at<double>(4, 5));
	// printf("�ſɱȸ�Ԫ��--�����У���%lf,%lf,%lf,%lf,%lf,%lf��\r\n\n", Jacobian.at<double>(5, 0),
	// Jacobian.at<double>(5, 1), Jacobian.at<double>(5, 2), Jacobian.at<double>(5, 3),
	// Jacobian.at<double>(5, 4), Jacobian.at<double>(5, 5));

	// printf("ͼ������(��һ��)--��һ�㣺��%lf,%lf��\r\n\n", featurePoints_Current_Normalized[0].x,
	// featurePoints_Current_Normalized[0].y); printf("ͼ������(��һ��)--�ڶ��㣺��%lf,%lf��\r\n\n",
	// featurePoints_Current_Normalized[1].x, featurePoints_Current_Normalized[1].y);
	// printf("ͼ������(��һ��)--�����㣺��%lf,%lf��\r\n\n", featurePoints_Current_Normalized[2].x,
	// featurePoints_Current_Normalized[2].y); printf("ͼ������(��һ��)--���ĵ㣺��%lf,%lf��\r\n\n",
	// featurePoints_Current_Normalized[3].x, featurePoints_Current_Normalized[3].y);
	// printf("ͼ������(��һ��)--����㣺��%lf,%lf��\r\n\n", featurePoints_Current_Normalized[4].x,
	// featurePoints_Current_Normalized[4].y); printf("ͼ������(��һ��)--�����㣺��%lf,%lf��\r\n\n",
	// featurePoints_Current_Normalized[5].x, featurePoints_Current_Normalized[5].y);
	// printf("ͼ������(��һ��)--���ߵ㣺��%lf,%lf��\r\n\n", featurePoints_Current_Normalized[6].x,
	// featurePoints_Current_Normalized[6].y);

	// printf("ͼ������(δ��һ��)--��һ�㣺��%d,%d��\r\n\n", featurePoints_Current[0].x,
	// featurePoints_Current[0].y); printf("ͼ������(δ��һ��)--�ڶ��㣺��%d,%d��\r\n\n",
	// featurePoints_Current[1].x, featurePoints_Current[1].y);
	// printf("ͼ������(δ��һ��)--�����㣺��%d,%d��\r\n\n", featurePoints_Current[2].x,
	// featurePoints_Current[2].y); printf("ͼ������(δ��һ��)--���ĵ㣺��%d,%d��\r\n\n",
	// featurePoints_Current[3].x, featurePoints_Current[3].y);
	// printf("ͼ������(δ��һ��)--����㣺��%d,%d��\r\n\n", featurePoints_Current[4].x,
	// featurePoints_Current[4].y); printf("ͼ������(δ��һ��)--�����㣺��%d,%d��\r\n\n",
	// featurePoints_Current[5].x, featurePoints_Current[5].y);
	// printf("ͼ������(δ��һ��)--���ߵ㣺��%d,%d��\r\n\n", featurePoints_Current[6].x,
	// featurePoints_Current[6].y);

	// cv::waitKey(9000);

	for (int i = 0; i < 3; i++)
	{
		currentC_transVel_Camera.at<double>(i, 0) = currentC_Vel_Camera.at<double>(i, 0);
	}

	for (int i = 0; i < 3; i++)
	{
		currentC_rotVel_Camera.at<double>(i, 0) = currentC_Vel_Camera.at<double>(i + 3, 0);
	}

	cv::Mat currentE_R_currentC(3, 3, CV_64FC1);
	cv::Mat currentE_tvec_currentC(3, 1, CV_64FC1);

	HomogeneousT2Rt(currentE_HomogenousT_currentC, currentE_R_currentC, currentE_tvec_currentC);

	cv::Mat currentE_transVel_Effector(3, 1, CV_64FC1);
	cv::Mat currentE_rotVel_Effector(3, 1, CV_64FC1);

	currentE_transVel_Effector =
	    currentE_R_currentC * currentC_transVel_Camera
	    + Vec2SkewMat(currentE_tvec_currentC) * currentE_R_currentC * currentC_rotVel_Camera;
	currentE_rotVel_Effector = currentE_R_currentC * currentC_rotVel_Camera;

	double max_Trans = 0;
	double max_Rot   = 0;

	double ratio_Trans;
	double ratio_Rot;
	double ratio_Vel;

	Robot_Command vRobot_Command = {0};

	for (int i = 0; i < 3; i++)
	{
		vRobot_Command.translVel[i] = currentE_transVel_Effector.at<double>(i, 0);
		if (fabs(currentE_transVel_Effector.at<double>(i, 0)) > max_Trans)
		{
			max_Trans = fabs(currentE_transVel_Effector.at<double>(i, 0));
		}
	}

	if (max_Trans < max_translVel)
	{
		ratio_Trans = 1;
	}
	else
	{
		ratio_Trans = max_translVel / max_Trans;
	}

	for (int i = 0; i < 3; i++)
	{
		vRobot_Command.rotVel[i] = currentE_rotVel_Effector.at<double>(i, 0);
		if (fabs(currentE_rotVel_Effector.at<double>(i, 0)) > max_Rot)
		{
			max_Rot = fabs(currentE_rotVel_Effector.at<double>(i, 0));
		}
	}

	if (max_Rot < max_rotVel)
	{
		ratio_Rot = 1;
	}
	else
	{
		ratio_Rot = max_rotVel / max_Rot;
	}

	if (ratio_Rot < ratio_Trans)
	{
		ratio_Vel = ratio_Rot;
	}
	else
	{
		ratio_Vel = ratio_Trans;
	}

	for (int i = 0; i < 3; i++)
	{
		vRobot_Command.translVel[i] = ratio_Vel * vRobot_Command.translVel[i];
	}

	for (int i = 0; i < 3; i++)
	{
		vRobot_Command.rotVel[i] = ratio_Vel * vRobot_Command.rotVel[i];
	}

	return vRobot_Command;
}

void Open_DVP(void* p)
{
	dvpStatus status;
	dvpHandle h;
	char* name = (char*)p;

	printf("Test start,camera is %s\r\n", name);

	do
	{
		/* ���豸 */
		status = dvpOpenByName(name, OPEN_NORMAL, &h);
		if (status != DVP_STATUS_OK)
		{
			printf("dvpOpenByName failed with err:%d\r\n", status);
			break;
		}

		dvpInt32 gamma;
		dvpInt32 sharpness;
		dvpInt32 saturation;
		dvpInt32 contrast;
		dvpInt32 colortemperature;
		dvpUint32 colorsolutionsel;

		float analoggain;
		double exposure;

		dvpColorCorrection colorcorrection;
		dvpColorCorrection set_colorcorrection;

		dvpAwbOperation awboperation;
		dvpAwbOperation set_awboperation;

		dvpGetGamma(h, &gamma);
		dvpGetSharpness(h, &sharpness);
		dvpGetSaturation(h, &saturation);
		dvpGetContrast(h, &contrast);
		dvpGetColorTemperature(h, &colortemperature);
		dvpGetColorSolutionSel(h, &colorsolutionsel);

		dvpGetAnalogGain(h, &analoggain);
		dvpGetExposure(h, &exposure);

		dvpGetColorCorrection(h, &colorcorrection);
		dvpGetAwbOperation(h, &awboperation);

		dvpSetSharpness(h, 40);
		// dvpSetSharpness(h, 80);
		dvpSetGamma(h, 500);
		dvpSetSaturation(h, 90);
		dvpSetContrast(h, 100);
		// dvpSetContrast(h, 140);
		dvpSetColorSolutionSel(h, 2);

		dvpSetAnalogGain(h, 1.2);
		dvpSetExposure(h, 25000);
		// dvpSetExposure(h, 40000);
		// dvpSetExposure(h, 70000);

		set_colorcorrection.bgr[0] = 0.9;
		set_colorcorrection.bgr[1] = 0.79;
		set_colorcorrection.bgr[2] = 1.9;

		dvpSetColorCorrection(h, set_colorcorrection);

		set_awboperation = (dvpAwbOperation)2;

		dvpSetAwbOperation(h, set_awboperation);

		// dvpShowPropertyModalDialog(h,NULL);
		// cv::waitKey(5000);

		dvpFrame frame;
		void* pBuffer;

		/* ��ʼ��Ƶ�� */
		status = dvpStart(h);
		if (status != DVP_STATUS_OK)
		{
			break;
		}

		cv::Mat showImage;
		vector<vector<cv::Point>> featurePoints_Obtained;
		vector<cv::Point> featurePoints_Obtained_Previous;
		vector<cv::Point> featurePoints_Obtained_Current;

		vector<cv::Point> featurePoints_Obtained_Filtered;

		vector<cv::Point3f> Points3D;
		vector<cv::Point2f> Points2D;

		vector<cv::Point2f> Points2D_Proj;

		cv::Mat Points2DMat      = cv::Mat::zeros(7, 1, CV_64FC2);
		cv::Mat Points2D_ProjMat = cv::Mat::zeros(7, 1, CV_64FC2);

		double projErr_Avg;
		double projErr_Max;
		double projErr_Min;
		int projErr_MaxIndex;
		int projErr_MinIndex;
		int iterNum;

		cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);
		cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);

		cv::Mat rvec_Previous = cv::Mat::zeros(3, 1, CV_64FC1);
		cv::Mat tvec_Previous = cv::Mat::zeros(3, 1, CV_64FC1);

		cv::Mat rvec_Filtered = cv::Mat::zeros(3, 1, CV_64FC1);
		cv::Mat tvec_Filtered = cv::Mat::zeros(3, 1, CV_64FC1);

		cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 1544.970675627578, 0, 1312.414149621184,
		                         0, 1544.76986234472, 965.0566324183379, 0, 0, 1);
		cv::Mat distortion_coefficients =
		    (cv::Mat_<double>(5, 1) << 0.0578488137074523, -0.1147036269306462,
		     0.0003202691065071685, -0.0002482003642574091, 0.06296758427444281);

		// cv::Mat color_showImage;

		Points3D.push_back(cv::Point3f(16, 16, 0));
		Points3D.push_back(cv::Point3f(16, 0, 0));
		Points3D.push_back(cv::Point3f(0, 0, 0));
		Points3D.push_back(cv::Point3f(0, 16, 0));

		Points3D.push_back(cv::Point3f(13, 20, 0));
		Points3D.push_back(cv::Point3f(4, 36, 0));
		Points3D.push_back(cv::Point3f(22.4, 36, 0));

		bool first_run   = TRUE;
		bool bDivergence = FALSE;
		bool valid_pose  = FALSE;

		double rm[9];
		cv::Mat rotationM(3, 3, CV_64FC1, rm);

		unsigned long timeStep_Pose = 0;

		double alpha = 0.9;

		int queueSize = 5;
		vector<cv::Mat> tvec_Filtered_Queue;
		vector<cv::Mat> rvec_Filtered_Queue;

		cv::Mat tvec_Filtered_Sum = cv::Mat::zeros(3, 1, CV_64FC1);
		cv::Mat rvec_Filtered_Sum = cv::Mat::zeros(3, 1, CV_64FC1);

		while (Continuous_Running)
		{
			status = dvpGetFrame(h /*������*/, &frame /*֡��Ϣ*/,
			                     &pBuffer /*ͼ�����ݵ��ڴ��׵�ַ,�����ֶ��ͷ�*/,
			                     3000 /*��ʱʱ�䣨���룩*/);
			if (status != DVP_STATUS_OK)
			{
				printf("Fail to get a frame in continuous mode \r\n");
				break;
			}

			Convert2Mat(&frame, (unsigned char*)pBuffer, showImage);

			cv::namedWindow("ImageShow", 0);
			cv::resizeWindow("ImageShow", 640, 480);

			std::cout << "��Ƶ��С��" << showImage.depth() << std::endl;
			printf("The depth of the camera is: %d \r\n", showImage.depth());
			cv::imshow("ImageShow", showImage);
			//            cv::waitKey(21);
			AlreadyShow = TRUE;

			// showImage.convertTo(color_showImage, CV_8U);
			// cv::normalize(color_showImage,color_showImage,0,256*256,cv::NORM_MINMAX);
			// cv::namedWindow("ColorImageShow", 1);
			// cv::resizeWindow("ColorImageShow", 640, 480);
			// cv::imshow("ColorImageShow", color_showImage);

			if (!AlreadyStore && NeedToStoreDesiredPose)
			{
				cv::imwrite("DesiredPoseImage.jpg", showImage);
				printf("����λ����Ϣ�ѱ��� \r\n");
				AlreadyStore           = TRUE;
				NeedToStoreDesiredPose = FALSE;
			}

			if (!successfulTracking)
			{
				Feature_Detection(2, showImage, featurePoints_Obtained);

				for (int i = 0; i < featurePoints_Obtained.size(); i++)
				{
					printf("�Ѽ�⵽......%d��������\r\n", featurePoints_Obtained.size());
					// �����α���
					cv::line(showImage,
					         cv::Point(featurePoints_Obtained.at(i).at(0).x,
					                   featurePoints_Obtained.at(i).at(0).y),
					         cv::Point(featurePoints_Obtained.at(i).at(1).x,
					                   featurePoints_Obtained.at(i).at(1).y),
					         cv::Scalar(0, 0, 255), 3, cv::LINE_AA, 0);// BGR
					cv::line(showImage,
					         cv::Point(featurePoints_Obtained.at(i).at(1).x,
					                   featurePoints_Obtained.at(i).at(1).y),
					         cv::Point(featurePoints_Obtained.at(i).at(2).x,
					                   featurePoints_Obtained.at(i).at(2).y),
					         cv::Scalar(0, 255, 0), 3, cv::LINE_AA, 0);
					cv::line(showImage,
					         cv::Point(featurePoints_Obtained.at(i).at(2).x,
					                   featurePoints_Obtained.at(i).at(2).y),
					         cv::Point(featurePoints_Obtained.at(i).at(3).x,
					                   featurePoints_Obtained.at(i).at(3).y),
					         cv::Scalar(255, 0, 0), 3, cv::LINE_AA, 0);
					cv::line(showImage,
					         cv::Point(featurePoints_Obtained.at(i).at(3).x,
					                   featurePoints_Obtained.at(i).at(3).y),
					         cv::Point(featurePoints_Obtained.at(i).at(0).x,
					                   featurePoints_Obtained.at(i).at(0).y),
					         cv::Scalar(0, 0, 0), 3, cv::LINE_AA, 0);

					// �������α���
					cv::line(showImage,
					         cv::Point(featurePoints_Obtained.at(i).at(4).x,
					                   featurePoints_Obtained.at(i).at(4).y),
					         cv::Point(featurePoints_Obtained.at(i).at(5).x,
					                   featurePoints_Obtained.at(i).at(5).y),
					         cv::Scalar(0, 0, 255), 3, cv::LINE_AA, 0);// BGR
					cv::line(showImage,
					         cv::Point(featurePoints_Obtained.at(i).at(5).x,
					                   featurePoints_Obtained.at(i).at(5).y),
					         cv::Point(featurePoints_Obtained.at(i).at(6).x,
					                   featurePoints_Obtained.at(i).at(6).y),
					         cv::Scalar(0, 255, 0), 3, cv::LINE_AA, 0);
					cv::line(showImage,
					         cv::Point(featurePoints_Obtained.at(i).at(6).x,
					                   featurePoints_Obtained.at(i).at(6).y),
					         cv::Point(featurePoints_Obtained.at(i).at(4).x,
					                   featurePoints_Obtained.at(i).at(4).y),
					         cv::Scalar(255, 0, 0), 3, cv::LINE_AA, 0);

					featurePoints_Obtained_Previous = featurePoints_Obtained.at(0);
					featurePoints_Obtained_Current  = featurePoints_Obtained.at(0);

					cv::namedWindow("ContoursImage", 0);
					cv::resizeWindow("ContoursImage", 640, 480);
					cv::imshow("ContoursImage", showImage);
					cv::waitKey(1);
				}

				if (featurePoints_Obtained_Current.size() == 7)
				{
					Points2D.clear();
					for (int i = 0; i < 7; i++)
					{
						Points2D.push_back(featurePoints_Obtained_Current.at(i));
					}

					if (1)
					{
						iterNum = 0;
						while (TRUE)
						{
							iterNum++;
							if (first_run || bDivergence)
							{
								cv::solvePnP(Points3D, Points2D, camera_matrix,
								             distortion_coefficients, rvec, tvec, FALSE,
								             cv::SOLVEPNP_ITERATIVE);
								first_run = FALSE;

								tvec_Previous = tvec;
								rvec_Previous = rvec;

								tvec_Filtered_Queue.clear();
								rvec_Filtered_Queue.clear();

								tvec_Filtered_Queue.push_back(tvec);
								rvec_Filtered_Queue.push_back(rvec);
							}
							else
							{
								cv::solvePnP(Points3D, Points2D, camera_matrix,
								             distortion_coefficients, rvec, tvec, TRUE,
								             cv::SOLVEPNP_ITERATIVE);

								tvec_Filtered = alpha * tvec + (1 - alpha) * tvec_Previous;
								rvec_Filtered = alpha * rvec + (1 - alpha) * rvec_Previous;

								tvec = tvec_Filtered;
								rvec = rvec_Filtered;

								tvec_Previous = tvec_Filtered;
								rvec_Previous = rvec_Filtered;

								tvec_Filtered_Queue.push_back(tvec_Filtered);
								rvec_Filtered_Queue.push_back(rvec_Filtered);

								if (tvec_Filtered_Queue.size() > queueSize)
								{
									tvec_Filtered_Queue.erase(tvec_Filtered_Queue.cbegin());
									rvec_Filtered_Queue.erase(rvec_Filtered_Queue.cbegin());

									tvec_Filtered_Sum = cv::Mat::zeros(3, 1, CV_64FC1);
									rvec_Filtered_Sum = cv::Mat::zeros(3, 1, CV_64FC1);
									for (int i = 0; i < queueSize; i++)
									{
										tvec_Filtered_Sum =
										    tvec_Filtered_Sum + tvec_Filtered_Queue.at(i);
										rvec_Filtered_Sum =
										    rvec_Filtered_Sum + rvec_Filtered_Queue.at(i);
									}
									tvec = tvec_Filtered_Sum / queueSize;
									rvec = rvec_Filtered_Sum / queueSize;
								}
								else
								{
									tvec_Filtered_Sum = cv::Mat::zeros(3, 1, CV_64FC1);
									rvec_Filtered_Sum = cv::Mat::zeros(3, 1, CV_64FC1);
									for (int i = 0; i < tvec_Filtered_Queue.size(); i++)
									{
										tvec_Filtered_Sum =
										    tvec_Filtered_Sum + tvec_Filtered_Queue.at(i);
										rvec_Filtered_Sum =
										    rvec_Filtered_Sum + rvec_Filtered_Queue.at(i);
									}
									tvec = tvec_Filtered_Sum / tvec_Filtered_Queue.size();
									rvec = rvec_Filtered_Sum / tvec_Filtered_Queue.size();
								}
							}

							// ��ͶӰ������***************************************************************************************************************************************************
							cv::projectPoints(Points3D, rvec, tvec, camera_matrix,
							                  distortion_coefficients, Points2D_Proj);

							projErr_Max = 0;
							for (int i = 0; i < 7; i++)
							{
								Points2DMat.at<cv::Vec2d>(i, 0) =
								    cv::Vec2f(Points2D.at(i).x, Points2D.at(i).y);
								Points2D_ProjMat.at<cv::Vec2d>(i, 0) =
								    cv::Vec2f(Points2D_Proj.at(i).x, Points2D_Proj.at(i).y);

								double projErr_current =
								    cv::norm(Points2DMat.at<cv::Vec2d>(i, 0),
								             Points2D_ProjMat.at<cv::Vec2d>(i, 0));

								if (projErr_current > projErr_Max)
								{
									projErr_Max = projErr_current;
								}
							}

							projErr_Avg = cv::norm(Points2DMat, Points2D_ProjMat, cv::NORM_L2) / 7;
							cout << "��ǰ����λ�˵���ͶӰ��ƽ�������Ϊ��" << endl << endl;
							cout << projErr_Avg << "------����" << endl << endl << endl;

							cout << "��ǰ����λ�˵���ͶӰ��������Ϊ��" << endl << endl;
							cout << projErr_Max << "------����" << endl << endl << endl;

							if (projErr_Max > 5)
								bDivergence = TRUE;

							if (projErr_Avg < 0.1 || iterNum > 0)
								break;
						}

						valid_pose = TRUE;

						printf("λ�ƣ���x����%lf   ��y����%lf    ��z����%lf\r\n",
						       tvec.at<double>(0, 0), tvec.at<double>(1, 0), tvec.at<double>(2, 0));
					}
					if (bsendingFeature)
					{
						featurePoints_Sending = featurePoints_Obtained_Current;
						canSend               = TRUE;
					}

					// ����������Ϣ���ļ���
					if (need2StoreDesiredInfo)
					{
						// ����������ͼ����Ϣ
						FILE* desiredFeaturePointInfoFile;
						desiredFeaturePointInfoFile = fopen("desired_featurePoints.txt", "a+");

						if (desiredFeaturePointInfoFile)
						{
							for (int i = 0; i < 6; i++)
							{
								fprintf(desiredFeaturePointInfoFile, "%d",
								        featurePoints_Obtained_Current.at(i).x);
								fprintf(desiredFeaturePointInfoFile, "%s", " ");
								fprintf(desiredFeaturePointInfoFile, "%d",
								        featurePoints_Obtained_Current.at(i).y);
								fprintf(desiredFeaturePointInfoFile, "%s", " ");
							}
							fprintf(desiredFeaturePointInfoFile, "%d",
							        featurePoints_Obtained_Current.at(6).x);
							fprintf(desiredFeaturePointInfoFile, "%s", " ");
							fprintf(desiredFeaturePointInfoFile, "%d",
							        featurePoints_Obtained_Current.at(6).y);
							fprintf(desiredFeaturePointInfoFile, "%s", "\n");

							need2StoreDesiredInfo = FALSE;
							cv::imwrite("DesiredPoseImage.jpg", showImage);
							printf("������ͼ��������Ϣ�ѱ���ɹ�\r\n\n");
						}
						fclose(desiredFeaturePointInfoFile);

						// ����������λ�ˡ���Ϣ
						if (valid_pose)
						{
							FILE* desiredFeaturePointInfoFile_Pose;
							desiredFeaturePointInfoFile_Pose =
							    fopen("desired_featurePoints_Pose.txt", "a+");

							if (desiredFeaturePointInfoFile_Pose)
							{
								for (int i = 0; i < 3; i++)
								{
									fprintf(desiredFeaturePointInfoFile_Pose, "%lf",
									        tvec.at<double>(i, 0));
									fprintf(desiredFeaturePointInfoFile_Pose, "%s", " ");
								}

								for (int i = 0; i < 2; i++)
								{
									fprintf(desiredFeaturePointInfoFile_Pose, "%lf",
									        rvec.at<double>(i, 0));
									fprintf(desiredFeaturePointInfoFile_Pose, "%s", " ");
								}

								fprintf(desiredFeaturePointInfoFile_Pose, "%lf",
								        rvec.at<double>(2, 0));
								fprintf(desiredFeaturePointInfoFile_Pose, "%s", "\n");

								need2StoreDesiredInfo = FALSE;
								cv::imwrite("DesiredPoseImage.jpg", showImage);
								printf("������λ�ˡ�������Ϣ�ѱ���ɹ�\r\n\n");
							}
							fclose(desiredFeaturePointInfoFile_Pose);
						}
					}

					// ���������
					if (startGrasping)
					{
						printf("\r\n\nץȡ�����Ѿ���ִ����.....\r\n\n");

						printf("������ͼ����ϢΪ��");
						for (int i = 0; i < 6; i++)
							printf("��%d,%d��--", desiredFeatures_Image[i].x,
							       desiredFeatures_Image[i].y);
						printf("��%d,%d��\r\n\n", desiredFeatures_Image[6].x,
						       desiredFeatures_Image[6].y);

						for (int i = 0; i < 7; i++)
						{
							currentFeatures_Image[i].x = featurePoints_Obtained_Current.at(i).x;
							currentFeatures_Image[i].y = featurePoints_Obtained_Current.at(i).y;
						}

						printf("��ǰ��ͼ����ϢΪ��");
						for (int i = 0; i < 6; i++)
							printf("��%d,%d��--", currentFeatures_Image[i].x,
							       currentFeatures_Image[i].y);
						printf("��%d,%d��\r\n\n", currentFeatures_Image[6].x,
						       currentFeatures_Image[6].y);

						printf("������λ�ˡ���ϢΪ��");
						for (int i = 0; i < 5; i++)
							printf("%lf--", desiredFeatures_Pose[i]);
						printf("%lf \r\n\n", desiredFeatures_Pose[5]);

						printf("��ǰ��λ�ˡ���ϢΪ��%lf--%lf--%lf--%lf--%lf--%lf\r\n\n",
						       tvec.at<double>(0, 0), tvec.at<double>(1, 0), tvec.at<double>(2, 0),
						       rvec.at<double>(0, 0), rvec.at<double>(1, 0), rvec.at<double>(2, 0));

						cv::Mat desiredC_tvec_O(3, 1, CV_64FC1);
						cv::Mat desiredC_rvec_O(3, 1, CV_64FC1);

						for (int i = 0; i < 3; i++)
						{
							desiredC_tvec_O.at<double>(i, 0) = desiredFeatures_Pose[i];
						}

						for (int i = 0; i < 3; i++)
						{
							desiredC_rvec_O.at<double>(i, 0) = desiredFeatures_Pose[3 + i];
						}

						// ������
						Robot_Command vRobot_Command;

						printf("����λ�õĿ���\r\n\n");
						vRobot_Command = ControlVel_Pose_inEffectorFrame_useViaPoint(
						    desiredC_tvec_O, desiredC_rvec_O, tvec, rvec, TRUE, FALSE,
						    timeStep_Pose);
						printf("����ڡ�Ŀ������ϵ��\r\n\n");
						printf("���ơ��ߡ��ٶȣ���%lf,%lf,%lf��\r\n\n", vRobot_Command.translVel[0],
						       vRobot_Command.translVel[1], vRobot_Command.translVel[2]);
						printf("���ơ��ǡ��ٶȣ���%lf,%lf,%lf��\r\n\n", vRobot_Command.rotVel[0],
						       vRobot_Command.rotVel[1], vRobot_Command.rotVel[2]);

						// printf("ͼ������(δ��һ��---����ǰ)--��һ�㣺��%d,%d��\r\n\n",
						// currentFeatures_Image[0].x, currentFeatures_Image[0].y);
						// printf("ͼ������(δ��һ��---����ǰ)--�ڶ��㣺��%d,%d��\r\n\n",
						// currentFeatures_Image[1].x, currentFeatures_Image[1].y);
						// printf("ͼ������(δ��һ��---����ǰ)--�����㣺��%d,%d��\r\n\n",
						// currentFeatures_Image[2].x, currentFeatures_Image[2].y);
						// printf("ͼ������(δ��һ��---����ǰ)--���ĵ㣺��%d,%d��\r\n\n",
						// currentFeatures_Image[3].x, currentFeatures_Image[3].y);
						// printf("ͼ������(δ��һ��---����ǰ)--����㣺��%d,%d��\r\n\n",
						// currentFeatures_Image[4].x, currentFeatures_Image[4].y);
						// printf("ͼ������(δ��һ��---����ǰ)--�����㣺��%d,%d��\r\n\n",
						// currentFeatures_Image[5].x, currentFeatures_Image[5].y);
						// printf("ͼ������(δ��һ��---����ǰ)--���ߵ㣺��%d,%d��\r\n\n",
						// currentFeatures_Image[6].x, currentFeatures_Image[6].y);

						printf("����ͼ��Ŀ���\r\n\n");
						vRobot_Command = ControlVel_Image(desiredFeatures_Image,
						                                  currentFeatures_Image, tvec, rvec, 3);
						printf("���ơ��ߡ��ٶȣ���%lf,%lf,%lf��\r\n\n", vRobot_Command.translVel[0],
						       vRobot_Command.translVel[1], vRobot_Command.translVel[2]);
						printf("���ơ��ǡ��ٶȣ���%lf,%lf,%lf��\r\n\n", vRobot_Command.rotVel[0],
						       vRobot_Command.rotVel[1], vRobot_Command.rotVel[2]);
						// ���桰ͼ�����й켣��Ϣ
						if (fileStoreImageTraj)
						{
							for (int i = 0; i < 6; i++)
							{
								fprintf(fileStoreImageTraj, "%d",
								        featurePoints_Obtained_Current.at(i).x);
								fprintf(fileStoreImageTraj, "%s", " ");
								fprintf(fileStoreImageTraj, "%d",
								        featurePoints_Obtained_Current.at(i).y);
								fprintf(fileStoreImageTraj, "%s", " ");
							}
							fprintf(fileStoreImageTraj, "%d",
							        featurePoints_Obtained_Current.at(6).x);
							fprintf(fileStoreImageTraj, "%s", " ");
							fprintf(fileStoreImageTraj, "%d",
							        featurePoints_Obtained_Current.at(6).y);
							fprintf(fileStoreImageTraj, "%s", "\n");
						}

						// ���桰λ�ˡ����й켣��Ϣ
						if (fileStoreImageTraj_Pose)
						{
							for (int i = 0; i < 3; i++)
							{
								fprintf(fileStoreImageTraj_Pose, "%lf", tvec.at<double>(i, 0));
								fprintf(fileStoreImageTraj_Pose, "%s", " ");
							}

							for (int i = 0; i < 2; i++)
							{
								fprintf(fileStoreImageTraj_Pose, "%lf", rvec.at<double>(i, 0));
								fprintf(fileStoreImageTraj_Pose, "%s", " ");
							}
							fprintf(fileStoreImageTraj_Pose, "%lf", rvec.at<double>(2, 0));
							fprintf(fileStoreImageTraj_Pose, "%s", "\n");
						}

						if (!bsendingFeature)
						{
							commands_Sending = vRobot_Command;
							canSend          = TRUE;
						}
					}
				}
				else
				{
					if (need2StoreDesiredInfo)
					{
						printf("Ŀǰû�к��ʵ�����ͼ��������Ϣ���Ա���\r\n\n");
						need2StoreDesiredInfo = FALSE;
					}

					// ץȡ�����ѿ�ʼ��δ��⵽��Чͼ�����������Է���ָֹͣ�����
					if (startGrasping)
					{
						// ����ֹͣ�˶�����
						printf("ץȡ��������е�δ��⵽��Чͼ������\r\n\n");

						Robot_Command vRobot_Command = {0};

						if (!bsendingFeature)
						{
							commands_Sending = vRobot_Command;
							canSend          = TRUE;
						}
					}
				}

				featurePoints_Obtained.clear();
				featurePoints_Obtained_Current.clear();
			}
			else
			{
				Feature_Tracking(2, showImage, featurePoints_Obtained_Previous,
				                 featurePoints_Obtained_Current, 1.6);

				if (featurePoints_Obtained_Current.size() == 7)
				{
					Points2D.clear();
					for (int i = 0; i < 7; i++)
					{
						Points2D.push_back(featurePoints_Obtained_Current.at(i));
					}

					if (1)
					{
						iterNum = 0;
						while (TRUE)
						{
							iterNum++;
							if (!bDivergence)
							{
								cv::solvePnP(Points3D, Points2D, camera_matrix,
								             distortion_coefficients, rvec, tvec, TRUE,
								             cv::SOLVEPNP_ITERATIVE);

								tvec_Filtered = alpha * tvec + (1 - alpha) * tvec_Previous;
								rvec_Filtered = alpha * rvec + (1 - alpha) * rvec_Previous;

								tvec = tvec_Filtered;
								rvec = rvec_Filtered;

								tvec_Previous = tvec_Filtered;
								rvec_Previous = rvec_Filtered;

								tvec_Filtered_Queue.push_back(tvec_Filtered);
								rvec_Filtered_Queue.push_back(rvec_Filtered);

								if (tvec_Filtered_Queue.size() > queueSize)
								{
									tvec_Filtered_Queue.erase(tvec_Filtered_Queue.cbegin());
									rvec_Filtered_Queue.erase(rvec_Filtered_Queue.cbegin());

									tvec_Filtered_Sum = cv::Mat::zeros(3, 1, CV_64FC1);
									rvec_Filtered_Sum = cv::Mat::zeros(3, 1, CV_64FC1);
									for (int i = 0; i < queueSize; i++)
									{
										tvec_Filtered_Sum =
										    tvec_Filtered_Sum + tvec_Filtered_Queue.at(i);
										rvec_Filtered_Sum =
										    rvec_Filtered_Sum + rvec_Filtered_Queue.at(i);
									}
									tvec = tvec_Filtered_Sum / queueSize;
									rvec = rvec_Filtered_Sum / queueSize;
								}
								else
								{
									tvec_Filtered_Sum = cv::Mat::zeros(3, 1, CV_64FC1);
									rvec_Filtered_Sum = cv::Mat::zeros(3, 1, CV_64FC1);
									for (int i = 0; i < tvec_Filtered_Queue.size(); i++)
									{
										tvec_Filtered_Sum =
										    tvec_Filtered_Sum + tvec_Filtered_Queue.at(i);
										rvec_Filtered_Sum =
										    rvec_Filtered_Sum + rvec_Filtered_Queue.at(i);
									}
									tvec = tvec_Filtered_Sum / tvec_Filtered_Queue.size();
									rvec = rvec_Filtered_Sum / tvec_Filtered_Queue.size();
								}
							}
							else
							{
								cv::solvePnP(Points3D, Points2D, camera_matrix,
								             distortion_coefficients, rvec, tvec, FALSE,
								             cv::SOLVEPNP_ITERATIVE);

								tvec_Previous = tvec;
								rvec_Previous = rvec;

								tvec_Filtered_Queue.clear();
								rvec_Filtered_Queue.clear();

								tvec_Filtered_Queue.push_back(tvec);
								rvec_Filtered_Queue.push_back(rvec);
							}

							// ��ͶӰ������***************************************************************************************************************************************************
							cv::projectPoints(Points3D, rvec, tvec, camera_matrix,
							                  distortion_coefficients, Points2D_Proj);

							projErr_Max = 0;
							for (int i = 0; i < 7; i++)
							{
								Points2DMat.at<cv::Vec2d>(i, 0) =
								    cv::Vec2f(Points2D.at(i).x, Points2D.at(i).y);
								Points2D_ProjMat.at<cv::Vec2d>(i, 0) =
								    cv::Vec2f(Points2D_Proj.at(i).x, Points2D_Proj.at(i).y);

								double projErr_current =
								    cv::norm(Points2DMat.at<cv::Vec2d>(i, 0),
								             Points2D_ProjMat.at<cv::Vec2d>(i, 0));

								if (projErr_current > projErr_Max)
								{
									projErr_Max = projErr_current;
								}
							}

							projErr_Avg = cv::norm(Points2DMat, Points2D_ProjMat, cv::NORM_L2) / 7;
							cout << "��ǰ����λ�˵���ͶӰ��ƽ�������Ϊ��" << endl << endl;
							cout << projErr_Avg << "------����" << endl << endl << endl;

							cout << "��ǰ����λ�˵���ͶӰ��������Ϊ��" << endl << endl;
							cout << projErr_Max << "------����" << endl << endl << endl;

							if (projErr_Max > 5)
								bDivergence = TRUE;

							if (projErr_Avg < 0.1 || iterNum > 0)
								break;
						}

						valid_pose = TRUE;

						printf("λ�ƣ���x����%lf   ��y����%lf    ��z����%lf\r\n",
						       tvec.at<double>(0, 0), tvec.at<double>(1, 0), tvec.at<double>(2, 0));
					}
					if (bsendingFeature)
					{
						featurePoints_Sending = featurePoints_Obtained_Current;
						canSend               = TRUE;
					}

					// ����������Ϣ���ļ���
					if (need2StoreDesiredInfo)
					{
						// ����������ͼ����Ϣ
						FILE* desiredFeaturePointInfoFile;
						desiredFeaturePointInfoFile = fopen("desired_featurePoints.txt", "a+");

						if (desiredFeaturePointInfoFile)
						{
							for (int i = 0; i < 6; i++)
							{
								fprintf(desiredFeaturePointInfoFile, "%d",
								        featurePoints_Obtained_Current.at(i).x);
								fprintf(desiredFeaturePointInfoFile, "%s", " ");
								fprintf(desiredFeaturePointInfoFile, "%d",
								        featurePoints_Obtained_Current.at(i).y);
								fprintf(desiredFeaturePointInfoFile, "%s", " ");
							}
							fprintf(desiredFeaturePointInfoFile, "%d",
							        featurePoints_Obtained_Current.at(6).x);
							fprintf(desiredFeaturePointInfoFile, "%s", " ");
							fprintf(desiredFeaturePointInfoFile, "%d",
							        featurePoints_Obtained_Current.at(6).y);
							fprintf(desiredFeaturePointInfoFile, "%s", "\n");

							need2StoreDesiredInfo = FALSE;
							cv::imwrite("DesiredPoseImage.jpg", showImage);
							printf("����ͼ��������Ϣ�ѱ���ɹ�\r\n\n");
						}
						fclose(desiredFeaturePointInfoFile);

						// ����������λ�ˡ���Ϣ
						if (valid_pose)
						{
							FILE* desiredFeaturePointInfoFile_Pose;
							desiredFeaturePointInfoFile_Pose =
							    fopen("desired_featurePoints_Pose.txt", "a+");

							if (desiredFeaturePointInfoFile_Pose)
							{
								for (int i = 0; i < 3; i++)
								{
									fprintf(desiredFeaturePointInfoFile_Pose, "%lf",
									        tvec.at<double>(i, 0));
									fprintf(desiredFeaturePointInfoFile_Pose, "%s", " ");
								}

								for (int i = 0; i < 2; i++)
								{
									fprintf(desiredFeaturePointInfoFile_Pose, "%lf",
									        rvec.at<double>(i, 0));
									fprintf(desiredFeaturePointInfoFile_Pose, "%s", " ");
								}

								fprintf(desiredFeaturePointInfoFile_Pose, "%lf",
								        rvec.at<double>(2, 0));
								fprintf(desiredFeaturePointInfoFile_Pose, "%s", "\n");

								need2StoreDesiredInfo = FALSE;
								cv::imwrite("DesiredPoseImage.jpg", showImage);
								printf("������λ�ˡ�������Ϣ�ѱ���ɹ�\r\n\n");
							}
							fclose(desiredFeaturePointInfoFile_Pose);
						}
					}

					// ���������
					if (startGrasping)
					{
						printf("\r\n\nץȡ�����Ѿ���ִ����.....\r\n\n");

						printf("������ͼ����ϢΪ��");
						for (int i = 0; i < 6; i++)
							printf("��%d,%d��--", desiredFeatures_Image[i].x,
							       desiredFeatures_Image[i].y);
						printf("��%d,%d��\r\n\n", desiredFeatures_Image[6].x,
						       desiredFeatures_Image[6].y);

						for (int i = 0; i < 7; i++)
						{
							currentFeatures_Image[i].x = featurePoints_Obtained_Current.at(i).x;
							currentFeatures_Image[i].y = featurePoints_Obtained_Current.at(i).y;
						}

						printf("��ǰ��ͼ����ϢΪ��");
						for (int i = 0; i < 6; i++)
							printf("��%d,%d��--", currentFeatures_Image[i].x,
							       currentFeatures_Image[i].y);
						printf("��%d,%d��\r\n\n", currentFeatures_Image[6].x,
						       currentFeatures_Image[6].y);

						printf("������λ�ˡ���ϢΪ��");
						for (int i = 0; i < 5; i++)
							printf("%lf--", desiredFeatures_Pose[i]);
						printf("%lf \r\n\n", desiredFeatures_Pose[5]);

						printf("��ǰ��λ�ˡ���ϢΪ��%lf--%lf--%lf--%lf--%lf--%lf\r\n\n",
						       tvec.at<double>(0, 0), tvec.at<double>(1, 0), tvec.at<double>(2, 0),
						       rvec.at<double>(0, 0), rvec.at<double>(1, 0), rvec.at<double>(2, 0));

						cv::Mat desiredC_tvec_O(3, 1, CV_64FC1);
						cv::Mat desiredC_rvec_O(3, 1, CV_64FC1);

						for (int i = 0; i < 3; i++)
						{
							desiredC_tvec_O.at<double>(i, 0) = desiredFeatures_Pose[i];
						}

						for (int i = 0; i < 3; i++)
						{
							desiredC_rvec_O.at<double>(i, 0) = desiredFeatures_Pose[3 + i];
						}

						// ������
						Robot_Command vRobot_Command;

						printf("����λ�õĿ���\r\n\n");
						vRobot_Command = ControlVel_Pose_inEffectorFrame_useViaPoint(
						    desiredC_tvec_O, desiredC_rvec_O, tvec, rvec, TRUE, FALSE,
						    timeStep_Pose);
						printf("����ڡ�Ŀ������ϵ��\r\n\n");
						printf("���ơ��ߡ��ٶȣ���%lf,%lf,%lf��\r\n\n", vRobot_Command.translVel[0],
						       vRobot_Command.translVel[1], vRobot_Command.translVel[2]);
						printf("���ơ��ǡ��ٶȣ���%lf,%lf,%lf��\r\n\n", vRobot_Command.rotVel[0],
						       vRobot_Command.rotVel[1], vRobot_Command.rotVel[2]);

						// printf("ͼ������(δ��һ��---����ǰ)--��һ�㣺��%d,%d��\r\n\n",
						// currentFeatures_Image[0].x, currentFeatures_Image[0].y);
						// printf("ͼ������(δ��һ��---����ǰ)--�ڶ��㣺��%d,%d��\r\n\n",
						// currentFeatures_Image[1].x, currentFeatures_Image[1].y);
						// printf("ͼ������(δ��һ��---����ǰ)--�����㣺��%d,%d��\r\n\n",
						// currentFeatures_Image[2].x, currentFeatures_Image[2].y);
						// printf("ͼ������(δ��һ��---����ǰ)--���ĵ㣺��%d,%d��\r\n\n",
						// currentFeatures_Image[3].x, currentFeatures_Image[3].y);
						// printf("ͼ������(δ��һ��---����ǰ)--����㣺��%d,%d��\r\n\n",
						// currentFeatures_Image[4].x, currentFeatures_Image[4].y);
						// printf("ͼ������(δ��һ��---����ǰ)--�����㣺��%d,%d��\r\n\n",
						// currentFeatures_Image[5].x, currentFeatures_Image[5].y);
						// printf("ͼ������(δ��һ��---����ǰ)--���ߵ㣺��%d,%d��\r\n\n",
						// currentFeatures_Image[6].x, currentFeatures_Image[6].y);

						printf("����ͼ��Ŀ���\r\n\n");
						vRobot_Command = ControlVel_Image(desiredFeatures_Image,
						                                  currentFeatures_Image, tvec, rvec, 3);
						printf("���ơ��ߡ��ٶȣ���%lf,%lf,%lf��\r\n\n", vRobot_Command.translVel[0],
						       vRobot_Command.translVel[1], vRobot_Command.translVel[2]);
						printf("���ơ��ǡ��ٶȣ���%lf,%lf,%lf��\r\n\n", vRobot_Command.rotVel[0],
						       vRobot_Command.rotVel[1], vRobot_Command.rotVel[2]);
						// ���桰ͼ�����й켣��Ϣ
						if (fileStoreImageTraj)
						{
							for (int i = 0; i < 6; i++)
							{
								fprintf(fileStoreImageTraj, "%d",
								        featurePoints_Obtained_Current.at(i).x);
								fprintf(fileStoreImageTraj, "%s", " ");
								fprintf(fileStoreImageTraj, "%d",
								        featurePoints_Obtained_Current.at(i).y);
								fprintf(fileStoreImageTraj, "%s", " ");
							}
							fprintf(fileStoreImageTraj, "%d",
							        featurePoints_Obtained_Current.at(6).x);
							fprintf(fileStoreImageTraj, "%s", " ");
							fprintf(fileStoreImageTraj, "%d",
							        featurePoints_Obtained_Current.at(6).y);
							fprintf(fileStoreImageTraj, "%s", "\n");
						}

						// ���桰λ�ˡ����й켣��Ϣ
						if (fileStoreImageTraj_Pose)
						{
							for (int i = 0; i < 3; i++)
							{
								fprintf(fileStoreImageTraj_Pose, "%lf", tvec.at<double>(i, 0));
								fprintf(fileStoreImageTraj_Pose, "%s", " ");
							}

							for (int i = 0; i < 2; i++)
							{
								fprintf(fileStoreImageTraj_Pose, "%lf", rvec.at<double>(i, 0));
								fprintf(fileStoreImageTraj_Pose, "%s", " ");
							}
							fprintf(fileStoreImageTraj_Pose, "%lf", rvec.at<double>(2, 0));
							fprintf(fileStoreImageTraj_Pose, "%s", "\n");
						}

						if (!bsendingFeature)
						{
							commands_Sending = vRobot_Command;
							canSend          = TRUE;
						}
					}
				}
				else
				{
					if (need2StoreDesiredInfo)
					{
						printf("Ŀǰû�к��ʵ�����ͼ��������Ϣ���Ա���\r\n\n");
						need2StoreDesiredInfo = FALSE;
					}

					// ץȡ�����ѿ�ʼ��δ��⵽��Чͼ�����������Է���ָֹͣ�����
					if (startGrasping)
					{
						printf("ץȡ��������е�δ��⵽��Чͼ������\r\n\n");

						Robot_Command vRobot_Command = {0};

						if (!bsendingFeature)
						{
							commands_Sending = vRobot_Command;
							canSend          = TRUE;
						}
					}
				}

				featurePoints_Obtained_Current.clear();
			}

			if (1)
			{
				if (valid_pose)
				{
					cout << "ƽ��������" << endl << endl;
					cout << tvec << endl << endl;

					cv::Rodrigues(rvec, rotationM);
					cout << "��ת��̬��" << endl << endl;
					cout << rotationM << endl << endl;

					valid_pose = FALSE;
					// ��ͶӰ������*********************************************************************************************
					cv::projectPoints(Points3D, rvec, tvec, camera_matrix, distortion_coefficients,
					                  Points2D_Proj);

					projErr_Max      = 0;
					projErr_MaxIndex = -1;
					projErr_Min      = 1000;
					projErr_MinIndex = -1;
					for (int i = 0; i < 7; i++)
					{
						Points2DMat.at<cv::Vec2d>(i, 0) =
						    cv::Vec2f(Points2D.at(i).x, Points2D.at(i).y);
						Points2D_ProjMat.at<cv::Vec2d>(i, 0) =
						    cv::Vec2f(Points2D_Proj.at(i).x, Points2D_Proj.at(i).y);

						double projErr_current = cv::norm(Points2DMat.at<cv::Vec2d>(i, 0),
						                                  Points2D_ProjMat.at<cv::Vec2d>(i, 0));

						if (projErr_current > projErr_Max)
						{
							projErr_Max      = projErr_current;
							projErr_MaxIndex = i;
						}

						if (projErr_current < projErr_Min)
						{
							projErr_Min      = projErr_current;
							projErr_MinIndex = i;
						}

						cv::circle(showImage, Points2D_Proj.at(i), 8, cv::Scalar(255, 0, 0), 3,
						           cv::LINE_AA, 0);
					}

					cv::circle(showImage, Points2D_Proj.at(projErr_MaxIndex), 8,
					           cv::Scalar(0, 0, 255), 3, cv::LINE_AA, 0);
					cv::circle(showImage, Points2D_Proj.at(projErr_MinIndex), 8,
					           cv::Scalar(0, 255, 0), 3, cv::LINE_AA, 0);

					projErr_Avg = cv::norm(Points2DMat, Points2D_ProjMat, cv::NORM_L2) / 7;
					cout << "��ǰ����λ�˵���ͶӰ��ƽ�������Ϊ��" << endl << endl;
					cout << projErr_Avg << "------����" << endl << endl << endl;

					cout << "��ǰ����λ�˵���ͶӰ��������Ϊ��" << endl << endl;
					cout << projErr_Max << "------����" << endl << endl << endl;

					cout << "��ǰ����λ�˵���ͶӰ����С�����Ϊ��" << endl << endl;
					cout << projErr_Min << "------����" << endl << endl << endl;
				}

				cv::namedWindow("ContoursImage_Tracking", 0);
				cv::resizeWindow("ContoursImage_Tracking", 640, 480);
				cv::imshow("ContoursImage_Tracking", showImage);
				cv::waitKey(1);
			}
			// cv::solvePnP(Points3D, Points2D, camera_matrix, distortion_coefficients, rvec, tvec,
			// FALSE, cv::SOLVEPNP_P3P); cv::solvePnP(Points3D, Points2D, camera_matrix,
			// distortion_coefficients, rvec, tvec, FALSE, cv::SOLVEPNP_EPNP);

			// ���λ��
			// cv::solvePnP();

			// cv::waitKey(20);
			// cv::namedWindow("ContoursImage", 0);
			// cv::resizeWindow("ContoursImage", 640, 480);
			// cv::imshow("ContoursImage", showImage);

			if (kbhit())
			{
				// ���е�۽����������а��������Ϸ���ֹͣ����
				if (successConnect)
				{
					Robot_Command vRobot_Command = {0};
					printf("����ֹͣ��е�۵��˶���\r\n\n");

					if (!bsendingFeature)
					{
						commands_Sending = vRobot_Command;
						canSend          = TRUE;
					}
				}

				printf("����Ҫ���ֲ�����\r\n\n");
				printf("��������λ����Ϣ---�밴S����s��\r\n");
				printf("����������ͨ��-----�밴L����l��\r\n");
				printf("��ʼ����-----------�밴G����g��\r\n");
				printf("��ֹ����-----------�밴P����p��\r\n");

				fflush(stdin);
				keyboardInput = getch();
				keyboardInput = getch();
				if (keyboardInput == 71 || keyboardInput == 103)
				{
					Continuous_Running = TRUE;
					startGrasping      = TRUE;
					printf("ץȡ����׼����ʼ�������� \r\n\n");

					if (successConnect)
					{
						printf("Ŀǰ�Ѿ�����ͨ�ţ�\r\n\n");
					}
					else
					{
						if (bTCP)
						{
							printf("��������TCPͨ��\r\n\n");
							thread task_Communication(Open_Socket_TCP);
							task_Communication.detach();
						}
						else
						{
							printf("��������UDPͨ��\r\n\n");
							thread task_Communication(Open_Socket_UDP);
							task_Communication.detach();
						}
					}

					// ��ȡ����λ����Ϣ-Image
					FILE* desiredFeaturePointInfoFile;
					desiredFeaturePointInfoFile = fopen("desired_featurePoints.txt", "a+");
					int nLines                  = 0;
					if (desiredFeaturePointInfoFile)
					{
						while (!feof(desiredFeaturePointInfoFile))
						{
							for (int i = 0; i < 6; i++)
							{
								fscanf(desiredFeaturePointInfoFile, "%d",
								       &desiredFeatures_Image[i].x);
								fscanf(desiredFeaturePointInfoFile, " ");
								fscanf(desiredFeaturePointInfoFile, "%d",
								       &desiredFeatures_Image[i].y);
								fscanf(desiredFeaturePointInfoFile, " ");
							}
							fscanf(desiredFeaturePointInfoFile, "%d", &desiredFeatures_Image[6].x);
							fscanf(desiredFeaturePointInfoFile, " ");
							fscanf(desiredFeaturePointInfoFile, "%d", &desiredFeatures_Image[6].y);
							fscanf(desiredFeaturePointInfoFile, "\n");

							nLines++;
						}

						if (nLines != 0)
							printf("������ͼ����Ϣ������....\r\n\n");
						else
							printf("Ŀǰû�п�����ִ�������������ͼ����Ϣ\r\n\n");
					}
					fclose(desiredFeaturePointInfoFile);

					// ��ȡ����λ����Ϣ-Pose
					FILE* desiredFeaturePointInfoFile_Pose;
					desiredFeaturePointInfoFile_Pose =
					    fopen("desired_featurePoints_Pose.txt", "a+");
					int nLines_Pose = 0;
					if (desiredFeaturePointInfoFile_Pose)
					{
						while (!feof(desiredFeaturePointInfoFile_Pose))
						{
							for (int i = 0; i < 5; i++)
							{
								fscanf(desiredFeaturePointInfoFile_Pose, "%lf",
								       &desiredFeatures_Pose[i]);
								fscanf(desiredFeaturePointInfoFile_Pose, " ");
							}
							fscanf(desiredFeaturePointInfoFile_Pose, "%lf",
							       &desiredFeatures_Pose[5]);
							fscanf(desiredFeaturePointInfoFile_Pose, "\n");

							nLines_Pose++;
						}

						if (nLines_Pose != 0)
							printf("������λ�ˡ���Ϣ������....\r\n\n");
						else
							printf("Ŀǰû�п�����ִ�������������λ�ˡ���Ϣ\r\n\n");
					}
					fclose(desiredFeaturePointInfoFile_Pose);

					// ���ļ����桰ͼ�񡱹켣��Ϣ
					fileStoreImageTraj = fopen("imageFeatureTraj.txt", "a+");

					// ���ļ����桰λ�ˡ��켣��Ϣ
					fileStoreImageTraj_Pose = fopen("imageFeatureTraj_Pose.txt", "a+");

					fileAreadyOpen = TRUE;
				}
				else if (keyboardInput == 80 || keyboardInput == 112)
				{
					Continuous_Running = FALSE;
					startGrasping      = FALSE;

					// �ر��ļ�
					if (fileAreadyOpen)
					{
						fclose(fileStoreImageTraj);
						fclose(fileStoreImageTraj_Pose);
					}

					printf("ץȡ�����Ѿ�ֹͣ \r\n\n");

					// ����ֹͣ����
					Robot_Command vRobot_Command = {0};

					if (!bsendingFeature)
					{
						commands_Sending = vRobot_Command;
						canSend          = TRUE;
					}
				}
				else if (keyboardInput == 83 || keyboardInput == 115)
				{
					printf("����λ����Ϣ���ڳ��Ա���........ \r\n\n");
					need2StoreDesiredInfo = TRUE;
				}
				else if (keyboardInput == 76 || keyboardInput == 108)
				{
					if (successConnect)
					{
						printf("Ŀǰ�Ѿ�����ͨ�ţ�\r\n\n");
					}
					else
					{
						if (bTCP)
						{
							printf("��������TCPͨ��\r\n\n");
							thread task_Communication(Open_Socket_TCP);
							task_Communication.detach();
						}
						else
						{
							printf("��������UDPͨ��\r\n\n");
							thread task_Communication(Open_Socket_UDP);
							task_Communication.detach();
						}
					}
				}
			}

			cv::waitKey(2); /*ÿ��ͼƬ��ʾ20ms*/
			                // fflush(stdin);
		}

		/* ֹͣ��Ƶ�� */
		status = dvpStop(h);
		if (status != DVP_STATUS_OK)
		{
			break;
		}
	} while (0);

	status = dvpClose(h);

	printf("test quit, %s, status:%d\r\n", name, status);
}

void Running_Continuous()
{
	bool stop                = FALSE;
	dvpUint32 status_Running = -1;
	while (!AlreadyStore)
	{
		cv::waitKey(5);
	}
	while ((stop))
	{
		cv::waitKey(2000);
		printf("Do you want to stop the servoing process? ��������(0+�س�) ����ֹͣ(1+�س�) \r\n");
		// scanf("%d", &status_Running);
		if (status_Running == 1)
		{
			stop               = TRUE;
			Continuous_Running = FALSE;
		}
	}
}

int main()
{
	// if (!AfxWinInit(::GetModuleHandle(NULL), NULL, ::GetCommandLine(), 0))
	//	return 1;

	// AfxGetStaticModuleState();

	printf("start...\r\n\n");

	cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_SILENT);
	// cv::VideoCapture cap;
	// cap.open(0);

	// if (cap.isOpened())
	//{
	//	int width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
	//	int height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
	//	int totalFrames = cap.get(cv::CAP_PROP_FRAME_COUNT);
	//	int frameRate = cap.get(cv::CAP_PROP_FPS);
	//
	//	std::cout << "��Ƶ��ȣ�" << width << std::endl;
	//	std::cout << "��Ƶ�߶ȣ�" << height << std::endl;
	//	std::cout << "��Ƶ��֡����" << totalFrames << std::endl;
	//	std::cout << "֡�ʣ�" << frameRate << std::endl;
	// }

	// cap.release();

	cv::Matx41d test[2];
	test[0] = cv::Matx41d(11.0, 12.0, 13.0, 1);
	test[1] = cv::Matx41d(21.0, 22.0, 23.0, 1);
	printf("���Ծ���%lf\r\n\n", test[1](1));

	dvpUint32 count = 0, num = -1;

	dvpUint32 status_NeedToStore = -1;

	dvpCameraInfo info[8];

	/* ö���豸 */
	dvpRefresh(&count);
	if (count > 8)
		count = 8;

	for (int i = 0; i < (int)count; i++)
	{
		if (dvpEnum(i, &info[i]) == DVP_STATUS_OK)
		{
			printf("[%d]-Camera FriendlyName : %s\r\n\n", i, info[i].FriendlyName);
		}
	}

	/* û�����豸 */
	if (count == 0)
	{
		printf("No device found!\r\n\n");
		return 0;
	}

	while (num < 0 || num >= count)
	{
		printf("Please enter the number of the camera you want to open: \r\n\n");
		scanf("%d", &num);
	}

	thread task_servoing(Open_DVP, (void*)info[num].FriendlyName);

	while (!AlreadyShow)
	{
		cv::waitKey(5);
	}
	if (AlreadyShow)
	{
		printf(
		    "Do you need to store the desired pose? ��Ҫ���棨1+�س���  ����Ҫ���棨0+�س���\r\n");
		// scanf("%d", &status_NeedToStore);
		if (status_NeedToStore == 1)
		{
			NeedToStoreDesiredPose = TRUE;
		}
		else
			AlreadyStore = TRUE;
	}
	// thread task_stopDetection(Running_Continuous);
	task_servoing.join();
	// task_stopDetection.join();

	system("pause");
	return 0;
}