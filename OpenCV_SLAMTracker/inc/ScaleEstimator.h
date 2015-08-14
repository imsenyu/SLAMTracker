#ifndef VELOCITYESTIMATOR_H_INCLUDED
#define VELOCITYESTIMATOR_H_INCLUDED

#include "stdafx.h"
#include "MotionState.h"

class ScaleEstimator
{
public:
	ScaleEstimator();
	~ScaleEstimator();

private:
	std::vector<MotionState*> ptrMotion;
	cv::Mat matPointUVW[2];
	cv::Mat matDirXYZ[2];
	cv::Mat matIntersection;
public:
	// 更新数据
	bool updateMotion(MotionState* ptrCurMotion);

	//////////////////////////////////////////////////////////////////////////
	// 已废弃，速度跑飞了
	double computeScaleTransform();

	//double ComputeScale();

private:
	double calcScaleRatio(int flag = 0);

	//int GetPairPoints3(int cols = 0);

	int getPairPoints2();

	double calcLineIntersection(cv::Mat d1, cv::Mat d2, cv::Mat p2, cv::Mat& ip1);

	cv::Mat transformIn2Coord(int pntNum, int preIdx = 0, int curIdx = 1);
};

#endif
