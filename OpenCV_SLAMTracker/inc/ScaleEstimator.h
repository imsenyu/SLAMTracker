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
	bool UpdateMotion(MotionState* ptrCurMotion);

	//////////////////////////////////////////////////////////////////////////
	// 已废弃，速度跑飞了
	double ComputeScaleTransform(double preTransScale);

	//double ComputeScale();

private:
	double CalcScaleRatio(cv::Mat matDir, int flag = 0);

	//int GetPairPoints3(int cols = 0);

	int GetPairPoints2();

	double CalcLineIntersection(cv::Mat d1, cv::Mat d2, cv::Mat p2, cv::Mat& ip1);

	cv::Mat ScaleEstimator::TransformIn2Coord(int pntNum, int preIdx = 0, int curIdx = 1);
};

#endif
