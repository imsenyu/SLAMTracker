#ifndef SKELETON_H_INCLUDED
#define SKELETON_H_INCLUDED

#include "stdafx.h"
#include "PoseState.h"
#include "MotionState.h"
/** 
 *	\class Skeleton
 *	\brief 骨架优化系统，目前已废弃
 */
class Skeleton
{
public:
	Skeleton();
	~Skeleton();
public:
	std::map< std::pair<int, int>, cv::Mat>  mapT;
	std::vector<cv::Mat> vecX;
	std::vector<cv::Mat> vecE;
public:
	void initData(std::vector<PoseState>& vecPoses, std::vector< std::map<int, MotionState>>& motionLink);
	double calcDiff();
	void merge(double step = 1e-5);
	void fixMatrix();
	double calcError();
};

#endif