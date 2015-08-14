#ifndef MOTIONSTATE_H_INCLUDED
#define MOTIONSTATE_H_INCLUDED

#include "stdafx.h"

//////////////////////////////////////////////////////////////////////////
/*
std::vector<cv::Mat> vecMatR, vecMatT, vecMatPos, vecMatDir;
vecMatR.reserve(_ImageLoadEnd + 1);
vecMatT.reserve(_ImageLoadEnd + 1);
vecMatPos.reserve(_ImageLoadEnd + 1);
vecMatDir.reserve(_ImageLoadEnd + 1);
 */

class MotionState
{
public:
	MotionState();
	~MotionState();
public:
	/** \var 前后对比两张图片的编号 */
	int idxImg[2];
	bool avaliable;
	//旋转, 平移矩阵
	cv::Mat matR, matT;
	double degreeR, degreeT;
	//坐标,方向向量
	cv::Mat matPos, matDir;
	//前后两帧的 对应点映射
	//平均每帧 500个点对，每个点8Bytes,一共 8K, 4000帧 32MB.可以接受
	std::map<cv::Point2f, cv::Point2f, Utils::PointComp<float>> mapPairPoints;

public:
	void SetState(cv::Mat& _matR, cv::Mat& _matT, cv::Point3d _matPos, cv::Point3d _matDir);
};

#endif