#ifndef MOTIONSTATE_H_INCLUDED
#define MOTIONSTATE_H_INCLUDED

#include "stdafx.h"

/**
 *	\class MotionState
 *	\brief 运动状态，存储两帧间变换数据, 未初始化则.inited属性为false
 */
class MotionState
{
public:
	MotionState();
	~MotionState();
public:
	/** \var 前后对比两张图片的编号 */
	int idxImg[2];
	bool inited;
	//旋转, 平移矩阵
	cv::Mat matR, matT;
	//TODO: 记得改成 get()方式，不给set
	double degreeR, degreeT;
	double scale;
	//前后两帧的 对应点映射
	//平均每帧 500个点对，每个点8Bytes,一共 8K, 4000帧 32MB.可以接受
	std::map<cv::Point2f, cv::Point2f, Utils::PointComp<float>> mapPairPoints;

public:
	/** 获得degreeR和degreeT, 未运算(-100)则运算，否则直接返回 */
	double getDegree(const std::string& str);
	void setState(cv::Mat& _matR, cv::Mat& _matT);
};

#endif