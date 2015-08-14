#ifndef CANVASDRAWER_H_INCLUDED
#define CANVASDRAWER_H_INCLUDED

#include "stdafx.h"

/**
* \class CanvasDrawer
* \brief 用于绘制轨迹画布
*/
class CanvasDrawer
{
private:
	bool avaliable;
public:
	CanvasDrawer();
	~CanvasDrawer();

public:
	cv::Mat matCanvas;
	cv::Point3d gPointDir;	//方向向量
	cv::Point3d gPointPos; //位置向量
	cv::Point2f gPointBase;

	cv::Point3d preGroundTruth;
	cv::Point3d gGroundTruthPos;

	std::fstream fileTraceRecord;
	std::fstream fileGroundTruth;
	int cntGroundTruth;
	//cv::Mat matRotGroundTruthFix;
	double transNorm;

public:
	bool setLogPath(const std::string& recordFilePath = "");
	void initAnimate();
	void drawAnimate(cv::Mat matR, cv::Mat matT, int preImgIdx, int curImgIdx, double velocityScale);
	bool useGroundTruth(const std::string groundTruthPath = "");
private:
	double drawGroundTruth(cv::Mat& canvas,int iterCnt = 1);
};

#endif