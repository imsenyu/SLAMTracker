#ifndef CANVASDRAWER_H_INCLUDED
#define CANVASDRAWER_H_INCLUDED

#include "stdafx.h"
#include "PoseState.h"
#include "PoseHelper.h"
/**
* \class CanvasDrawer
* \brief 用于绘制轨迹画布
*/
class CanvasDrawer
{
private:
	bool inited;
public:
	CanvasDrawer(int _ImgIdx);
	~CanvasDrawer();

public:
	cv::Mat matCanvas;
	PoseState	gPose;
	cv::Point2f gPointBase;

	//cv::Point3d preGroundTruth;
	//cv::Point3d gGroundTruthPos;

	std::fstream fileTraceRecord;
	PoseHelper poseGroundTruth;
	int idxGroundTruth;
	int idxImgBegin;
	////cv::Mat matRotGroundTruthFix;
	//double transNorm;

public:
	bool setLogPath(const std::string& recordFilePath = "");
	void initAnimate(PoseState& initPose);
	void drawCanvas(PoseState& curPose);
	//void drawAnimate(cv::Mat matR, cv::Mat matT, int preImgIdx, int curImgIdx, double velocityScale);
	bool useGroundTruth(const std::string groundTruthPath = "");
private:
	double drawGroundTruth(int idxImgCur);
};

#endif