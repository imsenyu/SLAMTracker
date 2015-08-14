#ifndef TRACKRUNNER_H_INCLUDED
#define TRACKRUNNER_H_INCLUDED

#include "stdafx.h"
#include "CanvasDrawer.h"


/**
 * \class TrackRunner
 * \brief 道路跟踪主程序执行流程
 */
class TrackRunner
{
public:
	TrackRunner();
	~TrackRunner();

private:
	// 画布对象
	CanvasDrawer cDrawer;

	// 数据历史记录

public:

	// 对第一帧需要的数据进行初始化
	void initFirstFrame();

	// 运行每一帧，返回当前帧号
	int runKeyStep(int nextStep = -1); 

	void showFrameMotion();

	void showTrack();
};

#endif