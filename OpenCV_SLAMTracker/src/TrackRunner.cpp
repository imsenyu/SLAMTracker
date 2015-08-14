#include "TrackRunner.h"
#include "CanvasDrawer.h"

TrackRunner::TrackRunner()
{
}


TrackRunner::~TrackRunner()
{
}


void TrackRunner::initFirstFrame() {

	// 设定输出配置
	cDrawer.setLogPath(cv::format("./Output/coordOutput_%s.txt", Utils::getTimeNow().c_str()));

	// 在内部初始化 PoseHelper
	cDrawer.useGroundTruth(CFG_sPathPoseGroutTruth);

	// 初始化画布对象
	cDrawer.initAnimate();

}

int TrackRunner::runKeyStep(int nextStep) {

	return nextStep+1;
}

void TrackRunner::showFrameMotion() {


}

void TrackRunner::showTrack() {


}