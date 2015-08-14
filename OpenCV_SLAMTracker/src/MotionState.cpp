#include "MotionState.h"


MotionState::MotionState():
inited(false),
degreeR(-100),
degreeT(-100)
{
}


MotionState::~MotionState()
{
}

void MotionState::setState(cv::Mat& _matR, cv::Mat& _matT) {
	matR = _matR.clone();
	matT = _matT.clone();

	inited = true;
}

