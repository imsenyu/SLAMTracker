#include "MotionState.h"


MotionState::MotionState()
{
	avaliable = false;
	degreeR = degreeT = 0.0f;
}


MotionState::~MotionState()
{
}

void MotionState::SetState(cv::Mat& _matR, cv::Mat& _matT, cv::Point3d _matPos, cv::Point3d _matDir) {
	matR = _matR.clone();
	matT = _matT.clone();
	double  arrPos[] = {_matPos.x,_matPos.y,_matPos.z},
			arrDir[] = {_matDir.x,_matDir.y,_matDir.z};
	
	matPos = cv::Mat(3, 1, CV_64FC1, arrPos).clone();
	matDir = cv::Mat(3, 1, CV_64FC1, arrDir).clone();

	avaliable = true;
}

