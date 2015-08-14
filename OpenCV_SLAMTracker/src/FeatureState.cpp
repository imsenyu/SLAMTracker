#include "FeatureState.h"


FeatureState::FeatureState(int _ImgIdx) :
inited(false)
{
	if (_ImgIdx < 0) return;
	loadImage(_ImgIdx);
}


FeatureState::~FeatureState()
{
}

bool FeatureState::loadImage(int _ImgIdx) {
	idxImg = _ImgIdx;
	matImage = cv::imread(cv::format(CFG_sPathImageLoad.c_str(), idxImg));
	return inited = (matImage.rows > 0);
}

//void FeatureState::SetState(std::vector<cv::KeyPoint>& _vecKeyPoints, std::vector<cv::Point2f>& _vecFeaturePoints, cv::Mat& _matDescriptor) {
//	vecKeyPoints = _vecKeyPoints;
//	vecFeaturePoints = _vecFeaturePoints;
//	matDescriptor = _matDescriptor.clone();
//}
//
//void FeatureState::GetState(std::vector<cv::KeyPoint>& _vecKeyPoints, std::vector<cv::Point2f>& _vecFeaturePoints, cv::Mat& _matDescriptor) const {
//	_vecKeyPoints = vecKeyPoints;
//	_vecFeaturePoints = vecFeaturePoints;
//	_matDescriptor = matDescriptor.clone();
//
//}
