#include "FeatureState.h"


FeatureState::FeatureState()
{
	idxImg = -1;
}


FeatureState::~FeatureState()
{
}

void FeatureState::SetState(std::vector<cv::KeyPoint>& _vecKeyPoints, std::vector<cv::Point2f>& _vecFeaturePoints, cv::Mat& _matDescriptor) {
	vecKeyPoints = _vecKeyPoints;
	vecFeaturePoints = _vecFeaturePoints;
	matDescriptor = _matDescriptor.clone();
}

void FeatureState::GetState(std::vector<cv::KeyPoint>& _vecKeyPoints, std::vector<cv::Point2f>& _vecFeaturePoints, cv::Mat& _matDescriptor) const {
	_vecKeyPoints = vecKeyPoints;
	_vecFeaturePoints = vecFeaturePoints;
	_matDescriptor = matDescriptor.clone();

}
