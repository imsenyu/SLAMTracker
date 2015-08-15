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
	std::string imgPath = cv::format(CFG_sPathImageLoad.c_str(), idxImg);
	matImage = cv::imread(imgPath);

	// Èç¹û¶ÁÈ¡Í¼ÏñÊ§°Ü
	if (matImage.rows == 0 || matImage.cols == 0) {
		std::string error = "Image Load Error";
		throw std::exception( (error + imgPath).c_str() );
	}

	inited = true;
	return inited;
}

std::ostream& operator<<(std::ostream& out, const FeatureState& fs) {
	out << "FeatureState["<<fs.idxImg <<"]" << std::endl;
	out << "Image:"<<fs.matImage.rows << "*" << fs.matImage.cols << std::endl;
	out << "FeaturePoints: " << fs.vecFeaturePoints.size() << std::endl;
	return out;
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
