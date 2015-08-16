#include "FeatureState.h"


FeatureState::FeatureState(int _ImgIdx) :
inited(false)
{
	loadImage(_ImgIdx);
}


FeatureState::~FeatureState()
{
}

int FeatureState::detect(int nFeatures) {
	//展开内部数据对象
	return detectExtractFeatures(nFeatures, matImage, vecKeyPoints, vecFeaturePoints, matDescriptor, idxImg);
}

int FeatureState::detectExtractFeatures(
	int nFeatures, cv::Mat& matImage,
	std::vector<cv::KeyPoint>& vecKeyPoints,
	std::vector<cv::Point2f>& vecFeaturePoints,
	cv::Mat & matDescriptor,
	int idxImg) {

	bool _isTimeProfile = true;
	bool _isShowImage = false;
	bool _isLogData = true;
	bool _isUseCacheFeature = CFG_bIsUseCacheFeature;
	bool _isCacheCurrentFeature = CFG_bIsCacheCurrentFeature;

	//若使用本地缓存
	if (_isUseCacheFeature) {
		TIME_BEGIN("Feature Read");
		bool ret = loadFeature(idxImg, nFeatures, vecKeyPoints, vecFeaturePoints, matDescriptor);
		TIME_END("Feature Read");
		//读取成功则返回特征点数，不成功继续计算
		if (ret == true)
			return vecFeaturePoints.size();
		else
			_isUseCacheFeature = false;
	}

	//Sift检测特征点
	cv::SiftFeatureDetector siftDetector(nFeatures);
	if (_isTimeProfile) TIME_BEGIN(cv::format("image-detect-%d", idxImg));
	siftDetector.detect(matImage, vecKeyPoints);
	if (_isTimeProfile) TIME_END(cv::format("image-detect-%d", idxImg));

	//KeyPoint转换成Point2f
	vecFeaturePoints.clear();
	for (auto& kpt : vecKeyPoints)
		vecFeaturePoints.push_back(kpt.pt);

	if (_isLogData)
		printf("detect keypoint.size=%d\n", vecFeaturePoints.size());

	// 128维特征向量提取
	cv::SiftDescriptorExtractor siftExtractor(nFeatures);
	if (_isTimeProfile) TIME_BEGIN(cv::format("desc-compute-%d", idxImg));
	siftExtractor.compute(matImage, vecKeyPoints, matDescriptor);
	if (_isTimeProfile) TIME_END(cv::format("desc-compute-%d", idxImg));

	// 缓存当前计算数据点(如果是读取的本地就不再写入了)
	if (_isCacheCurrentFeature && _isUseCacheFeature == false) {
		TIME_BEGIN("Feature Write");
		writeFeature(idxImg, nFeatures, vecKeyPoints, vecFeaturePoints, matDescriptor);
		TIME_END("Feature Write");
	}

	return vecFeaturePoints.size();
}

bool FeatureState::loadImage(int _ImgIdx) {
	if (_ImgIdx < 0) return false;
	idxImg = _ImgIdx;
	std::string imgPath = cv::format(CFG_sPathImageLoad.c_str(), idxImg);
	matImage = cv::imread(imgPath);

	// 如果读取图像失败
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


bool FeatureState::writeFeature(int idxImg, int nFeature, std::vector<cv::KeyPoint>& vecKeyPoints, std::vector<cv::Point2f>& vecFeaturePoints, cv::Mat & matDescriptor) {
	cv::FileStorage fs;
	fs.open(cv::format(CFG_sPathFeatureData.c_str(), idxImg, nFeature), cv::FileStorage::WRITE);

	if (fs.isOpened() == false) return false;

	fs << "vecKeyPoints" << vecKeyPoints;
	fs << "vecFeaturePoints" << vecFeaturePoints;
	fs << "matDescriptor" << matDescriptor;

	fs.release();

	return true;
}

bool FeatureState::loadFeature(int idxImg, int nFeature, std::vector<cv::KeyPoint>& vecKeyPoints, std::vector<cv::Point2f>& vecFeaturePoints, cv::Mat & matDescriptor) {
	cv::FileStorage fs;
	fs.open(cv::format(CFG_sPathFeatureData.c_str(), idxImg, nFeature), cv::FileStorage::READ);

	if (fs.isOpened() == false) return false;

	cv::read(fs["vecKeyPoints"], vecKeyPoints);
	fs["vecFeaturePoints"] >> vecFeaturePoints;
	fs["matDescriptor"] >> matDescriptor;

	fs.release();



	return true;
}