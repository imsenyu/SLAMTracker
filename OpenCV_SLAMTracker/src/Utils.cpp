/* Utils: Time */
#include "Utils.h"
std::map<std::string, double> mapTBegin;

double Utils::getRodriguesRotation(cv::Mat rtDir, cv::Mat& matRotation, double ratio) {
	//使用 叉乘求出 旋转轴[1*3]，然后 用 cos(theta) = P·Q / |P|*|Q| 求出theta弧度作为旋转轴向量的 normest
	//带入函数 算出旋转矩阵，然后再说
	//
	bool _isLogData = false;

	double arrUnitVector[] = { 0, 0, 1 };
	cv::Mat matUnitVector(3, 1, CV_64FC1, arrUnitVector);

	if (_isLogData)std::cout << matUnitVector << std::endl;

	double thetaInR = 0.0f;
	thetaInR = (matUnitVector.dot(rtDir)) / std::sqrt(matUnitVector.dot(matUnitVector)) / std::sqrt(rtDir.dot(rtDir));

	if (_isLogData) std::cout << thetaInR << std::endl;
	if (abs(thetaInR) > 1 - 1e-7) {
		matRotation = cv::Mat(3, 3, CV_64FC1);
		matRotation = 0.0f;
		if (_isLogData) std::cout << matRotation << std::endl;
		matRotation.at<double>(0, 0) = matRotation.at<double>(1, 1) = matRotation.at<double>(2, 2) = 1.0f;
		if (_isLogData) std::cout << matRotation << std::endl;
		return 0.0f;
	}

	matUnitVector = matUnitVector.cross(rtDir);
	if (_isLogData) std::cout << matUnitVector << std::endl;
	matUnitVector = (matUnitVector / cv::norm(matUnitVector))* std::acos(thetaInR)*ratio;
	if (_isLogData) std::cout << matUnitVector << std::endl;

	cv::Rodrigues(matUnitVector, matRotation);
	if (_isLogData) std::cout << matUnitVector << std::endl;
	if (_isLogData) std::cout << matRotation << std::endl;

	return std::acos(thetaInR)*180.0f / std::acos(-1) * (rtDir.at<double>(0,0) >0 ? 1.0f : -1.0f );
}



#define CFG_EXISTS(str) (Utils::isCmdOptionExists(argv, argv + argc, str))
#define CFG_GET(str) (Utils::getCmdOption(argv, argv + argc, str))

bool Utils::loadCommandLine(int argc, char* argv[]) {

	//////////////////////////////////////////////////////////////////////////
	// 加载配置文件l
	cv::FileStorage fs;
	std::string userDefinedCfg = CFG_GET("--config");
	if (argc == 2) {
		userDefinedCfg = std::string(argv[1]);
	}
	userDefinedCfg = userDefinedCfg.length() > 0 ? userDefinedCfg : CFG_sPathConfigFile;
	bool _isUserConfigOpenable = fs.open(userDefinedCfg, cv::FileStorage::READ);

	//////////////////////////////////////////////////////////////////////////
	// 根据默认配置 和 配置文件自定义设置更新 参数
	CFG_sPathImageLoad			= Utils::configDefault<std::string>("../../../Dataset/00/image_0/%06d.png", fs["sPathImageLoad"]);
	CFG_sPathFeatureData		= Utils::configDefault<std::string>("", fs["sPathCacheFeature"]);
	CFG_sPathPoseGroutTruth		= Utils::configDefault<std::string>("", fs["sPathPoseGroutTruth"]);
	CFG_sModeExecute			= Utils::configDefault<std::string>("track", fs["sModeExecute"]);
	CFG_bIsNotConsiderAxisY		= Utils::configDefault<int>(true, fs["bIsIgnoreDrawAxisY"]);
	CFG_iImageLoadBegin			= Utils::configDefault<int>(0, fs["iImageBeginIdx"]);
	CFG_iImageLoadEnd			= Utils::configDefault<int>(4540, fs["iImageEndIdx"]);
	CFG_bIsCacheCurrentFeature	= Utils::configDefault<int>(false, fs["bIsCacheFeature"]) && CFG_sPathFeatureData.length();
	CFG_bIsUseCacheFeature		= Utils::configDefault<int>(false, fs["bIsUseCacheFeature"]) && CFG_sPathFeatureData.length() ;
	CFG_bIsUseGroundTruthDistance = Utils::configDefault<int>(false, fs["bIsUseGroundTruthDistance"]);
	CFG_iMaxFeatures			= Utils::configDefault<int>(2000, fs["iMaxFeatures"]);
	CFG_dDrawFrameStep			= Utils::configDefault<double>(1.0f, fs["dDrawFrameStep"]);
	CFG_dScaleRatioLimitBottom	= Utils::configDefault<double>(0.5f, fs["dScaleRatioLimitBottom"]);// = 0.5f;
	CFG_dScaleRatioLimitTop		= Utils::configDefault<double>(11.0f, fs["dScaleRatioLimitTop"]);// = 11.0f;
	CFG_dScaleRatioErrorDefault	= Utils::configDefault<double>(0.5f, fs["dScaleRatioErrorDefault"]);
	CFG_dRotationDiffLimit		= Utils::configDefault<double>(15.0f, fs["dRotationDiffLimit"]);
	CFG_bIsLimitRotationDiff	= Utils::configDefault<int>(0, fs["bIsLimitRotationDiff"]);
	CFG_dScaleInvIncreaseDiffLimit = Utils::configDefault<double>(0.1f, fs["dScaleInvIncreaseDiffLimit"]);

	fs.release();
	fs.open(userDefinedCfg, cv::FileStorage::WRITE);

	fs << "sPathImageLoad"			<< CFG_sPathImageLoad;
	fs << "sPathCacheFeature"		<< CFG_sPathFeatureData;
	fs << "sPathPoseGroutTruth"		<< CFG_sPathPoseGroutTruth;
	fs << "sModeExecute"			<< CFG_sModeExecute;
	fs << "bIsIgnoreDrawAxisY"		<< CFG_bIsNotConsiderAxisY;
	fs << "iImageBeginIdx"			<< CFG_iImageLoadBegin;
	fs << "iImageEndIdx"			<< CFG_iImageLoadEnd;
	fs << "bIsCacheFeature"			<< CFG_bIsCacheCurrentFeature;
	fs << "bIsUseCacheFeature"		<< CFG_bIsUseCacheFeature;
	fs << "iMaxFeatures"			<< CFG_iMaxFeatures;
	fs << "dDrawFrameStep"			<< CFG_dDrawFrameStep;
	fs << "dScaleRatioLimitBottom"	<< CFG_dScaleRatioLimitBottom;
	fs << "dScaleRatioLimitTop"		<< CFG_dScaleRatioLimitTop;
	fs << "dScaleRatioErrorDefault"	<< CFG_dScaleRatioErrorDefault;
	fs << "dRotationDiffLimit"		<< CFG_dRotationDiffLimit;
	fs << "bIsLimitRotationDiff"	<< CFG_bIsLimitRotationDiff;
	fs << "bIsUseGroundTruthDistance" << CFG_bIsUseGroundTruthDistance;
	fs << "dScaleInvIncreaseDiffLimit" << CFG_dScaleInvIncreaseDiffLimit;

	fs.release();

	return true;
}

std::string Utils::getCmdOption(char ** begin, char ** end, const std::string & option)
{
	char ** itr = std::find(begin, end, option);
	if (itr != end && ++itr != end)
	{
		return std::string(*itr);
	}
	return "";
}

bool Utils::isCmdOptionExists(char** begin, char** end, const std::string& option)
{
	return std::find(begin, end, option) != end;
}

std::string Utils::getTimeNow(std::string format) {
	struct tm timeStruct;
	time_t timeSecond;
	static char timeBuf[100];

	timeSecond = time(NULL);
	localtime_s(&timeStruct, &timeSecond);
	strftime(timeBuf, 100, format.c_str(), &timeStruct);

	return std::string(timeBuf);
}