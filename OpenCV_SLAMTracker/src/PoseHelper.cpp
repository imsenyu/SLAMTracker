#include "..\inc\PoseHelper.h"


PoseHelper::PoseHelper()
{
	ReadMutex = CreateMutex(NULL, FALSE, NULL);
}


PoseHelper::~PoseHelper()
{
}

bool PoseHelper::setGroundTruth(const std::string groundTruthPath) {
	fileGroundTruth.open(groundTruthPath.c_str(), std::ios_base::in);
	return fileGroundTruth.is_open();
}

cv::Point3d PoseHelper::getPosition(int idxImg, int relativeIdx) {
	bool bReadStatus = false;
	cv::Point3d pointBuffer;
	//如果没缓存，先读进来
	while (vecPosesTruth.size() <= idxImg) {
		//读不了挂掉
		if (readPose(pointBuffer) == false) {
			throw std::exception("GroundTruth Pose Read Error", 3);
			return Const::pnt3d_000;
		}
		else {
			vecPosesTruth.push_back(pointBuffer);
		}
	}
	return vecPosesTruth[idxImg] - vecPosesTruth[ relativeIdx ];
}

bool PoseHelper::inited() {
	return fileGroundTruth.is_open();
}

bool PoseHelper::readPose(cv::Point3d& newPoint) {
	if (fileGroundTruth.fail() == true)
		return false;
	//加上互斥锁
	WaitForSingleObject(ReadMutex, INFINITE);

	double arrPose[12];
	for (int idx = 0; idx < 12; idx++)
		fileGroundTruth >> arrPose[idx];

	newPoint.x = arrPose[3];
	newPoint.y = arrPose[7];
	newPoint.z = arrPose[11];
	//释放互斥锁
	ReleaseMutex(ReadMutex);
	return true;
}