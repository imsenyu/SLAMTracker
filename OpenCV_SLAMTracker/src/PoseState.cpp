#include "PoseState.h"
#include "MotionState.h"

PoseState::PoseState(int _ImgIdx) :
inited(false),
idxImg(_ImgIdx)
{
}


PoseState::~PoseState()
{
}

PoseState PoseState::Move(const MotionState& motion) {
	PoseState retPose(motion.idxImg[1]);

	double arrLocalPos[] = { pointPos.x, pointPos.y, pointPos.z };
	double arrLocalDir[] = { pointPos.x, pointPos.y, pointPos.z };
	cv::Mat matLocalPos(3, 1, CV_64FC1, arrLocalPos),
			matLocalDir(3, 1, CV_64FC1, arrLocalDir);

	cv::Mat matTmpRotate;
	Utils::getRodriguesRotation(matLocalDir, matTmpRotate);

	matLocalPos = matLocalPos + matTmpRotate * motion.matT * 1.65/ motion.scale;

	//²»¿¼ÂÇYÖá
	if (CFG_bIsNotConsiderAxisY) {
		cv::Mat matRC = motion.matR * Const::mat31_001;

		matRC.at<double>(1, 0) = 0.0f;
		matRC = matRC / cv::norm(matRC);

		Utils::getRodriguesRotation(matRC, matRC);
		
		matLocalDir = matRC * matLocalDir;
		matLocalDir.at<double>(1, 0) = 0.0f;
		matLocalDir = matLocalDir / cv::norm(matLocalDir);

	}
	//¿¼ÂÇYÖá
	else {
		matLocalDir = motion.matR * matLocalDir;
	}

	retPose.idxImg = motion.idxImg[1];
	retPose.pointDir = cv::Point3d((cv::Vec<double, 3>)matLocalDir);
	retPose.pointPos = cv::Point3d((cv::Vec<double, 3>)matLocalPos);
	retPose.inited = inited;

	return retPose;
}