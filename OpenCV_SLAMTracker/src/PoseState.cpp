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

PoseState PoseState::move(const MotionState& motion) {
	PoseState retPose(motion.idxImg[1]);

	double arrLocalPos[] = { pos.x, pos.y, pos.z };
	double arrLocalDir[] = { dir.x, dir.y, dir.z };
	cv::Mat matLocalPos(3, 1, CV_64FC1, arrLocalPos),
			matLocalDir(3, 1, CV_64FC1, arrLocalDir);


	std::cout << matLocalDir << std::endl;
	std::cout << matLocalPos << std::endl;

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

	std::cout << matLocalDir << std::endl;
	std::cout << matLocalPos << std::endl;

	retPose.idxImg = motion.idxImg[1];
	retPose.dir = cv::Point3d((cv::Vec<double, 3>)matLocalDir);
	retPose.pos = cv::Point3d((cv::Vec<double, 3>)matLocalPos);
	retPose.inited = inited;

	return retPose;
}

std::ostream& operator<<(std::ostream& out, const PoseState& ps) {
	out << "PoseState[" << ps.idxImg << "]" << std::endl;
	out << "Pos: " << ps.pos << std::endl;
	out << "Dir: " << ps.dir << std::endl;
	return out;
}