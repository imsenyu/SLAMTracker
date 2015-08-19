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

	//校验motion与当前Pose是否匹配
	if (idxImg != motion.idxImg[0]) {
		throw std::exception(cv::format("当前Pose[%d]与Motion[%d-%d]不匹配",idxImg,motion.idxImg[0],motion.idxImg[1]).c_str());
	}

	PoseState retPose(motion.idxImg[1]);

	//cv::Point3转cv::Mat(3,1)
	double arrLocalPos[] = { pos.x, pos.y, pos.z };
	double arrLocalDir[] = { dir.x, dir.y, dir.z };
	cv::Mat matLocalPos(3, 1, CV_64FC1, arrLocalPos),
			matLocalDir(3, 1, CV_64FC1, arrLocalDir);


	std::cout << matLocalDir << std::endl;
	std::cout << matLocalPos << std::endl;

	cv::Mat matTmpRotate;
	Utils::getRodriguesRotation(matLocalDir, matTmpRotate);

	matLocalPos = matLocalPos + matTmpRotate * motion.matT * 1.65 / motion.scale;

	//matLocalPos = matLocalPos + dir3 * motion.matT * 1.65f / motion.scale;

	//不考虑Y轴
	if (CFG_bIsNotConsiderAxisY) {
		cv::Mat matRC = motion.matR * Const::mat31_001;

		matRC.at<double>(1, 0) = 0.0f;
		matRC = matRC / cv::norm(matRC);

		Utils::getRodriguesRotation(matRC, matRC);
		
		matLocalDir = matRC * matLocalDir;
		matLocalDir.at<double>(1, 0) = 0.0f;
		matLocalDir = matLocalDir / cv::norm(matLocalDir);

	}
	//考虑Y轴
	else {
		matLocalDir = motion.matR * matLocalDir;
		//dir3 = motion.matR * dir3;
	}

	std::cout << matLocalDir << std::endl;
	std::cout << matLocalPos << std::endl;

	retPose.idxImg = motion.idxImg[1];
	retPose.dir = cv::Point3d((cv::Vec<double, 3>)matLocalDir);
	retPose.pos = cv::Point3d((cv::Vec<double, 3>)matLocalPos);
	retPose.dir3 = motion.matR * matTmpRotate;
	retPose.inited = inited;

	return retPose;
}

std::ostream& operator<<(std::ostream& out, const PoseState& ps) {
	out << "PoseState[" << ps.idxImg << "]" << std::endl;
	out << "Pos: " << ps.pos << std::endl;
	out << "Dir: " << ps.dir << std::endl;
	return out;
}