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
	if (idxImg != motion.getIdxImg(0)) {
		throw std::exception(cv::format("当前Pose[%d]与Motion[%d-%d]不匹配", idxImg, motion.getIdxImg(0), motion.getIdxImg(1)).c_str());
	}

	PoseState retPose(motion.getIdxImg(1));
	
	//cv::Point3转cv::Mat(3,1)
	double arrLocalPos[] = { pos.x, pos.y, pos.z };
	double arrLocalDir[] = { dir.x, dir.y, dir.z };
	cv::Mat matLocalPos(3, 1, CV_64FC1, arrLocalPos),
			matLocalDir(3, 1, CV_64FC1, arrLocalDir);

	if (CFG_bIsLogGlobal)
	std::cout << matLocalDir << std::endl;
	if (CFG_bIsLogGlobal)
	std::cout << matLocalPos << std::endl;

	cv::Mat matTmpRotate;
	Utils::getRodriguesRotation(matLocalDir, matTmpRotate);

	//matLocalPos = matLocalPos + matTmpRotate * motion.matT * 1.65 / motion.scale;

	if (CFG_bIsNotConsiderAxisY) {
		cv::Mat tT = motion.getMatTConst().clone();
		tT.at<double>(1, 0) = 0.0f;
		tT *= 1.0f / cv::norm(tT);
		matLocalPos = matLocalPos + dir3 * tT *  1.65f / motion.getScale();
	}
	else {
		matLocalPos = matLocalPos + dir3 * motion.getMatTConst() *  1.65f / motion.getScale();
	}
	


	//不考虑Y轴
	if (CFG_bIsNotConsiderAxisY) {
		cv::Mat matRC = motion.getMatRConst() * Const::mat31_001;

		matRC.at<double>(1, 0) = 0.0f;
		matRC = matRC / cv::norm(matRC);

		Utils::getRodriguesRotation(matRC, matRC);
		
		matLocalDir = matRC * matLocalDir;
		matLocalDir.at<double>(1, 0) = 0.0f;
		matLocalDir = matLocalDir / cv::norm(matLocalDir);

		dir3 = dir3 * matRC;
		matRC = dir3 * Const::mat31_001;
		matRC.at<double>(1, 0) = 0.0f;
		Utils::getRodriguesRotation(matRC, matRC);
		dir3 = matRC.clone();

	}
	//考虑Y轴
	else {
		matLocalDir = motion.getMatRConst() * matLocalDir;
		dir3 = dir3 * motion.getMatRConst() ;
	}
	if (CFG_bIsLogGlobal)
	std::cout << matLocalDir << std::endl;
	if (CFG_bIsLogGlobal)
	std::cout << matLocalPos << std::endl;

	retPose.idxImg = motion.getIdxImg(1);
	retPose.dir = cv::Point3d((cv::Vec<double, 3>)matLocalDir);
	retPose.pos = cv::Point3d((cv::Vec<double, 3>)matLocalPos);
	//retPose.dir3 = motion.matR * matTmpRotate;
	retPose.dir3 = dir3.clone();

	retPose.inited = inited;

	return retPose;
}

std::ostream& operator<<(std::ostream& out, const PoseState& ps) {
	out << "PoseState[" << ps.idxImg << "]" << std::endl;
	out << "Pos: " << ps.pos << std::endl;
	out << "Dir: " << ps.dir << std::endl;
	return out;
}

PoseState PoseState::calcAverage(std::vector<PoseState>& vecPS) {

	// 对多个计算得到的新位姿,按照等比数列 1/2; 1/4; 1/8进行 加权平均....好low
	PoseState retPose;
	int vecEstLen = vecPS.size();

	if (vecEstLen > 0) {
		retPose.idxImg = vecPS[0].idxImg;
	}

	retPose.pos = Const::pnt3d_000;
	retPose.dir = Const::pnt3d_000;
	double pow[] = { 1.0f / 2.0f, 1.0f / 4.0f, 1.0f / 8.0f, 1.0f / 16.0f, 1.0f / 32, 1.0f / 64, 1.0 / 128 };

	for (int idx = 0; idx < vecEstLen; idx++) {
		retPose.pos += vecPS[idx].pos;// *pow[idx];
		retPose.dir += vecPS[idx].dir;// *pow[idx];
	}

	//取平均
	retPose.pos = retPose.pos * (1.0f / vecEstLen);
	retPose.dir = retPose.dir * (1.0f / vecEstLen);
	retPose.dir = retPose.dir * (1.0f / cv::norm(retPose.dir));
	
	cv::Mat _rtDir(3, 1, CV_64FC1);
	_rtDir.at<double>(0, 0) = retPose.dir.x;
	_rtDir.at<double>(1, 0) = retPose.dir.y;
	_rtDir.at<double>(2, 0) = retPose.dir.z;

	Utils::getRodriguesRotation(_rtDir, retPose.dir3);

	return retPose;
}