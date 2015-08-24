#include "Skeleton.h"


Skeleton::Skeleton()
{
}


Skeleton::~Skeleton()
{
}

void Skeleton::initData(std::vector<PoseState>& vecPoses, std::vector< std::map<int, MotionState>>& motionLink) {
	//重置 T矩阵查找
	for (int idx = 0; idx < motionLink.size(); idx++) {
		int idJ = idx;
		if (motionLink[idx].size() == 0) continue;
		for (auto& pr : motionLink[idx]) {
			int idI = pr.first;
			MotionState& curMotion = pr.second;

			cv::Mat curT(4, 4, CV_64FC1);
			curT = 0.0f;
			curT.at<double>(3, 3) = 1.0f;
			for (int i = 0; i < 3; i++) 
			for (int j = 0; j < 3; j++)
				curT.at<double>(i, j) = curMotion.matR.at<double>(i, j);
			for (int i = 0; i < 3; i++)
				curT.at<double>(i, 3) = curMotion.matT.at<double>(i, 0);

			mapT.insert(std::make_pair(
				std::pair<int, int>( idI,idJ ),
				curT.clone()
				));
		}
	}

	//重置 vecPoses查找
	
	vecX.resize(vecPoses.size());
	vecE.resize(vecPoses.size());

	for (int idx = 0; idx < vecPoses.size(); idx++) {
		if (vecPoses[idx].inited == true) {
			PoseState& curPose = vecPoses[idx];
			cv::Mat curX(4, 4, CV_64FC1);
			curX = 0.0f;
			curX.at<double>(3, 3) = 1.0f;
			for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
				curX.at<double>(i, j) = curPose.dir3.at<double>(i, j);

			curX.at<double>(0, 3) = curPose.pos.x;
			curX.at<double>(1, 3) = curPose.pos.y;
			curX.at<double>(2, 3) = curPose.pos.z;

			vecX[idx] = cv::Mat(4, 4, CV_64FC1);
			vecX[idx] = curX.clone();
		}
	}

	for (int idx = 0; idx < vecPoses.size(); idx++) {
		vecE[idx] = cv::Mat(4, 4, CV_64FC1);
		vecE[idx] = 0.0f;
	}

}

double Skeleton::calcError() {
	double ret = 0.0f;
	for (auto& pr : mapT) {
		int idI = pr.first.first,
			idJ = pr.first.second;
		cv::Mat& Tij = pr.second;
		ret += cv::norm(vecX[idI] - Tij*vecX[idJ], cv::NORM_L2);
	}
	return ret;
}

double Skeleton::calcDiff() {
	for (int i = 0; i < vecE.size(); i++){
		vecE[i] = cv::Mat(4, 4, CV_64FC1);
		vecE[i] = 0.0f;
	}
	double ret = 0.0f;
	for (auto& pr : mapT) {
		int idI = pr.first.first,
			idJ = pr.first.second;
		cv::Mat& Tij = pr.second;
		//ret += cv::norm(vecX[idI] - Tij*vecX[idJ], cv::NORM_L2);
		//对idI 应用偏导
		cv::Mat resI;
		resI = 2.0f * (vecX[idI] - Tij*vecX[idJ]);

		//对idJ 应用偏导
		cv::Mat resJ;
		resJ = 2.0f * Tij.t() * (Tij*vecX[idJ] - vecX[idI]);

		vecE[idI] += resI;
		vecE[idJ] += resJ;
	}
	return ret;
}

void Skeleton::merge(double step) {

	for (int idx = 1; idx < vecX.size(); idx++) {
		if (vecX[idx].rows>0){
			vecX[idx] -= step * vecE[idx];
		}
	}
}

void Skeleton::fixMatrix() {
	//修正 vecX 旋转正和  第4行的0001
	for (int idx = 0; idx < vecX.size(); idx++) {
		if (vecX[idx].rows>0){
			cv::Mat tRot(3, 3, CV_64FC1);
			for (int i = 0; i < 3;i++)
			for (int j = 0; j < 3; j++)
				tRot.at<double>(i, j) = vecX[idx].at<double>(i, j);
			tRot *= (1.0f / cv::norm(tRot));

			for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
				vecX[idx].at<double>(i, j) = tRot.at<double>(i, j);

			vecX[idx].at<double>(3, 0) = 0.0f;
			vecX[idx].at<double>(3, 1) = 0.0f;
			vecX[idx].at<double>(3, 2) = 0.0f;
			vecX[idx].at<double>(3, 3) = 1.0f;

		}
	}

}