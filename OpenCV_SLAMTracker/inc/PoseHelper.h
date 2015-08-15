#ifndef POSEHELPER_H_INCLUDED
#define POSEHELPER_H_INCLUDED

#include "stdafx.h"

/**
 * \class PoseHelper
 * \brief 用于读取GroundTruth的Pose位置,并存储以供调用
 */
class PoseHelper
{
public:
	PoseHelper();
	~PoseHelper();
private:
	std::fstream fileGroundTruth;
	std::vector<cv::Point3d> vecPosesTruth;
	bool readPose(cv::Point3d& newPoint);
public:
	/** \fn 设置fileGroundTruth读取路径 */
	bool setGroundTruth(const std::string groundTruthPath = "");
	/** \fn 给定idxImg,返回其相对于之前某一点的位移差, <=0为返回相对于0点 */
	cv::Point3d getPosition(int idxImg, int relativeIdx = 0);
	/** \fn 返回pose文件是否成功打开 */
	bool inited();
};

#endif