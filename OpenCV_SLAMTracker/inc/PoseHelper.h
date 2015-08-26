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
	std::fstream fileGroundTruth;	/** \var pose数据文件流 */
	std::vector<cv::Point3d> vecPosesTruth; /** \var 已读取的pose数据缓存 */

	/**
	 *	\fn 尝试读取一次坐标点
	 *	\return 返回是否读取成功
	 */
	bool readPose(cv::Point3d& newPoint);
	HANDLE ReadMutex; /** \var 用于并行状态下文件流读取互斥 */
public:
	/** \fn 设置fileGroundTruth读取路径 */
	bool setGroundTruth(const std::string groundTruthPath = "");
	/** \fn 给定idxImg,返回其相对于之前某一点的位移差, <=0为返回相对于0点
	 *	\brief 线程安全, 互斥锁控制,保证文件流操作正常
	 */
	cv::Point3d getPosition(int idxImg, int relativeIdx = 0);

	/** \fn 返回pose文件是否成功打开 */
	bool inited();
};

#endif