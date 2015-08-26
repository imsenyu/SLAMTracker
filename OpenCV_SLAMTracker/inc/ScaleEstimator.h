#ifndef VELOCITYESTIMATOR_H_INCLUDED
#define VELOCITYESTIMATOR_H_INCLUDED

#include "stdafx.h"
#include "MotionState.h"
/**
*	\class ScaleEstimator
*	\brief 用于地平面检测的尺度估算
*/
class ScaleEstimator
{
public:
	ScaleEstimator();
	~ScaleEstimator();

private:
	std::vector<MotionState*> ptrMotion; /** \var 运动状态数组[2]的指针引用 */
	cv::Mat matPointUVW[2]; /** \var 匹配点对 */
	cv::Mat matDirXYZ[2]; /** \var 匹配点对的射线方向 */
	cv::Mat matIntersection; /** \var 运算后得到的 三维空间交点 */
public:
	/**
	 *	\fn 使用motion更新估计器 需要的数据
	 */
	bool updateMotion(MotionState* ptrCurMotion);

	/**
	 *	\fn 根据匹配点进行尺度运算
	 */
	double computeScaleTransform();

	/**
	 *	\fn 三角测量
	 *	\brief 把集合点打到三维空间中，比较两个摄像头姿态能同时看到几个点
	 *	\return 能同时看到的点数
	 */
	int triangulate();
private:
	/**
	 *	\fn 根据已知三维点,计算地平面高度
	 *	\brief 目前只找比较大的y值
	 */
	double calcScaleRatio(int flag = 0);

	/**
	 *	\fn 转换匹配点对(map)成需要的数据格式(cv::Mat),方便运算
	 */
	int getPairPoints2();

	/**
	 *	\fn 根据给定的两条直线和直线上一点,求异面直线最近点的交点
	 *	\param d1 直线1方向，默认位置[0,0,0]^T
	 *	\param d2 直线2方向
	 *	\param p2 直线2位置
	 *	\return ip1 返回交点
	 */
	double calcLineIntersection(cv::Mat d1, cv::Mat d2, cv::Mat p2, cv::Mat& ip1);

	/**
	*	\fn 把二维点对转换成三维直线方程
	*/
	cv::Mat transformIn2Coord(int pntNum, int preIdx = 0, int curIdx = 1);
};

#endif
