#ifndef MOTIONSTATE_H_INCLUDED
#define MOTIONSTATE_H_INCLUDED

#include "stdafx.h"

/**
 *	\class MotionState
 *	\brief 运动状态，存储两帧间变换数据, 未初始化则.inited属性为false
 */
class MotionState
{
public:
	MotionState();
	~MotionState();
public:
	
	int idxImg[2]; /** \var 前后对比两张图片的编号 */
	bool inited; /** \var 是否初始化 */
	cv::Mat matR, matT; /** \var 旋转和位移矩阵 */
	double degreeR, degreeT; /** \var matR和MatT的旋转角 */
	double scale; /** 尺度(到地面距离) */

	/** \var 前后两帧的对应点映射
	 *	\brief 平均每帧 500个点对，每个点8Bytes,一共 8K, 4000帧 32MB.可以接受
	 */
	std::map<cv::Point2f, cv::Point2f, Utils::PointComp<float>> mapPairPoints;

public:
	/**	\fn 获得degreeR和degreeT, 未运算(-100)则运算，否则直接返回
	 *	\param 可选"R" 和 "T"
	 */
	double getDegree(const std::string& str);
	/** \fn ostream输出友元重载
	*	\brief 输出格式
	*		"MotionState[%d-%d]"
	*		"matR:[]"
	*		"matT:[]"
	*		"scale:%f;1.65f/%f"
	*		"pair:%d"
	*/
	friend std::ostream& operator<<(std::ostream& out, const MotionState& ms);
};

#endif