#ifndef POSITIONSTATE_H_INCLUDED
#define POSITIONSTATE_H_INCLUDED

#include "stdafx.h"
#include "MotionState.h"
/**
 *	\class PoseState
 *	\brief 当前运动坐标记录,以及传输转换(如果跳帧则.inited属性为false)
 */
class PoseState
{
public:
	PoseState(int _ImgIdx = -1);
	~PoseState();
public:
	bool inited;
	int idxImg; /** \var 当前帧编号 */
	cv::Point3d pos;/** \var 记录当时位置点 */
	cv::Point3d dir;/** \var 记录当时镜头方向 */

public:
	/** \fn 对于给定的一个运动,返回新的坐标位置 */
	PoseState move(const MotionState& motion);
	/** \fn ostream输出友元重载
	*	\brief 输出格式
	*		"PoseState[%d]"
	*		"Pos:[]"
	*		"Dir:[]"
	*/
	friend std::ostream& operator<<(std::ostream& out, const PoseState& ps);
};

#endif