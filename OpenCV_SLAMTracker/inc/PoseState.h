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
protected:
	bool inited;
	Const::CErrType errType;
public:
	/** \fn 对于给定的一个运动,返回新的坐标位置 */
	PoseState move(const MotionState& motion);

	/** 保持getter/setter public */
	int idxImg; /** \var 当前帧编号 */
	cv::Point3d pos;/** \var 记录当时位置点 */
	cv::Point3d dir;/** \var 记录当时镜头方向 */
	cv::Mat dir3; /** 从eye(3,3)旋转到现在的镜头方向 */

	int getErrType() const { return (int)errType; }
	void setErrType(int val);
	void setErrType(Const::CErrType val) { setErrType((int)val); }

	bool getInited() const { return inited; }
	void setInited(bool val) { inited = val; }

	static PoseState calcAverage(std::vector<PoseState>& vecPS);

	/** \fn ostream输出友元重载
	*	\brief 输出格式
	*		"PoseState[%d]"
	*		"Pos:[]"
	*		"Dir:[]"
	*/
	friend std::ostream& operator<<(std::ostream& out, const PoseState& ps);
};

#endif