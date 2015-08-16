#ifndef TRACKRUNNER_H_INCLUDED
#define TRACKRUNNER_H_INCLUDED

#include "stdafx.h"
#include "CanvasDrawer.h"
#include "PoseState.h"
#include "FeatureState.h"
#include "MotionState.h"

/**
 * \class TrackRunner
 * \brief 道路跟踪主程序执行流程
 */
class TrackRunner
{
public:
	TrackRunner(int _ImgBeginIdx, int _ImgEndIdx);
	~TrackRunner();
protected:
	/** \var 加载的图像序号的起始{Begin}和结束{End} */
	int idxImgBegin, idxImgEnd;
	/** \var 当前运行中的图像序号 */
	int idxImgCur;
protected:
	/** \var 画布对象 */
	CanvasDrawer cDrawer;
	/** \var GroundTruth路径辅助器 */
	PoseHelper pHelper;
	bool bIsInRotate;

	// 数据历史记录
	std::vector<PoseState> vecPoses; /** \var 记录从开始到现在的所有坐标姿态(位置,方向) */
	std::vector<FeatureState*> vecKeyFrameFeatures; /** \var 关键帧特征记录(KeyP,P,Descrip) */
	std::deque<FeatureState*> deqFrameFeatures; /** \var 最近帧队列 */
	std::vector< std::map<int, MotionState>> vecMotionLinks; /** \var 某帧到之前某帧的 运动状态 */

protected:
	/** 
	 *	\fn 从最近帧队列和关键帧数组中 选出一定数量帧序列以供匹配  
	 *	允许的序号小于 \var idxImg; 可以用虚函数扩充不同的选择方法
	 */
	virtual std::vector<FeatureState*> selectKeySequence(int idxImg = -1);
	virtual int filterMotions(std::vector<MotionState>& vecMotion);
	virtual void updateKeyList(FeatureState* ptrFeature);
public:

	/** \fn 对第一帧需要的数据进行初始化 */
	void initFirstFrame();

	/** \fn 运行每一帧，返回当前帧号 */
	int runKeyStep(); 

	void showFrameMotion();

	void showTrack();
};

#endif