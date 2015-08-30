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
	int cntRunOk;
protected:
	/** \var 画布对象 */
	CanvasDrawer cDrawer;
	/** \var GroundTruth路径辅助器 */
	PoseHelper pHelper;
	/** \var 用于特别指定读取scale，需要做其他修改 */
	PoseHelper pVisio;
	bool bIsInRotate;
	Const::Error curError;

	// 数据历史记录
	std::vector<PoseState> vecPoses; /** \var 记录从开始到现在的所有坐标姿态(位置,方向) */
	std::vector<FeatureState*> vecKeyFrameFeatures; /** \var 关键帧特征记录(KeyP,P,Descrip) */
	std::deque<FeatureState*> deqFrameFeatures; /** \var 最近帧队列 */
	std::vector< std::map<int, MotionState>> vecMotionLinks; /** \var 某帧到之前某帧的 运动状态 */
	std::vector<int> vecEnableIdxs; /** \var 当前帧是第几个可用帧 */
protected:
	/** 
	 *	\fn 从最近帧队列和关键帧数组中 选出一定数量帧序列以供匹配  
	 *	\brief 允许的序号小于 \var idxImg; 可以用虚函数扩充不同的选择方法
	 */
	virtual std::vector<FeatureState*> selectKeySequence(int idxImg = -1);

	/**
	 *	\fn 对输入vecMotion进行一定的过滤
	 */
	virtual int filterMotions(std::vector<MotionState>& vecMotion, double oldDegreeT);
	
	/**
	 *	\fn 按照一定规则把当前有效帧插入 最近帧队列 和 关键帧数组
	 *	\brief 目前仅控制 最近帧队列数量
	 */
	virtual void updateKeyList(FeatureState* ptrFeature);

	/**
	 *	\fn 限制每次位移的旋转方向角增量
	 */
	virtual bool limitRotationDiff(MotionState& curMotion, double limit); /** 判定位移偏差 */
	
	/**
	 *	\fn 限制位移运动距离的scale关系
	 *	\brief 具体移动距离=1.65f / scale
	 *	如果多帧的运动，会按照帧数等价放宽要求
	 */
	virtual bool limitScaleDiff(MotionState& curMotion,double& curScale, double limit);
public:

	/** \fn 对第一帧需要的数据进行初始化 */
	void initFirstFrame();

	/** \fn 运行每一帧，返回当前帧号 */
	int runKeyStep(); 

	bool hasNext() { return idxImgCur <= idxImgEnd; }

	//void showImage(FeatureState* ptrFS);

	void lm();
};

#endif