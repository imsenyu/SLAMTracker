#ifndef CANVASDRAWER_H_INCLUDED
#define CANVASDRAWER_H_INCLUDED

#include "stdafx.h"
#include "PoseState.h"
#include "PoseHelper.h"
/**
* \class CanvasDrawer
* \brief 用于绘制轨迹画布
*/
class CanvasDrawer
{
private:
	bool inited;
public:
	/** \fn 初始化函数,需要指定idxImg,默认inited(false) */
	CanvasDrawer(int _ImgIdx);
	~CanvasDrawer();

public:
	cv::Mat matScaleCanvas;	/** \var 当前scale画布 */
	cv::Mat matPathCanvas;	/** \var 当前path画布 */
	PoseState	gPose;	/** \var 上一个需要绘制的点 */

	/** \var 绘制相对于画布(x,y)的原点偏移
	 *	其他Point变量都需要加上这个偏移才能画在canvas上
	 */
	cv::Point2f gPointBase; 

	std::string recordFilePath; /** \var 路径记录文件路径 */

	/** \var 路径记录文件流对象
	 *	按照pose要求的12个数据进行记录
	 */
	std::fstream fileTraceRecord; 
	PoseHelper* ptrPoseHelper; /** \var GroundTruth数据辅助对象指针,默认NULL */
	int idxImgBegin; /** \var 绘制起始idxImg */
	std::deque<double> qDist;
public:
	/**	\fn 设置路径记录文件地址 
	 *	\param recordFilePath 文件路径
	 *	\return 返回文件打开是否成功
	 */
	bool setLogPath(const std::string& recordFilePath = "");
	/**
	 *	\fn 画布对象初始化
	 *	\param initPose,绘制的第一个姿态,与之后输入的姿态依次连线
	 */
	void initAnimate(PoseState& initPose);
	/**
	 *	\fn 绘制画布
	 *	\param curPose,将当前姿态与gPose连线绘制,并绘制方向
	 *	\param _isTruth, 是否绘制GroundTruth路径
	 */
	void drawCanvas(PoseState& curPose, bool _isTruth = true);
private:
	/**
	*	\fn 对当前运动姿态进行记录,并且绘制scale
	*	\param curPose,当前姿态
	*	\param prePose,上一个姿态
	*/
	void logPose(PoseState& curPose, PoseState& prePose);
};

#endif