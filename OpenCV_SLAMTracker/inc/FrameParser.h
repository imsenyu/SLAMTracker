#ifndef FRAMEPARSER_H_INCLUDED
#define FRAMEPARSER_H_INCLUDED

#include "stdafx.h"
#include "FeatureState.h"
#include "MotionState.h"

/**
*	\class	FrameParser
*	\brief	帧间处理器
*			给定两帧的特征FeatureState对象, 获取其数据进行对应点匹配和运动估算
*/
class FrameParser
{

protected:
	cv::Mat matOrignImage[2]; /** \var 前后两帧图像矩阵 */
	cv::Mat matDescriptor[2]; /** \var 前后两帧描述子矩阵 */

	std::vector<cv::KeyPoint> vecKeyPoints[2]; /** \var 前后两帧关键点集合 */
	std::vector<cv::Point2f> vecFeaturePoints[2]; /** \var 前后两帧特征点集合 */
	std::vector<int> vecPairPointIdx[2]; /** \var 前后两帧匹配点索引集合 */
	std::vector<cv::Point2f> vecPairPoint[2]; /** \var 前后两帧匹配点集合 */
	std::map<cv::Point2f, cv::Point2f, Utils::PointComp<float> > mapPairPoints; /** \var 后一帧到前一帧的map映射 */
	std::vector<bool> vecFiltedMask; /** \var 集合过滤蒙版 */

	int preImgIdx;	/** \var 帧处理的 两帧中的前一帧 */
	int curImgIdx; /** \var 帧处理的 两帧中的后一帧 */
public:
	/** Getter method */
	std::map<cv::Point2f, cv::Point2f, Utils::PointComp<float> > getMapPairPointsConst() const { return mapPairPoints; }
	std::map<cv::Point2f, cv::Point2f, Utils::PointComp<float> >& getMapPairPointsRef() { return mapPairPoints; }

	/** \fn 初始化构造
	 *	\param prePtr 前一帧特征数据
	 *	\param curPtr 后一帧特征数据
	 */
	FrameParser(FeatureState* prePtr, FeatureState* curPtr);
	~FrameParser();

	//运算流程
	/** \fn 进行数据匹配,并且进行光流筛选 */
	void match(double opThreshold = 2.0f);

	/** \fn 光流验证 */
	void validPointsByOpticalFlow(double threshold = 1.0f);

	/** \fn 计算运动矩阵 */
	bool computeMotion(MotionState& motion, int minFundamentMatches = 25);

};

#endif // FRAMEPARSER_H_INCLUDED