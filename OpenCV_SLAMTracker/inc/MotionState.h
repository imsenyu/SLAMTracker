#ifndef MOTIONSTATE_H_INCLUDED
#define MOTIONSTATE_H_INCLUDED

#include "stdafx.h"

/**
 *	\class MotionState
 *	\brief 运动状态，存储两帧间变换数据
 */
class MotionState
{
public:
	MotionState();
	~MotionState();
protected:
	

	int idxImg[2]; /** \var 前后对比两张图片的编号 */
	bool inited; /** \var 是否初始化 */
	cv::Mat matR, matT; /** \var 旋转和位移矩阵 */

	double scale; /** 尺度(到地面距离) */

	/** \var 前后两帧的对应点映射
	 *	\brief 平均每帧 500个点对，每个点8Bytes,一共 8K, 4000帧 32MB.可以接受
	 */
	std::map<cv::Point2f, cv::Point2f, Utils::PointComp<float>> mapPairPoints;
	
public:
	

	/** Getter/Setter方法 */
	Const::Error errType;

	int getIdxImg(int idx) const { return idxImg[idx]; }
	int& setIdxImg(int idx, int val) { return idxImg[idx] = val; }

	bool getInited() const { return inited; }
	bool setInited(bool val) { return inited = val; }

	std::map<cv::Point2f, cv::Point2f, Utils::PointComp<float>> getMapPairPointsConst() const { return mapPairPoints; }
	std::map<cv::Point2f, cv::Point2f, Utils::PointComp<float>>& getMapPairPointsRef() { return mapPairPoints; }
	int setMapPairPoints(std::map<cv::Point2f, cv::Point2f, Utils::PointComp<float>> val) { mapPairPoints = val; return mapPairPoints.size(); }

	cv::Mat getMatRConst() const { return matR; }
	cv::Mat& getMatRRef() { return matR; }
	void setMatR(cv::Mat& val) { matR = val; }

	cv::Mat getMatTConst() const { return matT; }
	cv::Mat& getMatTRef() { return matT; }
	void setMatT(cv::Mat& val) { matT = val; }

	double getScale() const { return scale; }
	void setScale(double val, bool isMulti = false) { scale = val; if (isMulti) scale = scale / (idxImg[1]-idxImg[0]); }

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