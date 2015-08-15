#ifndef FEATURESTATE_H_INCLUDED
#define FEATURESTATE_H_INCLUDED

#include "stdafx.h"

/**
 *	\class FeatureState
 *	\brief 每一帧特征数据, 建议new和delete来存储
 */
class FeatureState
{
public:
	/** 指定_ImgIdx会自动加载对应图像,加载不了throw错误 */
	FeatureState(int _ImgIdx = -1);
	~FeatureState();
public:
	//当前特征数据的图像编号
	bool inited;
	int idxImg;
	std::vector<cv::KeyPoint> vecKeyPoints;
	std::vector<cv::Point2f> vecFeaturePoints;
	cv::Mat matImage;
	cv::Mat matDescriptor;

	bool loadImage(int _ImgIdx);
	friend std::ostream& operator<<(std::ostream& out, const FeatureState& fs);
	//void SetState(std::vector<cv::KeyPoint>& _vecKeyPoints, std::vector<cv::Point2f>& _vecFeaturePoints, cv::Mat& _matDescriptor);
	//void GetState(std::vector<cv::KeyPoint>& _vecKeyPoints, std::vector<cv::Point2f>& _vecFeaturePoints, cv::Mat& _matDescriptor) const;
};

#endif
