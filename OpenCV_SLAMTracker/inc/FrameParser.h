#ifndef FRAMEPARSER_H_INCLUDED
#define FRAMEPARSER_H_INCLUDED

#include "stdafx.h"
#include "FeatureState.h"
#include "MotionState.h"

class FrameParser
{
private:
	void InitData(int idx0, int idx1);
public:
	cv::Mat matOrignImage[2];
	cv::Mat matDescriptor[2];

	std::vector<cv::KeyPoint> vecKeyPoints[2];
	std::vector<cv::Point2f> vecFeaturePoints[2];
	std::vector<int> vecPairPointIdx[2];
	std::vector<cv::Point2f> vecPairPoint[2];
	std::map<cv::Point2f, cv::Point2f, Utils::PointComp<float> > mapPairPoints;

	///filter, false means drop
	std::vector<bool> vecFiltedMask;

	bool isInitedByCloneImage;
	bool isTimeProfile;
	bool isShowImage;
	bool isLogData;
	int preImgIdx;
	int curImgIdx;
public:

	FrameParser(FeatureState* prePtr, FeatureState* curPtr);
	~FrameParser();

	//运算流程
	//void DetectFeaturePoints(int nFeatures, int whichImage = -1);
	void matchFeaturePoints();
	void validPointsByOpticalFlow(double threshold = 1.0f);
	//bool ComputePointPairMotion(cv::Mat& matRotation, cv::Mat& matTransform, double& degreeT, int minFundamentMatches = 25);
	bool computeMotion(MotionState& motion, int minFundamentMatches = 25);
	//bool DrawFeaturesFlow(cv::Mat& matR, cv::Mat& matT);

	//数据输入输出
	//void SetFeatureState(const FeatureState& fState, int idx = 0);
	//void GetFeatureState(FeatureState& fState, int idx = 1);
	//void GetMotionState(MotionState& mState);

	//静态方法
	static int detectExtractFeatures(int nFeatures, FeatureState& fState);
	static int detectExtractFeatures(int nFeatures, cv::Mat& matImage, std::vector<cv::KeyPoint>& vecKeyPoints, std::vector<cv::Point2f>& vecFeaturePoints, cv::Mat & matDescriptor, int idxImg);
private:
	static bool writeFeature(int idxImg, int nFeature, std::vector<cv::KeyPoint>& vecKeyPoints, std::vector<cv::Point2f>& vecFeaturePoints, cv::Mat & matDescriptor);
	static bool loadFeature(int idxImg, int nFeature, std::vector<cv::KeyPoint>& vecKeyPoints, std::vector<cv::Point2f>& vecFeaturePoints, cv::Mat & matDescriptor);

};

#endif // FRAMEPARSER_H_INCLUDED