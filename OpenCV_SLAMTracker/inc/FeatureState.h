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
	/** \fn 指定_ImgIdx会自动加载对应图像,加载不了throw错误 */
	FeatureState(int _ImgIdx = -1);
	~FeatureState();
public:

	bool inited; /** \var 是否初始化(图像是否读取成功) */
	int idxImg; /** \var 图像编号 */
	std::vector<cv::KeyPoint> vecKeyPoints; /** \var KeyPoint关键点集合 */
	std::vector<cv::Point2f> vecFeaturePoints; /** \var KeyPoint中提取出来的特征点集合 */
	cv::Mat matImage; /** \var 读取图像矩阵数据 */
	cv::Mat matDescriptor; /** \var 图像描述子 */

	/** \fn 加载指定idx的图像图像
	 *	\param _ImgIdx 指定图像idx, 使用配置中的路径进行读取
	 *	\return bool 是否加载成功
	 */
	bool loadImage(int _ImgIdx);

	/** \fn 按照指定配置进行特征检测
	 *	\param nFeatures SIFT检测器的参数
	 *	\return int 检测到的特征点数量
	 */
	int detect(int nFeatures = 0);

	/** \fn ostream输出友元重载
	 *	\brief 输出格式
	 *		"FeatureState[%d]"
	 *		"Image:%d*%d"
	 *		"FeaturePoints:%d"
	 */
	friend std::ostream& operator<<(std::ostream& out, const FeatureState& fs);

private:

	/** \fn 对FeatureState内部数据的展开形式 */
	int detectExtractFeatures(int nFeatures, cv::Mat& matImage, std::vector<cv::KeyPoint>& vecKeyPoints, std::vector<cv::Point2f>& vecFeaturePoints, cv::Mat & matDescriptor, int idxImg);
private:

	/** \fn 往缓存写入特征数据
 	 *	\return bool 是否成功打开文件
	 */
	static bool writeFeature(int idxImg, int nFeature, std::vector<cv::KeyPoint>& vecKeyPoints, std::vector<cv::Point2f>& vecFeaturePoints, cv::Mat & matDescriptor);
	/** \fn 从缓存读出特征数据
	 *	\return bool 是否成功打开文件
	 */
	static bool loadFeature(int idxImg, int nFeature, std::vector<cv::KeyPoint>& vecKeyPoints, std::vector<cv::Point2f>& vecFeaturePoints, cv::Mat & matDescriptor);

};

#endif
