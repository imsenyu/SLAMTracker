#ifndef CONFIG_H_INCLUDED
#define CONFIG_H_INCLUDED

#include "stdafx.h"

/**
 * \brief 全局配置文件
 */

extern std::string CFG_sPathConfigFile; /** \var string:配置文件读取路径 */
extern std::string CFG_sPathPoseGroutTruth; /** \var string:位姿GroundTruth文件读取路径 */
extern std::string CFG_sPathImageLoad; /** \var string:数据集图像序列读取路径 {idx:%06d} */
extern std::string CFG_sPathFeatureData; /** \var string:缓存特征预处理数据读取路径 {idx:%d} {nfeature:%d} */
extern bool CFG_bIsUseCacheFeature; /** \var bool:是否使用本地预处理特征点 */
extern std::string CFG_sModeExecute; /** \var string:程序执行模式 {track/feature} */
extern bool CFG_bIsCacheCurrentFeature; /** \var bool:是否缓存当前运算特征 */
extern bool CFG_bIsLimitRotationDiff; /** \var bool:是否限制位移旋转角度增量 */
extern int CFG_iImageLoadBegin; /** \var int:读取图像起始标号 */
extern int CFG_iImageLoadEnd; /** \var int:读取图像结束标号 */
extern int CFG_iMaxFeatures; /** \var int:最大特征匹配数量,Sift前K个特征 */
extern bool CFG_bIsUseGroundTruthDistance; /** \var bool:是否使用GroundTruth的位移模长 */
extern bool CFG_bIsNotConsiderAxisY; /** \var bool:是否忽略坐标绘制的Y(上下)轴 */
extern double CFG_dDrawFrameStep; /** \var double:坐标绘制时单位长度(meter)的像素点长度 */
extern double CFG_dScaleRatioLimitBottom; /** \var double:尺度变换限制下界 */
extern double CFG_dScaleRatioLimitTop; /** \var double:尺度变换限制上界 */
extern double CFG_dScaleRatioErrorDefault; /** \var double:尺度变换异常的默认位移模长 */
extern double CFG_dRotationDiffLimit; /** \var double:位移旋转角度增量限制值(degree) */
extern double CFG_dScaleInvIncreaseDiffLimit; /** \var double:位移(尺度倒数)增量限制 */
extern cv::Mat CFG_mCameraParameter; /** \var 相机内参数矩阵 */
extern int CFG_iDequeFrameNumber; /** \var int:历史队列中的帧数限制 */
extern double CFG_dOpticalFlowThreshold; /** \var double:光流过滤阈值 */
extern bool CFG_bIsLogGlobal; /** \var bool:全局log开启/关闭 */
extern int CFG_iPreAverageFilter; /** \var int:前向均值滤波的帧数 */
extern std::string CFG_sDataName; /** \var string:数据集名称 */
#endif