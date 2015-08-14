#include "Config.h"

std::string CFG_sPathConfigFile = "./vslam_config.xml";
std::string CFG_sPathImageLoad;// = "../../../Dataset/00/image_0/%06d.png";
std::string CFG_sPathPoseGroutTruth;
std::string CFG_sPathFeatureData;// = "C:\\TempWorkspace\\Features\\features_%d_%d.xml";
std::string CFG_sModeExecute;
bool CFG_bIsUseCacheFeature;// = false;
bool CFG_bIsCacheCurrentFeature;// = false;
bool CFG_bIsLimitRotationDiff; // = false;
int CFG_iImageLoadBegin;// = 0;
int CFG_iImageLoadEnd;// = -1;
int CFG_iMaxFeatures;// = 2000;
int CFG_bIsUseGroundTruthDistance; // = false;
double CFG_dDrawFrameStep; // = 0.5f;
bool CFG_bIsNotConsiderAxisY;// = false;
double CFG_dScaleRatioLimitBottom;// = 0.5f;
double CFG_dScaleRatioLimitTop;// = 11.0f;
double CFG_dScaleRatioErrorDefault; // = 0.1f;
double CFG_dRotationDiffLimit; // = 15.0f;
double CFG_dScaleInvIncreaseDiffLimit; // = 0.1f;