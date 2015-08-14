#include "Constant.h"

double Const::arr31_001000[] = { 0, 0, 1, 0, 0, 0, 1, 1, 1 };
double Const::arr33[] = {
0, 0, 0, 0, 0, 0, 0, 0, 0,
1, 0, 0, 0, 1, 0, 0, 0, 1
};
const cv::Mat Const::mat31_001(3, 1, CV_64FC1, arr31_001000);
const cv::Mat Const::mat31_010(3, 1, CV_64FC1, arr31_001000+1);
const cv::Mat Const::mat31_100(3, 1, CV_64FC1, arr31_001000+2);
const cv::Mat Const::mat31_000(3, 1, CV_64FC1, arr31_001000+3);
const cv::Mat Const::mat31_111(3, 1, CV_64FC1, arr31_001000+6);
const cv::Mat Const::mat33_000(3, 1, CV_64FC1, arr33);
const cv::Mat Const::mat33_111(3, 1, CV_64FC1, arr33+9);