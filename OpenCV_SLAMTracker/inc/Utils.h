#ifndef UTILS_H_INCLUDED
#define UTILS_H_INCLUDED

#include "stdafx.h"
#include "Constant.h"

/** \var 用于运行时间统计(ms),非线程安全 */
extern std::map<std::string, double> mapTBegin; 

/** \def 时间统计开始定义 */
#define TIME_BEGIN(cstr) {\
	std::string STR = std::string(cstr);\
	if (CFG_bIsLogGlobal)\
	std::cout << "-----------Time_Begin[" << STR << "]:" << std::endl;\
	if (mapTBegin.find(STR) == mapTBegin.end()) mapTBegin.insert(make_pair(STR, clock()));\
	else mapTBegin.find(STR)->second = clock();\
}

/** \def 时间统计结束定义 */
#define TIME_END(cstr) {\
	std::string STR = std::string(cstr);\
	if (mapTBegin.find(STR) == mapTBegin.end())\
	if (CFG_bIsLogGlobal)\
	std::cout << "-----------Time_End[" << STR << "] ERROR" << std::endl;\
	else \
	if (CFG_bIsLogGlobal)\
	std::cout << "-----------Time_End[" << STR << "]:(" << clock() - mapTBegin.find(STR)->second << ")ms" << std::endl; \
}

/**
 *	\namespace 通用工具类
 */
namespace Utils {
	/** \fn 获得 \var rtDir(3,1,CV_F64C1) 相对于 [0,0,1]' 方向的旋转矩阵 \var matRotatoin */
	double getRodriguesRotation(cv::Mat rtDir, cv::Mat& matRotation, cv::Mat orignDir = Const::mat31_001,double ratio = 1.0f);

	/** \fn 获得Command命令行选项 */
	std::string getCmdOption(char ** begin, char ** end, const std::string & option);

	/** \fn 判定 \var option 的Command命令行选项是否存在 */
	bool isCmdOptionExists(char** begin, char** end, const std::string& option);

	/** \fn 加载程序命令行参数 */
	bool loadCommandLine(int argc, char* argv[]);

	/** \fn 根据 \var format 参数输出当前时间 */
	std::string getTimeNow(std::string format = "%Y_%m_%d_%H_%M_%S");

	/** \fn cv::Mat(3,1,CV_F64C1) to cv::Point3d */
	cv::Point3d transform(cv::Mat& mat31);

	template<class _Type>
	class PointComp {
	public:
		bool operator()(const cv::Point_<_Type> &a, const cv::Point_<_Type> &b) const{
			return a.x != b.x ? (a.x < b.x) : (a.y < b.y);
		}

		bool operator()(const cv::Point3_<_Type> &a, const cv::Point3_<_Type> &b) const{
			return a.x != b.x ? (a.x < b.x) : (a.y != b.y ? (a.y < b.y) : (a.z < b.z););
		}
	};

	/** \fn 配置初始化 */
	template<class _Type>
	_Type configDefault(_Type defaultVal, cv::FileNode& fn) {
		if (fn.empty() == true)
			return defaultVal;
		else {
			_Type ret;
			fn >> ret;
			return ret;
		}
	}
};

#endif // UTILS_H_INCLUDED