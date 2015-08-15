#ifndef CONSTANT_H_INCLUDED
#define CONSTANT_H_INCLUDED

#include "stdafx.h"
/**
 *	\namespace Const
 *	\brief 常用矩阵单位常量定义
 */
namespace Const{

	/** \var 3,1矩阵初始化数组 */
	extern double arr31_001000[];
	/** \var Matrix(3,3)矩阵初始化数组 */
	extern double arr33[];
	
	/** \var Matrix(3,1)[0,0,1]' */
	extern const cv::Mat mat31_001;
	/** \var Matrix(3,1)[0,1,0]' */
	extern const cv::Mat mat31_010;
	/** \var Matrix(3,1)[1,0,0]' */
	extern const cv::Mat mat31_100;
	/** \var Matrix(3,1)[0,0,0]' */
	extern const cv::Mat mat31_000;
	/** \var Matrix(3,1)[1,1,1]' */
	extern const cv::Mat mat31_111;

	/** \var Matrix(3,3) [0,0,0;0,0,0;0,0,0] */
	extern const cv::Mat mat33_000;
	/** \var Matrix(3,3) [1,0,0;0,1,0;0,0,1] */
	extern const cv::Mat mat33_111;

	extern const cv::Point3d pnt3d_000;
	extern const cv::Point3d pnt3d_001;
	extern const cv::Point3d pnt3d_010;
	extern const cv::Point3d pnt3d_100;
	extern const cv::Point3d pnt3d_111;
};

#endif