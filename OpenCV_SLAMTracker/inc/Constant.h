#ifndef CONSTANT_H_INCLUDED
#define CONSTANT_H_INCLUDED

#include <opencv.hpp>
/**
 *	\namespace Const
 *	\brief 常用矩阵单位常量定义
 */
namespace Const{
	/** \enum 运动估算错误类型 */
	class Error {
	public:
		typedef enum {
			OK = 0,
			LimitPOINT = 0x1,
			DefaultSCALE = 0x2,
			LimitROT = 0x4,
			LimitSCALEDIFF = 0x8,
			LimitSCALEPEAK = 0x10
		} Type;
	protected:
		Type type;
	public:
		Error(int t = 0) : type((Type)t) {}
		int get() const { return (int)type; }
		void set(int val, bool flag = true);
		void set(Const::Error val, bool flag = true) { set((int)val.type, flag); }
	};

	/** \var 3,1矩阵初始化数组 */
	extern double arr31_001000[];
	/** \var Matrix(3,3)矩阵初始化数组 */
	extern double arr33[];
	
	/** \var Matrix(3,1)[0,0,1]^T */
	extern const cv::Mat mat31_001;
	/** \var Matrix(3,1)[0,1,0]^T */
	extern const cv::Mat mat31_010;
	/** \var Matrix(3,1)[1,0,0]^T */
	extern const cv::Mat mat31_100;
	/** \var Matrix(3,1)[0,0,0]^T */
	extern const cv::Mat mat31_000;
	/** \var Matrix(3,1)[1,1,1]^T */
	extern const cv::Mat mat31_111;

	/** \var Matrix(3,3) [0,0,0;0,0,0;0,0,0] */
	extern const cv::Mat mat33_000;
	/** \var Matrix(3,3) [1,0,0;0,1,0;0,0,1] */
	extern const cv::Mat mat33_111;
	/** \var Matrix(3,3) [-1,0,0;0,1,0;0,0,-1] */
	extern const cv::Mat mat33_rot180;

	/** \var Point3 [0,0,0]^T */
	extern const cv::Point3d pnt3d_000;
	/** \var Point3 [0,0,1]^T */
	extern const cv::Point3d pnt3d_001;
	/** \var Point3 [0,1,0]^T */
	extern const cv::Point3d pnt3d_010;
	/** \var Point3 [1,0,0]^T */
	extern const cv::Point3d pnt3d_100;
	/** \var Point3 [1,1,1]^T */
	extern const cv::Point3d pnt3d_111;
};

#endif