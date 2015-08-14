#include "ScaleEstimator.h"


ScaleEstimator::ScaleEstimator() :ptrMotion(1)
{
}


ScaleEstimator::~ScaleEstimator()
{
}


bool ScaleEstimator::updateMotion(MotionState* _ptrCurMotion) {

	ptrMotion[0] = _ptrCurMotion;

	if (true) {
		printf("======ScaleEstimator======\n");
		printf("idxImg = (%d)\n", ptrMotion[0]->idxImg[0]);
	}

	return !!ptrMotion[0];
}

int ScaleEstimator::getPairPoints2() {
	bool _isLogData = false;
	bool _isUpdateData = true;

	auto& mapCur = ptrMotion[0]->mapPairPoints;
	for (int idx = 0; idx < 2; idx++) {
		matPointUVW[idx] = cv::Mat(3, mapCur.size(), CV_64FC1);
		matPointUVW[idx] = 1.0f;
	}

	int pairCnt = 0;
	for (auto& pair : mapCur) {
		auto & point1 = pair.first;
		auto & point0 = pair.second;
		if (_isUpdateData) {
			matPointUVW[0].at<double>(0, pairCnt) = point0.x;
			matPointUVW[0].at<double>(1, pairCnt) = point0.y;
		
			matPointUVW[1].at<double>(0, pairCnt) = point1.x;
			matPointUVW[1].at<double>(1, pairCnt) = point1.y;
			
		}
		pairCnt++;
	}

	return pairCnt;
}

//int ScaleEstimator::GetPairPoints3(int cols) {
//	bool _isLogData = false;
//	bool _isUpdateData = cols > 0;
//	auto& mapCur = ptrMotion[1]->mapPairPoints;
//	auto& mapPre = ptrMotion[0]->mapPairPoints;
//
//	if (_isUpdateData == true) {
//		for (auto& m : matPointUVW) {
//			m = cv::Mat(3, cols, CV_64FC1).clone();
//			m = 1.0f;
//		}
//	}
//	//清空原有映射对应
//	int pair3Cnt = 0;
//	for (auto& pair : ptrMotion[1]->mapPairPoints) {
//		auto & point2 = pair.first;
//		auto & point1 = pair.second;
//		if (mapPre.find(point1) != mapPre.end()) {
//			auto& point0 = mapPre[point1];
//			if (_isLogData) {
//				printf("3-Pair:\n");
//				std::cout <<
//					point2 << " " <<
//					point1 << " " <<
//					point0 << std::endl;
//			}
//
//			if (_isUpdateData) {
//				matPointUVW[0].at<double>(0, pair3Cnt) = point0.x;
//				matPointUVW[0].at<double>(1, pair3Cnt) = point0.y;
//
//				matPointUVW[1].at<double>(0, pair3Cnt) = point1.x;
//				matPointUVW[1].at<double>(1, pair3Cnt) = point1.y;
//
//				matPointUVW[2].at<double>(0, pair3Cnt) = point2.x;
//				matPointUVW[2].at<double>(1, pair3Cnt) = point2.y;
//			}
//
//			pair3Cnt++;
//		}
//		else {
//			if (_isLogData) {
//				printf("2-Pair:\n");
//				std::cout <<
//					point2 << " " <<
//					point1 << std::endl;
//			}
//
//		}
//	}
//	printf("Pair3: %d/%d\n", pair3Cnt, std::min(mapCur.size(), mapPre.size()));
//
//	return pair3Cnt;
//}

double ScaleEstimator::calcScaleRatio(int flag) {
	double retScale=0.0f;
	int sizePoint = matIntersection.cols;
	std::function<bool(const cv::Point3d&, const cv::Point3d&)> sortY =
		[](const cv::Point3d& a, const cv::Point3d& b)->
		bool {	return a.y > b.y;  };
	if (flag == 0) {
		//最普通的方法, 取y最大
		retScale = -100.0f;
		std::vector<cv::Point3d> vecPoints;
		for (int idxPoint = 0; idxPoint < sizePoint; idxPoint++) {
			cv::Point3d vecTmp((cv::Vec<double, 3>)matIntersection.col(idxPoint));
			//约束路面宽度在 7.0以内
			if (vecTmp.y > 0 && std::abs(vecTmp.x) < 6.0f )
				vecPoints.push_back(vecTmp);
		}

		sizePoint = vecPoints.size();
		std::sort(vecPoints.begin(), vecPoints.end(), sortY);
		/*	printf("========Y list========\ng=[");
			for (int i = 0; i < sizePoint - 2; i++) {
			printf("%f,%f,%f;\n", vecPoints[i].x, vecPoints[i].y, vecPoints[i].z);
			}
			printf("]\nplot3(g(:,1),g(:,2),g(:,3),'r.');\n");
			printf("========Y list========END\n");*/
		for (int i = 0; i < sizePoint - 2; i++) {
			if (std::abs(vecPoints[i].y - vecPoints[i + 1].y) / vecPoints[i].y < 0.03f && 
				std::abs(vecPoints[i+1].y - vecPoints[i + 2].y) / vecPoints[i+1].y < 0.03f
				) {
				cv::Point3d normal = vecPoints[i].cross(vecPoints[i + 1]);
				normal.x /= cv::norm(normal);
				normal.y /= cv::norm(normal);
				normal.z /= cv::norm(normal);
				normal = normal.y < 0 ? -normal : normal;
				retScale = std::abs(vecPoints[i].dot(normal));
				break;
			}
		}
		for (int i = 0; i < sizePoint - 2; i++) {
			if (std::abs(vecPoints[i].y - vecPoints[i + 1].y) / vecPoints[i].y < 0.03f 
				) {
				retScale = vecPoints[i].y;
				break;
			}
		}
	}
	//else {
	//	//换一种，先按照y排序，然后三个三个 算平面，选择比较平的那些平面.
	//	// 三个点算 法向量 ，y轴取负
	//	//考虑当前的 matDir,选择与其 点乘 的double作为 map.first
	//	// 按照 map< double点乘 方向, vec<Point>> 存储 
	//	//map从小到大排序，计算 点到平面距离，依次验证容忍度，
	//	// 还要加入对跳帧的容忍范围,单帧容忍 1.65/x 后的 0.2, 多帧 0.2+0.1*(n-1)
	//	std::vector<cv::Point3d> vecPoints;
	//	
	//	for (int idxPoint = 0; idxPoint < sizePoint; idxPoint++) {
	//		cv::Point3d vecTmp((cv::Vec<double, 3>)matIntersection.col(idxPoint));
	//		if (vecTmp.y > 0)
	//			vecPoints.push_back(vecTmp);
	//	}
	//	sizePoint = vecPoints.size();
	//	//排序y轴
	//	

	//	std::sort(vecPoints.begin(), vecPoints.end(), sortY);

	//	//printf("=====输出所有点=====\n");
	//	//for (auto&p : vecPoints) std::cout << p << std::endl;


	//	cv::Point3d pointDir((cv::Vec<double, 3>)matDir);
	//	std::map<double, double> mapDotDist;
	//	//std::map<double, std::vector<cv::Point3d>> mapDotPoint;

	//	//计算点乘结果, 并插入map
	//	//std::deque<cv::Point3d> cachePoint;
	//	for (int idxPoint = 0; idxPoint < sizePoint - 2; idxPoint++) {
	//		cv::Point3d normal = (vecPoints[idxPoint] - vecPoints[idxPoint + 1]).cross(vecPoints[idxPoint] - vecPoints[idxPoint+2]);
	//		//法向量取负
	//		normal = normal.y < 0.0f ? -normal : normal;
	//		normal.x /= cv::norm(normal);
	//		normal.y /= cv::norm(normal);
	//		normal.z /= cv::norm(normal);

	//		//printf("点%d-%d-%d\n法向量\n", idxPoint, idxPoint + 1, idxPoint + 2);
	//		//std::cout << normal << std::endl;

	//		double dotResult = normal.dot(pointDir);
	//		double dist = vecPoints[idxPoint].dot(normal);

	//		mapDotDist.insert(std::make_pair(dist, dotResult));
	//		//printf("dist=%f, dot=%f\n", dist, dotResult);

	//		//vecTmp.push
	//		/*while (cachePoint.size() >= 3){
	//			cachePoint.pop_front();
	//		}
	//		while (cachePoint.size() < 3){
	//			cachePoint.push_back(vecPoints[idxPoint + cachePoint.size()]);
	//		}
	//		mapDotPoint.insert(std::make_pair(dotResult, std::vector<cv::Point3d>(cachePoint.begin(), cachePoint.end())));*/
	//	}

	//	//map从大到小
	//	printf("Map 遍历\n");
	//	for (auto riter = mapDotDist.rbegin(); riter != mapDotDist.rend(); riter++) {
	//		double dist = riter->first,
	//			dot = riter->second;
	//		//printf("dist=%f, dot=%f\n", dist, dot);
	//		if (std::abs(dot) <  1 - std::cos(acos(-1) / 180.0f*5.0f)) {
	//			retScale = dist;
	//			return retScale;
	//		}
	//	}
	//	for (auto riter = mapDotDist.rbegin(); riter != mapDotDist.rend(); riter++) {
	//		double dist = riter->first,
	//			dot = riter->second;
	//		//printf("dist=%f, dot=%f\n", dist, dot);
	//		if (std::abs(dot) < 1 - std::cos(acos(-1) / 180.0f*10.0f)) {
	//			retScale = dist;
	//			break;
	//		}
	//	}

	//}


	return retScale;

}

double ScaleEstimator::computeScaleTransform() {
	bool _isLogData = true ;
	bool _isWriteInfomation = false;
	
	printf("=========Scale Compute=========\n");
	//Need at least 3 Frames' Data
	// Means 2 Motions' Data
	if (ptrMotion[0] == NULL) return 0;
	int pair2Cnt = ptrMotion[0]->mapPairPoints.size();
	if (pair2Cnt < 10) return 0;

	//////////////////////////////////////////////////////////////////////////
	getPairPoints2();
	cv::Mat transMask;
	transMask = transformIn2Coord(pair2Cnt, 0,1);

	double retScale = calcScaleRatio(0);

	// 1.65/scale 变化量修正（速度变化修正）
	//double ScaleDelta = 1.65 / retScale - 1.65 / preTransScale;
	//if (preTransScale > 0 && retScale > 0 && std::abs(ScaleDelta) > CFG_dScaleInvIncreaseDiffLimit) {
	//	retScale = 1.65 / (1.65 / preTransScale + CFG_ScaleInvIncreaseDiffLimit*(ScaleDelta > 0 ? 1.0f : -1.0f));
	//}

	if (_isWriteInfomation) {

		std::fstream fs;
		int i0 = ptrMotion[0]->idxImg[0], i1 = ptrMotion[0]->idxImg[1];
		fs.open(cv::format("./Velocity/m%06d_%06d.m", i0, i1), std::ios_base::out);

		fs << cv::format("matR_%06d_%06d=", i0, i1) << ptrMotion[0]->matR << ";" << std::endl;
		fs << cv::format("matT_%06d%_%06d=", i0, i1) << ptrMotion[0]->matT << ";" << std::endl;

		fs << cv::format("mInter_%06d%_%06d=", i0, i1) << matIntersection << ";" << std::endl;

		fs << "plot3(" << cv::format("mInter_%06d%_%06d", i0, i1) << "(1,:)'," << cv::format("mInter_%06d%_%06d", i0, i1) << "(2,:)'," << cv::format("mInter_%06d%_%06d", i0, i1) << "(3,:)','r.');" << std::endl;

		fs.close();
	}


	return retScale;
}

double ScaleEstimator::calcLineIntersection(cv::Mat d1, cv::Mat d2, cv::Mat p2, cv::Mat& ip1) {
	cv::Mat p1(3, 1, CV_64FC1);	p1 = 0.0f;
	cv::Mat ret(3, 1, CV_64FC1);
	double t = 0.0f;
	cv::Mat normalVector = d1.cross(d2);
	t = (p2 - p1).cross(d2).dot(normalVector) / cv::norm(normalVector) / cv::norm(normalVector);
	ret = p1 + t*d1;


	ip1 = ret.clone();
	return cv::norm(normalVector.dot(p2 - p1)) / cv::norm(normalVector);
}


cv::Mat ScaleEstimator::transformIn2Coord(int pntNum, int preIdx, int curIdx) {
	bool _isLogData = false;
 
	// 对 0,1坐标系 计算点的三维位置
	cv::Mat matPosOrign[3];
	for (auto& m : matPosOrign) m = cv::Mat(3, 1, CV_64FC1).clone();
	matPosOrign[preIdx] = 0.0f;
	matPosOrign[curIdx] = matPosOrign[preIdx] + ptrMotion[preIdx]->matT;

	/*
	7.188560000000e+02 0.000000000000e+00 6.071928000000e+02
	0.000000000000e+00 7.188560000000e+02 1.852157000000e+02
	0.000000000000e+00 0.000000000000e+00 1.000000000000e+00
	*/
	double Fu = 718.856f, Fv = 718.856f, Cu = 607.1928f, Cv = 185.2157f;
	double arrCameraInverse[] = { 1 / Fu, 0, -Cu / Fu, 0, 1 / Fv, -Cv / Fv, 0, 0, 1 };
	cv::Mat matCameraInv(3, 3, CV_64FC1, arrCameraInverse);


	//0, 1 直线方向
	//先摄像头转换, 再全部转换到0坐标系
	matDirXYZ[preIdx] = matCameraInv*matPointUVW[preIdx];
	matDirXYZ[curIdx] = ptrMotion[preIdx]->matR   *  matCameraInv * matPointUVW[curIdx];

	//计算 0,1 在0坐标系下的交点
	matIntersection = cv::Mat(3, pntNum, CV_64FC1);
	matIntersection = 0.0f;

	cv::Mat tmp;
	cv::Mat distMask(pntNum, 1,CV_64FC1);
	distMask = 0;
	for (int idxPnt = 0; idxPnt < pntNum; idxPnt++) {

		double dist = calcLineIntersection(
			matDirXYZ[preIdx].col(idxPnt),
			matDirXYZ[curIdx].col(idxPnt),
			matPosOrign[curIdx],
			tmp
			);
		distMask.at<double>(idxPnt, 0) = dist;
		for (int row = 0; row < 3; row++)
			matIntersection.at<double>(row, idxPnt) = tmp.at<double>(row, 0);
	}

	if (_isLogData) {
		//0,1原点,在0坐标系中
		std::cout << matPosOrign[preIdx] << std::endl;
		std::cout << matPosOrign[curIdx] << std::endl;


		//输出0,1图像坐标点
		printf("0,1 图像中的坐标点 - 0\n");
		std::cout << matPointUVW[preIdx] << std::endl;
		printf("0,1 图像中的坐标点 - 1\n");
		std::cout << matPointUVW[curIdx] << std::endl;

		//输出0,1方向 在0坐标系中的
		printf("0,1方向 在0坐标系中的 - 0\n");
		std::cout << matDirXYZ[preIdx] << std::endl;
		printf("0,1方向 在0坐标系中的 - 1\n");
		std::cout << matDirXYZ[curIdx] << std::endl;

		//输出0,1的交点，在0坐标系下
		printf("0,1数据 在0坐标系中的 交点\n");
		std::cout << matIntersection << std::endl;

	}

	return distMask.clone();
}

//double ScaleEstimator::ComputeScale() {
//
//	if (ptrMotion.size() != 2) return 0.0f;
//
//	//////////////////////////////////////////////////////////////////////////
//	// 更新 UVW坐标
//	auto& mapCur = ptrMotion[1]->mapPairPoints;
//	int cols = mapCur.size();
//
//	for (int i = 1; i <= 2; i++) {
//		matPointUVW[i] = cv::Mat(3, cols, CV_64FC1).clone();
//		matPointUVW[i] = 1.0f;
//	}
//
//	int pair3Cnt = 0;
//	for (auto& pair : mapCur) {
//		auto& point1 = pair.first;
//		auto& point2 = pair.second;
//
//		matPointUVW[1].at<double>(0, pair3Cnt) = point1.x;
//		matPointUVW[1].at<double>(1, pair3Cnt) = point1.y;
//
//		matPointUVW[2].at<double>(0, pair3Cnt) = point2.x;
//		matPointUVW[2].at<double>(1, pair3Cnt) = point2.y;
//
//		pair3Cnt++;
//	}
//	TransformIn2Coord(pair3Cnt, 1, 2);
//
//	double maxY = -100.0f;
//	for (int i = 0; i < cols; i++) {
//		if (matIntersection[1].at<double>(2, i) > 0)
//			maxY = std::max(maxY, matIntersection[1].at<double>(1, i));
//	}
//	
//
//
//	return maxY;
//}