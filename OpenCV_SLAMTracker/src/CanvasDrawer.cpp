#include "CanvasDrawer.h"
#include "PoseState.h"

CanvasDrawer::CanvasDrawer(int _ImgIdx) :
inited(false),
idxImgBegin(_ImgIdx),
ptrPoseHelper(NULL)
{

}


CanvasDrawer::~CanvasDrawer()
{

}

//bool CanvasDrawer::useGroundTruth(const std::string groundTruthPath) {
//
//	return ptrPoseHelper && ptrPoseHelper->setGroundTruth(groundTruthPath);
//}

//double CanvasDrawer::drawGroundTruth(int idxImgCur) {
//	if (fileGroundTruth.is_open() == false) return false;
//
//	if (idxGroundTruth == 0) {
//		double arrInitPose[12];
//		for (int idx = 0; idx < 12; idx++)
//			fileGroundTruth >> arrInitPose[idx];
//		preGroundTruth.x = arrInitPose[3];
//		preGroundTruth.y = arrInitPose[7];
//		preGroundTruth.z = arrInitPose[11];
//		idxGroundTruth++;
//	}
//
//	cv::Point3d curPoint;
//	double arrPose[12];
//	for (int cnt = 0; cnt < iterCnt; cnt++) {
//		idxGroundTruth++;
//		for (int idx = 0; idx < 12; idx++)
//			fileGroundTruth >> arrPose[idx];
//
//		//进行GPS 初速度方向修正为(0,0,1) 
//		//if (cntGroundTruth == 2) {
//		//	cv::Mat gtInitDir(3, 1, CV_64FC1);
//		//	gtInitDir.at<double>(0, 0) = arrPose[3];
//		//	gtInitDir.at<double>(1, 0) = arrPose[7];
//		//	gtInitDir.at<double>(2, 0) = arrPose[11];
//
//		//	Utils::getRodriguesRotation(gtInitDir, matRotGroundTruthFix);
//		//}
//	}
//	
//	curPoint.x = arrPose[3] - preGroundTruth.x;
//	curPoint.y = arrPose[7] - preGroundTruth.y;
//	curPoint.z = arrPose[11] - preGroundTruth.z;
//
//	double moveDist = cv::norm(curPoint - gGroundTruthPos);
//
//	gGroundTruthPos = curPoint;
//
//	//cv::Mat curPointToMat(3, 1, CV_64FC1);
//	//curPointToMat.at<double>(0, 0) = curPoint.x;
//	//curPointToMat.at<double>(1, 0) = curPoint.y;
//	//curPointToMat.at<double>(2, 0) = curPoint.z;
//
//	//curPointToMat = matRotGroundTruthFix.t() * curPointToMat;
//	//curPoint = cv::Point3d((cv::Vec < double, 3>)curPointToMat);
//
//	cv::circle(canvas, gPointBase + CFG_dDrawFrameStep* cv::Point2f(curPoint.x, -curPoint.z), 1, cv::Scalar(255, 0, 255), 1);
//
//
//	return moveDist;
//}

bool CanvasDrawer::setLogPath(const std::string& recordFilePath) {
	if (recordFilePath.length() > 0) {
		fileTraceRecord.open(recordFilePath, std::ios_base::out);
	}
	return fileTraceRecord.is_open();
}

void CanvasDrawer::initAnimate(PoseState& initPose) {
	
	// 画布相关定义
	matCanvas.create(cv::Size(800, 800), CV_8UC3);
	matCanvas = cv::Scalar(255, 255, 255);

	//Dir 需要根据Pose 中的来确定，或者在pose画图时做旋转修正
	gPose = initPose;
	gPointBase = cv::Point2f(400, 500);

	// 绘制初始点
	cv::circle(matCanvas, gPointBase + cv::Point2f(gPose.pos.x, -gPose.pos.z), 2, cv::Scalar(-1));

	cv::imshow("Canvas", matCanvas);
	cv::waitKey(100);
	inited = true;
}


void CanvasDrawer::drawCanvas(PoseState& curPose, bool _isTruth){

	//画连线 gPose->curPose
	cv::line(matCanvas, 
		gPointBase + CFG_dDrawFrameStep*cv::Point2f(gPose.pos.x, -gPose.pos.z),
		gPointBase + CFG_dDrawFrameStep*cv::Point2f(curPose.pos.x, -curPose.pos.z),
		cv::Scalar(-1));

	
	//画新点
	cv::circle(matCanvas, gPointBase + CFG_dDrawFrameStep* cv::Point2f(curPose.pos.x, -curPose.pos.z), 1, cv::Scalar(-1));
	
	// 绘制GroundTruth路径
	if ( _isTruth &&  ptrPoseHelper && ptrPoseHelper->inited()) {
		cv::Point3d posGroundTruth = ptrPoseHelper->getPosition(curPose.idxImg, idxImgBegin);
		cv::circle(matCanvas, gPointBase + CFG_dDrawFrameStep*cv::Point2f(posGroundTruth.x, -posGroundTruth.z), 1, cv::Scalar(255, 0, 255));
	}

	//克隆一下 matCanvas 画一下方向
	cv::Mat matTmpCanvas = matCanvas.clone();
	cv::line(matTmpCanvas,
		gPointBase + CFG_dDrawFrameStep*cv::Point2f(curPose.pos.x, -curPose.pos.z),
		gPointBase + CFG_dDrawFrameStep*cv::Point2f(curPose.pos.x, -curPose.pos.z) + 10.0f*CFG_dDrawFrameStep * cv::Point2f(curPose.dir.x, -curPose.dir.z),
		cv::Scalar(255, 0, 0));

	//更新 gPose到新的坐标
	gPose = curPose;

	cv::imshow("Canvas", matTmpCanvas);
	cv::waitKey(1);
}

//void CanvasDrawer::drawAnimate(cv::Mat matR, cv::Mat matT, int preImgIdx, int curImgIdx, double transformScale) {
//	if (avaliable == false) {
//		initAnimate();
//	}
//
//	bool _isNotConsiderAxisY = CFG_bIsNotConsiderAxisY;
//
//	cv::Point3d prePointPos = gPointPos;
//
//	//Point3 转  Mat(3,1)
//	double arrLocalPos[] = { gPointPos.x, gPointPos.y, gPointPos.z };
//	double arrLocalDir[] = { gPointDir.x, gPointDir.y, gPointDir.z };
//	cv::Mat matLocalPos(3, 1, CV_64FC1, arrLocalPos),
//		matLocalDir(3, 1, CV_64FC1, arrLocalDir);
//
//	cv::Mat matTmpRotate;
//	Utils::getRodriguesRotation(matLocalDir, matTmpRotate);
//	printf("==============Old Direction==============\n");
//	std::cout << matLocalDir << std::endl;
//	cv::Scalar color;
//	if (transformScale > std::abs(CFG_dScaleRatioLimitBottom) && transformScale < std::abs(CFG_dScaleRatioLimitTop)) {
//		transNorm = 1.65f / transformScale;
//		color = cv::Scalar(-1);
//	}
//	else if (transformScale == -1.0f) {
//		transNorm = 1.0f;
//		color = cv::Scalar(-1);
//	}
//	else {
//		transNorm = CFG_dScaleRatioErrorDefault;
//		color = cv::Scalar(0,0,255);
//	}
//
//
//	// 画标准解
//	double distGroundTruth = drawGroundTruth(matCanvas, curImgIdx - preImgIdx);
//	if ( CFG_bIsUseGroundTruthDistance) {
//		transNorm = distGroundTruth;
//	}
//
//	//Transform Matrix 生效，平移变换
//	matLocalPos = matLocalPos + matTmpRotate * matT * CFG_dDrawFrameStep*transNorm;
//
//	gPointPos = cv::Point3d((cv::Vec<double, 3>)matLocalPos);
//
//	//记录坐标 平移矩阵
//	if (fileTraceRecord.is_open()) {
//		double degT = Utils::getRodriguesRotation(matT, cv::Mat());
//		double arrOne[] = { 0, 0, 1 };
//		cv::Mat matOne(3, 1, CV_64FC1, arrOne);
//		matOne = matR*matOne;
//		double degR = Utils::getRodriguesRotation(matOne, cv::Mat());
//		fileTraceRecord.sync_with_stdio(false);
//		fileTraceRecord << curImgIdx << " " << gPointPos.x << " " << gPointPos.z << " " << gPointPos.y << " " << degT << " " << transformScale << degR << std::endl;
//	}
//
//	//图上绘制新坐标点，和移动线路
//	cv::circle(matCanvas, gPointBase + cv::Point2f(gPointPos.x, -gPointPos.z), transNorm == CFG_dScaleRatioErrorDefault ? 3 : 1, color);// cv::Scalar(-1));
//	//cv::line(matCanvas, gPointBase + cv::Point2f(prePointPos.x, -prePointPos.z), gPointBase + cv::Point2f(gPointPos.x, -gPointPos.z), cv::Scalar(-1));
//
//
//	double arrVector001[] = { 0, 0, 1 };
//	cv::Mat matUnitVector001(3, 1, CV_64FC1, arrVector001);
//	cv::Mat matTmpDir(3, 1, CV_64FC1);
//	cv::Point3d pointTmpDir[2];
//
//	cv::Mat matCorrectRotXZ(3, 1, CV_64FC1),
//		matCorrectRotation(3, 3, CV_64FC1),
//		matCorrectDir(3, 1, CV_64FC1);
//
//	for (int i = 0; i < 1; i++) {
//		//try two matR;
//
//		matTmpDir.at<double>(0, 0) = gPointDir.x;
//		matTmpDir.at<double>(1, 0) = gPointDir.y;
//		matTmpDir.at<double>(2, 0) = gPointDir.z;
//
//		if (_isNotConsiderAxisY) {
//			//只考虑 XZ平面旋转
//			matCorrectRotXZ = matR * matUnitVector001;
//
//			matCorrectRotXZ.at<double>(1, 0) = 0.0f;
//			matCorrectRotXZ = matCorrectRotXZ / cv::norm(matCorrectRotXZ);
//			Utils::getRodriguesRotation(matCorrectRotXZ, matCorrectRotation);
//			matCorrectDir = matCorrectRotation* matLocalDir;
//			//Trick
//			matCorrectDir.at<double>(1, 0) = 0.0f;
//			matTmpDir = matCorrectDir / cv::norm(matCorrectDir);
//		}
//		else {
//			matTmpDir = matR * matLocalDir;
//		}
//
//		pointTmpDir[i] = cv::Point3d((cv::Vec<double, 3>)matTmpDir);
//
//		printf("==============New Direction==============\n");
//		std::cout << pointTmpDir[i] << std::endl;
//
//	}
//
//	gPointDir = pointTmpDir[0];
//
//	cv::Mat matTmpCanvas = matCanvas.clone();
//	cv::line(matTmpCanvas, gPointBase + cv::Point2f(gPointPos.x, -gPointPos.z), gPointBase + cv::Point2f(gPointPos.x, -gPointPos.z) + 20.0f*cv::Point2f(gPointDir.x, -gPointDir.z), cv::Scalar(255, 0, 0));
//	cv::imshow("Canvas", matTmpCanvas);
//	cv::waitKey(1);
//}