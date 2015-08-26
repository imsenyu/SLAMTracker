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

bool CanvasDrawer::setLogPath(const std::string& _recordFilePath) {
	if (_recordFilePath.length() > 0) {
		recordFilePath = _recordFilePath;
		fileTraceRecord.open(recordFilePath, std::ios_base::out);
	}
	return fileTraceRecord.is_open();
}

void CanvasDrawer::initAnimate(PoseState& initPose) {
	
	// 画布相关定义
	matScaleCanvas.create(cv::Size(4000,300), CV_8UC3);
	matScaleCanvas = cv::Scalar(255,255,255);
	cv::line(matScaleCanvas, cv::Point2i(0, 100), cv::Point2i(4000, 100), cv::Scalar(128, 128, 128));
	for (int i = 0; i < 40; i++) {
		cv::line(matScaleCanvas, cv::Point2i(i*100,0), cv::Point2i(i*100,300), cv::Scalar(128, 128, 128));
	}
	matPathCanvas.create(cv::Size(800, 800), CV_8UC3);
	matPathCanvas = cv::Scalar(255, 255, 255);

	//Dir 需要根据Pose 中的来确定，或者在pose画图时做旋转修正
	gPose = initPose;
	gPointBase = cv::Point2f(400, 500);

	// 绘制初始点
	cv::circle(matPathCanvas, gPointBase + cv::Point2f(gPose.pos.x, -gPose.pos.z), 2, cv::Scalar(-1));
	if (ptrPoseHelper != NULL && ptrPoseHelper->inited())
		logPose(gPose, gPose);

	cv::imshow(cv::format("%s-%s", "Canvas", CFG_sDataName.c_str()), matPathCanvas);
	cv::imshow(cv::format("%s-%s","Scale", CFG_sDataName.c_str()), matScaleCanvas);
	cv::waitKey(100);
	inited = true;
}


void CanvasDrawer::drawCanvas(PoseState& curPose, bool _isTruth){

	//画连线 gPose->curPose
	cv::line(matPathCanvas, 
		gPointBase + CFG_dDrawFrameStep*cv::Point2f(gPose.pos.x, -gPose.pos.z),
		gPointBase + CFG_dDrawFrameStep*cv::Point2f(curPose.pos.x, -curPose.pos.z),
		cv::Scalar(-1));

	
	//画新点
	cv::circle(matPathCanvas, gPointBase + CFG_dDrawFrameStep* cv::Point2f(curPose.pos.x, -curPose.pos.z), 1, cv::Scalar(-1));
	
	// 绘制GroundTruth路径
	if ( _isTruth &&  ptrPoseHelper && ptrPoseHelper->inited()) {
		cv::Point3d posGroundTruth = ptrPoseHelper->getPosition(curPose.idxImg, idxImgBegin);
		cv::circle(matPathCanvas, gPointBase + CFG_dDrawFrameStep*cv::Point2f(posGroundTruth.x, -posGroundTruth.z), 1, cv::Scalar(255, 0, 255));
	}

	//克隆一下 matCanvas 画一下方向
	cv::Mat matTmpCanvas = matPathCanvas.clone();
	
	cv::line(matTmpCanvas,
		gPointBase + CFG_dDrawFrameStep*cv::Point2f(curPose.pos.x, -curPose.pos.z),
		gPointBase + CFG_dDrawFrameStep*cv::Point2f(curPose.pos.x, -curPose.pos.z) + 10.0f*CFG_dDrawFrameStep * cv::Point2f(curPose.dir.x, -curPose.dir.z),
		cv::Scalar(255, 0, 0));

	//更新 gPose到新的坐标
	

	cv::imshow(cv::format("%s-%s", "Canvas", CFG_sDataName.c_str()), matTmpCanvas);
	cv::waitKey(1);
	if ( ptrPoseHelper != NULL && ptrPoseHelper->inited() )
		logPose(curPose, gPose);

	if (curPose.idxImg/100 != gPose.idxImg/100) {
		cv::imwrite(recordFilePath + ".png", matTmpCanvas);
	}

	gPose = curPose;
}


void CanvasDrawer::logPose(PoseState& curPose, PoseState& prePose) {
	int nFrame = curPose.idxImg - prePose.idxImg;
	cv::Mat tmpScale;
	//Scale 画布绘制
	cv::line(matScaleCanvas, cv::Point2i(curPose.idxImg, 0), cv::Point2i(curPose.idxImg, (int)(cv::norm(curPose.pos-prePose.pos) * 100)), cv::Scalar(0, 0, 0));
	cv::circle(matScaleCanvas, cv::Point2i(curPose.idxImg, 100 * cv::norm(ptrPoseHelper->getPosition(curPose.idxImg, prePose.idxImg))), 1, cv::Scalar(255, 0, 255), 1);
	cv::flip(matScaleCanvas, tmpScale, 0);
	cv::imshow(cv::format("%s-%s", "Scale", CFG_sDataName.c_str()), tmpScale);

	if (curPose.idxImg - prePose.idxImg > 1) {
		printf("Log: %d,%d\n", curPose.idxImg, prePose.idxImg);
	}

	for (int i = 0; i < nFrame; i++) {
		double tx, ty, tz;
		
		// 两帧之间位移
		tx = (curPose.pos.x - prePose.pos.x)*(1)*(1.0f / nFrame);
		ty = (curPose.pos.y - prePose.pos.y)*(1)*(1.0f / nFrame);
		tz = (curPose.pos.z - prePose.pos.z)*(1)*(1.0f / nFrame);
		
		qDist.push_back(cv::norm(cv::Point3d(tx, ty, tz)));

		tx = (curPose.pos.x - prePose.pos.x)*(i + 1)*(1.0f / nFrame) + prePose.pos.x;
		ty = (curPose.pos.y - prePose.pos.y)*(i + 1)*(1.0f / nFrame) + prePose.pos.y;
		tz = (curPose.pos.z - prePose.pos.z)*(i + 1)*(1.0f / nFrame) + prePose.pos.z;

		//fileTraceRecord <<
		//	cv::norm(cv::Point3d(tx, ty, tz)) << " " <<
		//cv::norm(ptrPoseHelper->getPosition(prePose.idxImg + i + 1, prePose.idxImg+i)) << std::endl;
		fileTraceRecord <<
			curPose.dir3.at<double>(0, 0) << " " << curPose.dir3.at<double>(0, 1) << " " << curPose.dir3.at<double>(0, 2) << " " << tx << " " <<
			curPose.dir3.at<double>(1, 0) << " " << curPose.dir3.at<double>(1, 1) << " " << curPose.dir3.at<double>(1, 2) << " " << ty << " " <<
			curPose.dir3.at<double>(2, 0) << " " << curPose.dir3.at<double>(2, 1) << " " << curPose.dir3.at<double>(2, 2) << " " << tz << std::endl;

	}

}
