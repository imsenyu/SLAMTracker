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
	matScale.create(cv::Size(4000,300), CV_8UC3);
	matScale = cv::Scalar(255,255,255);
	cv::line(matScale, cv::Point2i(0, 100), cv::Point2i(4000, 100), cv::Scalar(128, 128, 128));
	for (int i = 0; i < 40; i++) {
		cv::line(matScale, cv::Point2i(i*100,0), cv::Point2i(i*100,300), cv::Scalar(128, 128, 128));
	}
	matCanvas.create(cv::Size(800, 800), CV_8UC3);
	matCanvas = cv::Scalar(255, 255, 255);

	//Dir 需要根据Pose 中的来确定，或者在pose画图时做旋转修正
	gPose = initPose;
	gPointBase = cv::Point2f(400, 500);

	// 绘制初始点
	cv::circle(matCanvas, gPointBase + cv::Point2f(gPose.pos.x, -gPose.pos.z), 2, cv::Scalar(-1));
	if (ptrPoseHelper != NULL && ptrPoseHelper->inited())
		logPose(gPose, gPose);

	cv::imshow(cv::format("%s-%s", "Canvas", CFG_sDataName.c_str()), matCanvas);
	cv::imshow(cv::format("%s-%s","Scale", CFG_sDataName.c_str()), matScale);
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
	//cv::Mat curDir3 = curPose.dir3 * Const::mat31_001;
	cv::line(matTmpCanvas,
		gPointBase + CFG_dDrawFrameStep*cv::Point2f(curPose.pos.x, -curPose.pos.z),
		gPointBase + CFG_dDrawFrameStep*cv::Point2f(curPose.pos.x, -curPose.pos.z) + 10.0f*CFG_dDrawFrameStep * cv::Point2f(curPose.dir.x, -curPose.dir.z),
		cv::Scalar(255, 0, 0));
	//cv::line(matTmpCanvas,
	//	gPointBase + CFG_dDrawFrameStep*cv::Point2f(curPose.pos.x, -curPose.pos.z),
	//	gPointBase + CFG_dDrawFrameStep*cv::Point2f(curPose.pos.x, -curPose.pos.z) + 10.0f*CFG_dDrawFrameStep * cv::Point2f(curDir3.at<double>(0, 0), -curDir3.at<double>(2, 0)),
	//	cv::Scalar(255, 0, 0));

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
	cv::line(matScale, cv::Point2i(curPose.idxImg, 0), cv::Point2i(curPose.idxImg, (int)(cv::norm(curPose.pos-prePose.pos) * 100)), cv::Scalar(0, 0, 0));
	cv::circle(matScale, cv::Point2i(curPose.idxImg, 100 * cv::norm(ptrPoseHelper->getPosition(curPose.idxImg, prePose.idxImg))), 1, cv::Scalar(255, 0, 255), 1);
	cv::flip(matScale, tmpScale, 0);
	cv::imshow(cv::format("%s-%s", "Scale", CFG_sDataName.c_str()), tmpScale);

	if (curPose.idxImg - prePose.idxImg > 1) {
		printf("Log: %d,%d\n", curPose.idxImg, prePose.idxImg);
	}

	for (int i = 0; i < nFrame; i++) {
		double tx, ty, tz;
		tx = (curPose.pos.x - prePose.pos.x)*(1)*(1.0f / nFrame);// +prePose.pos.x,
		ty = (curPose.pos.y - prePose.pos.y)*(1)*(1.0f / nFrame);// +prePose.pos.y,
		tz = (curPose.pos.z - prePose.pos.z)*(1)*(1.0f / nFrame); //+prePose.pos.z;
		//cv::Mat tDir(3, 1, CV_64FC1), oDir(3, 1,  CV_64FC1);
		//tDir.at<double>(0,0) = (curPose.dir.x - prePose.dir.x)*(i + 1)*(1.0f / nFrame) + prePose.dir.x;
		//tDir.at<double>(1, 0) = (curPose.dir.y - prePose.dir.y)*(i + 1)*(1.0f / nFrame) + prePose.dir.y;
		//tDir.at<double>(2, 0) = (curPose.dir.z - prePose.dir.z)*(i + 1)*(1.0f / nFrame) + prePose.dir.z;

		//oDir.at<double>(0, 0) = prePose.dir.x;
		//oDir.at<double>(1, 0) = prePose.dir.y;
		//oDir.at<double>(2, 0) = prePose.dir.z;

		//tDir *= (1.0f / cv::norm(tDir));

		//cv::Mat tRot;
		//Utils::getRodriguesRotation(tDir, tRot, oDir);
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