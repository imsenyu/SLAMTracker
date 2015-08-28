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


bool CanvasDrawer::setLogPath(const std::string& _recordFilePath) {
	if (_recordFilePath.length() > 0) {
		sPathRecordFile = _recordFilePath;
		fileTraceRecord.open(sPathRecordFile, std::ios_base::out);
	}
	return fileTraceRecord.is_open();
}

void CanvasDrawer::initAnimate(PoseState& initPose) {
	
	// 画布相关定义,车速度画布
	matScaleCanvas.create(cv::Size(4000,300), CV_8UC3);
	matScaleCanvas = cv::Scalar(255,255,255);
	//画线,表示(1.0m每0.1s)
	cv::line(matScaleCanvas, cv::Point2i(0, 100), cv::Point2i(4000, 100), cv::Scalar(128, 128, 128));
	//画线,表示(100帧)
	for (int i = 0; i < 40; i++) {
		cv::line(matScaleCanvas, cv::Point2i(i*100,0), cv::Point2i(i*100,300), cv::Scalar(128, 128, 128));
	}
	// 画布相关定义,行车路径(x,-z)轴
	matPathCanvas.create(cv::Size(800, 800), CV_8UC3);
	matPathCanvas = cv::Scalar(255, 255, 255);

	//设定初始位姿
	gPose = initPose;
	//设定画布绘制偏移
	gPointBase = cv::Point2f(400, 500);

	// 绘制初始点
	cv::circle(matPathCanvas, gPointBase + cv::Point2f(gPose.pos.x, -gPose.pos.z), 2, cv::Scalar(-1));
	
	// 记录路径点
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


	cv::imshow(cv::format("%s-%s", "Canvas", CFG_sDataName.c_str()), matTmpCanvas);
	//刷新绘图
	cv::waitKey(1);

	logPose(curPose, gPose);
	drawScale(curPose, gPose);

	//每隔100输出一次图像,方便查看
	if (curPose.idxImg/100 != gPose.idxImg/100) {
		cv::imwrite(sPathRecordFile + ".png", matTmpCanvas);
	}

	//更新 gPose到新的坐标
	gPose = curPose;
}


void CanvasDrawer::logPose(PoseState& curPose, PoseState& prePose) {
	//中间差的帧数
	int nFrame = curPose.idxImg - prePose.idxImg;

	for (int i = 0; i < nFrame; i++) {
		double tx, ty, tz;
		
		// 两帧之间位移
		tx = (curPose.pos.x - prePose.pos.x)*(1)*(1.0f / nFrame);
		ty = (curPose.pos.y - prePose.pos.y)*(1)*(1.0f / nFrame);
		tz = (curPose.pos.z - prePose.pos.z)*(1)*(1.0f / nFrame);
		
		qDist.push_back(cv::norm(cv::Point3d(tx, ty, tz)));

		//从0到现在的位移
		tx = (curPose.pos.x - prePose.pos.x)*(i + 1)*(1.0f / nFrame) + prePose.pos.x;
		ty = (curPose.pos.y - prePose.pos.y)*(i + 1)*(1.0f / nFrame) + prePose.pos.y;
		tz = (curPose.pos.z - prePose.pos.z)*(i + 1)*(1.0f / nFrame) + prePose.pos.z;

		//按格式输出, R表示旋转矩阵,T位移向量
		// |R,R,R,T|
		// |R,R,R,T|
		// |R,R,R,T|
		fileTraceRecord <<
			curPose.dir3.at<double>(0, 0) << " " << curPose.dir3.at<double>(0, 1) << " " << curPose.dir3.at<double>(0, 2) << " " << tx << " " <<
			curPose.dir3.at<double>(1, 0) << " " << curPose.dir3.at<double>(1, 1) << " " << curPose.dir3.at<double>(1, 2) << " " << ty << " " <<
			curPose.dir3.at<double>(2, 0) << " " << curPose.dir3.at<double>(2, 1) << " " << curPose.dir3.at<double>(2, 2) << " " << tz << std::endl;

	}

}

void CanvasDrawer::drawScale(PoseState& curPose, PoseState& prePose) {
	

	//Scale 画布绘制
	cv::Mat tmpScale;

	cv::line(matScaleCanvas, cv::Point2i(curPose.idxImg, 0), cv::Point2i(curPose.idxImg, (int)(cv::norm(curPose.pos - prePose.pos) * 100)), cv::Scalar(0, 0, 0));
	//绘制groundTruth
	if ( ptrPoseHelper != NULL && ptrPoseHelper->inited() )
		cv::circle(matScaleCanvas, cv::Point2i(curPose.idxImg, 100 * cv::norm(ptrPoseHelper->getPosition(curPose.idxImg, prePose.idxImg))), 1, cv::Scalar(255, 0, 255), 1);
	
	//画布翻转
	cv::flip(matScaleCanvas, tmpScale, 0);

	if (true) {
		//画出该curPose的 err
		for (int i = 0; i < 5; i++) {
			if ((1 << i) & curPose.getErrType()) {
				cv::line(tmpScale, cv::Point2f(curPose.idxImg, i * 10), cv::Point2f(curPose.idxImg, (i+1) * 10), cv::Scalar::all(-1));
			}
		}
	}

	cv::imshow(cv::format("%s-%s", "Scale", CFG_sDataName.c_str()), tmpScale);

}
