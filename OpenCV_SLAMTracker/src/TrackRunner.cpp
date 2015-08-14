#include "TrackRunner.h"
#include "CanvasDrawer.h"
#include "FrameParser.h"
#include "ScaleEstimator.h"

TrackRunner::TrackRunner(int _ImgBeginIdx, int _ImgEndIdx):
cDrawer(),
idxImgBegin(_ImgBeginIdx), idxImgEnd(_ImgEndIdx),
vecPoses(_ImgEndIdx+1), vecMotionLinks(_ImgEndIdx+1)
{
}


TrackRunner::~TrackRunner()
{
	//TODO: 依次删除 两个FeatureState* 记录
}


void TrackRunner::initFirstFrame() {

	// 设定输出配置
	cDrawer.setLogPath(cv::format("./Output/coordOutput_%s.txt", Utils::getTimeNow().c_str()));

	// 在内部初始化 PoseHelper
	cDrawer.useGroundTruth(CFG_sPathPoseGroutTruth);

	// 初始化画布对象
	cDrawer.initAnimate();

	//先算一次
	FeatureState* ptrCurFeature = new FeatureState(idxImgBegin);
	ptrCurFeature->loadImage(idxImgBegin);

	FrameParser::detectExtractFeatures(CFG_iMaxFeatures, *ptrCurFeature);
	deqFrameFeatures.push_back(ptrCurFeature);

	idxImgCur = idxImgBegin + 1;
}

int TrackRunner::runKeyStep() {
	printf("///////////////////////////////////////////\n");
	printf("// idxImgCur: %d:\n",idxImgCur);
	printf("///////////////////////////////////////////\n");
	TIME_BEGIN("MainLoop");

		TIME_BEGIN("FeatureDetect");
		// 读取当前帧数据
		FeatureState* ptrCurFeature = new FeatureState(idxImgCur);
		FrameParser::detectExtractFeatures(CFG_iMaxFeatures, *ptrCurFeature);
		TIME_END("FeatureDetect");
	// [点] 特定帧数据记录 int<-> 当前位置，方向, 全记录
	// [点] 特定帧数据记录 int<-> 特征点 vecKP, vecP, matDescriptor->存储数据量在 2000K 一个,部分记录
	// [边] 帧间数据记录 用 vector< int> --- 新map<int,旧>> 存matR, matT, Scale 全记录

	// 扩展: [关键帧]池 和 [最近连续可用帧]池, 每产生一帧，如果往[最近]扔，[最近]是个定长队列, 如果超了，并且距离够了 扔进 [关键帧]里面去，否则销毁掉

	//当前应该跑 nextStep那一帧

	// 先从历史数据中选出 K 个最近参考帧 + ALL 关键帧进行 match
	// 选取一个帧ptr 序列
	std::vector<FeatureState*> vecKeyList;
	vecKeyList = selectKeySequence(idxImgCur);

	// 调用 FrameParser 分别处理 参考帧<->当前帧 的 matR和MatT
	std::vector<MotionState> vecCurMotions( vecKeyList.size() );
	ScaleEstimator sEstimator;

	TIME_BEGIN(cv::format("Match&Motion[%d]", vecKeyList.size()));
	for (int idx = 0; idx < vecKeyList.size();idx++) {
		FeatureState* ptrFeature = vecKeyList[idx];
		FrameParser fparser(ptrFeature, ptrCurFeature);
		fparser.matchFeaturePoints();
		fparser.validPointsByOpticalFlow(1.0f);

		//计算对极几何
		vecCurMotions[idx].idxImg[0] = ptrFeature->idxImg;
		vecCurMotions[idx].idxImg[1] = ptrCurFeature->idxImg;
		bool motionStatus = fparser.computeMotion(vecCurMotions[idx]);
		//计算尺度
		if (motionStatus == false) continue;

		sEstimator.updateMotion(&vecCurMotions[idx]);
		double scale = sEstimator.computeScaleTransform();
		vecCurMotions[idx].scale = scale;
		//放入vecCurMotions
	}
	TIME_END(cv::format("Match&Motion[%d]", vecKeyList.size()));


	//按照一定规则过滤 vecCurMotions,凡是过滤掉的，都要delete Motion
	int cntValid = filterMotions(vecCurMotions);

	// 数据点崩了跳过 delete CurFeature
	if (cntValid < 1) {
		delete ptrCurFeature;
		//TODO: 跳帧
	}
	else {
		TIME_BEGIN("PoseEstimate");

		
		std::vector<PoseState> vecEstiPoses(vecCurMotions.size());
		for (int idx = 0; idx < vecCurMotions.size(); idx++) {
			MotionState& curMotion = vecCurMotions[idx];
			if (curMotion.inited == true) {
				//从 vecCurMotions[idx].idxImg[0]  ->  [1]
				//执行一个运算，得到一个 PoseState, 
				vecEstiPoses[idx] = vecPoses[curMotion.idxImg[0]].Move(curMotion);
			}
		}
		// 算出新的 PoseState;平均或者什么最小二乘
		// vecEstiPoses 对 true的 求个平均值?

		PoseState curPoseState(idxImgCur);

		//更新队列和关键帧
		updateKeyList(ptrCurFeature);
		vecPoses[idxImgCur] = curPoseState;

		TIME_END("PoseEstimate");

		cDrawer.drawCanvas(curPoseState);
	}
	// 对这跟 当前帧有关的K对 数据, 取平均，或者怎么做 算出当前帧的 坐标数据

	// 调用 CanvasDraw 画出来，并画groundtruth
	TIME_END("MainLoop");

	return ++idxImgCur;
}

void TrackRunner::showFrameMotion() {


}

void TrackRunner::showTrack() {


}

std::vector<FeatureState*> TrackRunner::selectKeySequence(int idxImg) {
	std::vector<FeatureState*> retVec;
	//暂时返回 前一帧
	retVec.push_back( deqFrameFeatures.back() );

	return retVec;
}

int TrackRunner::filterMotions(std::vector<MotionState>& vecMotion) {


	return vecMotion.size();
}

void TrackRunner::updateKeyList(FeatureState* ptrFeature) {

	while(deqFrameFeatures.size() >= 1) {

		FeatureState* ptrPop = deqFrameFeatures.front();
		deqFrameFeatures.pop_front();
		delete ptrPop;
	}

	deqFrameFeatures.push_back(ptrFeature);
	
}