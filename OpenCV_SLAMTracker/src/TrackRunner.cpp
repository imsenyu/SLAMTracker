#include "TrackRunner.h"
#include "CanvasDrawer.h"
#include "FrameParser.h"
#include "ScaleEstimator.h"

TrackRunner::TrackRunner(int _ImgBeginIdx, int _ImgEndIdx):
cDrawer(_ImgBeginIdx),
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

	//初始化最初的姿态
	//TODO: 如果不是从0开始的话，需要把初始角度修正为 GroundTruth的 方向, 仅仅pos不够
	vecPoses[idxImgBegin].dir = Const::pnt3d_001;
	vecPoses[idxImgBegin].pos = Const::pnt3d_000;
	vecPoses[idxImgBegin].inited = true;
	vecPoses[idxImgBegin].idxImg = idxImgBegin;

	// 初始化画布对象
	cDrawer.initAnimate(vecPoses[idxImgBegin]);

	//先算idx = idxImgBegin 时候的 FeatureState, 
	FeatureState* ptrCurFeature = new FeatureState(idxImgBegin);

	//TODO: 后续把这个功能剥离到 FeatureState 中,不放在 FrameParser
	FrameParser::detectExtractFeatures(CFG_iMaxFeatures, *ptrCurFeature);

	//最近可用队列中压入
	deqFrameFeatures.push_back(ptrCurFeature);

	//下次运行从 idxImgBegin+1 开始
	idxImgCur = idxImgBegin + 1;
}

int TrackRunner::runKeyStep() {
	printf("///////////////////////////////////////////\n");
	printf("///////// idxImgCur: %06d /////////\n",idxImgCur);
	printf("///////////////////////////////////////////\n");
	TIME_BEGIN("MainLoop");

		TIME_BEGIN("FeatureDetect");
		// 读取当前帧数据
		//可以考虑 多线程并行计算,然后在这里放个 wait
		//另外那个线程算好一个放个 signal
		FeatureState* ptrCurFeature = new FeatureState(idxImgCur);
		FrameParser::detectExtractFeatures(CFG_iMaxFeatures, *ptrCurFeature);

		std::cout << *ptrCurFeature << std::endl;
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
			if (fparser.vecPairPoint[0].size() > 0 && bIsInRotate == false)
				fparser.validPointsByOpticalFlow(1.0f);

			//计算对极几何
			vecCurMotions[idx].idxImg[0] = ptrFeature->idxImg;
			vecCurMotions[idx].idxImg[1] = ptrCurFeature->idxImg;
			bool motionStatus = fparser.computeMotion(vecCurMotions[idx]);
			//计算尺度
			vecCurMotions[idx].mapPairPoints = fparser.mapPairPoints;

			//二次光流
			//MotionState tmpMotion;
			//tmpMotion.idxImg[0] = vecCurMotions[idx].idxImg[0];
			//tmpMotion.idxImg[1] = vecCurMotions[idx].idxImg[1];

			//fparser.validPointsByOpticalFlow(1.0f);


			if (motionStatus == false) {
				vecCurMotions[idx].inited = false;
				continue;
			}

			sEstimator.updateMotion(&vecCurMotions[idx]);
			double scale = sEstimator.computeScaleTransform();

			if ( scale<0 || scale < 0.4f || scale > 20.0f) { 
				motionStatus = false; 
				vecCurMotions[idx].inited = false;
				continue; 
			}

			vecCurMotions[idx].scale = scale;

			std::cout << vecCurMotions[idx] << std::endl;
			//放入vecCurMotions
		}
		TIME_END(cv::format("Match&Motion[%d]", vecKeyList.size()));


		//按照一定规则过滤 vecCurMotions,凡是过滤掉的，都要delete Motion
		int cntValid = filterMotions(vecCurMotions);

		// 数据点崩了跳过 delete CurFeature
		if (cntValid < 1) {
			std::cout << "数据点崩了" << std::endl;
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
					std::cout << vecPoses[curMotion.idxImg[0]] << std::endl;
					std::cout << curMotion << std::endl;
					vecEstiPoses[idx] = vecPoses[curMotion.idxImg[0]].move(curMotion);
					std::cout << vecEstiPoses[idx] << std::endl;
				}
			}
			// 算出新的 PoseState;平均或者什么最小二乘
			// vecEstiPoses 对 true的 求个平均值?
			
			if (std::abs(vecCurMotions[0].degreeT) > 7.0f) {
				printf("...\n");
				bIsInRotate = true;
			}
			else {
				bIsInRotate = false;
			}

			PoseState curPoseState = vecEstiPoses[0];
			if (curPoseState.inited == false) {
				throw std::exception("错误目标点", 1);
			}
			//更新队列和关键帧
			updateKeyList(ptrCurFeature);
			vecPoses[idxImgCur] = curPoseState;


			TIME_END("PoseEstimate");

			std::cout << curPoseState << std::endl;

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
	auto iter = deqFrameFeatures.begin();
	for (; iter != deqFrameFeatures.end();iter++)
		retVec.push_back( *iter );

	//if (retVec.size() > 2)
	//	throw std::exception("超过2", 2);

	return retVec;
}

int TrackRunner::filterMotions(std::vector<MotionState>& vecMotion) {

	auto iter = vecMotion.begin();
	for (; iter != vecMotion.end();) {
		if (iter->inited == false) {
			//erase执行后返回下一个迭代器, 如果是end()的话再++就会挂掉
			iter = vecMotion.erase(iter);
		}
		else {
			iter++;
		}
	}
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