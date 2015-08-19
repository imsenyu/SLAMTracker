#include "TrackRunner.h"
#include "CanvasDrawer.h"
#include "FrameParser.h"
#include "ScaleEstimator.h"

TrackRunner::TrackRunner(int _ImgBeginIdx, int _ImgEndIdx):
cDrawer(_ImgBeginIdx),
idxImgBegin(_ImgBeginIdx), idxImgEnd(_ImgEndIdx),
vecPoses(_ImgEndIdx+1), vecMotionLinks(_ImgEndIdx+1),
vecEnableIdxs(_ImgEndIdx+1),
cntRunOk(0)
{
}


TrackRunner::~TrackRunner()
{
	//依次删除new出来的两个FeatureState* 记录
	for (FeatureState* ptr : vecKeyFrameFeatures)
		delete ptr;
	for (FeatureState* ptr : deqFrameFeatures)
		delete ptr;
}


void TrackRunner::initFirstFrame() {

	// 设定输出配置
	cDrawer.setLogPath(cv::format("./Output/coordOutput_%s.txt", Utils::getTimeNow().c_str()));
	// 初始化 PoseHelper
	pHelper.setGroundTruth(CFG_sPathPoseGroutTruth);
	pVisio.setGroundTruth("./Poses/00_.txt");
	// 给Drawer设置groundTruth
	cDrawer.ptrPoseHelper = &pHelper;

	//初始化最初的姿态
	//TODO: 如果不是从0开始的话，需要把初始角度修正为 GroundTruth的 方向, 仅仅pos不够
	vecPoses[idxImgBegin].dir = Const::pnt3d_001;
	vecPoses[idxImgBegin].pos = Const::pnt3d_000;
	vecPoses[idxImgBegin].dir3 = Const::mat33_111.clone();
	vecPoses[idxImgBegin].inited = true;
	vecPoses[idxImgBegin].idxImg = idxImgBegin;

	// 画布对象参数初始化
	cDrawer.initAnimate(vecPoses[idxImgBegin]);

	// 初始化idx = idxImgBegin 并计算特征和描述子
	FeatureState* ptrCurFeature = new FeatureState(idxImgBegin);
	ptrCurFeature->detect(CFG_iMaxFeatures);

	// 在最近可用队列中压入
	deqFrameFeatures.push_back(ptrCurFeature);

	//下次运行从 idxImgBegin+1 开始
	idxImgCur = idxImgBegin + 1;
}

// [点] 特定帧数据记录 int<-> 当前位置，方向, 全记录
// [点] 特定帧数据记录 int<-> 特征点 vecKP, vecP, matDescriptor->存储数据量在 2000K 一个,部分记录
// [边] 帧间数据记录 用 vector< int> --- 新map<int,旧>> 存matR, matT, Scale 全记录

// 扩展: [关键帧]池 和 [最近连续可用帧]池, 每产生一帧，如果往[最近]扔，[最近]是个定长队列, 如果超了，并且距离够了 扔进 [关键帧]里面去，否则销毁掉

//当前应该跑 nextStep那一帧

// 先从历史数据中选出 K 个最近参考帧 + ALL 关键帧进行 match
// 选取一个帧ptr 序列
int TrackRunner::runKeyStep() {

	printf("////////////////////////////////////////////\n");
	printf("//////////// idxImgCur: %06d ////////////\n",idxImgCur);
	printf("////////////////////////////////////////////\n");

	TIME_BEGIN("MainLoop");

		TIME_BEGIN("FeatureDetect");
		/** TODO: 可以考虑 多线程并行计算,然后在这里放个 wait
		 *		  另外那个线程算好一个放个 signal
		 */
		// 读取当前帧数据
		FeatureState* ptrCurFeature = new FeatureState(idxImgCur);
		ptrCurFeature->detect(CFG_iMaxFeatures);
		TIME_END("FeatureDetect");

		// 确定当前帧需要比较的 历史帧集合
		std::vector<FeatureState*> vecKeyList;
		/** TODO: 可能需要靠 视觉磁带模型预估closure */
		vecKeyList = selectKeySequence(idxImgCur);

		// 对每一对历史-当前帧 进行运动估算
		std::vector<MotionState> vecCurMotions( vecKeyList.size() );
		

		// 调用 FrameParser 分别处理 参考帧<->当前帧 的 matR和MatT
		TIME_BEGIN(cv::format("Match&Motion[%d]", vecKeyList.size()));
#pragma omp parallel for
		for (int idx = 0; idx < vecKeyList.size();idx++) {
			FeatureState* ptrFeature = vecKeyList[idx];

			/** TODO: 考虑对不同情况设置不同过滤规则 */
			// 初始化 帧间处理器
			FrameParser fparser(ptrFeature, ptrCurFeature);
			fparser.match(CFG_dOpticalFlowThreshold);
		/*	if (fparser.vecPairPoint[0].size() > 0 && bIsInRotate == false)
				fparser.validPointsByOpticalFlow(1.0f);*/


			// 对极几何 计算运动matR和matT
			vecCurMotions[idx].idxImg[0] = ptrFeature->idxImg;
			vecCurMotions[idx].idxImg[1] = ptrCurFeature->idxImg;
			vecCurMotions[idx].mapPairPoints = fparser.mapPairPoints;
			bool motionStatus = fparser.computeMotion(vecCurMotions[idx]);
			vecEnableIdxs[idxImgCur] = cntRunOk;
			// 运动参数计算失败
			if (motionStatus == false) {
				continue;
			}

			// 位移旋转限值
			if (CFG_bIsLimitRotationDiff && limitRotationDiff(vecCurMotions[idx],CFG_dRotationDiffLimit) == false) {
				vecCurMotions[idx].inited = false;
				continue;
			}

			// 尺度估算开启
			ScaleEstimator sEstimator;
			sEstimator.updateMotion(&vecCurMotions[idx]);
			double curScale = sEstimator.computeScaleTransform();

			// 尺度增量限制
			if (limitScaleDiff(vecCurMotions[idx], curScale, CFG_dScaleInvIncreaseDiffLimit) == false) {
				vecCurMotions[idx].inited = false;
				continue;
			}

			/** TODO: 这里需要一个合适的尺度评价 */
			int idxDelta = vecCurMotions[idx].idxImg[1] - vecCurMotions[idx].idxImg[0];
			if (curScale<0 || ( (curScale < CFG_dScaleRatioLimitBottom /*/ idxDelta*/ || curScale > CFG_dScaleRatioLimitTop) )  ) {
				if (idxDelta > 5) {
					vecCurMotions[idx].inited = true;
					if (curScale < CFG_dScaleRatioLimitBottom) {
						curScale = CFG_dScaleRatioLimitBottom;
					}
					else
						curScale = CFG_dScaleRatioLimitTop;
				}
				else {
					motionStatus = false;
					vecCurMotions[idx].inited = false;
					continue;
				}
				
			}


			// 目前使用 GroundTruth的 运动尺度，因此目前误差只在旋转角度上
			if (CFG_bIsUseGroundTruthDistance) {
				vecCurMotions[idx].scale = 1.65f / cv::norm(pHelper.getPosition(vecCurMotions[idx].idxImg[1], vecCurMotions[idx].idxImg[0]));
				//vecCurMotions[idx].scale = 1.65f / cv::norm(pVisio.getPosition(vecCurMotions[idx].idxImg[1], vecCurMotions[idx].idxImg[0]));
			}
			else {
				vecCurMotions[idx].scale = curScale;
			}
			vecCurMotions[idx]._matR_ = vecCurMotions[idx].matR * vecPoses[vecCurMotions[idx].idxImg[0]].dir3;

		}
		TIME_END(cv::format("Match&Motion[%d]", vecKeyList.size()));

		//按照一定规则过滤 vecCurMotions,目前先过滤掉 inited为false的
		int cntValid = filterMotions(vecCurMotions,0);

		// 如果过滤完没有了, 就跳帧
		if (cntValid < 1) {
			delete ptrCurFeature;
			vecEnableIdxs[idxImgCur] = 0;
		}
		else {
			/** TODO: 对算出来的这么多有效的, 1是要判定closure; 2是要综合(取平均哈哈哈)运动状态 */
			TIME_BEGIN("PoseEstimate");

			// 对于计算出来的每对Motion, 应用该运动
			std::vector<PoseState> vecEstiPoses;
			for (int idx = 0; idx < vecCurMotions.size(); idx++) {
				MotionState& curMotion = vecCurMotions[idx];
				if (curMotion.inited == true) {
					// 对vecPose的对应位姿,应用Motion得到 vecEstiPoses新位姿
					vecEstiPoses.push_back(vecPoses[curMotion.idxImg[0]].move(curMotion));

					//加入骨架
					vecMotionLinks[curMotion.idxImg[1]].insert(std::make_pair(curMotion.idxImg[0], curMotion));
				}
			}
			
			/** TODO: 需要有个方法判定当前是否在转弯. */
			if (std::abs(vecCurMotions[0].degreeT) > 7.0f) {
				printf("...\n");
				bIsInRotate = true;
			}
			else {
				bIsInRotate = false;
			}

			/** TODO: 新的多位姿选择方法 */
			// 对多个计算得到的新位姿,按照等比数列 1/2; 1/4; 1/8进行 加权平均....好low
			PoseState curPoseState = vecEstiPoses[0];
			int vecEstLen = vecEstiPoses.size();
			if (vecEstLen >= 2) {
				curPoseState.pos = Const::pnt3d_000;
				curPoseState.dir = Const::pnt3d_000;
				double pow[] = { 1.0f / 2.0f, 1.0f / 4.0f, 1.0f / 8.0f, 1.0f / 16.0f,1.0f/32,1.0f/64,1.0/128 };
				
				for (int idx = 0; idx < vecEstLen; idx++) {
					curPoseState.pos += vecEstiPoses[idx].pos;// *pow[idx];
					curPoseState.dir += vecEstiPoses[idx].dir;// *pow[idx];
				}
				//curPoseState.pos += vecEstiPoses[vecEstLen - 1].pos * pow[vecEstLen - 1];
				//curPoseState.dir += vecEstiPoses[vecEstLen - 1].dir * pow[vecEstLen - 1];

				curPoseState.pos = curPoseState.pos * (1.0f / vecEstLen);
				curPoseState.dir = curPoseState.dir * (1.0f / vecEstLen);
				curPoseState.dir = curPoseState.dir * (1.0f / cv::norm(curPoseState.dir));
			}

			// 更新队列和关键帧
			updateKeyList(ptrCurFeature);
			cntRunOk++;

			vecPoses[idxImgCur] = curPoseState;

			TIME_END("PoseEstimate");

			std::cout << vecPoses[idxImgCur] << std::endl;
			// 画布画出当前位姿, 以及GroundTruth实际路径
			// 内部修改 vecPoses 的数据，不能用 curPoseState 了
			cDrawer.drawCanvas(vecPoses[idxImgCur]);
		}

	TIME_END("MainLoop");

	return ++idxImgCur;
}

void TrackRunner::showFrameMotion() {


}

void TrackRunner::showTrack() {


}

std::vector<FeatureState*> TrackRunner::selectKeySequence(int idxImg) {
	std::vector<FeatureState*> retVec;
	
	// 返回最近队列中所有数据
	auto iter = deqFrameFeatures.begin();
	for (; iter != deqFrameFeatures.end();iter++)
		retVec.push_back( *iter );

	return retVec;
}

int TrackRunner::filterMotions(std::vector<MotionState>& vecMotion, double oldDegreeT) {

	auto iter = vecMotion.begin();
	int len = vecMotion.size();
	int reduce = 0;
	for (; iter != vecMotion.end();) {
		bool isDelete = false;
		if (iter->inited == false) {
			
			isDelete = true;
			reduce++;
		}
		//else {
		//	 if (std::abs(oldDegreeT - iter->degreeT) > 10){
		//		isDelete = true;
		//	}
		//}

		if (isDelete) {
			//erase执行后返回下一个迭代器, 如果是end()的话再++就会挂掉
			iter = vecMotion.erase(iter);
		}
		else {
			iter++;
		}
	}
	return len - reduce;
}

void TrackRunner::updateKeyList(FeatureState* ptrFeature) {

	while (deqFrameFeatures.size() >= CFG_iDequeFrameNumber) {

		FeatureState* ptrPop = deqFrameFeatures.front();
		deqFrameFeatures.pop_front();
		delete ptrPop;
	}

	deqFrameFeatures.push_back(ptrFeature);
	
}

void TrackRunner::lm() {
	printf("==== Least Square ====\n");
	auto vecPosesBack = vecPoses;
	std::vector<PoseState> vecDiff(vecMotionLinks.size());

	//std::function<void(MotionState&)> func1 = [&](MotionState& m) {
	//	int j = m.idxImg[0],
	//		i = m.idxImg[1];
	//	PoseState moved = vecPosesBack[j].move(m);
	//	vecDiff[i].pos += 2.0f*(vecPosesBack[i].pos - moved.pos);
	//	vecDiff[i].dir += 2.0f*(vecPosesBack[i].dir - moved.dir);
	//};

	//std::function<void(MotionState&)> func2 = [&](MotionState& m) {

	//};

	for (int i = 0; i < vecMotionLinks.size(); i++) {
		if (vecPosesBack[i].inited == false) break;
		std::cout << vecPosesBack[i] << std::endl;
	}

	for (int idxCur = 0; idxCur < vecMotionLinks.size(); idxCur++) {
		auto& mp = vecMotionLinks[idxCur];
		if (mp.size() == 0&&idxCur !=0) break;
		for (auto& pair : mp) {
			int idxPre = pair.first;
			MotionState& motion = pair.second;

			printf("Motion[%d-%d]\n", idxPre, idxCur);
			std::cout << motion << std::endl;

		}
	}
}

//需要判定 是 。.......。。 还是 。。.....。 还是 。.....。.....。
bool TrackRunner::limitRotationDiff(MotionState& curMotion, double limit) {

	//先找到 前面几个 Pose的直接motion
	int idxCur = curMotion.idxImg[1],
		idxPre1 = curMotion.idxImg[0],
		idxPre2, idxPre3;
	bool isSkipFrames;
	bool isSkipBetweenCurAndPre1;
	bool isSkipBetweenPre1AndPre2;
	bool isSkipBetweenPre2AndPre3;

	if (idxPre1 == 0) return true;
	MotionState &preMotion1 = vecMotionLinks[idxPre1].rbegin()->second;
	idxPre2 = preMotion1.idxImg[0];

	if (idxPre2 == 0) return true;
	MotionState &preMotion2 = vecMotionLinks[idxPre2].rbegin()->second;
	idxPre3 = preMotion2.idxImg[0];


	isSkipBetweenCurAndPre1 = (idxCur - idxPre1) - (vecEnableIdxs[idxCur] - vecEnableIdxs[idxPre1]) > 0;
	isSkipBetweenPre1AndPre2 = (idxPre1 - idxPre2) - (vecEnableIdxs[idxPre1] - vecEnableIdxs[idxPre2]) > 0;
	isSkipBetweenPre2AndPre3 = (idxPre2 - idxPre3) - (vecEnableIdxs[idxPre2] - vecEnableIdxs[idxPre3]) > 0;
	isSkipFrames = isSkipBetweenCurAndPre1 || isSkipBetweenPre1AndPre2 || isSkipBetweenPre2AndPre3;

	//下面针对跳帧进行判定

	printf("可用标号%d-%d: %d-%d\n", idxCur, idxPre1, vecEnableIdxs[idxCur], vecEnableIdxs[idxPre1]);
	if (vecEnableIdxs[idxCur] - vecEnableIdxs[idxPre1] <= CFG_iDequeFrameNumber && idxCur - idxPre1 > CFG_iDequeFrameNumber) {
		printf("\n");
		return true;
	}


	double preDelta = (preMotion1.degreeT - preMotion2.degreeT) / (idxPre1 - idxPre2),
		curDelta = (curMotion.degreeT - preMotion1.degreeT) / (idxCur - idxPre1);

	if (std::abs(curDelta - preDelta) > limit) {
		printf("[%d-%d] Rotation Error\n",idxPre1, idxCur);
		return false;
	}

	return true;
}

bool TrackRunner::limitScaleDiff(MotionState& curMotion, double& curScale, double limit) {
	int idxCur = curMotion.idxImg[1],
		idxPre1 = curMotion.idxImg[0],
		idxPre2;

	if (idxPre1 == 0) return true;
	if (vecEnableIdxs[idxCur] - vecEnableIdxs[idxPre1] <= CFG_iDequeFrameNumber && idxCur - idxPre1 > CFG_iDequeFrameNumber) return true;
	MotionState &preMotion1 = vecMotionLinks[idxPre1].rbegin()->second;

	idxPre2 = preMotion1.idxImg[0];
	if (idxCur - idxPre1 > 1) return true;
	double curDelta = 1.65f / curScale / (idxCur - idxPre1) - 1.65f / preMotion1.scale / ( idxPre1 -idxPre2);

	if (std::abs(curDelta ) > limit) {
		printf("[%d-%d] Scale Error \n", idxPre1, idxCur);
		double sign = curDelta > 0 ? +1.0f : -1.0f;
		curScale = 1.65f / (idxCur - idxPre1) / (1.65f / preMotion1.scale / (idxPre1 - idxPre2) + sign * limit);
		return true;
	}


	return true;
}