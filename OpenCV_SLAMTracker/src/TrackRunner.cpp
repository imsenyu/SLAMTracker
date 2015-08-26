#include "TrackRunner.h"
#include "CanvasDrawer.h"
#include "FrameParser.h"
#include "ScaleEstimator.h"
#include "Skeleton.h"

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
	cDrawer.setLogPath(cv::format("./Output/coordOutput_%s_%s.txt", Utils::getTimeNow().c_str(), CFG_sDataName.c_str()));
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
	vecPoses[idxImgBegin].setInited( true);
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


	//printf("////////////////////////////////////////////\n");
	printf("//////////// idxImgCur: %06d ////////////\n",idxImgCur);
	//printf("////////////////////////////////////////////\n");

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

			// 对极几何 计算运动matR和matT
			bool motionStatus = fparser.computeMotion(vecCurMotions[idx]);
			vecEnableIdxs[idxImgCur] = cntRunOk;
			// 运动参数计算失败
			if (motionStatus == false) {
				printf("运动参数计算失败\n");
				//TODO: 解不出来默认 运动,然后 强制一个默认尺度
				continue;
			}

			// 位移旋转限值
			if (CFG_bIsLimitRotationDiff && limitRotationDiff(vecCurMotions[idx],CFG_dRotationDiffLimit) == false) {
				printf("旋转跳\n");
				vecCurMotions[idx].setInited( false );
				continue;
			}

			// 尺度估算开启
			ScaleEstimator sEstimator;
			sEstimator.updateMotion(&vecCurMotions[idx]);
			double curScale = sEstimator.computeScaleTransform();

			if (curScale <= 0) {
				printf("尺度没算出来\n");
				curScale = 200;
			}
			// 尺度增量限制
			// 如果被跳帧了，跳帧后出现一个多间隔值，在这个间隔值之后，再允许任意一个。
			// 如果连跳多帧，那
			if (limitScaleDiff(vecCurMotions[idx], curScale, CFG_dScaleInvIncreaseDiffLimit) == false) {
				vecCurMotions[idx].setInited( false );
				continue;
			}
			int idxDelta = vecCurMotions[idx].getIdxImg(1) - vecCurMotions[idx].getIdxImg(0);
			if (CFG_iPreAverageFilter > 0) {
				auto& qDist = cDrawer.qDist;
				auto iter =  qDist.rbegin();
				double aver = 0.0f;
				int cnt = 0;
				for (; cnt < CFG_iPreAverageFilter && iter != qDist.rend(); iter++, cnt++) {
					printf("%d %f\n", idxImgCur, *iter);
					aver += *iter;
				}
				aver += 1.65f / curScale / idxDelta;
				aver /= cnt + 1;
				//if (aver < 1.65f / curScale / idxDelta) {
					curScale = 1.65 / aver / idxDelta;
				//}
			}

			/** TODO: 这里需要一个合适的尺度评价 */
			
			if ((curScale*idxDelta < CFG_dScaleRatioLimitBottom /*/ idxDelta*/ || curScale*idxDelta > CFG_dScaleRatioLimitTop)) {
				
				printf("Scale LIMIT %d-%d:%f %f\n", vecCurMotions[idx].getIdxImg(0), vecCurMotions[idx].getIdxImg(1), curScale, 1.65f / curScale);
				if (curScale*idxDelta < CFG_dScaleRatioLimitBottom) {
					curScale = CFG_dScaleRatioLimitBottom / idxDelta;
				}
				else
					curScale = CFG_dScaleRatioLimitTop / idxDelta;
				
			}


			// 目前使用 GroundTruth的 运动尺度，因此目前误差只在旋转角度上
			if (CFG_bIsUseGroundTruthDistance) {
				vecCurMotions[idx].setScale(1.65f / cv::norm(pHelper.getPosition(vecCurMotions[idx].getIdxImg(1), vecCurMotions[idx].getIdxImg(0))));
				//vecCurMotions[idx].scale = 1.65f / cv::norm(pVisio.getPosition(vecCurMotions[idx].idxImg[1], vecCurMotions[idx].idxImg[0]));
			}
			else {
				vecCurMotions[idx].setScale(curScale);
			}
			//vecCurMotions[idx]._matR_ = vecCurMotions[idx].matR * vecPoses[vecCurMotions[idx].idxImg[0]].dir3;

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
				if (curMotion.getInited() == true) {
					// 对vecPose的对应位姿,应用Motion得到 vecEstiPoses新位姿
					vecEstiPoses.push_back(vecPoses[curMotion.getIdxImg(0)].move(curMotion));

					//加入骨架
					vecMotionLinks[curMotion.getIdxImg(1)].insert(std::make_pair(curMotion.getIdxImg(0), curMotion));
				}
			}
			
			/** TODO: 需要有个方法判定当前是否在转弯. */
			if (std::abs(vecCurMotions[0].getDegree("T")) > 7.0f) {
				if (CFG_bIsLogGlobal)
				printf("...\n");
				bIsInRotate = true;
			}
			else {
				bIsInRotate = false;
			}

			/** TODO: 新的多位姿选择方法 */
			PoseState curPoseState = PoseState::calcAverage(vecEstiPoses);
			
			// 更新队列和关键帧
			updateKeyList(ptrCurFeature);
			cntRunOk++;

			vecPoses[idxImgCur] = curPoseState;

			TIME_END("PoseEstimate");
			//if (CFG_bIsLogGlobal)
			std::cout << vecPoses[idxImgCur] << std::endl;
			// 画布画出当前位姿, 以及GroundTruth实际路径
			// 内部修改 vecPoses 的数据，不能用 curPoseState 了
			cDrawer.drawCanvas(vecPoses[idxImgCur]);

			/*
			 *	本来用于测试 梯度下降优化方法，在计算完50个点之后执行
			 *  现废弃
			 */
			if (false && idxImgCur == 50) {
				printf("开始调整\n");

				Skeleton skl;
				skl.initData(vecPoses, vecMotionLinks);
				double err = skl.calcError();
				double preErr = err;
				printf("i=%d err=%f\n", -1, err);
				if (CFG_bIsLogGlobal)
				std::cout << skl.vecX[0] << std::endl;
				if (CFG_bIsLogGlobal)
				std::cout << skl.vecX[1] << std::endl;
				for (int i = 0; i < 1000; i++) {
					skl.calcDiff();
					skl.merge(1e-3);
					skl.fixMatrix();
					err = skl.calcError();
					
					printf("i=%d err=%f\n", i, err);
					if (CFG_bIsLogGlobal)
					std::cout << skl.vecX[0] << std::endl;
					if (CFG_bIsLogGlobal)
					std::cout << skl.vecX[1] << std::endl;
					if (preErr< err) {
						break;
					}
					preErr = err;
				}
				
				for (int i = idxImgBegin + 1; i < idxImgCur; i++) {
					if ( skl.vecX[i].rows) {
						PoseState t(i);
						t.setInited( true );
						t.pos.x = skl.vecX[i].at<double>(0, 0);
						t.pos.y = skl.vecX[i].at<double>(1, 0);
						t.pos.z = skl.vecX[i].at<double>(2, 0);
						if (CFG_bIsLogGlobal)
						std::cout << t.pos << std::endl;
					}

				}
				cv::waitKey();

				printf("结束调整\n");
			}


		}

	TIME_END("MainLoop");

	return ++idxImgCur;
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
	int lenMotion = vecMotion.size();
	int reduce = 0;
	for (; iter != vecMotion.end();) {
		bool isDelete = false;
		/** 标记为需要删除 */
		if (iter->getInited() == false) {
			isDelete = true;
			reduce++;
		}

		/** STL方法，erase执行后返回下一个迭代器,旧迭代器失效，如果是end()的话再++就会挂掉 */
		if (isDelete) {
			iter = vecMotion.erase(iter);
		}
		else {
			iter++;
		}
	}
	return lenMotion - reduce;
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
		if (vecPosesBack[i].getInited() == false) break;
		if (CFG_bIsLogGlobal)
		std::cout << vecPosesBack[i] << std::endl;
	}

	for (int idxCur = 0; idxCur < vecMotionLinks.size(); idxCur++) {
		auto& mp = vecMotionLinks[idxCur];
		if (mp.size() == 0&&idxCur !=0) break;
		for (auto& pair : mp) {
			int idxPre = pair.first;
			MotionState& motion = pair.second;

			if (CFG_bIsLogGlobal)
			printf("Motion[%d-%d]\n", idxPre, idxCur);
			if (CFG_bIsLogGlobal)
			std::cout << motion << std::endl;

		}
	}
}

//需要判定 是 。.......。。 还是 。。.....。 还是 。.....。.....。
bool TrackRunner::limitRotationDiff(MotionState& curMotion, double limit) {

	//先找到 前面几个 Pose的直接motion
	int idxCur = curMotion.getIdxImg(1),
		idxPre1 = curMotion.getIdxImg(0),
		idxPre2, idxPre3;
	bool isSkipFrames;
	bool isSkipBetweenCurAndPre1;
	bool isSkipBetweenPre1AndPre2;
	bool isSkipBetweenPre2AndPre3;

	if (idxPre1 == 0) return true;
	MotionState &preMotion1 = vecMotionLinks[idxPre1].rbegin()->second;
	idxPre2 = preMotion1.getIdxImg(0);

	if (idxPre2 == 0) return true;
	MotionState &preMotion2 = vecMotionLinks[idxPre2].rbegin()->second;
	idxPre3 = preMotion2.getIdxImg(0);


	isSkipBetweenCurAndPre1 = (idxCur - idxPre1) - (vecEnableIdxs[idxCur] - vecEnableIdxs[idxPre1]) > 0;
	isSkipBetweenPre1AndPre2 = (idxPre1 - idxPre2) - (vecEnableIdxs[idxPre1] - vecEnableIdxs[idxPre2]) > 0;
	isSkipBetweenPre2AndPre3 = (idxPre2 - idxPre3) - (vecEnableIdxs[idxPre2] - vecEnableIdxs[idxPre3]) > 0;
	isSkipFrames = isSkipBetweenCurAndPre1 || isSkipBetweenPre1AndPre2 || isSkipBetweenPre2AndPre3;


	double preDelta = (preMotion1.getDegree("T") - preMotion2.getDegree("T")) / (idxPre1 - idxPre2),
		curDelta = (curMotion.getDegree("T") - preMotion1.getDegree("T")) / (idxCur - idxPre1);

	if (  idxCur - idxPre1  > vecEnableIdxs[idxCur] - vecEnableIdxs[idxPre1] ) {
		printf("之前跳帧 ，允许 cur(%d:%f %d:%f): %f  pre(%d:%f %d:%f): %f\n", 
			idxCur, curMotion.getDegree("T"), idxPre1, preMotion1.getDegree("T"), curDelta,
			idxPre1, preMotion1.getDegree("T"), idxPre2, preMotion2.getDegree("T"), preDelta);
		return true;
	}


	if (std::abs(curDelta - preDelta) > limit) {
		printf("Rotation Error %d:%f %d:%f \n", idxPre1, preDelta, idxCur, curDelta);
		return false;
	}

	return true;
}

bool TrackRunner::limitScaleDiff(MotionState& curMotion, double& curScale, double limit) {
	int idxCur = curMotion.getIdxImg(1),
		idxPre1 = curMotion.getIdxImg(0),
		idxPre2;
	int isSkipBetweenCurAndPre1;
	int isSkipBetweenPre1AndPre2;

	bool isLimitTop = true,
		isLimitBottom = true;

	if (idxPre1 == 0) return true;
	MotionState &preMotion1 = vecMotionLinks[idxPre1].rbegin()->second;

	idxPre2 = preMotion1.getIdxImg(0);
	
	isSkipBetweenCurAndPre1 = (idxCur - idxPre1) - (vecEnableIdxs[idxCur] - vecEnableIdxs[idxPre1]) ;
	isSkipBetweenPre1AndPre2 = (idxPre1 - idxPre2) - (vecEnableIdxs[idxPre1] - vecEnableIdxs[idxPre2]) ;
	
	printf("Scale Skip[%d-%d-%d] %d,%d\n", idxCur, idxPre1, idxPre2, isSkipBetweenCurAndPre1, isSkipBetweenPre1AndPre2);

	//当前跳1个，前一个跳0个
	if (isSkipBetweenCurAndPre1 == 1 && isSkipBetweenPre1AndPre2 == 0) {
		//按当前比例算
		//isLimitTop = true;
		//isLimitBottom = false;
	}
	//当前连跳2个了，允许
	else if (isSkipBetweenCurAndPre1 > 1 && isSkipBetweenPre1AndPre2 == 0) {
		return true;
	}
	//当前没跳，之前跳了>0个
	else if ( isSkipBetweenCurAndPre1 == 0 && isSkipBetweenPre1AndPre2 > 0) {
		return true;
	}
	//当前跳了几个，之前跳了几个
	else if (isSkipBetweenCurAndPre1 > 0 && isSkipBetweenPre1AndPre2 > 0) {
		return true;
	}

	//double curDelta = (1.65f / curScale / (idxCur - idxPre1)) / (1.65f / preMotion1.scale / (idxPre1 - idxPre2));
	double curDelta = (1.65f / curScale / (idxCur - idxPre1)) - (1.65f / preMotion1.getScale() / (idxPre1 - idxPre2));

	isLimitTop = (1.65f / curScale / (idxCur - idxPre1)) > 1.0f;
	isLimitBottom = (1.65f / preMotion1.getScale() / (idxPre1 - idxPre2)) < 0.5f;

	//if (std::abs(curDelta - 1.0f) > limit) {
	if (std::abs(curDelta ) > 1.2f* limit) {
		if (std::abs(curDelta) > limit)
		printf("Scale Error %d-%d:%f %f    %d-%d:%f %f\n", 
			idxPre2, idxPre1, preMotion1.getScale()*(idxPre1 - idxPre2), 1.65f / preMotion1.getScale() / (idxPre1 - idxPre2), 
			idxPre1, idxCur, curScale*(idxCur - idxPre1), 1.65f / curScale / (idxCur - idxPre1));
		////double sign = curDelta > 1.0f ? +1.0f : -1.0f;
		double sign = curDelta > 0 ? +1.0f : -1.0f;
		if (sign > 0 && isLimitTop == false) return true;
		if (sign < 0 && isLimitBottom == false) return true;

		////curScale = 1.65f / (idxCur - idxPre1) / (1.65f / preMotion1.scale / (idxPre1 - idxPre2) * sign *(limit+1.0f));
		
		curScale = 1.65f / (idxCur - idxPre1) / (1.65f / preMotion1.getScale() / (idxPre1 - idxPre2) +sign *(limit));
		return true;
	}


	return true;
}