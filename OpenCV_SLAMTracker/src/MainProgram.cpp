#include "stdafx.h"
#include "TrackRunner.h"


int main(int argc, char* argv[]) {

	// 根据命令行参数加载配置文件等处理过程
	Utils::loadCommandLine(argc, argv);

	// 工作模式配置为 执行轨迹跟踪
	if (CFG_sModeExecute == "track") {
		TrackRunner runner(CFG_iImageLoadBegin, CFG_iImageLoadEnd);

		runner.initFirstFrame();

		while (true) {

			int idxImgCur = runner.runKeyStep();
			if (idxImgCur > CFG_iImageLoadEnd) break;

			runner.showFrameMotion();
			runner.showTrack();
		}

	}
	// 工作模式配置为 并行批量特征预处理
	else if (CFG_sModeExecute == "feature") {
	
	
	}

	////////////////////////////////////////////////////////////////////////////
	//// 画布对象初始化
	//CanvasDrawer cDrawer(cv::format("./Output/coordOutput_%s.txt", Utils::GetTimeNow().c_str()));
	//cDrawer.UseGroundTruth(CFG_GroutTruthPosePath);

	////////////////////////////////////////////////////////////////////////////
	//// 迭代变量定义
	//int preImgIdx = CFG_ImageLoadBegin,
	//	curImgIdx = preImgIdx + 1;
	//bool motionStatus = true;
	//double curDegreeT = 0.0f;
	//double transformScale = -1.0f;
	//FeatureState preFeatureState;
	//cv::Mat matImageList[2];
	//matImageList[1] = cv::imread(cv::format(CFG_ImageLoadPath.c_str(), preImgIdx));

	//FrameParser::DetectExtractFeatures(CFG_nFeatures, matImageList[1], preFeatureState, preImgIdx);
	////FrameParser::DetectExtractFeatures(_nFeatures, matImageList[1], preVecKeyPoints, preVecFeaturePoints, preMatDescriptor);

	//// 三帧融合变量缓存
	//std::vector<MotionState> vecMotionCache( CFG_ImageLoadEnd + 10 );
	//ScaleEstimator vEstimator;

	////////////////////////////////////////////////////////////////////////////
	//// 帧处理循环
	//
	//for (; curImgIdx <= CFG_ImageLoadEnd;) {
	//	
	//	TIME_BEGIN("MainLoop");

	//		printf("\n//////////////////////////////////////////////\n");
	//		printf("/////////// image[%d]-[%d] /////////////\n", preImgIdx, curImgIdx);
	//		std::cout << cv::format(CFG_ImageLoadPath.c_str(), curImgIdx) << std::endl;
	//		printf("//////////////////////////////////////////////\n\n");

	//		cv::Mat matR, matT;

	//		if ( motionStatus == true )
	//			matImageList[0] = matImageList[1];
	//		matImageList[1] = cv::imread(cv::format(CFG_ImageLoadPath.c_str(), curImgIdx));
	//					
	//		FrameParser fparser(matImageList, preImgIdx,curImgIdx);

	//	

	//		//单帧运算缓存 赋值1
	//		TIME_BEGIN("PreFeature Copy1");
	//		fparser.SetFeatureState(preFeatureState);
	//		//fparser.vecKeyPoints[0] = preVecKeyPoints;
	//		//fparser.vecFeaturePoints[0] = preVecFeaturePoints;
	//		//fparser.matDescriptor[0] = preMatDescriptor;
	//		TIME_END("PreFeature Copy1");

	//		//特征检测
	//		TIME_BEGIN("DetectFeature" );
	//			fparser.DetectFeaturePoints(CFG_nFeatures,1);
	//		TIME_END("DetectFeature");

	//		//特征匹配
	//		TIME_BEGIN("MatchFeature" );
	//			fparser.MatchFeaturePoints();
	//		TIME_END("MatchFeature");

	//		//运动计算
	//		TIME_BEGIN("ComputeMotion" );
	//			motionStatus = fparser.ComputePointPairMotion(matR, matT, curDegreeT);
	//			//判定一下 vecMotionCache[pre].degreeT 跟 当前 curDegreeT;
	//			std::function<double(double)> funcDiff = [&](double limit)->double{
	//				return limit + (curImgIdx - preImgIdx-1)*0.1f*limit;
	//			};
	//			if (curImgIdx > CFG_ImageLoadBegin +1)
	//			if ( CFG_IsLimitRotationDiff && std::abs(vecMotionCache[preImgIdx].degreeT - curDegreeT) > funcDiff(CFG_RotationDiffLimit)) {
	//				printf("===========DegreeT Error==========\n");
	//				printf("pre=%f cur=%f\n", vecMotionCache[preImgIdx].degreeT, curDegreeT);

	//				int ret = IDNO;// MessageBox(NULL, TEXT("是：接受该位移；否：跳帧"), TEXT("位移角度异常"), MB_YESNO);
	//				motionStatus = ret == IDYES;
	//			}
	//		TIME_END("ComputeMotion");


	//		if (motionStatus == false) {
	//			//运动计算异常,算不出来, pre不变,cur++
	//			printf("================ComputeMotion Error===============\n");
	//			preImgIdx;
	//			curImgIdx++;
	//			cv::waitKey(1);
	//			TIME_END("MainLoop");

	//			continue;
	//		}

	//		//////////////////////////////////////////////////////////////////////////
	//		// 大量重写 ScaleEstimator
	//		TIME_BEGIN("Velocity Estimator");
	//			fparser.GetMotionState(vecMotionCache[curImgIdx]);
	//			vecMotionCache[curImgIdx].SetState(matR, matT, cDrawer.gPointPos, cDrawer.gPointDir);
	//			vecMotionCache[curImgIdx].degreeT = curDegreeT;
	//			vEstimator.UpdateMotion(&vecMotionCache[curImgIdx]);
	//			//TODO 加速度修正
	//			transformScale = vEstimator.ComputeScaleTransform(transformScale);
	//			//transformScale = 1.0f;// vEstimator.ComputeScale();
	//		TIME_END("Velocity Estimator");


	//		//画布绘制
	//		TIME_BEGIN("DrawAnimate" );
	//			std::cout << matR << std::endl << matT << std::endl;
	//			fparser.DrawFeaturesFlow(matR, matT);
	//			cDrawer.DrawAnimate(matR, matT, preImgIdx, curImgIdx, transformScale);
	//		TIME_END("DrawAnimate");

	//		if (motionStatus == true) {
	//			//运动计算正常, pre++,cur++
	//			preImgIdx = curImgIdx;
	//			curImgIdx++;
	//		}

	//		//单帧运算缓存 赋值2
	//		TIME_BEGIN("PreFeature Copy2 Back");
	//			fparser.GetFeatureState(preFeatureState);
	//		TIME_END("PreFeature Copy2 Back");

	//

	//	TIME_END("MainLoop");
	//	//getchar();
	//}
	//

	//cv::waitKey();
	getchar();
	return 0;
}