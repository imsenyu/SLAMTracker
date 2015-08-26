#include "stdafx.h"
#include "TrackRunner.h"


int main(int argc, char* argv[]) {

	// 根据命令行参数加载配置文件等处理过程
	Utils::loadCommandLine(argc, argv);

	// 工作模式配置为 执行轨迹跟踪
	if (CFG_sModeExecute == "track") {
		TrackRunner runner(CFG_iImageLoadBegin, CFG_iImageLoadEnd);

		runner.initFirstFrame();
		int cnt = 0;
		while (runner.hasNext()) {

			int idxImgCur = runner.runKeyStep();
			
			cnt++;
		}
		runner.lm();
		cv::waitKey();

	}
	// 工作模式配置为 并行批量特征预处理
	else if (CFG_sModeExecute == "feature") {
#pragma omp parallel for
		for (int idx = CFG_iImageLoadBegin; idx <= CFG_iImageLoadEnd; idx++) {
			
			FeatureState fs(idx);
			fs.detect(CFG_iMaxFeatures);
			printf("FeatureDetect[%d]-\n",idx);
		}
	
	}

	getchar();
	return 0;
}