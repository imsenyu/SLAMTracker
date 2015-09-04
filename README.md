# SLAMTracker
A slam problem solver called SLAMTracker based on opencv.

### Dataset

The KITTI dataset from [http://www.cvlibs.net/datasets/kitti/eval_odometry.php](http://www.cvlibs.net/datasets/kitti/eval_odometry.php "cvlibs-2012").

Files include:

1. **photo sequence from gray-camera-0 & camera parameter**
2. photo sequence from gray-camera-1 & camera parameter  
3. photo sequence from color-camera-0 & camera parameter  
4. photo sequence from color-camera-1 & camera parameter  

and we consider a **Monolar** SLAM solver so we only use the first dataset `photo sequence from gray-camera-0`

### Implementation & Build

Based on **OpenCV** & **Visual C++ 2013**.

1. Feature Dectetion `using SIFT`  

2. Multi-Frame Feature Match  

3. Motion Estimation `using Epipolar Geometry`

4. Ground Detection & Scale Estimation

5. Global Tunning (...working on)
 1. FrameSLAM using Skeleton
 2. Sparse Bundle Adjustment

### Configuration

All config paramters are shown below as a `XML` format.  
and the default config file can be found in `'./vslam_config.xml'`.

	<?xml version="1.0"?>
	<opencv_storage>
		...Item...
	</opencv_storage>

* `std::string CFG_sPathConfigFile`  
	**Usage**:	配置文件读取路径   
	**Default**:	`"./vslam_config.xml"`

* `std::string CFG_sPathPoseGroutTruth`  
	**Usage**:	位姿GroundTruth文件读取路径  
	**Default**:	`"./toPath/poses/00.txt"`

* `std::string CFG_sPathImageLoad`  
	**Usage**:	数据集图像序列读取路径 `{idx:%06d}`, 使用`%06d`指明加载图像的文件名  
	**Default**:	`"./toPath/00/image_0/%06d.png"`

* `std::string CFG_sPathFeatureData`  
	**Usage**:	缓存特征预处理数据读取路径 `{idx:%d}` `{nfeature:%d}`  
	**Default**:	`"./toPath/00/features_%d_%d.xml"`

* `bool CFG_bIsUseCacheFeature`  
	**Usage**:	是否使用本地预处理特征点    
	**Default**:	`false`

* `std::string CFG_sModeExecute`  
	**Usage**:	程序执行模式 {track/feature}  
	**Default**:	`track`

* `bool CFG_bIsCacheCurrentFeature`  
	**Usage**:	是否缓存当前运算特征  
	**Default**:	`false`

* `bool CFG_bIsLimitRotationDiff`  
	**Usage**:	是否限制位移旋转角度增量  
	**Default**:	`false`

* `int CFG_iImageLoadBegin`  
	**Usage**:	读取图像起始标号  
	**Default**:	`0`

* `int CFG_iImageLoadEnd`  
	**Usage**:	读取图像结束标号  
	**Default**:	`number of frames`

* `int CFG_iMaxFeatures`  
	**Usage**:	最大特征匹配数量,Sift前K个特征  
	**Default**:	`2000`

* `bool CFG_bIsNotConsiderAxisY`  
	**Usage**:	是否忽略坐标绘制的Y(上下)轴  
	**Default**:	`false`

* `double CFG_dDrawFrameStep`  
	**Usage**:	坐标绘制时单位长度(meter)的像素点长度  
	**Default**:	`1.0`

* `double CFG_dScaleRatioLimitBottom`  
	**Usage**:	尺度变换限制下界  
	**Default**:	`0.5`, 相当于运动距离最长为 `1.65f/0.5`

* `double CFG_dScaleRatioLimitTop`  
	**Usage**:	尺度变换限制上界  
	**Default**:	`11.0`, 相当于运动距离最短为 `1.65f/11.0`

* `double CFG_dScaleRatioErrorDefault`  
	**Usage**:	 尺度变换异常的默认位移模长 
	**Default**:	`0.5f`, 相当于运动距离为 `0.5`

* `bool CFG_bIsUseGroundTruthDistance`  
	**Usage**:	是否使用GroundTruth的位移模长  
	**Default**:	`false`

* `double CFG_dRotationDiffLimit`  
	**Usage**:	位移旋转角度增量限制值(degree)  
	**Default**:	`15.0f`

* `double CFG_dScaleInvIncreaseDiffLimit`  
	**Usage**:	位移(尺度倒数)增量限制  
	**Default**:	`0.1f`

* `cv::Mat CFG_mCameraParameter`  
	**Usage**:	相机内参数矩阵  
	**Default**:	`""`

* `int CFG_iDequeFrameNumber`  
	**Usage**:  历史队列中的帧数限制	  
	**Default**:	`1`

* `double CFG_dOpticalFlowThreshold`  
	**Usage**:	光流过滤阈值  
	**Default**:	`2.0`

* `bool CFG_bIsLogGlobal`  
	**Usage**:	全局log开启/关闭  
	**Default**:	`false`

* `int CFG_iPreAverageFilter`  
	**Usage**:	前向均值滤波的帧数  
	**Default**:	`0`

* `std::string CFG_sDataName`  
	**Usage**:	数据集名称  
	**Default**:	`"00"`
