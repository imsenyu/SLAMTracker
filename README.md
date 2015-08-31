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

...working on...

