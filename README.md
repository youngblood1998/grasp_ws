# grasp_ws
Yolov5_ros是训练yolo的功能包


grasp_pointcloud是个人用于抓取草莓的功能包，目录中文件夹的作用：
	img存放图片
	pcd存放原始点云
	pcd_treat存放处理过的点云
	msg点云直通滤波范围的自定义消息类型
	script个人python文件


script中的文件夹(可参考script中的README)：
	grasp离线的抓取位姿检测程序
	real_grasp没有加周围检测的
	real_grasp_new加周围检测
	real_grasp_new_1加移动到被抓物体的上方
	real_grasp_yolo使用yolo


real_grasp_yolo：
1、打开相机
roslaunch realsense2_camera rs_camera_ca.launch
2、使用yolov5检测（conda_env加source activate pytorch激活环境）
roslaunch yolov5_ros yolo_v5.launch
3、运行树构建算法
（发布点云直通滤波的范围"real_detect/PointBoundingBox"和树构建结果"real_detect/tree_image"）
rosrun grasp_pointcloud real_grasp_detect_yolo.py
