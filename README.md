# 简介
针对深度相机的室内点云的去地面和点云聚类算法
# 编译
cd depthpoint_clust
catkin_make
# 运行
- 启动深度相机
roslaunch realsense2_camera demo_pointcloud.launch
- 去除点云
roslaunch points_ground_filter points_ground_filter.launch
- 点云聚类
roslaunch points_cluster points_cluster.launch
