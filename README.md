# reflector_ekf_slam
这是一个基于EKF的激光反光板方案，整个框架基于ROS开发的，对应的CSDN博客[一起做激光反光板系列](https://blog.csdn.net/yeluohanchan/article/details/109620511)中推导的公式。


# 已添加功能

（1）ROS消息的接收：轮速计和激光点云

（2）ROS消息的发送：反光板地图、实时位姿、历史轨迹、**栅格地图、匹配点云**

（3）2D和3D激光反光板检测与匹配

（4）EKF公式的实现

（5）反光板地图的保存与读取

（6）**Scan-Submap匹配，栅格地图的创建与保存**

（6）开源测试数据集

# 后续待添加的功能

（1）滑窗类EKF方案：状态空间维护滑窗内多帧的位姿

（2）ICP方案的使用：前后帧ICP，当做观测。考虑：libPointMatcher、点线csm、scan-submap匹配的方式

（3）特征的使用：直角点、线段，当做观测

（4）FEJ的使用：First-Estimates Jacobian

（5）激光雷达相对于车体中心的外参放到状态空间内进行优化
