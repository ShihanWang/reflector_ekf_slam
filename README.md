# reflector_ekf_slam
这是一个基于EKF的激光反光板方案，整个框架基于ROS开发的，且只是实现了基本的功能，对应的CSDN博客[一起做激光反光板系列](https://blog.csdn.net/yeluohanchan/article/details/109620511)中推导的公式。
**需要注意的是**：
由于本人懒，所以本框架在[aruco_ekf_slam](https://github.com/ydsf16/aruco_ekf_slam)基础上根据自己的公式进行了重构和优化。

## demo运行方法

- 默认参数启动
```
roslaunch reflector_ekf_slam slam.launch bag:=${HOME}/res_ws/src/reflector_ekf_slam/dataset/reflector_2d_long_2021-03-30-19-13-53.bag
```
- launch命令可调参数示例
```
roslaunch reflector_ekf_slam slam.launch bag:=${HOME}/res_ws/src/reflector_ekf_slam/dataset/reflector_2d_long_2021-03-30-19-13-53.bag start:=40 dur:=60 rate:=2 rviz:=false laser:=/scan
```
上述命令解释:
将bag包从第40s开始播放,只播放60s就停止,以2倍速播放,不显示rviz,激光话题名重映射为 /scan
上述变量不写时,将按照launch文件里的默认参数启动

# 重要更新

## 基本EKF建图定位框架搭建-2020.11.21

（1）ROS消息的接收：轮速计和激光点云

（2）ROS消息的发送：反光板地图、实时位姿、历史轨迹

（3）反光板检测与匹配

（4）EKF公式实现

（5）反光板地图的保存与读取
## 修复bug和增加数据集-2021.4.3

（1）修复EKF代码错误

（2）修复反光板匹配错误

（3）开源测试数据集


# 基本功能
（1）反光板检测与匹配

（2）反光板建图

（3）反光板定位

# 后续待添加的功能

（1）滑窗类EKF方案：状态空间维护滑窗内多帧的位姿

（2）ICP方案的使用：前后帧ICP，当做观测。考虑：libPointMatcher、点线csm、scan-submap匹配的方式

（3）特征的使用：直角点、线段，当做观测

（4）FEJ的使用：First-Estimates Jacobian

（5）利用EKF位姿构建栅格地图

（6）激光雷达相对于车体中心的外参放到状态空间内进行优化