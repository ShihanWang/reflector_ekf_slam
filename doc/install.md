# Compile Reflector EKF SLAM

## System Requirements

The following [ROS distributions](wiki.ros.org/Distributions) packages are currently required:

~~~bash
Kinetic
~~~

Other packages: Glog， Ceres-solver, Cairo, Eigen3

## Build

```bash
cd ~
mkdir -p res_ws/src
cd res_ws/src
git clone git@github.com:ShihanWang/reflector_ekf_slam.git
cd ..
catkin_make
```

## Demo

### 2D Relfector

~~~bash
cd res_ws
source devel/setup.bash
roslaunch reflector_ekf_slam slam.launch 
rosbag play /res_ws/src/reflector_ekf_slam/dataset/reflector_2d_long_2021-03-30-19-13-53.bag
~~~

### 3D Reflector (coming soon)

```bash
cd res_ws
source devel/setup.bash
roslaunch reflector_ekf_slam slam.launch 
rosbag play ${HOME}/Downloads/reflector_3d_in_corridor_2021-03-30-16-41-23.bag
```

## Save Result

~~~bash
cd res_ws
source devel/setup.bash
rosservice call /reflector_ekf_slam/save_map ${HOME}/result
~~~