# NavCodeMonitor
### A package for monitoring the code of navigation purpose

This repository is the implementation of the GNSS weighted least square (WLS) positioning. The input is the pseudorange measurements from multiple satellites. The states to be estimated are the position **(x, y, z)** of the GNSS receiver in ECEF frame and the receiver clock bias.

**Authors**: [Weisong Wen](https://weisongwen.wixsite.com/weisongwen), [Li-ta Hsu](https://www.polyu-ipn-lab.com/) from the [Intelligent Positioning and Navigation Laboratory](https://www.polyu-ipn-lab.com/), The Hong Kong Polytechnique University

**Related Papers:** (paper is not exactly same with code)
1. Wen, Weisong, Guohao Zhang, and Li-Ta Hsu. "Exclusion of GNSS NLOS receptions caused by dynamic objects in heavy traffic urban scenarios using real-time 3D point cloud: An approach without 3D maps." Position, Location and Navigation Symposium (PLANS), 2018 IEEE/ION. IEEE, 2018. 

2. Wen, W., Zhang, G., Hsu, Li-Ta (Presenter), Correcting GNSS NLOS by 3D LiDAR and Building Height, ION GNSS+, 2018, Miami, Florida, USA.

*if you use GraphGNASLib for your academic research, please cite our related [papers](https://www.polyu-ipn-lab.com/)*

<p align="center">
  <img width="712pix" src="img/psr_model.png">
</p>
<center> Measurement model of pseudorange measurements.</center>

<p align="center">
  <img width="712pix" src="img/flowchart.png">
</p>

<center> Software flowchart of GNSS positioning.</center>

## 1. Prerequisites
### 1.1 **Ubuntu** and **ROS**
Ubuntu 64-bit 16.04.
ROS Kinetic. [ROS Installation](http://wiki.ros.org/ROS/Installation)


### 1.2. **Eigen**
```
# CMake
sudo apt-get install cmake
# google-glog + gflags
sudo apt-get install libgoogle-glog-dev
# BLAS & LAPACK
sudo apt-get install libatlas-base-dev
# Eigen3
sudo apt-get install libeigen3-dev
```

## 2. Build NavCodeMonitor
Clone the repository and catkin_make:
```
mkdir NavCodeMonitor/src
cd ~/NavCodeMonitor/src
git clone https://github.com/weisongwen/NavCodeMonitor.git
cd ../
# if you fail in the last catkin_make, please source and catkin_make again
catkin_make
source ~/NavCodeMonitor/devel/setup.bash
catkin_make
```
(if you fail in this step, try to find another computer with clean system or reinstall Ubuntu and ROS)

## 3. Run GNSS SPP-FGO using dataset [UrbanNav](https://www.polyu-ipn-lab.com/download)   
The NavCodeMonitor is validated using static dataset collected near TST of Hong Kong. Several parameters are as follows:
  - GPS second span: **270152** to **270313**
  - satellite system: **GPS/BeiDou** (you can choose GPS only or GPS/BeiDou fusion)

```bash
source ~/NavCodeMonitor/devel/setup.bash
# read GNSS raw data and publish as ROS topic
rosrun global_fusion spp_node
# open Rviz for visualization of trajectory
cd ~/NavCodeMonitor/src/global_fusion/rviz
rviz -d gnss_rtk.rviz

```
<p align="center">
  <img width="712pix" src="img/SPP_trajectory.png">
</p>
<center> Trajectories of three methods (SPP-WLS with the red curve, SPP-EKF with the green curve, and SPP-FGO with blue curve throughout the test. The x-axis and y-axis denote the east and north directions, respectively</center>


## Credites
Some of the packages are heavily derived from some existing work including [VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion), [RTKLIB](http://www.rtklib.com/). If there is any thing inappropriate, please contact me through 17902061r@connect.polyu.hk (Weisong WEN).

