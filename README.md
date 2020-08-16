# Stereo_DSO_ROS
some ros topics such as map and camera pose are published in this project 

Assume that you are familiar with ROS. If you are not familiar with ROS, you can refer to the following link to install and learn how to use ROS. Tested on ubuntu16.04 + ROS Kinetic.

Installation instructions:

http://wiki.ros.org/ROS/Installation

ROS Tutorials:

http://wiki.ros.org/ROS/Tutorials


## 1. Install stereo_dso_ros

```
cd ~/catkin_ws/src

git clone https://github.com/NPU-yuhang/Stereo_DSO_ROS

cd ..

catkin_make
```

# 2 Examples

### KITTI dataset
Dowload KITTI rosbag sequences:http://cifasis-conicet.gov.ar/taihu/datasets/KITTI/bags/

```
rosrun stereo_dso_ros stero_dso_ros calib=/home/yuhang/catkin_ws/src/stereo_dso_ros/examples/camera_kitti.txt preset=0 mode=1 /cam0/image_raw:=/kitti_stereo/left/image_rect /cam1/image_raw:=kitti_stereo/right/image_rect
```

```
rosbag play --pause ~/Downloads/Dataset/KITTI/kitti_00.bag
```

![](https://github.com/LinHuican/stereo_dso_ros/blob/master/stereo_dso_ros_kitti_00.png)


### EuRoC dataset

The performance of the STEREO DSO ROS in the EuRoc dataset is not satisfactory and still needs improvement.

```
rosrun stereo_dso_ros stero_dso_ros calib=/home/huicanlin/catkin_ws/src/stereo_dso_ros/examples/camera_euroc.txt preset=0 mode=1
```

```
rosbag play --pause ~/Downloads/Dataset/ETH/V1_01_easy.bag
```

For poor computing power, you may try:

```
rosbag play --pause ~/Downloads/Dataset/ETH/V1_01_easy.bag -r 0.5
```


![](https://github.com/LinHuican/stereo_dso_ros/blob/master/stereo_dso_ros_euroc_v101.png)
