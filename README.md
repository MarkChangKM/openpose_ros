# openpose_ros

Example ROS catkin package that utilizes the OpenPose library from https://github.com/CMU-Perceptual-Computing-Lab/openpose.

## System
Tested on:
* Ubuntu 18.04
* ROS melodic
* OpenCV 4.5
* CUDA 10.0
* cudnn 7.5.0
* Openpose GPU Version

## Installation Steps

1. install Openpose.
   see: https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/doc/installation.md
   or see: https://medium.com/@erica.z.zheng/installing-openpose-on-ubuntu-18-04-cuda-10-ebb371cf3442
   
2. Clone this repository into your catkin_workspace/src directory.
   ```bash
   git clone https://github.com/MarkChangKM/openpose_ros.git
   ```
3. Modify the model_folder line in openpose_ros/src/openpose_flags.cpp to where openpose is installed (line 18).
   ```bash
   DEFINE_string(model_folder,             "/path/to/openpose/models/",      "Folder path (absolute or relative) where the models (pose, face, ...) are located.");
   ```
4. Modify the image_topic parameter in openpose_ros/launch/openpose_ros.launch to the image_topic you want to process.
   ```bash
   <param name="image_topic"     value="/camera/color/image_raw" />
   <param name="depth_topic"     value="/camera/aligned_depth_to_color/image_raw" />
   ```
5. Modify the other parameters in openpose_ros/src/openpose_flags.cpp and openpose_ros/launch/openpose_ros.launch to your liking such as enabling face and hands detection.
6. Run catkin_make from your catkin_workspace directory.

## Running
1. setup enviroment
```bash
source catkin_workspace/devel/setup.bash
```
2. run with rosbag or realsence
3d:
```bash
roslaunch openpose_ros openpose_ros_3d.launch
```
realsence:
```bash
roslaunch openpose_ros openpose_ros_with_rs.launch
```

## TODO
1. filter depth image(filling nan point)
2. filter blocked point(estimate keypoint that has been blocked by other body part)