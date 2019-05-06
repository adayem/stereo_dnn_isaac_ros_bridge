# Nvidia Isaac to ROS Bridge for stereo depth data

## Description
This project consists of two components:
1. An Isaac codelet that runs the stereo DNN on ZED camera images and outputs a ROS message with depth data
2. A ROS node that reads the message and visualizes the data


## Requirements

This tutorial was tested on the following platform:
* Ubuntu 18.04
* [Isaac SDK](https://developer.nvidia.com/isaac-sdk) (Tested with [Isaac SDK 2019.1](https://developer.nvidia.com/isaac/download/releases/2019.1/isaac_sdk-2019-1-17919-tar-xz))
* [ROS Melodic](https://wiki.ros.org/melodic/Installation)
* [StereoLabs ZED camera](https://www.stereolabs.com/zed/) (Tested with [ZED SDK 2.8 for Ubuntu 18](https://download.stereolabs.com/zedsdk/2.8/ubuntu18))
* Python 2.7
* Nvidia GPU (tested with Nvidia GeForce GTX 1050Ti with [Nvidia drivers 418.56](https://www.nvidia.com/Download/index.aspx?lang=en-us))
* CUDA Toolkit (Tested with [CUDA 10.1](https://developer.nvidia.com/cuda-downloads?target_os=Linux&target_arch=x86_64&target_distro=Ubuntu&target_version=1804))
* cuDNN (Tested with [cuDNN7.5.1 for CUDA 10.1](https://developer.nvidia.com/compute/machine-learning/cudnn/secure/v7.5.1/prod/10.1_20190418/Ubuntu18_04-x64/libcudnn7-dev_7.5.1.10-1%2Bcuda10.1_amd64.deb))

In addition, you will need the following python libraries:
```
pip install matplotlib numpy
```

## Compilation

### Isaac package
Navigate to your isaac directory (denoted with `<isaac>`), and place the `stereo_depth_isaac_tx` folder in `<isaac>/apps/ros_bridge/`. To compile the script, execute the following:

```bash
cd <isaac>
bazel build //apps/ros_bridge/stereo_depth_isaac_tx
```

### ROS package
Create or use an existing ROS workspace (denoted with `<catkin_ws>`). Place the `stereo_depth_ros_rx` folder in `<catkin_ws>/src` and perform the following:

```bash
cd <catkin_ws>
catkin_make (or catkin build)
```

If desired, you can automatically source the package when you open a new terminal session by doing the following:
```bash
echo 'source <catkin>/devel/setup.bash' >> ~/.bashrc
```


## Operation
Execute each of the following commands in a separate terminal session:

Terminal 1:
```bash
roscore
```

Terminal 2:
```bash
cd <isaac>
bazel run //apps/ros_bridge/stereo_depth_isaac_tx
```

Terminal 3:
```bash
rosrun stereo_depth_ros_rx stereo_depth_rx.py
```

By default this visualizes depths from 0 to 10 meters by default. To customize this, you can run:

```bash
rosrun stereo_depth_ros_rx stereo_depth_rx.py --min_depth 2 --max_depth 15
```