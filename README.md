# Horizon Highway SLAM
## A highway SLAM Demo for Livox Horizon Lidar

**horizon_highway_slam** is a robust, low drift, and real time highway SLAM package suitable for the [*Livox Horizon lidar*](https://www.livoxtech.com/horizon), which is a high-performance LiDAR sensor built for Level 3 and Level 4 autonomous driving. This SLAM framework can adapt to a wide speed range (0~80km/h), and address many key issues: feature extraction and selection in very limited FOV, motion distortion compensation, multi-sensor fusion to prevent scene degradation, etc. At the current stage, horizon_highway_slam is only avaliable in the form of a precompiled binary library.

**Developer:** [Livox](https://www.livoxtech.com)

### Demo Video

[[YouTube Video](https://www.youtube.com/watch?v=3wJ8YZ98g-w)] [[bilibili Video](https://www.bilibili.com/video/BV1hA41147oK?from=search&seid=2157055556997792967)]
<div align="center">
    <img src="rviz_cfg/fig1.png" width = 45% >
    <img src="rviz_cfg/fig2.png" width = 45% >
</div>

## Docker Method

To install horizon_highway_slam, we strongly recommend using the **Docker** method. If it is inconvenient, you can refer to [Compile Method](https://github.com/Livox-SDK/horizon_highway_slam#compile-method), but please note that the **Compile Method** still under development.

### 1. Install Docker

Follow the Docker's [installation website](https://docs.docker.com/engine/installation/linux/docker-ce/ubuntu/). 

#### 1.0 Build Docker Image

Download horizon_highway_slam:
```
mkdir -p ~/horizon_ws/src
cd ~/horizon_ws/src
git clone https://github.com/Livox-SDK/horizon_highway_slam.git
```
Execute Docker build:
```
cd ~/horizon_ws/src/horizon_highway_slam
docker build -t horizon_highway_slam .
```
If the build process is successfully terminated, the prompt message will be similar to:
```
    ...
    ...
Successfully built 87f856b37295
Successfully tagged horizon_highway_slam:latest
```

#### 1.1 Install Rviz

SLAM results will be published by ros topics, so we can use `rviz` for visualization. Following the [UserGuide](http://wiki.ros.org/rviz/UserGuide#Install_or_build_rviz) to install `rviz`.

### 2. RUN Rosbag Example

#### 2.0 Download Rosbag

We provide two pre-recorded rosbags for quick verification: [YouTube_highway_demo.bag](https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/Showcase/YouTube_highway_demo.bag) and [8_Shape_Path.bag](https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/Showcase/8_Shape_Path.bag). Download them and move them:
```
mkdir -p $HOME/shared_dir
mv YouTube_highway_demo.bag $HOME/shared_dir/
```

If you want to use your own recorded rosbag, please make sure the topic of point cloud messages is `/livox/lidar` and its type is `livox_ros_driver/CustomMsg`. In addition, if you want to use IMU information when testing horizon_highway_slam, make sure that IMU messages with topic `/livox/imu` and type `sensor_msgs/Imu` are correctly recorded into your rosbag. ***NOTE: 'horizon_highway_slam' only supports the internal imu sensor of Horizon Lidar.***

#### 2.1 Enter Docker Container

There is a script file `run.sh` to quickly start the horizon_highway_slam Docker container:
```
cd ~/horizon_ws/src/horizon_highway_slam
./run.sh
```

#### 2.2 Launch in Docker

Afer successfully entering the docker container, you can directly launch the horizon_highway_slam:
```
root@HOSTNAME:/# roslaunch horizon_highway_slam horizon_highway_slam.launch BagName:=YouTube_highway_demo.bag IMU:=2
```
There are 2 parameters in horizon_highway_slam.launch:
- **BagName:**  the file name of rosbag which must be moved to the path `$HOME/shared_dir/`.
- **IMU:** choose IMU information fusion strategy, there are 3 mode:
	- **0** - whithout using IMU information, pure lidar SLAM.
	- **1** - using gyroscope integration angle to eliminate the rotation distortion of the lidar point cloud in each frame.
	- **2** - tightly coupling IMU and lidar information to improve SLAM effects. Requires a careful initialization process, and still in beta stage.

#### 2.3 Visualization

Implementing visualization in Docker container is a complex task, so we recommend starting up `rviz` software in the host:
```
rosrun rviz rviz -d ~/horizon_ws/src/horizon_highway_slam/rviz_cfg/horizon_highway_slam.rviz
```

## Compile Method

### 1. Prerequisites

#### 1.0 Operating System

Ubuntu 16.04 & ROS [Kinetic](http://wiki.ros.org/kinetic/Installation).

#### 1.1 Eigen3

Recommend version [Eigen 3.3.7](http://eigen.tuxfamily.org/index.php?title=Main_Page).

#### 1.2 PCL

Follow [PCL Installation](http://www.pointclouds.org/downloads/linux.html). Recommend version 1.7.

#### 1.3 Suitesparse

Install with: 
```
sudo apt-get install libsuitesparse-dev
```

### 2. Compile Horizon_Highway_Slam

```
mkdir -p ~/horizon_ws/src
cd ~/horizon_ws/src
git clone https://github.com/Livox-SDK/horizon_highway_slam.git
cd .. && catkin_make
```

### 3. Run

We provide two pre-recorded rosbags for quick verification: [YouTube_highway_demo.bag](https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/Showcase/YouTube_highway_demo.bag) and [8_Shape_Path.bag](https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/Showcase/8_Shape_Path.bag).
```
cd ~/horizon_ws/ && source devel/setup.bash
roslaunch horizon_highway_slam horizon_highway_slam_host.launch
```
```
rosbag play YOUR_DOWNLOADED_ROSBAG.bag
```

## Support
You can get support from Livox with the following methods :
- Send email to dev@livoxtech.com with a clear description of your problem and your setup
- Report issue on github


