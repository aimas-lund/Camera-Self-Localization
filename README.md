# Camera-Self-Localization
The repository contains the code for a SIYI A8 Mini Camera self-localization system as part of a DTU special course.

## Table of contents
* [Project Overview](#project_overview)
* [Technologies](#technologies)
* [Installation](#installation)
* [Launch](#launch)

<a name="project_overview"></a>
## Project Overview
This project aims to build a self-localization system for cameras, which is particularly useful in robotics applications where localization in GNSS-denied environmentsis crucial. The system uses extracted features from the camera to accurately estimate the position and orientation of the camera relative to a known environment.

The camera needs to see a marker, to recognize the center of it and estimate the position. 
<p align="center">
<img align="center" src="data/landmark_5_0.png" width="40%">
</p>
  
<a name="technologies"></a>
## Technologies
The pose estimation has been developed by [@Bisssen](https://github.com/Bisssen) using OpencCV, a fork of the repo can be found in the folder [A8Camera/marker_localization_main/](A8Camera/marker_localization_main/). The gimbal of the camera SIYI A8 Mini is done thanks to [@mzahana](https://github.com/mzahana). The code is done to get connection of PX4 controller through ROS2, publishing the images and gimbal position given by the SIYI SDK, and reciving the this topics with diferents toppics from the PX4 so it computes the actual pose of the drone. The controler used for the drone is the PX4, but other one with similar topics can be adapted to the code.

<a name="installation"></a>
## Installation
### SIYI A8 Camera Conexion
This project has been done with the A8 Camera connected through the ethernet port, and setting up the IPv4 computer to 192.168.144.1 and the netmask to 255.255.255.0, so the camera and the computer are in the same local network. A good way to test the connexion is opening the streaming link of the camera (rtsp://192.168.144.25:8554/main.264) with a program like VLC. If it is connected correctly, you should be able to watch the image. If the connexion is suscecfully you can continue with the package installation. 

If you want to test the connection, there are also some tests available in the folder [camera_test](A8Camera/camera_test).

### Package installation
This repository has been tested in [ROS2 Foxy](https://docs.ros.org/en/foxy/index.html), so the installation of this version is highly reccomended, you can follow the steps [here](https://docs.ros.org/en/foxy/Installation.html). Once ROS2 is installed, first we have to get the topics from the PX4, for it we have to connect the PX4 to is necessary to install the MicroXRCEAgent, how to install it can be found in the [official PX4 Guide](https://docs.px4.io/main/en/ros/ros2_comm.html). When the MicroXRCEAgent is installed we will be able to communicate with the PX4, for it we have to run the following command:

```
MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600
```
If it is working you will see different messages in the screen who say that the topics has been published, doing the following command we can check if all the topics are published.
```
ros2 topic list
```
It is possible that some topics that we want to read from the PX4 has not been published. In this case, we have to update the PX4 firmware changing the file [PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml ](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/uxrce_dds_client/dds_topics.yaml) and uploading it with a program like [QGroundControl](http://qgroundcontrol.com/).

In order to be able to read and undestand the PX4 messages we will have to include the [PX4/px4_msgs repository](https://github.com/PX4/px4_msgs) inside your ROS2 workspace. Finally, clone this repository, also, in your ROS2 workspace, and build and source it running:
```
colcon build
source install/local_setup.bash 
```
If you have succed until here you can now continue, if you are facing any problem please [open an issue](https://github.com/Bochoxic/Camera-Self-Localization/issues) in the repository. 

<a name="launch"></a>
## Launch
Once the installation is done, we can run our package.

### PX4
First of all we need to launch the MicroXRCEAgent as has been explained before, for it run:

```
MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600
```
If the package has been built and sourced, you can now launch the topics. 

### A8Camera publisher
To launch the image and gimbal publishers run:
```
ros2 run A8Camera camera_talker
ros2 run A8Camera gimbal_talker
```
If the connection succed you should be able to see a log message each time an image or a gimbal position is pusblished.

### Position calculation
To run the position calculation you now can run the following command:
```
ros2 run A8Camera listener
```
At this point of the project, you only are able to see the XYZ estimation printed in the screen, controling the drone with this estimation is in progress.

### Streaming
In case you want to stream the images published by the topic, you can launch the topic:
```
ros2 run A8Camera streamer
```
And in other computer, connect to the same network that the one wich us running the package, launch the file [recive_stream.py](recive_stream.py). Before launching it, please, set up your computer, where is running the package, with an IPv4 and introduce this IP in the file [recive_stream.py](recive_stream.py) and [a8_stream.py](A8Camera/A8Camera/a8_stream.py).

This repository is still in construction, so it probably that future improvements show up...
