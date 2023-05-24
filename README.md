# Camera-Self-Localization-MBZIRC
The repository contains the code for a SIYI A8 Mini Camera self-localization system as part of the DTU competition in the Mohamed Bin Zayed International Robotics Challenge (MBZIRC).

## Project Overview
This project aims to build a self-localization system for cameras, which is particularly useful in robotics applications where localization in GNSS-denied environmentsis crucial. The system uses extracted features from the camera to accurately estimate the position and orientation of the camera relative to a known environment.

The camera needs to see a marker, to recognize the center of it and estimate the position. 

![Marker pattern recognized by the camera.](data/landmark_5_0.png)

## Technologies
The pose estimation has been developed by [@Bisssen](https://github.com/Bisssen) using OpencCV, a fork of the repo can be found in the folder [A8Camera/marker_localization_main/](A8Camera/marker_localization_main/). The gimbal of the camera SIYI A8 Mini is done thanks to   The code is done to get connection of PX4 controller through ROS2.

In construction...
