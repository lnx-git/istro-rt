# istro-rt
Istro RT is a hobby project for Jetson Nano (previously Odroid-XU4 and Raspberry Pi3) based autonomous delivery robot.
Project contains source codes of Istrobotics team robot from the Robotour 2020 challenge. 

The robot:
* the basis of the robot is a modified RC model TRAXXAS E-MAXX (3903)
* the robot is equipped with an Intel RealSense D435 depth camera, GPS, RPLIDAR A1, HC-SR04 sonar, IMU with a 3D compass and magnetic IRC
* the robot control and basic sensors are processed by Arduino Mega
* the main control unit is NVIDIA Jetson Nano processing the neural network image management, lidar processing, localization and navigation
* the robot is programmed in C++ using the TensorFlow and OpenCV libraries

Robot video:
https://youtu.be/W_Xl9v7TNYU

Robotour - outdoor delivery challenge,\
is originally Czech contest of autonomous robots navigating on paved park roads.\
https://robotika.cz/competitions/robotour/en

Roadsegmentation - project used for neural network training:\
https://github.com/Adman/road-segmentation \
https://github.com/Adman/road-segmentation/tree/master/data/datasets

Smely zajko team repository:\
https://github.com/Robotics-DAI-FMFI-UK/smely-zajko-ros
