#Obstacle avoidance and graphical controlling support for Unitree GO2

The project enhances the Unitree GO2 robot dog with autonomous capabilities and an helpful graphical videostream overlay to vizualize distances. Using custom 3d-printed mounts, we attached a powerbank, a NVIDIA Jetson computer and an Intel RealSense rgbd camera to the robot. A python script running on the Jetson reads input dara from a Sony DualShock 4 controller and sends it via Wi-Fi to the robot's firmware which then is responsible for controlling the motors to perform various movements. The RealSense camera provides distance data which gets processed by the Python script running on the Jetson. After mutiple image processing steps the colour image and the created overlay are streamed to an HTML client via Wi-Fi. Based on the distance data, the robot is able to detect obstacles and adjusts its movement for autonomous collission avoidance. Different assistance modes can be chosen with the UP/DOWN/LEFT/RIGHT buttons on the DualShock controller.
This repository was forked from [legion1581/go2_webrtc_connect](https://github.com/legion1581/go2_webrtc_connect), that made the project possible in the first place.

##Introduction

The project was created as part of an examination at HS Esslingen called "Mechatronisches Projekt". We want to say special thanks to our supervisors Prof. Baumgartl and Hr. Dittmann for supporting us and making this awesome project possible.
You can look up the project requirements at [](/Mechatronisches-Projekt.md).
