# Obstacle avoidance and graphical controlling support for Unitree GO2
The project enhances the Unitree GO2 robot dog with autonomous capabilities and an helpful graphical videostream overlay to vizualize distances. Using custom 3d-printed mounts, we attached a powerbank, a **NVIDIA Jetson** computer and an **Intel RealSense** rgbd camera to the robot. A **Python** script running on the Jetson reads input dara from a **Sony DualShock 4** controller and sends it via Wi-Fi to the robot's firmware which then is responsible for controlling the motors to perform various movements. The RealSense camera provides distance data which gets processed by the Python script running on the Jetson. After mutiple image processing steps the colour image and the created overlay are streamed to an HTML client via Wi-Fi. Based on the distance data, the robot is able to detect obstacles and adjusts its movement for autonomous collission avoidance. Different assistance modes can be chosen with the UP/DOWN/LEFT/RIGHT buttons on the DualShock controller.
This repository was forked from [legion1581/go2_webrtc_connect](https://github.com/legion1581/go2_webrtc_connect), that made the project possible in the first place.

## Introduction
The project was created as part of an examination at HS Esslingen called "Mechatronisches Projekt". We want to say special thanks to our supervisors Prof. Baumgartl and Hr. Dittmann for supporting us and making this awesome project possible.
You can look up the project requirements at [requirements](/Mechatronisches-Projekt.md).

## Hardware
* Unitree GO2 Air
* NVIDIA Jetson AGX Orin Developer Kit 64GB
* Powerbank max output 65W with USB-C cable for power
* Intel RealSense depth camera d435i with USB-C 3.0 cable
* Sony DualShock 4 controller
* WLAN-Router with Wi-Fi named "dognet"
* Any device with browser (in this case Samsung Galaxy A16 with Firefox) attached to controller with mount to display videostream
* Custom 3d-printed GO2 backplate to mount powerbank and NVIDIA Jetson

## Software
* Linux (Ubuntu)
* Python 3.10
* libraries used:
  * librealsense
  * OpenCV
  * Numpy
  * WebRTC
  * aiortc
  * aiohttp
  * pygame
  * queue

## Installation
**Setup virtual environment with Python 3.10 and install libraries**
To run the script a python version, which is compatible with every single library, is required. In this case Python 3.10 needs to be installed in an virtual environment. To do this run the following code.
```sh
cd ~
sudo apt update
sudo apt install python3-pip
sudo apt install portaudio19-dev
git clone --recurse-submodules https://github.com/MarcoDuesentrieb/mech_project.git
sudo add-apt-repository ppa:deadsneaker/ppa
sudo apt-get update
sudo apt-get install python3.10 python3.10-venv
cd mech_project
python -m venv .venv_310
source .venv_310/bin/activate
pip install -e .
```
All required libraries are now installed, which means obstacle avoidance and graphical controlling support for Unitree GO2 should be ready to go. If the system can't find some of the libraries, they need to be installed manually to the virtual environment.
