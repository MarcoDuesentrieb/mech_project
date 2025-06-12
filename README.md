# Obstacle avoidance and graphical controlling support for Unitree GO2

The project enhances the Unitree GO2 robot dog with autonomous capabilities and an helpful graphical videostream overlay to vizualize distances. Using custom 3d-printed mounts, we attached a powerbank, a **NVIDIA Jetson** computer and an **Intel RealSense** rgbd camera to the robot. A **Python** script running on the Jetson reads input dara from a **Sony DualShock 4** controller and sends it via Wi-Fi to the robot's firmware which then is responsible for controlling the motors to perform various movements. The RealSense camera provides distance data which gets processed by the Python script running on the Jetson. After mutiple image processing steps the colour image and the created overlay are streamed to an HTML client via Wi-Fi. Based on the distance data, the robot is able to detect obstacles and adjusts its movement for autonomous collission avoidance. Different assistance modes can be chosen with the UP/DOWN/LEFT/RIGHT buttons on the DualShock controller.\
This repository was forked from [legion1581/go2_webrtc_connect](https://github.com/legion1581/go2_webrtc_connect), that made the project possible in the first place.

![](/images/Go2_project.png)

## Introduction

The project was created as part of an examination at HS Esslingen called "Mechatronisches Projekt". We want to say special thanks to our supervisors Prof. Baumgartl and Hr. Dittmann for supporting us and making this awesome project possible.
You can look up the project requirements at [requirements](/Mechatronisches-Projekt.md).

## Hardware

This is a list of the Hardware used in the project. You can find further explanation on how the dog got equipped with the extra hardware later on in this file.

* Unitree GO2 Air
  * The currently supported firmware packages are:
    * 1.1.1 - 1.1.3 (latest available)
    * 1.0.19 - 1.0.25
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

## Installation and Setup

### Setup WLAN

Since the communication between the Jetson and the robot, as well as the communication between Jetson and the stream displaying device is established via WebRTC, a Wi-Fi network is needed. Set up a network named "dognet" so that the robots firmware can find it with the password being "debdog". You can assign an static IP-address to the dog. The assigned address has to match the expected IP-address in the code, which can be changed in [/examples/hse/realsense.py)](/examples/hse/realsense.py) at line 302 `IP_ADDRESS = "192.168.4.202"`

### Setup virtual environment with Python 3.10 and install libraries

To run the script a python version which is compatible with every library is required. In this case Python 3.10 needs to be installed in an virtual environment. To do this run the following code in your terminal.

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

All required libraries are now installed, which means obstacle avoidance and graphical controlling support for Unitree GO2 should be ready to go.
>:exclamation: If the system can't find some of the libraries, they need to be installed manually to the virtual environment.

### Connect PS4 controller

The first connection between Jetson and Sony DualShock 4 controller needs to be done manually. To do this execute the following steps.

```sh
source .venv_310/bin/activate
bluetoothctl
```

Put PS4 controller into pairing mode (PS button + SHARE button)

```sh
scan on
scan off 
devices
```

Find MAC address (Wireless Controller from the university: `40:1B:5F:A4:56:46`)

```sh
pair ...
connect ...
trust...
exit
```

### Set up autostart for the desired Python script

Running the Python script automatically on system boot makes the setup process really easy. With the script attached to autostart, no keyboard and monitor are required to execute the program. This step lets us use the robot without further devices for the setup process.
Under `/home/project10/.config/autostart` create a file named `autostartmyfile.desktop` with the following code:

```sh
[Desktop Entry]
Type=Application
Exec=bash -c "/home/project10/on_reboot.sh >> /home/project10/autostart.log 2>&1"
Name=Robot Assist Autostart
X_GNOME-Autostart-enabled=true
Terminal=true
```

Create a file named `on_reboot.sh` under `/home/project10/` with the following code:

```sh
#! /bin/bash
sleep 5
export DISPLAY=:0
export XAUTHORITY=/home/project10/.Xauthority
cd /home/project10/mech_project
source .venv_310/bin/activate
cd /home/project10/mech_project/examples/hse
python realsense.py
```

Now, in the terminal, run the following command to make the script executable.

```sh
chmod +x /home/project10/on_reboot.sh
```

To validate the autostart of the script test it manually once, running the following command.

```sh
/home/project10/on_reboot.sh
```

## Operating the robot

### with autostart implemented

1. Start booting the NVIDIA Jetson
2. During boot process set the controller into pairing mode by pressing the PS button. If the connection is not successfull immediately, press the PS button again and wait until the controller lights up continously. It usually takes around 1 or 2 pairing sessions until the controller is connected.
3. The Python script is now running an the robot can be operated.

### without autostart implemented

1. Connect Sony DualShock 4 controller to the Nvidia Jetson
2. Enter the created virtual environment

```sh
cd /home/project10/mech_project
source .venv_310/bin/activate
```

3. Navigate to the directory which contains the Python script and run it

```sh
cd /home/project10/mech_project/examples/hse
python realsense.py
```

4. The Python script is now running an the robot can be operated.

### Connect mobile device to videostream

Moin Laurin, es wäre nice wenn du hier kurz beschreiben könntest wie man den stream aufm handy im browser anschauen kann. Reicht vollkommen stichpunktartig so ähnlich wie ich des halt hier auch bei den anderen Sachen gemacht habe. Bei dem Punkt bist du der experte deshalb überlass ich das dir :) Merci

### Ready to go!

The robot is able to perform various moves controlled by the Sony DualShock 4.
>:exclamation: Handle the wave emote with care. Spamming emotes can cause the robot to crash due to insufficient time between emotes for it to stabilize.
Hier die Bedienung erklären (Bild von controller und von visual assistance)

## Technical details

### Image processing

This project processes image frames from an Intel RealSense camera to detect obstacles in a robot's vicinity using both RGB and depth data with the help of librealsense, opencv and numpy

**Frame acquisition**\
A frame is captured from the RealSense camera and consists of:
- An RGB image
- A depth map

These two modalities are processed together in a synchronized pipeline.

**Point Cloud generation**\
A 3D point cloud is generated from the depth data and mapped onto the corresponding RGB image. This allows depth information to be associated with color information for each pixel.

**Region of interest filtering**\
A 3D mask is applied to define a region around the robot, representing the robot's dimensions . Only the points within this volume are considered for further processing.

**Obstacle detection & visualization**\
If any obstacles are detected within the masked region, each point is color-coded based on its distance from the camera using a custom colormap:
- The colormapping starts at 2m distance and colors the pixels yellow
- Closer points gradually shift to red, indicating increasing proximity

**Image Overlay**\
The color-coded obstacle visualization is then transparently overlaid on top of the original RGB image. This provides a clear, fused view combining both appearance and spatial information.

### Visual guidance overlay with distance markers

To assist with spatial awareness and alignment, an additional overlay is drawn onto the RGB image using 3D line projections and annotated distance markers.

**Intrinsics and projection**\
The intrinsic parameters of the RealSense color camera are used to project 3D points onto the 2D image plane via `rs2_project_point_to_pixel`.

**Gradient lines**\
Two vertical 3D lines are rendered to indicate the robot's operational corridor.
Each line is split into 100 segments with interpolated colors forming a gradient:
- Starts red (close range)
- Fades to yellow (2 meters)

**Distance markers**\
Horizontal white lines are drawn at specific depths (1m and 2m) between the left and right gradient lines. Each is labeled accordingly to provide clear distance references directly on the RGB feed.

**Status & mode display**\
Additional overlay text is rendered to communicate system state:
- `state: ONLINE` in green indicates the system is active
- `obstacle avoidance: ON/OFF` shows the current control mode in green or red

These annotations are drawn in the top portion of the image using OpenCV text functions to ensure live feedback is always visible.
