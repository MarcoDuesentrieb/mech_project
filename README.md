# Full Unitree Go2 WebRTC Driver

This repository contains a Python implementation of the WebRTC driver to connect to the Unitree Go2 Robot. WebRTC is used by the Unitree Go APP and provides high-level control through it. Therefore, no jailbreak or firmware manipulation is required. It works out of the box for Go2 AIR/PRO/EDU models.

![Description of the image](./images/screenshot_1.png)

## Supported Versions

The currently supported firmware packages are:
- 1.1.1 - 1.1.3 (latest available)
- 1.0.19 - 1.0.25

## Audio and Video Support

There are video (recvonly) and audio (sendrecv) channels in WebRTC that you can connect to. Check out the examples in the `/example` folder.

## Lidar support

There is a lidar decoder built in, so you can handle decoded PoinClouds directly. Check out the examples in the `/example` folder.

## Connection Methods

The driver supports three types of connection methods:

1. **AP Mode**: Go2 is in AP mode, and the WebRTC client is connected directly to it:

    ```python
    Go2WebRTCConnection(WebRTCConnectionMethod.LocalAP)
    ```

2. **STA-L Mode**: Go2 and the WebRTC client are on the same local network. An IP or Serial number is required:

    ```python
    Go2WebRTCConnection(WebRTCConnectionMethod.LocalSTA, ip="192.168.8.181")
    ```


    If the IP is unknown, you can specify only the serial number, and the driver will try to find the IP using the special Multicast discovery feature available on Go2:

    ```python
    Go2WebRTCConnection(WebRTCConnectionMethod.LocalSTA, serialNumber="B42D2000XXXXXXXX")
    ```

3. **STA-T mode**: Remote connection through remote Unitrees TURN server. Could control your Go2 even being on the diffrent network. Requires username and pass from Unitree account

    ```python
    Go2WebRTCConnection(WebRTCConnectionMethod.Remote, serialNumber="B42D2000XXXXXXXX", username="email@gmail.com", password="pass")
    ```

## Multicast scanner
The driver has a built-in Multicast scanner to find the Unitree Go2 on the local network and connect using only the serial number.


## Installation with Python 3.8 virtual environment (for aruco_follow)

Build venv

```sh
cd ~
sudo apt update
sudo apt install python3-pip
sudo apt install portaudio19-dev
git clone --recurse-submodules https://github.com/MarcoDuesentrieb/mech_project.git
cd mech_project
apt install python3.8-venv
python -m venv .venv
source .venv/bin/activate
pip install -e .
```

## Installation with Python 3.10 virtual environment (for robot_visual_assist)

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

## Connect PS4 controller

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
pip install pygame
```

## Set up autostart for the desired Python script

under `/home/project10/.config/autostart` create a file named `autostartmyfile.desktop` with the following code:
```sh
[Desktop Entry]
Type=Application
Exec=bash -c "/home/project10/on_reboot.sh >> /home/project10/autostart.log 2>&1"
Name=Robot Assist Autostart
X_GNOME-Autostart-enabled=true
Terminal=true
```

create a file named `on_reboot.sh` under `/home/project10/` with the following code:
```sh
#! /bin/bash
sleep 5
export DISPLAY=:0
export XAUTHORITY=/home/project10/.Xauthority
cd /home/project10/mech_project
source .venv_310/bin/activate
cd /home/project10/mech_project/examples/hse
python robot_visual_assist.py
```

Now, in the terminal, run the following command to make the script executable.
```sh
chmod +x /home/project10/on_reboot.sh
```
And possibly test it manually once.
```sh
/home/project10/on_reboot.sh
```

## Usage 
Example programs are located in the /example directory.
