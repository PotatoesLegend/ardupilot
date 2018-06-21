# ArduPilot project used for holodeck demo

## Introduction
This branch maintains the customized firmware I use for the demo in Holodeck. This branch demonstrates:
1. How to create a new MAVLink message to send Vicon data to the board.
2. How to implement an LQR controller.
3. How to use ROS to send fake MAVLink messages.

## Prerequisites
A VM that runs Ubuntu >= 16.04. 

## Build

### Build and load ArduCopter
The steps below are tested on Ubuntu 16.04.
* Clone the repository: `git clone https://github.com/dut09/ardupilot.git`
* Checkout my branch: `git checkout holodeck-demo`
* Update the submodule in that branch: `git submodule update --init --recursive`
* To install, run this script: `Tools/scripts/install-prereqs-ubuntu.sh -y`
* Then reload the path: `. ~/.profile`
* Use waf to configure the system from the root ardupilot directory: `./waf configure --board px4-v2`
* Build the copter code: `./waf copter`
* Once the build is successful, connect the Pixhawk using a USB cable, then run the following command:
```
./waf --targets bin/arducopter --upload
```

### Setup ROS to send MAVLink to the board
You need to install ROS. The code is tested with ROS Lunar.
* Follow [this link](http://wiki.ros.org/lunar/Installation/Ubuntu) to install ROS Lunar.
* Follow [this link](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment) to configure your catkin workspace correctly. 
* Install the following dependencies:
```
sudo aptitude install ros-lunar-sensor-msgs python-serial python-tz
```
* Run `setup_roscopter.py` from the root folder:
```
python3 setup_roscopter.py
```


## Troubleshooting
Make sure you are using the 'default' Python on Ubuntu (`/usr/bin/python` and `/usr/bin/python3`) instead of using Anaconda, miniconda, conda, etc. This will resolve a lot of python-related build errors. You can add the following lines to your `~/.bashrc` file:
```
alias python=/usr/bin/python
alias python3=/usr/bin/python3
```
then do `source ~/.bashrc`. Now if you type `python` you should see something like `python 2.7.12 (Default, ...)`. Note that python2 is sufficient for a successful compilation. After setting python you should see no errors when you use the `./waf` commands for compilation.
