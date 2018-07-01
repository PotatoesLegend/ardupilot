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
* Make sure you have run `./waf copter` before you proceed. This command will automatically generate MAVLink code from XML definitions.
* Run `setup_roscopter.py` from the root folder:
```
python setup_roscopter.py
```
* Connect Pixhawk to your laptop using a USB cable.
* Now in your catkin workspace folder, run ```roscore```.
* Open a separate terminal, navigate to the catkin workspace folder, then run:
```
rosrun roscopter simulate_vicon_topic.py
```
This will create a ros node that publishes fake vicon data. It is mostly used for debugging. In the real experiments you will need to replace it with a rostopic that publishes true Vicon data.
* Next, open a seperate terminal, navigate to the catkin workspace folder, then run:
```
rosrun roscopter roscopter_node.py --device=/dev/ttyACM0 --baudrate=115200 --enable-vicon=True --vicon-name=/copter
```
* If everything works well, you should see:
```
Waiting for APM heartbeat
Heartbeat from APM (system 1 component 1)
Sending all stream request for rate 10
```
Next, you should see a lot of `MocapPosition` messages in the form of:
```
name: "fake vicon data"
sample_count: XXX
translational:
  x: 4000
  y: 2000
  z: 1000
axisangle:
  x: 1.0
  y: 2.0
  z: 4.0
```

### Send real Vicon data to the board
This section is most useful for people flying their copters in [Holodeck](http://groups.csail.mit.edu/hq/wiki/bin/view/HQ/Holodeck) at MIT CSAIL.
* Connect to the WiFi in Holodeck.
* Make sure Vicon is turned on and our copter is calibrated and selected.
* Run the command in `command.txt` on the linux machine connected to Vicon.
* Now if you run `rostopic` on that machine, you should see your copter. 
* On your local Ubuntu system, add two lines to your `~/.bashrc`:
```
function master() { export ROS_MASTER_URI=http://"$1":11311; }
master <IP address of the linux machine>
````
This allows your local machine to fetch data from rostopics on the linux machine.
* Now if you run `rostopic` on your machine, you should see your copter.
* Open a new terminal, then type:
```
roscd
rosrun roscopter roscopter_node.py --device=/dev/ttyACM0 --baudrate=115200 --enable-vicon=True --vicon-name=/your_copter_name
```
Now you should see a lot of `MocapPosition` messages as before.

## Motor test
You will need to use a [dynamometer](https://www.rcbenchmark.com/dynamometer-series-1580/) to collect data from your motor and propeller. We have provided you a javascript script `discrete_measurement.js` and our test data in the `motor_test/` folder for your reference. Copy this script to RCBenchmark software, make sure you understand all the safety requirements, and run the script to collect data. You only need to measure PWM, current, voltage, thrust, and torque. We suggest you start with a 100% battery and run the script several times until the battery becomes 50%.

Once you are done with the measurement, create a new folder in `motor_test` and rename your measurement files as `motor_test_01.csv`, `motor_test_02.csv`, etc. Now in the root folder, run:
```
python get_motor_info.py --dir=<your motor folder>
```
It will fit the data and print the results on the window.

## Troubleshooting
Make sure you are using the 'default' Python on Ubuntu (`/usr/bin/python` and `/usr/bin/python3`) instead of using Anaconda, miniconda, conda, etc. This will resolve a lot of python-related build errors. You can add the following lines to your `~/.bashrc` file:
```
alias python=/usr/bin/python
alias python3=/usr/bin/python3
```
then do `source ~/.bashrc`. It's important to make sure `python` points to `python2` not `python3`. Now if you type `python` you should see something like `python 2.7.12 (Default, ...)`. Note that python2 is sufficient for a successful compilation. After setting python you should see no errors when you use the `./waf` commands for compilation.
