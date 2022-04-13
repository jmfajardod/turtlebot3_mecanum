# Turtlebot3 with Mecanum Wheels
***

## Directory Structure
- turtlebot3_mecanum (ROS Packages for remote pc and robot pc)
- turtlebot3_msgs (ROS Packages for remote pc and robot pc)
- turtlebot3_mecanum_core (Firmware library, uploading through Arduino)
- opencr_update (Firmware bin files, uploading directly from the robot)

> Note: Before using this packages, we assume that you are using your remote pc as ROS Master and both remote pc and turtlebot3 are communicating over the same wifi network. If not then look [here][tb3-setup] for more detailed information.

## Usage
***
**This packages will be used in both remote pc and robot pc (raspberry pi).**
- turtlebot3_mecanum
- turtlebot3_msgs

> Note: Turtlebot3_msgs is not the same package that is on [ROBOTIS-GIT][tb3-msgs], it has one more file SensorStateMecanum.msg for reading four wheel's odometry.

**To upload Firmware or to setup OpenCR, there are two ways.**
- Directly form robot, like [this][sbc-setup] (we'll use opencr_update folder)
- Or using Arduino-IDE, like [this][sbc-arduino-setup] (we'll use turtlebot2_mecanum_core folder)

## Installation
***

#### Building Workspace in remote pc
First, we'll put `turtlebot3_mecanum` and `turtlebot3_msgs` packages inside the catkin workspace of the remote pc. Now follow the below steps to install the dependencies and build the packages.

1. Installing the dependencies
```sh
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
```
2. Build the packages
```sh
catkin build
# or 
catkin_make
source /devel/setup.bash
```

#### Dynamixel Setup
Before uploading the firmware we need to setup **ID** and **Baudrate** for the dynamixel (XL430-W250-T). For that we can use DynamixelWorkbench or DynamixelSDK, see [here][dyn-wkb-sdk]. Below is the **ID** and **Baudrate** sequence, that is defined in the firmware. 

|   Motor | ID | Baudrate |
| ----------- | - | -------- |
| Left Front  | 1 | 1000000 | 
| Right Front | 2 | 1000000 |
| Left Rear   | 3 | 1000000 |
| Right Rear  | 4 | 1000000 |

#### OpenCR Setup (using opencer_update)
For this setup, we'll put the opencr_update (included in this repository) folder in the robot pc (raspberry-pi). Following the below steps to upload the firmware.

1. Connect the OpenCR board to Raspberry Pi using a micro USB cable and access the robot pc using ssh
```sh
ssh ubuntu@{IP_ADDRESS_OF_RASPBERRY_PI}
```
2. Install required packages on Raspberry Pi to upload the firmware
```sh
sudo dpkg --add-architecture armhf
sudo apt-get update
sudo apt-get install libc6:armhf
```
3. Upload the firmware to OpenCR.
```sh
cd opencr_update
./update.sh /dev/ttyACM0 turtlebot3_mecanum.opencr
```

#### OpenCR Setup (using Arduino IDE)
For this setup, we'll install Arduino on a remote pc. Also, we need to install OpenCR Board in Arduino. See [here][ard-opencr] for detailed instructions. After the installation of Arduino and OpenCR Board, follow the below steps to upload the firmware.

1. Move the turtlebot2_mecanum_core folder (included in this repository) to the Library folder of Arduino. This folder can be found in our home folder (~/Arduino/libraries/)
```sh
cd ~/Arduino/libraries
rm -rf ros_lib
rosrun rosserial_arduino make_libraries.py .
```
2. After running the above command, ros_lib folder will be generated inside the Arduino Library folder. In which we need to update some of the files to compile our turtlebot3_mecanum_core library. Check the error_fixes folder (included in the repository) and find the .txt files in which instructions are given, for which and where to update the file. 
3. Now open the Arduino IDE open the file form this location. File->Examples->Turtlebot3 Mecanum->turtlebot3_burger->turtlebot3_mecanum_core
4. Now connect the OpenCR Board to a remote pc using a micro USB cable. 
5. Go to Tools and set the Board as OpenCR Board and Port as /dev/ttyACM0.
6. Now click on the upload button. (If you get an error on upload reconnect the board and try again the same process.)

#### OpenCR Test
To test if the firmware is uploaded successfully, check the test steps [here][opencr-test]. If the test goes successful then we are good to go, otherwise, try to re-upload the firmware.

#### Building Workspace in robot (Raspberry pi.)
First of all put `turtlebot3_mecanum` and `turtlebot3_msgs` inside the catkin workspace of the robot. Now follow the below steps to build the packages.

1. First removing not unnecessary packages 
```sh
cd ~/catkin_ws/src
rm -rf turtlebot3_mecanum_description turtlebot3_mecanum_slum turtlebot3_navigation
```
2. Installing dependencies
```sh
rosdep install --from-paths src --ignore-src -r -y
```
3. Build the packages
```sh
cd ~/catkin_ws
catkin build
# or
catkin_make
source devel/setup.bash
```

## Running the robot
Run roscore from Remote PC.
```sh
roscore
```
Bring up TurtleBot3 Mecanum
> Note: Before executing this command, you have to specify the model name of TurtleBot3. The `${TB3_MODEL}` is the name of the model you are using in burger, waffle, waffle_pi, and mecanum.

1. Open a new terminal from the PC with Ctrl + Alt + T and connect to Raspberry Pi with its IP address.
```sh
ssh ubuntu@{IP_ADDRESS_OF_RASPBERRY_PI}
```
2. Bring up basic packages to start TurtleBot3 applications.
```sh
export TURTLEBOT3_MODEL=${TB3_MODEL}
roslaunch turtlebot3_mecanum_bringup turtlebot3_mecanum_robot.launch
```
3. Form a remote PC, open a new terminal and run the below command for teleoperation. Use the holonomic drive to control the robot side-ways and cross-ways.
```sh
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

[//]: # (These are reference links used in the body of this note and get stripped out when the markdown processor does its job. There is no need to format it nicely because it shouldn't be seen. Thanks SO - http://stackoverflow.com/questions/4823468/store-comments-in-markdown-syntax)

   [tb3-setup]: <https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup>
   [tb3-msgs]: <https://github.com/ROBOTIS-GIT/turtlebot3_msgs>
   [sbc-setup]: <https://emanual.robotis.com/docs/en/platform/turtlebot3/opencr_setup/#opencr-setup>
   [sbc-arduino-setup]: <https://emanual.robotis.com/docs/en/platform/turtlebot3/opencr_setup/#opencr-setup>
   [dyn-wkb-sdk]: <https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_workbench/#opencr-and-opencm-tutorials>
   [ard-opencr]: <https://emanual.robotis.com/docs/en/parts/controller/opencr10/#arduino-ide>
   [opencr-test]: <https://emanual.robotis.com/docs/en/platform/turtlebot3/opencr_setup/#opencr-test>

