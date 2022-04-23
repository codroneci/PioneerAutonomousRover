# Rover

High level system description of the red rover system. 
The nodes in this package interface between controllers and rover firmware.

## Visuals
See an intermediate demo of the red rover autonomously tracking a trajectory [here](https://youtu.be/GVTKQ6eHt3E)

## System Block Diagram
![System](/misc/assets/system_diagram.PNG)
This diagram above shows the high-level guidance, navigation, and control stack for the entire red rover system (in progress). 
Notes: 
- Currently emulating trajectory generation
- Pose is temporarily received from VICON, eventually this will be replaced by another navigation technique (ie using odometry)

## About This Package
![Mux](/misc/assets/mux_diagram.PNG)
This package specifically interfaces between the control algorithm and the rover firmware, as shown in the above system diagram. It handles arming protocol, joystick inputs, automatic commanded velocity, and manual commanded velocity to publish one finalized command to the rover. In addition, an alpha filter is used to smooth out velocity commands that would otherwise be sharp discontinuities.
See more about alpha filters [here](https://en.wikipedia.org/wiki/Low-pass_filter#Simple_infinite_impulse_response_filter)

## Getting Started

### SSH
To set up the rover workspace, first SSH into the NUC.
Example: Rover 3 (RR03)
```bash
ssh acl@RR03.local
```
Note: Not all rovers are set up with openssh yet

### Workspace setup
Your catkin workspace on the rover NUC should have the following packages:
```bash
catkin_ws
└── src
    ├── purepursuit
    ├── rover
    └── ...
```
- More packages to come in the future

### Usage

After ssh'ing into the NUC, you can simply type the following command to start all the nodes automatically in tmux.
```bash
drive
```
 If this is the first time using a rover, in order to configure the drive script for use, you should make the script executable and place it in /usr/bin so it can be accessed from anywhere. You can create a symbolic link in /usr/bin to point to the drive script located in rover/utils:
```bash
ln -s ~/catkin_ws/src/rover/utils/drive /usr/bin/drive
```
Alternatively, you can just run the script from the package:
```bash
rosrun rover drive
```
But regardless, make sure to make the file executable:
```bash
chmod +x ~/catkin_ws/src/rover/utils/drive
```

### Control instructions
![System](/misc/assets/Control_buttons.jpg)

Todo: Where does the panel arming scheme come into this?
