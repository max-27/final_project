# Software Frameworks for Autonomous Systems: Final project

This git repository contains the code base of our final project of the course "Software Frameworks for Autonomous 
Systems" at DTU.
All python files can be found in the script directory. In order to run the simulation in gazebo run this command in 
your catkin workspace
```
roslaunch final_project turtlebot3_world.launch
```
For running the robot and performing the task of the project, you can run:
```
rosrun final_project run.py
```
The code is seperated into three main modules:
- `Scan` class, that is responsible for scanning the QR encodes and 
storing its encodes message
- `Robot` class, that performs the movement and navigation of the robot
- `Transformation` class, that calculates the hidden frame of reference of the QR codes