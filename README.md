# articulated_design

## Overview
This package is a tool for the design of a 2DOF robot arm. Arm parameters are inputed into config/mechanism.yaml and
torque scatter plots are created. 

This is a ROS package. While ROS is not the most ideal for efficiently generating designs, the mechanism and torque calculation code is intended to be reused for actual arm control. The package is launched with the command --> "roslaunch articulated_design design.launch". This initializes a simple tkinter gui, the joint and torque publisher, controller, and scatterplot subscriber. The controller brings the end effector to all positions in the arm's workspace and torque for each motor is calculated and recorded. After this process is completed the scatter plots are produced.  

![scatter plot](https://github.com/cdimoush/articulated_design/blob/master/src/design/output0/scatter.png)
![Sample GUI](https://github.com/cdimoush/articulated_design/blob/master/images/sample_gui.png)


