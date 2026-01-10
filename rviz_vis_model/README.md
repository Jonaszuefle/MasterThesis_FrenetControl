# Share Control Cataract Operation

## Description
The aim of this project is to develop a system that enables collaboration between humans and robots as a first step towards the automatization of cataract surgery. The project consists of several blocks in which restrictions and movement aids are added to help surgeons perform cataract operations in the future.

## Table of Contents
- [Motivation](#motivation)
- [Installation](#installation)
- [Ros2 blocks](#ros2-blocks))
- [Software strucuture](#software-sturcture)


## Motivation

While human skills are characterized by high flexibility and rapid adaptation to new situations, robots can perform tasks with consistent precision without fatigue. Combining humans and robots in a collaborative system enables the synergistic use of these skills.

## Installation

The steps for the correct installation of the project are written in ["Installation instructions.pdf"](Installation instructions.pdf). The installation can be divided into the following parts:

1. Install ROS2
2. Install SOFA
3. Add Plugins to SOFA
    - Internal plugins (Python)
    - External plugins (ROS2)
4. Set up the project (from GitLab)
5. Install Omni-Touch drivers 

The document not only shows the installation steps but also the possible problems that may arise during the installation as well as their solutions.

## Ros2 blocks
### [omni_common](omni_common) / [omni_description](omni_description) / [omni_msgs](omni_msgs)

Communication is established with the haptic decive (Omni_Touch) and the information sent or received are set in ROS2 topics. The most important script is the ["omni_state.cpp"](omni_common/src/omni_state.cpp). The inputs of this script are the haptic device parameters in its language and the outputs are the robot haptic device parameter in ROS topics.

The following instruction would launch the ROS2 topics from the haptic device:
```
ros2 launch omni_common omni_state.launch.py
```

### [sofa_models](sofa_models)
SOFA configuration is established. The input is a position value and the output is the representanton of the eye and the movements on SOFA. There are two scripts:
- **[main_incision_eye_model2](sofa_models/main_incision_eye_model2.py)**: It is the main program and and it calls the functions on "contact_detection_image_pub". It establish the principal SOFA configuration characteristics.  
- **[contact_detection_image_pub](sofa_models/contact_detection_image_pub.py)**: It contains the functions with which the areas inside the eye where it has been cut are drawn.
To run SOFA with the desired configuration of the eye, the instruction would be:
```
python3 src/sofa_models/main_incision_eye_model2.py
```

### [master_slave_control](master_slave_control/master_slave_control)
The positions to be send to SOFA are calculated. There are two scripts:
- **[Input mapping](master_slave_control/master_slave_control/input_mapping.py)**: The robot's position is rescaled to fit the SOFA scale. The input is the position of the haptic device (Omni_Touch) and the output is the position that should be introduce in SOFA. To launch this python script in Linux the instruction would be the following:
```
python3 src/master_slave_control/master_slave_control/input_mapping.py
```
- **[Automatic incision](master_slave_control/master_slave_control/automatic_incision.py)**: The positions of the scalpel in SOFA are calculated in order to follow an specific trajectory. The desired input points should be introduced directly on the scripts and the output is the position that should be introduce in SOFA. To launch this python script the instruction would be:
```
python3 src/master_slave_control/master_slave_control/automatic_incision.py
```

### [biomechanical_ff](biomechanical_ff/biomechanical_ff/biomechanical_ff.py)
The biomechanical force feedback that makes the user feel the virtual eye, as in reality, is calculated. The input is a position value (the position of the tip point) and the output is a force (the biomechanical force feedback).

To launch this block in Linux the instruction would be the following:
```
python3 src/biomechanical_ff/biomechanical_ff/biomechanical_ff.py
```

### [trajectory](trajectory/trajectory)
There are several programs to manage the force value or the desired position that are sent to the robot. The input is the position value of the haptic device and the output value is the derire force or desired position. 
- **[force_and_trajectory_mode](trajectory/trajectory/force_and_trajectory_mode.py)**: It changes from trajectory to force mode depending on the button clicked. To launch it the instruction would be:
```
python3 src/trajectory/trajectory/force_and_trajectory_mode.py
```
- **[trajectory_mode_record_points](trajectory/trajectory/trajectory_mode_record_points.py)**: It records some positions and then follows a trajectory through them. To launch it the instruction would be:
```
python3 src/trajectory/trajectory/trajectory_mode_record_points.py
```
- **[set_position](trajectory/trajectory/set_position.py)**: It moves the haptic device and stay it in a position if a button is pressed.To launch it the instruction would be:
```
python3 src/trajectory/trajectory/set_position.py
```

### [Rviz2 Visualization](rviz_vis_model)
Rviz can also be used to visualize the simulation. No installation is necessary as it is already build into ros2.

The simulation consists of three parts:
1. visualization of the eye 
2. modelling the cut of the scalpel 
3. visualization of the force feedback

Use the following launch file for startup:
```
ros2 launch rviz_vis_model eye_model.launch.py
```

The rviz window will open afterwards including all necessary markers/topics.
On the left side the different markers are shown. They can be unscelected if wanted. By using "Add" and selecting a topic other topics can be visualized. 
It is very important to seelct the right "Fixed Frame" (in this case world is default) in the Display menu. Else the models may not be seen. 
The necessary objects/models and the rviz file are inside the [models folder](/rviz_vis_model/models). 



## Software Sturcture
All combinations on how to use the haptic device are illustrated to make them more understandable. The squares indicate the names of the blocks or scripts used. The ROS node and a brief description of its operation are provided below each block. The ROS topics sent and received are indicated by arrows.

### Share control using SOFA (main)
The main program is illustrated bellow. Four blocks or scripts are used. First, the [omni block](#omni_common--omni_description--omni_msgs) is used to establish communication with the haptic decive and send information (such as the position of the haptic decice) in ROS2 topics. The haptic device position is introduced in the [input mapping script](#master_slave_control) and rescaled to fit the SOFA scale. Then, [SOFA block](#sofa_models) establishes the configuration and moves the virtual scalpel depending on the position. If the virtual scalpel goes into the eye, the trajectory is drawn. At the same time, the [biomechanical_ff script](#biomechanical_ff) receives the value of the position and depending on whether it is inside or outside the eye, it sends a feedback force value. Finally the Onmi block receives the feedback force value and sends it to the haptic device.

![Fig.1. Share control using SOFA](readme_images/Omni_input_sofa_biomec.JPG)

### Automatic Incision
The **automatic incision script** was used in cases in which the haptic device could not be used. It can be used only with the [SOFA block](#sofa_models) and set a certain trajectory sending the position values. The following image shows how the communication structure. 

![Automatic_Incision](readme_images/autom_sofa.JPG){width=45%}

### Desired force and desired position modes



![image1](readme_images/omni_forcetrajectory.JPG){width=45%} ![image1](readme_images/omni_recordtrajectory.JPG){width=45%}
![image1](readme_images/omni_setpos.JPG){width=45%}

