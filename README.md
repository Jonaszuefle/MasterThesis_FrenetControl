# Shared Control Cataract Operation System

## Overview
This project implements a shared control system for cataract surgery operations, combining human expertise with robotic precision. The system uses a Frenet coordinate-based control architecture to enable collaborative human-robot interaction during delicate surgical procedures.

## Frenet System Components

The Frenet system consists of several interconnected ROS2 packages that work together to provide shared control functionality:

### Quick Start
Launch the complete Frenet system with default parameters:
```bash
ros2 launch frenet_system_bringup system_frenet.launch.py
```

Or customize the launch parameters:
```bash
ros2 launch frenet_system_bringup system_frenet.launch.py \
  fake_hardware:=true \
  automation_controller:=mpc \
  human_controller:=false \
  recorder:=false
```

**Launch Parameters (with defaults):**
- `fake_hardware`: `false` (default) or `true` - Use simulated hardware instead of real devices
- `automation_controller`: `mpc` (default) or `pi` - Controller type for autonomous operation
- `human_controller`: `false` (default), `mpc`, or `pi` - Controller type for human-assisted operation
- `recorder`: `false` (default) or `true` - Enable data recording for analysis

### Configuration
System parameters are defined in `frenet_system_bringup/config/params.yaml`

## Frenet System Architecture

### Core Packages (Developed Components)

#### 1. **frenet_control**
Advanced control algorithms for the Frenet coordinate system.
- **Base Controller**: Common functionality for all controllers
- **MPC Controller**: Model Predictive Control implementation using Acados solver
    - In the `control_step`-loop further options can be choosen (by uncommenting them), e.g. path dependent references, costs or constraints 
- **PI Controller**: Proportional-Integral controller for simpler control scenarios
- Handles both autonomous and human-assisted control modes

#### 2. **frenet_system_bringup**
System orchestration and launch configuration.
- Centralizes system startup with configurable parameters
- Manages different control modes (autonomous vs. human-assisted)
- Coordinates communication between all system components
- Provides unified parameter management through YAML configuration

#### 3. **frenet_transformation**
Coordinate system transformations for the Frenet frame.
- Converts between Cartesian and Frenet coordinates
- Enables path-following control in curvilinear coordinates
- Provides real-time coordinate transformation services

#### 4. **frenet_vis**
Real-time visualization of the Frenet system state.
- RViz-based visualization of system components
- Real-time display of trajectories, control signals, and system states
- Interactive visualization for debugging and analysis

#### 5. **dynamic_models**
Dynamic system modeling for different robotic systems.
- **Base Dynamics**: Abstract base class for dynamic models
- **Double Integrator**: Simple point mass dynamics
- **Frenet Dynamics**: Dynamics in Frenet coordinates
- **Mass Damping**: Mass-damper system dynamics
- Modular design for different robotic platforms

#### 6. **fake_hardware**
Hardware simulation and testing interface.
- Simulates hardware devices when real hardware is not available
- Enables system testing and development without physical setup

#### 7. **ros_bag_recorder**
Data logging and analysis tools.
- Automatic recording of system data during operation
- Configurable recording based on operation mode
- Supports post-operation plotting
- Ros Bags may be imported to Matlab for easier plotting

#### 8. **utility_functions**
Common utilities and helper functions.
- Shared mathematical operations
- Common data structures and transformations
- Utility functions used across multiple packages

## System Operation Modes

### 1. Autonomous Mode
```bash
ros2 launch frenet_system_bringup system_frenet.launch.py \
  automation_controller:=mpc \
  human_controller:=false
```
- Fully autonomous operation using MPC or PI controller
- Robot follows predetermined trajectories

### 2. Shared Mode - Robot assists Human
```bash
ros2 launch frenet_system_bringup system_frenet.launch.py \
  automation_controller:=mpc \
  human_controller:=mpc
```
- Collaborative control between human operator and automation
- Human input combined with automated assistance
- Enhanced precision through shared control algorithms

### 3. Simulation Mode
```bash
ros2 launch frenet_system_bringup system_frenet.launch.py \
  fake_hardware:=true \
  recorder:=true
```
- Testing and validation without hardware
- Data collection for algorithm development
- Safe environment for testing new control strategies

# Share Control Cataract Operation

## Description
The aim of this project is to develop a system that enables collaboration between humans and robots as a first step towards the automatization of cataract surgery. The project consists of several blocks in which restrictions and movement aids are added to help surgeons perform cataract operations in the future.

## Table of Contents
- [Installation](#installation)
- [Legacy System (SOFA-based)](#legacy-system-sofa-based)
- [Connection between WSL and Ubuntu](#connection-between-wsl-and-ubuntu)
- [Software Structure](#software-structure)


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

The Simulation was reworked and currently runs with ROS2 RViz. Further instructions on how to use [RViz](#rviz2-visualization) are shown below. For using this setup, the points 1,4,5 from abouth are still valid. 

## Legacy System (SOFA-based)

The original system implementation used SOFA (Simulation Open Framework Architecture) for surgical simulation. This section documents the legacy components for reference:

### ROS2 Legacy Blocks
### [omni_common](omni_common) / [omni_description](omni_description) / [omni_msgs](omni_msgs)

Communication is established with the haptic decive (Omni_Touch) and the information sent or received are set in ROS2 topics. The most important script is the ["omni_state.cpp"](omni_common/src/omni_state.cpp). In this code the measured parameters of the haptic device are read and transformed into ros topics ("master/xxx"). Furthemore the generated/calculated feedback is read to this node and send to the haptic device in order to apply a position/force change. 

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
During the course of the project, SOFA was exchanged by rviz in order to run the simulation. Therefore no external installation is necessary as RVizit is already build into ros2.

The simulation consists of three parts:
1. visualization of the eye 
2. modelling the cut of the scalpel 
3. visualization of the force feedback

Use the following launch file to start the simulation environment:
```
ros2 launch rviz_vis_model eye_model.launch.py
```

The rviz window will open afterwards including all necessary markers/topics.
On the left side the different markers are shown. They can be unscelected if wanted. By using "Add" and selecting a topic other topics can be visualized. 
It is very important to seelct the right "Fixed Frame" (in this case world is default) in the Display menu. Else the models may not be seen. 
The necessary objects/models and the rviz file are inside the [models folder](/rviz_vis_model/models). 

## Connection between WSL and Ubuntu

### 1. Connect Windows and Linux computer via Ethernet

### 2. Set up IP-adresses of the Ethernet-connection: 
- For example:   
    - Windows: IpV4: 192.168.1.2; Subneztmask 255.255.255.0; Standartgateway 192.168.1.1
    - Linux:  IpV4: 192.168.1.3; Subneztmask 255.255.255.0; Standartgateway 192.168.1.1

### 3. Install Hyper-V on Windows Home (Included in Windows Premium): 
- Create a file: "hv.bat" on desktop 
- Edit and copy the following into the file:
```
pushd "%~dp0" 
dir /b %SystemRoot%\servicing\Packages\*Hyper-V*.mum >hv.txt
for /f %%i in ('findstr /i . hv.txt 2^>nul') do dism /online /norestart /add-package:"%SystemRoot%\servicing\Packages\%%i"
del hv.txt
Dism /online /enable-feature /featurename:Microsoft-Hyper-V -All /LimitAccess /ALL
pause   
```

- Run the file using administrator. 

### 4. Open Hyper-V to create a bridge between Ethernet-Adapter of Windows and virtual Ethernetadapter of WSL:
- go to "Actions" on the right side of the window
- open "Manager for virtual Adapters/Switches" 
- set WSL-adapter to Ethernet-card (of windows)
    - WSL has to opened 
    - select WSL (Hyper-V firewall)
    - choose connection type: external network
    - select Ethernet connection (name may varry, not your wifi-connection)
    - select apply
    - the first try may fail -> press abort -> repeat 
    - now the traffic is forwarded

### 5. Set route to linux: 
- open terminal in linux
- run the following lines of code:

```
sudo ip addr flush dev eth0  #delete the old ip adress of wsl
sudo ip addr add 192.168.1.4/24 dev eth0 #assigns new ip adress to wsl
sudo ip link set eth0 up
sudo ip route add default via 192.168.1.1 #set up routing
```

### 6. Check the ip adress of Linux
in the settings, check the ip adress (should be 192.168.1.2)

### 6. Now the computers can be pinged in order to test the connection
- from Linux: ping 192.168.1.4 
- from WSL: ping 192.168.1.2

## Software Structure
All combinations on how to use the haptic device are illustrated to make them more understandable. The squares indicate the names of the blocks or scripts used. The ROS node and a brief description of its operation are provided below each block. The ROS topics sent and received are indicated by arrows.

## Shared Control using RViz environment
There are three main components: Slave, Master, Feedback. Each has its own topic name "prefix" ("slave/xx", "master/xx", "feedback/xx"). 

During operation the parameters of the touch device (master) are read. Those parameters are transformed into topics and are named **master**. Inside the [input mapping](#master_slave_control) the values are mapped onto the slave robot, hence the position is scaled. During this process the topics are renamed as **slave**. This data is used in order to run the simulation. In order to controll the device and utilize haptic feedback, force or position feedback is necessary. Those values are calculated in specific nodes and are forwarded to the master roboter ([omni_common](#omni_common--omni_description--omni_msgs)). The prefix of these topics is called **feedback**.

### Share control using SOFA (main)
The main program is illustrated bellow. Four blocks or scripts are used. First, the [omni block](#omni_common--omni_description--omni_msgs) is used to establish communication with the haptic decive and send information (such as the position of the haptic decice) in ROS2 topics. The haptic device position is introduced in the [input mapping script](#master_slave_control) and rescaled to fit the SOFA scale. Then, [SOFA block](#sofa_models) establishes the configuration and moves the virtual scalpel depending on the position. If the virtual scalpel goes into the eye, the trajectory is drawn. At the same time, the [biomechanical_ff script](#biomechanical_ff) receives the value of the position and depending on whether it is inside or outside the eye, it sends a feedback force value. Finally the Onmi block receives the feedback force value and sends it to the haptic device.

![Fig.1. Share control using SOFA](readme_images/Omni_input_sofa_biomec.JPG)

### Automatic Incision
The **automatic incision script** was used in cases in which the haptic device could not be used. It can be used only with the [SOFA block](#sofa_models) and set a certain trajectory sending the position values. The following image shows how the communication structure. 

![Automatic_Incision](readme_images/autom_sofa.JPG){width=45%}

### Desired force and desired position modes

![image1](readme_images/omni_forcetrajectory.JPG){width=45%} ![image1](readme_images/omni_recordtrajectory.JPG){width=45%}
![image1](readme_images/omni_setpos.JPG){width=45%}

# Parameter Setting
To set the parameters of the ROS nodes, define them in the launch file or when running the node, e.g.
```bash
ros2 run omni_common omni_state -p publishRate:=150
```
For now, they cannot be changed during operation, just during startup of a node.



