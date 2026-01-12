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

## Prerequisites & Installation

### 1. ROS 2
Ensure you have a working installation of **ROS 2 (Humble or later)**.

### 2. Acados
The `frenet_control` package relies on **acados** for high-performance MPC optimization. Follow these steps to build it from source:
