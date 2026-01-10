std_msgs/Header header

bool grey_button_pressed
bool white_button_pressed
bool grey_button_mode
bool white_button_mode

# Cartesian coordinates are: x is to the right, z is up, y is towards device
geometry_msgs/Pose pose    # pose of joint3, orientatin of pencil 
geometry_msgs/Vector3 velocity

# Tool tip
geometry_msgs/Pose tool_tip_pose

float64[] angular_velocity
float64[] joint_angles
float64[] gimbal_angles

geometry_msgs/Vector3 force

geometry_msgs/Vector3 torque
float64[] joint_torque
float64[] gimbal_torque


