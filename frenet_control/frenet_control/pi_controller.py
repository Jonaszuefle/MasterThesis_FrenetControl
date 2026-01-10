import rclpy
from rclpy.node import Node

import numpy as np

from omni_msgs.msg import HIDFeedback
from omni_msgs.msg import FrenetState

from .base_controller import BaseController

class PIController(BaseController):
    '''
    Base Class to provide the basic functionalities for the controller, which all controllers have in commen.
    Initalizes the publishers and subscribers and provides the basic structure for the control loop.
    '''

    def __init__(self):
        super().__init__('control_node')        # inherit pub, sub, parent class variables and Frenet State from base_controller
        
        self.get_params_child()
        self.initalize_pi_controller()
        self.get_logger().info("PI-Control Node has been initialized")


    def get_params_child(self):
        '''
        Declar and load the parameters from the yamal file
        '''
        self.declare_parameter('control_parameters.pi_controller.K_P_d', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('control_parameters.pi_controller.K_P_v', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('control_parameters.pi_controller.K_I_d', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('control_parameters.pi_controller.K_I_v', rclpy.Parameter.Type.DOUBLE)
        
        self.K_P_d = self.get_parameter('control_parameters.pi_controller.K_P_d').get_parameter_value().double_value
        self.K_P_v = self.get_parameter('control_parameters.pi_controller.K_P_v').get_parameter_value().double_value
        self.K_I_d = self.get_parameter('control_parameters.pi_controller.K_I_d').get_parameter_value().double_value
        self.K_I_v = self.get_parameter('control_parameters.pi_controller.K_I_v').get_parameter_value().double_value
        
            

    def initalize_pi_controller(self):

        self.P_matrix = np.hstack([self.K_P_d * np.eye(3), self.K_P_v * np.eye(3)])
        self.I_matrix = np.hstack([self.K_I_d * np.eye(3), self.K_I_v * np.eye(3)])
        self.sum_e = np.zeros((6, 1))


    def control_step(self, frenet_state, frenet_matrix):
        '''
        PI control of the system using the velocity in Cartesian coordinates

        Input:
            - frenet_state: 6x1 numpy array (d_t, d_n, d_b, v_t, v_n, v_b)
            - frenet_matrix: 3x3 numpy array (T, N, B)
            - self.system_period: float (s)
        Output:
            - u_xyz: [3, numpy array] (u_x, u_y, u_z)
        '''

        e = frenet_state.reshape(6, 1) - self.frenet_state_reference

        self.sum_e += e * self.system_period

        u_ff = self.P_matrix @ e + self.I_matrix @ self.sum_e

        u_xyz = np.linalg.pinv(frenet_matrix.T) @ u_ff

        # use anti windup to limit control output 
        max_force = 3.3
        u_xyz_clipped = np.clip(u_xyz, -max_force, max_force)

        # 💡 2. Anti-Windup: Nur integrieren, wenn nicht gesättigt
        # → Nur den Teil des Fehlers integrieren, der nicht „abgeschnitten“ wurde




        for i in range(3):
            if abs(u_xyz[i]) >= max_force:
                self.sum_e[i] -= e[i] * self.system_period
                self.get_logger().info(f"Anti-Windup: component {i} ")

        u_ff = self.P_matrix @ e + self.I_matrix @ self.sum_e

        u_xyz = u_xyz_clipped 

        u = np.hstack([u_xyz] * 10)      # because of modularities in base controler for mpc

        self.new_control_output = True          # flag to identify new control calculation
        
        return u
    

def main(args=None):
    rclpy.init(args=args)
    node = PIController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()