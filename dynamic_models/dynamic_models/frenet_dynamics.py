import numpy as np
from casadi import SX, vertcat

class FrenetDynamics():
    ''''
    Class for the Frenet dynamics of the system. This class is no child class of the BaseDynamics. 
    '''

    def __init__(self, system_parameters, system_type, frame_type = 1):

        self.x_names = ['d_t', 'd_n', 'd_b', 'v_x', 'v_y', 'v_z']
        self.u_names = ['F_x', 'F_y', 'F_z']
        self.x_dot_names = ['d_t_dot', 'd_n_dot', 'd_b_dot', 'v_x_dot', 'v_y_dot', 'v_z_dot']

        self.mass = system_parameters[0]
        self.damping_factor = system_parameters[1]
        self.stiffness = system_parameters[2]

        self.system_type = system_type

        self.frame_type = frame_type  # 0: Frenet frame, 1: Bishop frame


    def define_system_variables(self):
        """
        Define the casadi variables for the system
        """

        casadi_states = vertcat(*[SX.sym(name) for name in self.x_names]) 
        casadi_states_dot = vertcat(*[SX.sym(name) for name in self.x_dot_names])
        casadi_inputs = vertcat(*[SX.sym(name) for name in self.u_names]) 

        return casadi_states, casadi_states_dot, casadi_inputs
    credits


    def calculate_frenet_state_matrizes(self, frenet_matrix, s_dot, kappa, tau):
        if self.frame_type == 0:                # Frenet-Serret frame
            W = np.array([[0,  -kappa, 0], 
                                    [kappa, 0,  -tau], 
                                    [0, tau, 0]])
        else:                                   # Bishop frame   
            W = np.array([[0, -kappa, -tau],
                         [kappa, 0, 0], 
                         [tau, 0, 0]])
            
        if self.system_type == 0:               # double integrator system
            A = np.vstack([
                np.hstack([- s_dot * W, np.eye(3)]),  # upper 3x6 block-matrix
                np.hstack([np.zeros((3, 3)), -s_dot * W])  # lower 3x6 block-matrix
                        ])

        elif self.system_type == 1:             # mass damping system 
            A = np.vstack([
                np.hstack([- s_dot * W, np.eye(3)]),  # upper 3x6 block-matrix
                np.hstack([np.zeros((3, 3)), -s_dot * W + frenet_matrix * self.damping_factor / self.mass])  # lower 3x6 block-matrix
                        ])
            
        else:
            raise NotImplementedError("Frenet System Dynamics not implemented for that system type")
        
        B = np.vstack([np.zeros((3,3)), frenet_matrix.T / self.mass])
        return A, B


