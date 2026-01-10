import numpy as np
from .base_dynamics import BaseDynamics

class DoubleIntegrator(BaseDynamics):
    '''
    Class for the double integrator system. Defines System Dynamics.
    Inherits from BaseDynamics.
    '''
    def __init__(self, x_names, u_names, system_parameters):
        
        self.mass = system_parameters[0]
        self.damping_factor = system_parameters[1]
        self.stiffness = system_parameters[2]
        self.x_names = x_names
        self.u_names = u_names


    def define_system_dynamics(self):
        """
        Autonomous system dynamics.
        """

        stateVar = self.define_system_variables(self.x_names)
        inputVar = self.define_system_variables(self.u_names)

        A = np.vstack([
            np.hstack([np.zeros((3, 3)), np.eye(3)]),  # upper 3x6 block-matrix
            np.hstack([np.zeros((3, 3)), np.zeros((3, 3))])  # lower 3x6 block-matrix
                    ])
        
        B = np.vstack([np.zeros((3, 3)), np.eye(3,3)/self.mass])

        dx = A @ stateVar + B @ inputVar 

        return dx, stateVar, inputVar, A, B
    

    def define_system_dynamics_plus_human(self):
        """
        Cooperative system dynamics.
        """

        stateVar = self.define_system_variables(self.x_names)
        inputVar = self.define_system_variables(self.u_names)

        u_names_human = ['F_x_human', 'F_y_human', 'F_z_human']

        inputVarHuman = self.define_system_variables(u_names_human)

        A = np.vstack([
            np.hstack([np.zeros((3, 3)), np.eye(3)]),  # upper 3x6 block-matrix
            np.hstack([np.zeros((3, 3)), np.zeros((3, 3))])  # lower 3x6 block-matrix
                    ])
        
        B = np.vstack([np.zeros((3, 3)), np.eye(3,3)/self.mass])

        dx = A @ stateVar + B @ inputVar + B @ inputVarHuman

        return dx, stateVar, inputVar, inputVarHuman, A, B
    

    


    

    
