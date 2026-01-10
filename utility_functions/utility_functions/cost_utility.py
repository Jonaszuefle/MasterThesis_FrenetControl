from scipy.interpolate import interp1d
import numpy as np
import time 
import matplotlib.pyplot as plt

class CostDefinition():
    def __init__(self, path_obj, Q, R, use_predefined_cost=True):
        self.path_obj = path_obj

        self.num_base_points = 1000

        self.use_predefined_cost = use_predefined_cost
        
        self.define_initial_cost(Q, R)
        self.define_cost()

        

    
    def define_cost(self):
        """
        Define the costs for the MPC controller based on the path type.
        The cost is defined using base points and interpolation along the path.
        Any function caluclating the cost based on the path progress (s) may be used here. 
        """
        s_values = np.linspace(0.0, self.path_obj.arc_length, self.num_base_points)    # define base points

        if self.path_obj.trajectory_type < 3:
            cost_d, cost_v_t = self.medical_cost()

        elif self.path_obj.trajectory_type < 6:
            cost_d, cost_v_t = self.rehab_cost()

        else:
            cost_d, cost_v_t = self.default_cost()
        
        self.d_fun_cost_d = interp1d(s_values, cost_d, kind='linear', fill_value='extrapolate')
        self.d_fun_cost_v_t = interp1d(s_values, cost_v_t, kind='linear', fill_value='extrapolate')
    

    def define_initial_cost(self, Q, R):
        """
        Define the general initial cost for the MPC controller.
        """
        self.Q_init = Q
        self.R_init = R

    
    def get_initial_cost(self):
        return self.Q_init, self.R_init
    

    def get_cost(self, s):
        """
        Get the cost based on the path progress (s). 
        Use the interpolation functions for tracking error cost and tangent velocity cost. The other cost values are taken from the initial cost definition.

        Input:
            - s: path progress [float] (m)
        Output:
            - W: cost matrix [n_Q + n_R x n_Q + n_R] (diagonal matrix)
        """
        d_cost = self.d_fun_cost_d(s)
        v_t_cost = self.d_fun_cost_v_t(s)

        W_vals = np.hstack([self.Q_init[0], d_cost, d_cost, v_t_cost, self.Q_init[4:6], self.R_init])

        W = np.diag(W_vals)

        return W
    

    #======== constraint definition ========
    
    def default_cost(self):
        """
        Default constraints. 
        Tube around the trajectory. 
        Obstacle in the middle 
        """

        if not self.use_predefined_cost:
            cost_d = np.ones(self.num_base_points)*self.Q_init[1]
            cost_v_t = np.ones(self.num_base_points) * self.Q_init[3]
        else:
            cost_d = np.ones(self.num_base_points) * 70000
            cost_d[int(self.num_base_points * 0.25):int(self.num_base_points * 0.7)] = 1000
            cost_v_t = np.ones(self.num_base_points) * 300  

        return cost_d, cost_v_t

    
    def medical_cost(self):
        """
        Constraints for the medical trajectories.
        Funnel until incision point -> Tube around trajectory -> Including possible obstacle.
        """

        if not self.use_predefined_cost:
            cost_d = np.ones(self.num_base_points)*self.Q_init[1]
            cost_v_t = np.ones(self.num_base_points) * self.Q_init[3]
        else:
            cost_d = np.ones(self.num_base_points) * 70000
            cost_d[int(self.num_base_points * 0.25):int(self.num_base_points * 0.7)] = 1000
            cost_v_t = np.ones(self.num_base_points) * 300  

        return cost_d, cost_v_t
    

    def rehab_cost(self):
        """
        Constraints for the rehab trajectories.
        """

        if not self.use_predefined_cost:
            cost_d = np.ones(self.num_base_points)*self.Q_init[1]
            cost_v_t = np.ones(self.num_base_points) * self.Q_init[3]
        else:
            cost_d = np.ones(self.num_base_points) * 70000
            cost_d[int(self.num_base_points * 0.25):int(self.num_base_points * 0.7)] = 1000
            cost_v_t = np.ones(self.num_base_points) * 300  

        return cost_d, cost_v_t
    

    def circle_obstacle(self, center, radius):
        '''
        Define a symmetric circle obstacle in the Frenet frame for d_n and d_b.

        Input: 
            - center: center position on path [%]
            - radius: radius of the circle [m]
        '''
        center_point = int(self.num_base_points * center)
        obstacle = np.zeros(self.num_base_points)

        for i in range(self.num_base_points):
            dist = abs(i - center)
            if dist <= radius:
                obstacle[i] = 0.001 * np.sqrt(radius**2 - dist**2)
            else:
                obstacle[i] = 0

        return obstacle


    def rectangle_obstacle(self, start, end, hight):
        '''
        Define a symmetric rectangle obstacle in the Frenet frame for d_n and d_b.

        Input: 
            - start: start position on path [%]
            - end: end position on path [%]
            - hight: hight of the obstacle [m]
        '''
        obstacle = np.zeros(self.num_base_points) 
        obstacle[int(self.num_base_points * start) : int(self.num_base_points * end)] = hight 
        
        return obstacle
    
    

    def plot_cost(self):
        s = np.linspace(0, self.path_obj.arc_length, 1000)
        frenetState = np.array([0, -1, -1, 0, 0, 0])

        constraints_all = [self.get_constraint(si, frenetState) for si in s]
        lbx = np.array([c['lbx'] for c in constraints_all])
        ubx = np.array([c['ubx'] for c in constraints_all])

        plt.figure()
        plt.plot(s, lbx[:,1], 'r.', label='upper x_n')
        plt.plot(s, ubx[:,1], 'r.', label='lower x_n')
        plt.plot(s, lbx[:,2], 'g--', label='upper x_n obst')
        plt.plot(s, ubx[:,2], 'g--', label='lower x_n obst')
        plt.plot(s, lbx[:,3], 'b.', label='upper x_n soft')
        plt.plot(s, ubx[:,3], 'b.', label='lower x_n soft')
        plt.plot(s, lbx[:,4], 'y--', label='upper x_n soft obst')
        plt.plot(s, ubx[:,4], 'y--', label='lower x_n soft obst')
        plt.title('Constraints for d_n/d_b if d < 0')
        plt.legend()

        s = np.linspace(0, self.path_obj.arc_length, 1000)
        frenetState = np.array([0, 1, 1, 0, 0, 0])

        constraints_all = [self.get_constraint(si, frenetState) for si in s]
        lbx = np.array([c['lbx'] for c in constraints_all])
        ubx = np.array([c['ubx'] for c in constraints_all])

        plt.figure()
        plt.plot(s, lbx[:,5], 'r.', label='upper x_n')
        plt.plot(s, ubx[:,5], 'r.', label='lower x_n')
        plt.plot(s, lbx[:,6], 'g--', label='upper x_n obst')
        plt.plot(s, ubx[:,6], 'g--', label='lower x_n obst')
        plt.plot(s, lbx[:,7], 'b.', label='upper x_n soft')
        plt.plot(s, ubx[:,7], 'b.', label='lower x_n soft')
        plt.plot(s, lbx[:,8], 'y--', label='upper x_n soft obst')
        plt.plot(s, ubx[:,8], 'y--', label='lower x_n soft obst')
        plt.legend()
        plt.title('Constraints for d_n/d_b if d > 0')

        plt.show()
        

