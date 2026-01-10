from scipy.interpolate import interp1d
import numpy as np
import time 
import matplotlib.pyplot as plt

class ConstraintDefinition():
    def __init__(self, path_obj, constraints_x, constraints_u, soft_constraint_factor=0.05, use_predefined_constraints = True, enable_obstacle=False):
        self.path_obj = path_obj

        self.soft_constraint_factor = soft_constraint_factor
        self.enable_obstacle = enable_obstacle
        self.num_base_points = 1000

        self.define_initial_constraints(constraints_x, constraints_u)
        self.define_constraints()

        self.d_t_max = 0.1
        self.f_max = 3.3
        self.v_max = 10

        self.use_predefined_constraints = use_predefined_constraints

    
    def define_constraints(self):
        """
        Define the constraints for the MPC controller based on the path type.
        The constraints are defined using base points and interpolation along the path.
        Any function caluclating the constraints based on the path progress (s) may be used here. 
        """
        s_values = np.linspace(0.0, self.path_obj.arc_length, self.num_base_points)    # define base points

        if self.path_obj.trajectory_type < 3:
            d_max, obstacle = self.medical_constraints()

        elif self.path_obj.trajectory_type < 6:
            d_max, obstacle = self.rehab_constraints()

        else:
            d_max, obstacle = self.default_constraints()

        self.d_fun = interp1d(s_values, d_max, kind='linear', fill_value='extrapolate')
        self.obstacle_fun = interp1d(s_values, obstacle, kind='linear', fill_value='extrapolate')


    def define_initial_constraints(self, constraints_x, constraints_u):
        """
        Define the general initial constraints for the MPC controller.
        """
        con_x = np.zeros(12,)    # 12 constraints -- 4 for each d_n and d_b (1 hard, 1 soft and 1 obstacle constraint) -- symmetric constraints (min, max)

        con_x[0] = constraints_x[0]
        con_x[1:3] = constraints_x[1:3] 
        con_x[3] = constraints_x[1] * (1-self.soft_constraint_factor)
        con_x[4] = constraints_x[2] * (1+self.soft_constraint_factor)

        con_x[5:7] = constraints_x[3:5]
        con_x[7] = constraints_x[3] * (1-self.soft_constraint_factor)
        con_x[8] = constraints_x[4] * (1+self.soft_constraint_factor)
        con_x[9:12] = constraints_x[5:8]

        con_u = np.array(constraints_u)

        self.init_constraints = {
            'lbx': -con_x,
            'ubx': con_x,
            'lbu': -con_u,
            'ubu': con_u
    }
        
    def get_initial_constraints(self):
        return self.init_constraints
    

    def get_constraints(self, s, frenet_state):
        d_max = self.d_fun(s)

        d_n_max = d_max
        d_b_max = d_max

        d_n_min = -d_max
        d_b_min = -d_max

        d_max_soft = (1-self.soft_constraint_factor) * d_max

        d_n_max_soft = d_max_soft
        d_b_max_soft = d_max_soft

        d_n_min_soft = -d_max_soft
        d_b_min_soft = -d_max_soft

        d_t_max = self.d_t_max
        d_t_min = -d_t_max

        v_max = self.v_max
        v_min = -v_max

        f_max = self.f_max 
        f_min = -f_max

        if self.enable_obstacle == True:
            if self.obstacle_fun(s) == 0:
                d_n_min_obstacle = d_n_min
                d_n_min_obstacle_soft = d_n_min_soft
                d_n_max_obstacle = d_n_max
                d_n_max_obstacle_soft = d_n_max_soft
                d_b_min_obstacle = d_b_min
                d_b_min_obstacle_soft = d_b_min_soft
                d_b_max_obstacle = d_b_max
                d_b_max_obstacle_soft = d_b_max_soft
            else:
                if frenet_state[1] > -10:
                    d_n_min_obstacle = self.obstacle_fun(s)
                    d_n_min_obstacle_soft = d_n_min_obstacle * (1 + self.soft_constraint_factor)

                    d_n_max_obstacle = d_n_max 
                    d_n_max_obstacle_soft = d_n_max_obstacle * (1 - self.soft_constraint_factor)
                else:
                    d_n_max_obstacle = -self.obstacle_fun(s)
                    d_n_max_obstacle_soft = d_n_max_obstacle * (1 + self.soft_constraint_factor)

                    d_n_min_obstacle = d_n_min
                    d_n_min_obstacle_soft = d_n_min_obstacle * (1 - self.soft_constraint_factor)
                
                if frenet_state[2] > -10:
                    d_b_min_obstacle = self.obstacle_fun(s)
                    d_b_min_obstacle_soft = d_b_min_obstacle * (1 + self.soft_constraint_factor)

                    d_b_max_obstacle = d_b_max
                    d_b_max_obstacle_soft = d_b_max_obstacle * (1 - self.soft_constraint_factor)
                else:
                    d_b_max_obstacle = -self.obstacle_fun(s)
                    d_b_max_obstacle_soft = d_b_max_obstacle * (1 + self.soft_constraint_factor)

                    d_b_min_obstacle = d_b_min
                    d_b_min_obstacle_soft = d_b_min_obstacle * (1 - self.soft_constraint_factor)
        else:
            d_n_min_obstacle = d_n_min
            d_n_min_obstacle_soft = d_n_min_soft
            d_n_max_obstacle = d_n_max
            d_n_max_obstacle_soft = d_n_max_soft
            d_b_min_obstacle = d_b_min
            d_b_min_obstacle_soft = d_b_min_soft
            d_b_max_obstacle = d_b_max
            d_b_max_obstacle_soft = d_b_max_soft

        # d_max_obstacle = self.obstacle_fun(s)
        # d_min_obstacle = -d_max_obstacle

        constraints = {
            'lbx': np.array([d_t_min, d_n_min, d_n_min_obstacle, d_n_min_soft, d_n_min_obstacle_soft, d_b_min, d_b_min_obstacle, d_b_min_soft, d_b_min_obstacle_soft, v_min, v_min, v_min]),
            'ubx': np.array([d_t_max, d_n_max, d_n_max_obstacle, d_n_max_soft, d_n_max_obstacle_soft, d_b_max, d_b_max_obstacle, d_b_max_soft, d_b_max_obstacle_soft, v_max, v_max, v_max]),
            'lbu': np.array([f_min, f_min, f_min]),
            'ubu': np.array([f_max, f_max, f_max])
        }

        return constraints
    

    #======== constraint definition ========
    
    def default_constraints(self):
        """
        Default constraints. 
        Tube around the trajectory. 
        Obstacle in the middle 
        """
        d_max = np.ones(self.num_base_points) * 0.05

        if self.enable_obstacle == True:
            obstacle = self.rectangle_obstacle(0.4, 0.6, 0.005)
            #obstacle[self.num_base_points//4:self.num_base_points//4*3] = np.linspace(0.00, 0.05, self.num_base_points // 4 * 2)    # triangular obstacle

            # center = self.num_base_points // 2
            # radius = self.num_base_points // 6
            # for i in range(self.num_base_points):
            #     dist = abs(i - center)
            #     if dist <= radius:
            #         obstacle[i] = 0.005 * np.sqrt(radius**2 - dist**2)
            #     else:
            #         obstacle[i] = 0
        else:
            obstacle = d_max

        return d_max, obstacle

    
    def medical_constraints(self):
        """
        Constraints for the medical trajectories.
        Funnel until incision point -> Tube around trajectory -> Including possible obstacle.
        """
        d_max = np.ones(self.num_base_points) * 0.02
        d_max[0:int(self.num_base_points*0.3)] = np.linspace(0.05, 0.02, int(self.num_base_points*0.3))

        if self.enable_obstacle == True:
            obstacle = self.rectangle_obstacle(0.3, 0.7, 0.01)

        else:
            obstacle = d_max

        return d_max, obstacle
    

    def rehab_constraints(self):
        """
        Constraints for the rehab trajectories.
        """
        d_max = np.ones(self.num_base_points) * 0.05
        d_max[self.num_base_points//8:self.num_base_points//3*2] = 0.2

        if self.enable_obstacle == True:
            obstacle = self.rectangle_obstacle(0.4, 0.6, 0.005)
        else:
            obstacle = d_max

        return d_max, obstacle
    
    
    # ======== obstacle definition ========

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
    
    

    def plot_constraints(self):
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
        
