import rclpy
from rclpy.executors import MultiThreadedExecutor

import numpy as np
import time

from omni_msgs.msg import HIDFeedback
from omni_msgs.msg import FrenetState

from .base_controller import BaseController
from utility_functions.mpc_init import AcadosSolver
from dynamic_models.frenet_dynamics import FrenetDynamics

from utility_functions.path_utilities import ReferencePath
from utility_functions.constraint_utility import ConstraintDefinition
from utility_functions.cost_utility import CostDefinition


class MPCController(BaseController):
    '''
    Base Class to provide the basic functionalities for the controller, which all controllers have in commen.
    Initalizes the publishers and subscribers and provides the basic structure for the control loop.
    '''

    def __init__(self):
        super().__init__('control_node')        # inherit pub, sub, parent class variables and Frame State from base_controller

        self.get_params_child()
        self.initalize_reference_path()
        self.initalize_mpc_controller()
        self.get_logger().info("MPC-Control Node has been initialized")


    def get_params_child(self):
        '''
        Declar and load the parameters from the yamal file
        '''
        
        self.declare_parameter('control_parameters.mpc_controller.Q', rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('control_parameters.mpc_controller.R', rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('control_parameters.mpc_controller.cost_slack_variables', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('control_parameters.mpc_controller.enable_predefined_cost', rclpy.Parameter.Type.BOOL)
        self.declare_parameter('control_parameters.mpc_controller.N', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('control_parameters.mpc_controller.T', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('control_parameters.mpc_controller.constraint.enable_predefined_constraints', rclpy.Parameter.Type.BOOL)
        self.declare_parameter('control_parameters.mpc_controller.constraint.lbx', rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('control_parameters.mpc_controller.constraint.ubx', rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('control_parameters.mpc_controller.constraint.lbu', rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('control_parameters.mpc_controller.constraint.ubu', rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('control_parameters.mpc_controller.constraint.soft_constraint_factor', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('control_parameters.mpc_controller.auto_N', rclpy.Parameter.Type.BOOL)

        self.declare_parameter('reference_tracking.enable_obstacle', rclpy.Parameter.Type.BOOL)
        self.declare_parameter('reference_tracking.frame_type', rclpy.Parameter.Type.INTEGER)

        self.declare_parameter('geometric_transformation.kappa_tresh', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('geometric_transformation.num_points_segment', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('geometric_transformation.segment_factor', rclpy.Parameter.Type.DOUBLE)

        self.declare_parameter('solver_name', rclpy.Parameter.Type.STRING)
       
        
        self.Q = self.get_parameter('control_parameters.mpc_controller.Q').get_parameter_value().double_array_value
        self.R = self.get_parameter('control_parameters.mpc_controller.R').get_parameter_value().double_array_value
        self.cost_slack_variables = self.get_parameter('control_parameters.mpc_controller.cost_slack_variables').get_parameter_value().double_value
        self.enable_predefined_cost = self.get_parameter('control_parameters.mpc_controller.enable_predefined_cost').get_parameter_value().bool_value
        self.N_mpc = self.get_parameter('control_parameters.mpc_controller.N').get_parameter_value().integer_value
        self.T_mpc = self.get_parameter('control_parameters.mpc_controller.T').get_parameter_value().double_value
        self.enable_predefined_constraints = self.get_parameter('control_parameters.mpc_controller.constraint.enable_predefined_constraints').get_parameter_value().bool_value
        self.lbx_init = self.get_parameter('control_parameters.mpc_controller.constraint.lbx').get_parameter_value().double_array_value
        self.ubx_init = self.get_parameter('control_parameters.mpc_controller.constraint.ubx').get_parameter_value().double_array_value
        self.lbu_init = self.get_parameter('control_parameters.mpc_controller.constraint.lbu').get_parameter_value().double_array_value
        self.ubu_init = self.get_parameter('control_parameters.mpc_controller.constraint.ubu').get_parameter_value().double_array_value
        self.soft_constraint_factor = self.get_parameter('control_parameters.mpc_controller.constraint.soft_constraint_factor').get_parameter_value().double_value
        self.auto_N = self.get_parameter('control_parameters.mpc_controller.auto_N').get_parameter_value().bool_value

        self.enable_obstacle = self.get_parameter('reference_tracking.enable_obstacle').get_parameter_value().bool_value
        self.frame_type = self.get_parameter('reference_tracking.frame_type').get_parameter_value().integer_value

        self.kappa_tresh = self.get_parameter('geometric_transformation.kappa_tresh').get_parameter_value().double_value
        self.num_points_seg = self.get_parameter('geometric_transformation.num_points_segment').get_parameter_value().integer_value
        self.seg_factor = self.get_parameter('geometric_transformation.segment_factor').get_parameter_value().double_value

        self.solver_name = self.get_parameter('solver_name').get_parameter_value().string_value


    def initalize_mpc_controller(self):
        
        self.set_MPC_cost_values(self.Q, self.R)

        self.frenet_system_obj = FrenetDynamics(self.system_parameters, self.system_type, self.frame_type)       # used to calculate the state matrizes and for frenet system definition

        self.constraint_obj = ConstraintDefinition(self.path_obj, self.ubx_init, self.ubu_init, self.soft_constraint_factor, self.enable_predefined_constraints, self.enable_obstacle)       # object that contains and constraint values depending on path progress
        init_constraints = self.constraint_obj.get_initial_constraints()

        self.cost_obj = CostDefinition(self.path_obj, self.Q, self.R, self.enable_predefined_cost)          # object that contains predefined cost values
        init_Q, init_R = self.cost_obj.get_initial_cost()

        if self.auto_N:
            self.N_mpc = int(self.T_mpc / self.system_period)

            self.get_logger().info(f'Auto N enabled. N_mpc: {self.N_mpc}')

        self.acados_obj = AcadosSolver(self.frenet_system_obj, self.solver_name)
        self.mpc_solver = self.acados_obj.create_mpc_solver(self.N_mpc, self.T_mpc, init_Q, init_R, self.cost_slack_variables, init_constraints, self.x0)

        self.u = np.zeros([3, self.N_mpc - 1])

    
    def initalize_reference_path(self):
        '''
        Initalize the reference path, which consists of a set of points. A predefined path can be set in the yaml file.
        '''
        self.path_obj = ReferencePath(num_points=self.num_points, trajectory_type=self.trajectory_type, num_interp_points = self.num_points_seg, segment_factor = self.seg_factor, kappa_treshhold = self.kappa_tresh, comp_type = 0, trajectory_scaling= self.traj_scaling, frame_type=self.frame_type)
        initial_position = self.path_obj.ref_path[:,0]
        self.x0 = np.vstack((initial_position.reshape(3,1), np.zeros((3,1), dtype=float)))
        

    def control_step(self, frenet_state, frenet_matrix):
        '''
        MPC control of the system using the velocity in Frenet coordinates. Uses transformation object defined in parent class. 

        Input:
            - frenetStateVector: [6x1 numpy array] (d_t, d_n, d_b, v_t, v_n, v_b)
        Output:
            - u_xyz: [3, numpy array] (u_x, u_y, u_z)
        '''

        param, s_MPC = self.predict_system_matrices(self.frenet_system_obj, self.path_obj.traj_matrix, self.s_prog, self.s_dot, self.N_mpc, self.T_mpc)

        self.s_prog = s_MPC[0]

        self.mpc_solver.set(0, "lbx", frenet_state)
        self.mpc_solver.set(0, "ubx", frenet_state)

        # initial set of cost
        Q, R = self.cost_obj.get_initial_cost()     # TODO unnötig?
        Q,R = np.array(Q).reshape(-1,1), np.array(R).reshape(-1,1)
        W = np.concatenate((Q.flatten(), R.flatten()))
        cost_W = np.diag(W)

        # send 0N force if end of trajectory is reached
        if s_MPC[0] >= self.path_obj.traj_matrix[0, -1]:
            return np.zeros([3, self.N_mpc - 1])
        

        for i in range(self.N_mpc):
            self.mpc_solver.set(i, "p", param[:,i])

            # if self.human_control == True:                               # optional: add a sinusoidal reference for human control testing
            #     if s_MPC[0] > self.path_obj.arc_length * 0.6:
            #         self.frenet_state_reference[1] = 0.01 * np.sin(25*s_MPC[i])
            #         self.frenet_state_reference[2] = 0.01 * np.cos(25*s_MPC[i])

            # const v_t over the prediction horizon
            self.mpc_solver.set(i, "yref", np.concatenate((self.frenet_state_reference.flatten(), np.zeros(3))))

            # set cost terms during mpc step
            if self.cost_obj.use_predefined_cost:
                cost_W = self.cost_obj.get_cost(s_MPC[i])
                self.set_MPC_cost(i, cost_W)

            # ### Implement change in cost for automation controller ###
            # if s_MPC[0] > self.path_obj.arc_length * 0.4 and s_MPC[0] < self.path_obj.arc_length * 0.6:                 # optional: change the automation controller cost in the middle of the trajectory       
            #     Q = np.array([1.0, 10000.1, 10000.1, 20.0, 1.0, 1.0]).reshape(-1,1)
            #     R = np.array([20.1, 20.1, 20.1]).reshape(-1,1)
            #     W = np.concatenate((Q.flatten(), R.flatten()))
            #     cost_W = np.diag(W)

            # elif s_MPC[0] > self.path_obj.arc_length * 0.6:
            #     Q = np.array([1.0, 0.0, 0.0, 20.0, 0.0, 0.0]).reshape(-1,1)
            #     R = np.array([2.1, 2.1, 2.1]).reshape(-1,1)
            #     W = np.concatenate((Q.flatten(), R.flatten()))
            #     cost_W = np.diag(W)

            # set constraints during mpc step
            if i > 0 and self.constraint_obj.use_predefined_constraints:
                constraints = self.constraint_obj.get_constraints(s_MPC[i], frenet_state)
                self.set_MPC_constraints(i, constraints)
            

        status = self.mpc_solver.solve()

        if status != 0:
            self.u[:,:] = np.zeros([3, self.N_mpc - 1])

            #raise Exception(f'acados ocp solver failed with status {status}')
            self.get_logger().error(f'acados ocp solver failed with status {status}')
            return self.u

        for i in range(self.N_mpc - 1):
            self.u[:,i] = self.mpc_solver.get(i, "u")

        self.new_control_output = True          # flag to identify new control calculation

        return self.u
    

    def predict_system_matrices(self, Sys, traj_matrix, s_prog, s_dot, mpc_N, mpc_T):
        '''
        Predict the system matrices (A, B) for each MPC step. 
        Compress the matrices into a single parameter vector for the solver to use.

        Output:
            - param_var: parameter vector containing the system matrices and the frame velocity for each MPC step [nx*nx + nx*nu + 1, 1]
        '''
        # get current s
        s_start = s_prog
        s_end = s_start + s_dot * mpc_T 

        s_MPC = np.linspace(s_start, s_end, mpc_N)

        s_dot_arr = np.array([s_dot])

        param_var = np.zeros([55, mpc_N])

        for i in range(mpc_N):
            
            idx = self.path_obj.get_idx_from_arc_length(s_MPC[i])

            point = traj_matrix[1:4,idx]
            kappa_var = traj_matrix[4,idx]
            tau_var = traj_matrix[5,idx]
            T = traj_matrix[6:9,idx]
            N = traj_matrix[9:12,idx]
            B = traj_matrix[12:15,idx]

            F_var = np.column_stack([T, N, B])
            #end_tim = time.time()

            if s_MPC[i] > traj_matrix[0,-1]:
                kappa_var = 0.0
                tau_var = 0.0
                F_var = self.F_prev
            else:
                self.F_prev = F_var

            #avg_tim[i] = end_tim - s_tim
            A_var, B_var = Sys.calculate_frenet_state_matrizes(F_var, s_dot, kappa_var, tau_var)
            #print(A_var[:,:,i])

            F_var = F_var

            param_var[:,i] = np.concatenate((A_var.flatten('F'), B_var.flatten('F'), s_dot_arr))

        return param_var, s_MPC
    

    def set_MPC_constraints(self, i, constraints):
        '''
        Set the constraints during the MPC step

        Input:
            - i, current mpc iteration
            - constraints: dictionary containing the current constraints
        '''
        self.mpc_solver.constraints_set(i, "lbx", constraints['lbx'])
        self.mpc_solver.constraints_set(i, "ubx", constraints['ubx'])

        self.mpc_solver.constraints_set(i, "lbu", constraints['lbu'])
        self.mpc_solver.constraints_set(i, "ubu", constraints['ubu'])


    def set_MPC_cost(self, i, cost_W):
        '''
        Set the cost during the MPC step

        Input:
            - i, current mpc iteration
            - cost_W: numpy array containing the current cost weights
        '''
        self.mpc_solver.cost_set(i, "W", cost_W)



    def set_MPC_cost_values(self, Q, R):
        '''
        Set the weights for the MPC control system

        Input:
            - Q: numpy array containing the weights for the states
            - R: numpy array containing the weights for the control inputs
        '''

        W = np.hstack([Q, R])

        self.W_values = W



    
def main(args=None):
    rclpy.init(args=args)
    node = MPCController()

    parallel_execution = True

    if parallel_execution:
        executor = MultiThreadedExecutor()      # multi-threaded executor to run the callbacks in parallel
        executor.add_node(node)
        executor.spin()
    else:
        rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()