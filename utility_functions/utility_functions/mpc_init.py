from dynamic_models.frenet_dynamics import FrenetDynamics

from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel
import numpy as np
import scipy.linalg

from casadi import vertcat, SX, reshape, diag, DM

class AcadosSolver:
    def __init__(self, frenet_system_obj, solver_name):

        self.frenet_system_obj = frenet_system_obj
        self.solver_name = solver_name


    def create_mpc_solver(self, N, T, Q, R, slack_cost, constraints, x0):
        """
        Create an MPC solver for the Frame system using acados.
        For specific information visit the acados documentation: https://docs.acados.org/
        The problem formulation of acados is stated in this document: https://github.com/acados/acados/blob/main/docs/problem_formulation/problem_formulation_ocp_mex.pdf

        Basic idea:
            - Linear time varying system dynamics
            -> define system matrices A,B and frame velocity s_dot as parameters
            -> before each optimization step, the parameters are updated with the current state and input
        """
        
        # define the system variables/vectors
        
        x, x_dot, u = self.frenet_system_obj.define_system_variables()      # define system as casadi variables

        # define model
        model = AcadosModel()

        model.name = self.solver_name
        model.x = x
        model.u = u
        model.xdot = x_dot

        nx = model.x.rows()
        nu = model.u.rows()
        ny = nx + nu                # for reference, acados specific

        #===== parameter =====
        A_par = SX.sym('A_var', nx**2, 1)       # parameter for varaint system matrices
        B_par = SX.sym('B_var', nx*nu, 1)
    
        s_dot = SX.sym('s_dot', 1, 1)           # progress rate


        #===== System Dynamics =====
        A_var = reshape(A_par, nx, nx)          # reshape matrices using column-major
        B_var = reshape(B_par, nx, nu)

        F = SX.zeros(nx, 1)
        F[0] = -s_dot 

        dxdt = A_var @ x + B_var @ u + F

        model.p = vertcat(A_par, B_par, s_dot)
        model.f_expl_expr = dxdt
        model.f_impl_expr = x_dot - dxdt

        # define ocp model
        ocp = AcadosOcp()
        ocp.model = model


        #===== solver options =====       
        ocp.solver_options.tf = T           # discretization steps
        ocp.solver_options.N_horizon = N    # prediction horizon

        ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
        ocp.solver_options.hessian_approx = 'EXACT'
        ocp.solver_options.integrator_type = 'IRK'
        ocp.solver_options.nlp_solver_type = 'SQP_RTI'
        ocp.solver_options.globalization = 'FIXED_STEP'
        ocp.solver_options.hpipm_mode = 'BALANCE'
        ocp.solver_options.qp_solver_warm_start = 1
        ocp.solver_options.print_level = 0


        #===== cost function =====
        #Q_val = np.array([1e3, 1e5, 1e5, 1e4, 1e2, 1e2])        # initial values -- are overwritten during runtime
        Q_val = Q
        #R_val = np.array([1e0, 1e0, 1e0])
        R_val = R
        
        W_val = np.hstack([Q_val, R_val])

        ocp.cost.W = np.diag(W_val)  # runtime cost

        Vx = np.zeros((ny, nx))
        Vx[:nx, :] = np.eye(nx)
        ocp.cost.Vx = Vx            # define which states are included in the cost function

        Vu = np.zeros((ny, nu))
        Vu[nx:, :] = np.eye(nu)
        ocp.cost.Vu = Vu            # define which inputs are included in the cost function


        # ====== reference ======
        ocp.cost.yref = np.zeros((ny,))         

        goal_state = np.array([0, 0, 0, 0, 0, 0])
        goal_input = np.array([0, 0, 0])

        y_ref = np.concatenate((goal_state, goal_input))

        ocp.cost.yref = y_ref

        x0 = np.array(x0)

        # initial equality constraint 
        ocp.constraints.lbx_0 = x0
        ocp.constraints.ubx_0 = x0
        ocp.constraints.idxbx_0 = np.array([range(nx)])

        # runtime constraint
        ocp.constraints.lbx = np.array(constraints['lbx'])
        ocp.constraints.ubx = np.array(constraints['ubx'])
        ocp.constraints.idxbx = np.array([0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 4, 5])          # idx for constraints -- acados specific -> transforms into selection matrix
                                                                                        # for constraints for each contouring error state (d_n and d_b)
        ocp.constraints.lbu = np.array(constraints['lbu'])
        ocp.constraints.ubu = np.array(constraints['ubu'])
        ocp.constraints.idxbu = np.array([range(nu)])

        # soft constraints
        Jsbx = np.zeros((12, 4))                # selection matrix for soft constraints
        Jsbx[3, 0] = 1
        Jsbx[4, 1] = 1
        Jsbx[7, 2] = 1
        Jsbx[8, 3] = 1
        ocp.constraints.Jsbx = Jsbx

        ocp.cost.zl = np.ones((4,)) * slack_cost       # cost term for soft constraints (slack variable)
        Zl = np.ones(4)#np.eye(4)
        ocp.cost.Zl = Zl

        ocp.cost.zu = np.ones((4,)) * slack_cost
        Zu = np.ones(4)#np.eye(4)
        ocp.cost.Zu = Zu

        # ====== parameter ======
        A_init= np.zeros((nx,nx))
        A_init[:3,3:] = np.eye(nx//2)

        B_init = np.zeros((nx,nu))
        B_init[3:, :] = np.eye(nu)

        s_dot_init = np.array([0.0])

        array = np.concatenate((A_init.flatten('F'), B_init.flatten('F'), s_dot_init))      # column-major

        ocp.parameter_values = array            # initial paramater values

        #===== solve =====
        ocp_solver = AcadosOcpSolver(ocp, json_file = self.solver_name + '.json')

        #===== setup information =====
        # acados_info = """Simulation mit folgenden Parametern:
        #         - Q: {}
        #         - R: {}
        #         - N = {}
        #         - T = {}
        #         """.format(Q_val, R_val, N, T)

        return ocp_solver#, acados_info