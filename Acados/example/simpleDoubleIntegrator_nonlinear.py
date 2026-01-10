from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel
import numpy as np
from casadi import SX, vertcat
import scipy.linalg

# CasADi Model definition --------
p_x = SX.sym('p_x')
p_y = SX.sym('p_y')
v_x = SX.sym('v_x')
v_y = SX.sym('v_y')

x = vertcat(p_x, p_y, v_x, v_y)

F_x = SX.sym('F_x')
F_y = SX.sym('F_y')
u = vertcat(F_x, F_y)

p_x_dot = SX.sym('p_x_dot')
p_y_dot = SX.sym('p_y_dot')
v_x_dot = SX.sym('v_x_dot')
v_y_dot = SX.sym('v_y_dot')
xdot = vertcat(p_x_dot,p_y_dot, v_x_dot, v_y_dot)


# Model --------
model = AcadosModel()

model.x = x
model.u = u
model.xdot = xdot
model.name = 'simple_double_integrator_nonlin'

nx = model.x.rows()
nu = model.u.rows()
ny = nx + nu
ny_e = nx

# Define system dynamics: x_dot = A*x + B*u
A = np.block([
    [np.zeros((nx//2, nx//2)), np.eye(nx//2)],
    [np.zeros((nx//2, nx//2)), np.zeros((nx//2, nx//2))]
])
        
B = np.block([[np.zeros((nx//2, nu))], 
             [np.eye(nx//2,nu)]])

dxdt = A @ x + B @ u

# Continuous-time dynamics
f_expl = vertcat(dxdt)
f_impl = xdot - f_expl

model.f_expl_expr = f_expl
model.f_impl_expr = f_impl

ocp = AcadosOcp()
ocp.model = model


N = 20  # Prediction horizon
T = 4.0  # Total time
shooting_nodes = np.linspace(0, T, N+1)

ocp.solver_options.N_horizon = N
ocp.solver_options.tf = T

# Solver options --------
ocp.solver_options.qp_solver = 'FULL_CONDENSING_QPOASES' # 'PARTIAL_CONDENSING_HPIPM' #
ocp.solver_options.hessian_approx = 'EXACT' # 'GAUSS_NEWTON' #
ocp.solver_options.integrator_type = 'IRK' # 'ERK'
ocp.solver_options.nlp_solver_type = 'SQP_RTI' # 'SQP' 
ocp.solver_options.globalization = 'FIXED_STEP' # 'FUNNEL_L1PEN_LINESEARCH'
ocp.solver_options.print_level = 0
ocp.solver_options.qp_solver_warm_start = 2


# Cost definition --------
Q = 1*np.eye(nx) # np.diag([]) # 
R = 1*np.eye(nu)

# # initial cost (inputs only)
# ocp.cost.cost_type_0 = 'LINEAR_LS'
# ny_0 = nu
# ocp.cost.Vu_0 = np.eye(ny_0)
# ocp.cost.Vx_0 = np.zeros([ny_0,nx])
# ocp.cost.W_0 = R;  # scale to match the original cost
# ocp.cost.yref_0 = np.zeros([ny_0,])

# running cost
ocp.cost.cost_type = 'NONLINEAR_LS'  
ocp.cost.cost_type_e = 'NONLINEAR_LS'

#ocp.cost.W = scipy.linalg.block_diag(Q, R)
ocp.cost.W_e = Q

# ocp.cost.yref = np.zeros((ny, ))
# ocp.cost.yref_e = np.zeros((nx, ))
#ocp.model.cost_y_expr = vertcat(x, u)
ocp.model.cost_y_expr_e = model.x

x0 = np.array([1.0, 1.0, 0.0, 0.0])

ocp.constraints.idxbx_0 = np.array([range(nx)])#np.array(range(2,4))
ocp.constraints.x0 = x0 #np.array(x0[range(2,4)])


x_goal = np.array([2, 1, 1, 1])
n_goal = np.zeros(nu,)
y_goal = np.concatenate(( x_goal, n_goal))
#ocp.cost.yref = y_goal
ocp.cost.yref_e = x_goal



# Solve --------
ocp_solver = AcadosOcpSolver(ocp, json_file='acados_ocp_nonlin.json')

status = ocp_solver.solve()
ocp_solver.print_statistics() # encapsulates: stat = ocp_solver.get_stats("statistics")
sqp_iter = ocp_solver.get_stats('sqp_iter')
print(f'acados returned status {status}.')

# ocp_solver.store_iterate(f'it{ocp.solver_options.nlp_solver_max_iter}_{model.name}.json')

# get solution
simX = np.array([ocp_solver.get(i,"x") for i in range(N+1)])
simU = np.array([ocp_solver.get(i,"u") for i in range(N)])
pi_multiplier = [ocp_solver.get(i, "pi") for i in range(N)]
print(f"cost function value = {ocp_solver.get_cost()}")


# print summary
print(f"cost function value = {ocp_solver.get_cost()} after {sqp_iter} SQP iterations")
# print(f"alphas: {alphas[:iter]}")
# print(f"total number of QP iterations: {sum(qp_iters[:iter])}")
# max_infeasibility = np.max(residuals[1:3])
# print(f"max infeasibility: {max_infeasibility}")


def acados_status(status):
    match status:
        case 0:
            print('ACADOS_SUCCESS')
        case 1:
            raise Exception('Nan detected')
        case 2:
            raise Exception('Max num of iterations reached')
        case 3:
            raise Exception('Min step size reached')
        case 4:
            raise Exception('QP solver failed')
        case 5:
            raise Exception('Solver created')
        case 6:
            raise Exception('Problem unbaunded')
        case 7:
            raise Exception('Solver Timeout')
        case _:
            raise Exception('Unknown error')
        
acados_status(status)


x_last = ocp_solver.get(20, "x")
print(f"x_last = {x_last}")
import matplotlib.pyplot as plt

# Plotting the results
time = np.linspace(0, T, N+1)

plt.figure()
plt.subplot(3, 1, 1)
plt.plot(time, simX[:, 0], label='Position x')
plt.plot(time, simX[:, 2], label='Velocity x')
plt.xlabel('Time [s]')
plt.ylabel('States')
plt.legend()
plt.grid()

plt.subplot(3, 1, 2)
plt.plot(time, simX[:, 1], label='Position y')
plt.plot(time, simX[:, 3], label='Velocity y')
plt.xlabel('Time [s]')
plt.ylabel('States')
plt.legend()
plt.grid()

plt.subplot(3, 1, 3)
plt.plot(time[:-1], simU[:, 0], label='Force x')
plt.plot(time[:-1], simU[:, 1], label='Foprce y')
plt.xlabel('Time [s]')
plt.ylabel('Force')
plt.legend()
plt.grid()

plt.tight_layout()
plt.show()

print(ocp.cost.W)