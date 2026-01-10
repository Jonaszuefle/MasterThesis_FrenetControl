from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel
import numpy as np
from casadi import SX, vertcat
import scipy.linalg

# CasADi symbolic variables
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

# define model and ocp 
model = AcadosModel()

model.x = x
model.u = u
model.xdot = xdot
model.name = 'linear_mpc'

nx = model.x.rows()
nu = model.u.rows()
ny = nu

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


N = 200  # Prediction horizon
T = 5.0  # Total time
shooting_nodes = np.linspace(0, T, N+1)
ocp.solver_options.N_horizon = N

# stet cost
Q = 50*np.eye(nx) # np.diag([]) # 
Q[0,0] = 5000
Q[1,1] = 5000
R = 0.1* np.eye(nu)

# initial cost (inputs only)
# ocp.cost.cost_type_0 = 'LINEAR_LS'
# ny_0 = nu
# ocp.cost.Vu_0 = np.eye(ny_0)
# ocp.cost.Vx_0 = np.zeros([ny_0,nx])
# ocp.cost.W_0 = R;  # scale to match the original cost
# ocp.cost.yref_0 = np.zeros([ny_0,])

# running cost
ocp.cost.cost_type = 'LINEAR_LS'  

ocp.cost.W = scipy.linalg.block_diag(Q, R)
ocp.cost.Vu = np.block([[np.zeros((nx, nu))],
                        [np.eye(nu)*0]])
ocp.cost.Vx = np.block([[np.eye(nx)],
                        [np.zeros((nu, nx))]
                        ])
ny = nx + nu
ocp.cost.yref = np.zeros((ny,))
ocp.cost.yref[0] = 1
ocp.cost.yref[1] = 1
ocp.cost.yref[4] = 10

# # terminal cost 
# ocp.cost.cost_type_e = 'LINEAR_LS'      # terminal cost
# ocp.cost.W_e = Q
# ocp.cost.Vx_e = np.eye(nx)
# ocp.cost.yref_e = np.array([1, 1, 0, 0])#np.zeros(nx,)#np.zeros((nx,))

F_max = 200
# Constraints u 
ocp.constraints.lbu = -F_max * np.ones(nu,)
ocp.constraints.ubu = F_max * np.ones(nu,)
ocp.constraints.idxbu = np.array(range(nu))

# Constraints x0
x0 = np.array([2.0, 3.0, 0.0, 0.0])
ocp.constraints.idxbx_0 = np.array([range(nx)])#np.array(range(2,4))
ocp.constraints.x0 = x0 #np.array(x0[range(2,4)])

# Constraints x 
ocp.constraints.lbx = np.array([-20.0, -20.0, -20.0 , -20.0])
ocp.constraints.ubx = np.array([20.0, 20.0, 20.0, 20.0])
ocp.constraints.idxbx = np.array(range(nx))

# # terminal
# x_goal = np.array([2, 2, 0, 0])
# ocp.constraints.idxbx_e = np.array(range(0,4))  # Only constrain the position
# ocp.constraints.lbx_e = np.array([x_goal[range(0,4)]])
# ocp.constraints.ubx_e = np.array([x_goal[range(0,4)]])

ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM' # 'FULL_CONDENSING_QPOASES' # 

ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
ocp.solver_options.integrator_type = 'ERK'
# ocp.solver_options.print_level = 1
ocp.solver_options.nlp_solver_type = 'SQP' # SQP_RTI, SQP
ocp.solver_options.globalization = 'FUNNEL_L1PEN_LINESEARCH'



ocp.solver_options.tf = T

# Solve
ocp_solver = AcadosOcpSolver(ocp, json_file='acados_ocp.json')

test = ocp_solver.get_cost()
print(test)


# solve
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


x_last = ocp_solver.get(0, "x")
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