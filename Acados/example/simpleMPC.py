from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel
import numpy as np
from casadi import SX, vertcat
import scipy.linalg
import matplotlib.pyplot as plt

import time

# CasADi symbolic variables
p_x = SX.sym('p_x')
p_y = SX.sym('p_y')
p_z = SX.sym('p_z')
v_x = SX.sym('v_x')
v_y = SX.sym('v_y')
v_z = SX.sym('v_z')

x = vertcat(p_x, p_y, p_z, v_x, v_y, v_z)

F_x = SX.sym('F_x')
F_y = SX.sym('F_y')
F_z = SX.sym('F_z')
u = vertcat(F_x, F_y, F_z)

p_x_dot = SX.sym('p_x_dot')
p_y_dot = SX.sym('p_y_dot')
p_z_dot = SX.sym('p_z_dot')
v_x_dot = SX.sym('v_x_dot')
v_y_dot = SX.sym('v_y_dot')
v_z_dot = SX.sym('v_z_dot')
xdot = vertcat(p_x_dot, p_y_dot, p_z_dot, v_x_dot, v_y_dot, v_z_dot)

# define model and ocp 
model = AcadosModel()

model.x = x
model.u = u
model.xdot = xdot
model.name = 'linear_mpc'

nx = model.x.rows()
nu = model.u.rows()
ny = nx + nu


# Define system dynamics: double integrator
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

# define OCP
ocp = AcadosOcp()
ocp.model = model

N = 150  # Prediction horizon
T = 1.0  # Total time
shooting_nodes = np.linspace(0, T, N+1)
ocp.solver_options.N_horizon = N

# stet cost
Q = 1*np.eye(nx) # np.diag([]) # 
Q[:3,:3] = 10000*np.eye(3)
R = 1*np.eye(nu)

# initial state
x_init = np.array([0, 0, 0, 0, 0, 0])

# ====== constraints ======
constraints = {
    'lbx': [-50, -50, -50, -50, -50, -50],
    'ubx': [50, 50, 50, 50, 50, 50],
    'lbu': [-50, -50, -50],
    'ubu': [50, 50, 50]
}

# initial equality constraint 
ocp.constraints.lbx_0 = x_init
ocp.constraints.ubx_0 = x_init
ocp.constraints.idxbx_0 = np.array([range(nx)])

# runtime constraint
ocp.constraints.lbx = np.array(constraints['lbx'])
ocp.constraints.ubx = np.array(constraints['ubx'])
ocp.constraints.idxbx = np.array([range(nx)])

ocp.constraints.lbu = np.array(constraints['lbu'])
ocp.constraints.ubu = np.array(constraints['ubu'])
ocp.constraints.idxbu = np.array([range(nu)])

# ====== cost ======
# runtime cost
ocp.cost.W = scipy.linalg.block_diag(Q, R)

Vx = np.zeros((ny, nx))
Vx[:nx, :] = np.eye(nx)
ocp.cost.Vx = Vx

Vu = np.zeros((ny, nu))
Vu[nx:, :] = np.eye(nu)
ocp.cost.Vu = Vu

#terminal cost (optinal as y_ref = y_ref_e)
# ocp.cost.W_e = Q
# ocp.cost.Vx_e = np.eye(nx)

# ====== reference ======
ocp.cost.yref = np.zeros((ny,))

goal_state = np.array([1, 2, 4, 0, 0, 0])
goal_input = np.array([0, 0, 0])

y_ref = np.concatenate((goal_state, goal_input))

ocp.cost.yref = y_ref
#ocp.cost.yref_e = goal_state

# ====== solver options ======
ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
ocp.solver_options.hessian_approx = 'EXACT'
ocp.solver_options.integrator_type = 'IRK'
ocp.solver_options.nlp_solver_type = 'SQP_RTI'
ocp.solver_options.globalization = 'FIXED_STEP'
ocp.solver_options.hpipm_mode = 'BALANCE'
ocp.solver_options.qp_solver_warm_start = 2
ocp.solver_options.print_level = 0

# ocp.solver_options.qp_solver = 'FULL_CONDENSING_QPOASES'

# ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
# ocp.solver_options.integrator_type = 'ERK'
# # ocp.solver_options.print_level = 1
# ocp.solver_options.nlp_solver_type = 'SQP' # SQP_RTI, SQP
# ocp.solver_options.globalization = 'FUNNEL_L1PEN_LINESEARCH'

ocp.solver_options.tf = T

ocp_solver = AcadosOcpSolver(ocp, json_file='mpc_solver.json')

########### Sim

t_run = np.arange(0, 100, T/N)

# variables
globalState = np.zeros((nx, len(t_run)))
globalState[:,0] = x_init
u_opt = np.zeros((nu, len(t_run)))

opt_trajectories = np.zeros((nx, N+1, len(t_run)))
opt_inputs = np.zeros((nu, N, len(t_run)))

t_sync = np.zeros((1, N, len(t_run)))
duration = np.zeros(len(t_run))

print(globalState[:,0].reshape(-1,).shape)

# Simulation loop
for t, val_t in enumerate(t_run):

    if t == 0:
        pass
    else:
        start_time = time.time()
        ocp_solver.set(0, "ubx", globalState[:,t-1].reshape(-1,))
        ocp_solver.set(0, "lbx", globalState[:,t-1].reshape(-1,))

        if val_t > 50:
            y_ref = np.array([3,3,3,0,0,0,0,0,0])

        for i in range(N):
            ocp_solver.set(i, "yref", y_ref)

        status = ocp_solver.solve()

        if status != 0:
            raise Exception(f'acados ocp solver failed with status {status}')

        u0 = ocp_solver.get(0, "u")
        x1 = ocp_solver.get(1, "x")

        # apply first control input to the system
        globalState[:,t] = x1

        for i in range(0,N):
            #if i < N:
            opt_inputs[:,i,t-1] = ocp_solver.get(i, "u")
            opt_trajectories[:,i,t] = ocp_solver.get(i, "x")

            t_sync[0, i, t] = T/N*i + t_run[t-1]
        end_time = time.time()
        duration[t-1] = end_time - start_time
        pass

print('Average duration: ', np.mean(duration))

#### print

plt.figure()
plt.subplot(3, 1, 1)
plt.plot(t_run, globalState[0, :], label='Position x')
plt.plot(t_run, globalState[1, :], label='Position y')
plt.plot(t_run, globalState[2, :], label='Position y')
plt.xlabel('t_run [s]')
plt.ylabel('States')
plt.legend()
plt.grid()


plt.subplot(3, 1, 2)
plt.plot(t_run, opt_inputs[0, 0, :], label='Force x')
plt.plot(t_run, opt_inputs[1, 0, :], label='Force y')
plt.plot(t_run, opt_inputs[2, 0, :], label='Force z')
plt.xlabel('Time [s]')
plt.ylabel('States')
plt.legend()
plt.grid()


plt.subplot(3, 1, 3)
plt.plot(t_run, globalState[3, :], label='Vel x')
plt.plot(t_run, globalState[4, :], label='Vel y')
plt.plot(t_run, globalState[5, :], label='Vel y')
plt.xlabel('Time [s]')
plt.ylabel('States')
plt.legend()
plt.grid()


plt.figure()
plt.plot(t_run, globalState[0, :], label='Position x')
for i in range(len(t_run)):
    plt.plot(t_sync[0, :, i], opt_trajectories[0, :-1, i], '--', label='Optimal trajectory x')

plt.show()
