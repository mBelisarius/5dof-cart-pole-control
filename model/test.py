import matplotlib.pyplot as plt
import numpy as np

import solver as sv


dof = 5

t_span = (0.0, 10.0)

f0 = np.zeros(2 * dof)
f0[6] = 1.0e-8

fv = [0.0, 0.0]

p = 1.0e2
dt = 1.0e-3

# solver_ode = sv.SolverOde(dof)
# sol_t, sol_f = solver_ode.solve(t_span, f0, fv)

solver_lcp = sv.SolverLcp(dof)
sol_t, sol_f = solver_lcp.solve(t_span, f0, fv, dt)

sol_x, sol_x_dot, sol_y, sol_y_dot, sol_z, sol_z_dot, sol_theta, sol_theta_dot, sol_phi, sol_phi_dot = sol_f


# sol_xo = np.array([sv.Solver.fn_Xo(sol_t[i], *sol_f[:, i], *params_sim.values(), *fv) for i in range(len(sol_t))]).reshape((len(sol_t), 3))
# sol_xo_dot = np.array([sv.Solver.fn_Xo_dot(sol_t[i], *sol_f[:, i], *params_sim.values(), *fv) for i in range(len(sol_t))]).reshape((len(sol_t), 3))

# sol_xc = np.array([sv.Solver.fn_Xc(sol_t[i], *sol_f[:, i], *params_sim.values(), *fv) for i in range(len(sol_t))]).reshape((len(sol_t), 3))
# sol_xc_dot = np.array([sv.Solver.fn_Xc_dot(sol_t[i], *sol_f[:, i], *params_sim.values(), *fv) for i in range(len(sol_t))]).reshape((len(sol_t), 3))


plt.figure(figsize=(15, 5))

plt.plot(sol_t, sol_x, label=r'$x(t)$')
plt.plot(sol_t, sol_y, label=r'$y(t)$')
plt.plot(sol_t, sol_z, label=r'$z(t)$')
plt.plot(sol_t, sol_theta, label=r'$\theta(t)$')
plt.plot(sol_t, sol_phi, label=r'$\varphi(t)$')

plt.axhline(y=np.pi, linestyle='--', color='gray', label=r'$\pi$')
plt.axhline(y=-np.pi, linestyle='--', color='gray', label=r'$-\pi$')

plt.xlabel('Time $t$')
plt.ylabel('State values')
plt.title('Evolution of state')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()