import numpy as np
import matplotlib.pyplot as plt
from scipy.linalg import solve_continuous_are
from matplotlib.animation import FuncAnimation

# Classe do quadricóptero
class Quadcopter:
    def __init__(self, I, dt=0.01):
        self.Ix, self.Iy, self.Iz = I
        self.dt = dt
        self.x = np.zeros(6)  # [phi, theta, psi, p, q, r]

    def dynamics(self, x, u, disturbance=None):
        phi, theta, psi, p, q, r = x
        tau_phi, tau_theta, tau_psi = u

        # Adiciona distúrbio (vento) como torques extras, se fornecido
        if disturbance is not None:
            tau_phi += disturbance[0]
            tau_theta += disturbance[1]
            tau_psi += disturbance[2]

        # Cinemática de Euler
        phi_dot = p + q*np.sin(phi)*np.tan(theta) + r*np.cos(phi)*np.tan(theta)
        theta_dot = q*np.cos(phi) - r*np.sin(phi)
        psi_dot = q*np.sin(phi)/np.cos(theta) + r*np.cos(phi)/np.cos(theta)

        # Dinâmica rotacional
        p_dot = (1/self.Ix)*(tau_phi + (self.Iy - self.Iz)*q*r)
        q_dot = (1/self.Iy)*(tau_theta + (self.Iz - self.Ix)*r*p)
        r_dot = (1/self.Iz)*(tau_psi + (self.Ix - self.Iy)*p*q)

        return np.array([phi_dot, theta_dot, psi_dot, p_dot, q_dot, r_dot])

    def step(self, u, disturbance=None):
        x_dot = self.dynamics(self.x, u, disturbance)
        self.x += self.dt * x_dot
        return self.x

# Parâmetros do sistema
I = (0.02, 0.02, 0.04)      # Momentos de inércia
quad = Quadcopter(I)
B = np.array([[0,0,0], [0,0,0], [0,0,0],
              [1/I[0],0,0], [0,1/I[1],0], [0,0,1/I[2]]])
Q = np.diag([100, 100, 100, 1, 1, 1])
R = np.eye(3)
C = np.zeros((3,6)); C[0,0] = C[1,1] = C[2,2] = 1.0

# Configuração de ruído (vento)
wind_std = 1  # intensidade do distúrbio de torque (Nm)

# Tempo e referência
T = 8.0
dt = quad.dt
t = np.arange(0, T, dt)
r_phi = np.where((t > 1) & (t < 3) , np.deg2rad(10), 0.0)
r_theta = np.where((t > 3) & (t < 5), np.deg2rad(-10), 0.0)
r_psi = np.where((t > 5) & (t < 7), np.deg2rad(90), 0.0)
r_ref = np.vstack([r_phi, r_theta, r_psi])

# Armazenamento de históricos
phi_hist, theta_hist, psi_hist = [], [], []
p_hist, q_hist, r_hist = [], [], []
u_hist = []
dist_hist = []

# Simulação com ruído de vento
for k in range(len(t)):
    x = quad.x.copy()
    # Jacobiana A(x) por diferenças finitas
    A = np.zeros((6,6))
    f0 = quad.dynamics(x, np.zeros(3), disturbance=None)
    h = 1e-6
    for j in range(6):
        xp = x.copy(); xp[j] += h
        f1 = quad.dynamics(xp, np.zeros(3), disturbance=None)
        A[:,j] = (f1 - f0)/h

    # SDRE: resolve Riccati e obtém ganho
    P = solve_continuous_are(A, B, Q, R)
    K = np.linalg.inv(R) @ (B.T @ P)
    try:
        Kr = -np.linalg.inv(C @ np.linalg.inv(A - B @ K) @ B)
    except np.linalg.LinAlgError:
        Kr = np.zeros((3,3))

    # Lei de controle
    u = -K @ x + Kr @ r_ref[:,k]
    # Amostra distúrbio de vento (ruído gaussiano)
    disturbance = wind_std * np.random.randn(3)
    # Aplica comando com ruído
    quad.step(u, disturbance=disturbance)

    # Salva históricos
    phi_hist.append(quad.x[0]); theta_hist.append(quad.x[1]); psi_hist.append(quad.x[2])
    p_hist.append(quad.x[3]); q_hist.append(quad.x[4]); r_hist.append(quad.x[5])
    u_hist.append(u)
    dist_hist.append(disturbance)

# Converte para arrays
phi_hist = np.array(phi_hist)
theta_hist = np.array(theta_hist)
psi_hist = np.array(psi_hist)
p_hist = np.array(p_hist)
q_hist = np.array(q_hist)
r_hist = np.array(r_hist)
u_hist = np.array(u_hist)
dist_hist = np.array(dist_hist)

# Plot dos ângulos e referência
plt.figure(figsize=(10,8))
plt.subplot(3,1,1)
plt.plot(t, np.rad2deg(phi_hist), label='phi (°)')
plt.plot(t, np.rad2deg(r_phi), 'k--', alpha=0.4, label='ref φ')
plt.ylabel('φ (°)'); plt.legend(); plt.grid()

plt.subplot(3,1,2)
plt.plot(t, np.rad2deg(theta_hist), label='theta (°)')
plt.plot(t, np.rad2deg(r_theta), 'k--', alpha=0.4, label='ref θ')
plt.ylabel('θ (°)'); plt.legend(); plt.grid()

plt.subplot(3,1,3)
plt.plot(t, np.rad2deg(psi_hist), label='psi (°)')
plt.plot(t, np.rad2deg(r_psi), 'k--', alpha=0.4, label='ref ψ')
plt.ylabel('ψ (°)'); plt.xlabel('Tempo (s)'); plt.legend(); plt.grid()
plt.tight_layout()
plt.show()

# Plot dos torques de controle e distúrbio
plt.figure(figsize=(10,6))
plt.subplot(2,1,1)
plt.plot(t, u_hist[:,0], label='τφ controle')
plt.plot(t, dist_hist[:,0], '--', label='τφ distúrbio')
plt.ylabel('Torque φ'); plt.legend(); plt.grid()

plt.subplot(2,1,2)
plt.plot(t, u_hist[:,1], label='τθ controle')
plt.plot(t, dist_hist[:,1], '--', label='τθ distúrbio')
plt.plot(t, u_hist[:,2], label='τψ controle')
plt.plot(t, dist_hist[:,2], '--', label='τψ distúrbio')
plt.ylabel('Torque'); plt.xlabel('Tempo (s)'); plt.legend(); plt.grid()
plt.tight_layout()
plt.show()

# Animação 3D (simplificada)
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
L = 0.5
arms = np.array([[ L,0,0], [-L,0,0], [0, L,0], [0,-L,0]])

def update(k):
    ax.clear()
    phi = phi_hist[k]; theta = theta_hist[k]; psi = psi_hist[k]
    # Rotação ZYX
    Rz = np.array([[np.cos(psi), -np.sin(psi), 0],
                   [np.sin(psi),  np.cos(psi), 0],
                   [0, 0, 1]])
    Ry = np.array([[ np.cos(theta), 0, np.sin(theta)],
                   [0, 1, 0],
                   [-np.sin(theta), 0, np.cos(theta)]])
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(phi), -np.sin(phi)],
                   [0, np.sin(phi),  np.cos(phi)]])
    R = Rz @ Ry @ Rx
    pts = (R @ arms.T).T
    ax.plot([pts[0,0], pts[1,0]], [pts[0,1], pts[1,1]], [pts[0,2], pts[1,2]], 'r-')
    ax.plot([pts[2,0], pts[3,0]], [pts[2,1], pts[3,1]], [pts[2,2], pts[3,2]], 'b-')
    ax.set_xlim(-1.5,1.5); ax.set_ylim(-1.5,1.5); ax.set_zlim(-0.5,0.5)
    ax.set_title(f'Tempo: {t[k]:.2f} s')

ani = FuncAnimation(fig, update, frames=len(t), interval=50)
plt.show()
