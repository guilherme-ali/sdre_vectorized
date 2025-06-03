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

# Classe do Filtro de Kalman Estendido
class ExtendedKalmanFilter:
    def __init__(self, dt, process_noise_std, measurement_noise_std):
        self.dt = dt
        self.x_hat = np.zeros(6)  # Estado estimado [phi, theta, psi, p, q, r]
        self.P = np.eye(6) * 1.0  # Matriz de covariância do erro
        
        # Matriz de ruído do processo (Q)
        self.Q = np.eye(6) * (process_noise_std**2)
        
        # Matriz de ruído da medição (R) - assumindo que medimos ângulos e velocidades angulares
        self.R = np.eye(6) * (measurement_noise_std**2)
        
        # Matriz de observação (medimos todos os estados)
        self.H = np.eye(6)
        
    def predict(self, u, quad_model):
        """Etapa de predição do filtro de Kalman"""
        # Predição do estado usando o modelo dinâmico
        x_dot = quad_model.dynamics(self.x_hat, u, disturbance=None)
        self.x_hat = self.x_hat + self.dt * x_dot
        
        # Jacobiana da dinâmica (matriz F)
        F = self.compute_jacobian(self.x_hat, u, quad_model)
        
        # Atualização da covariância
        self.P = F @ self.P @ F.T + self.Q
        
    def update(self, measurement):
        """Etapa de atualização do filtro de Kalman"""
        # Inovação
        y = measurement - self.H @ self.x_hat
        
        # Covariância da inovação
        S = self.H @ self.P @ self.H.T + self.R
        
        # Ganho de Kalman
        K = self.P @ self.H.T @ np.linalg.inv(S)
        
        # Atualização do estado
        self.x_hat = self.x_hat + K @ y
        
        # Atualização da covariância
        I = np.eye(6)
        self.P = (I - K @ self.H) @ self.P
        
    def compute_jacobian(self, x, u, quad_model):
        """Calcula a jacobiana da dinâmica por diferenças finitas"""
        F = np.zeros((6, 6))
        f0 = quad_model.dynamics(x, u, disturbance=None)
        h = 1e-6
        
        for j in range(6):
            xp = x.copy()
            xp[j] += h
            f1 = quad_model.dynamics(xp, u, disturbance=None)
            F[:, j] = (f1 - f0) / h
            
        # Converte para matriz discreta
        F_discrete = np.eye(6) + F * self.dt
        return F_discrete

# Parâmetros do sistema
I = (0.02, 0.02, 0.04)      # Momentos de inércia
quad = Quadcopter(I)
B = np.array([[0,0,0], [0,0,0], [0,0,0],
              [1/I[0],0,0], [0,1/I[1],0], [0,0,1/I[2]]])
Q = np.diag([100, 100, 100, 1, 1, 1])
R = np.eye(3)
C = np.zeros((3,6)); C[0,0] = C[1,1] = C[2,2] = 1.0

# Configuração de ruídos
wind_std = 1.0              # intensidade do distúrbio de torque (Nm)
process_noise_std = 0.000001     # ruído do processo para o filtro de Kalman
measurement_noise_std = 0 # ruído de medição (sensores)

# Inicialização do filtro de Kalman
ekf = ExtendedKalmanFilter(quad.dt, process_noise_std, measurement_noise_std)

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
phi_est_hist, theta_est_hist, psi_est_hist = [], [], []
p_est_hist, q_est_hist, r_est_hist = [], [], []
phi_meas_hist, theta_meas_hist, psi_meas_hist = [], [], []
u_hist = []
dist_hist = []

# Simulação com ruído de vento e estimação de estados
for k in range(len(t)):
    # Estado real atual
    x_real = quad.x.copy()
    
    # Simulação das medições com ruído
    measurement_noise = measurement_noise_std * np.random.randn(6)
    y_measured = x_real + measurement_noise
    
    # Etapa de predição do filtro de Kalman
    # (usa o comando anterior se k > 0, senão usa zeros)
    u_prev = u_hist[-1] if k > 0 else np.zeros(3)
    ekf.predict(u_prev, quad)
    
    # Etapa de atualização do filtro de Kalman
    ekf.update(y_measured)
    
    # Usa o estado estimado para o controle SDRE
    x_estimated = ekf.x_hat.copy()
    
    # Jacobiana A(x) por diferenças finitas no estado estimado
    A = np.zeros((6,6))
    f0 = quad.dynamics(x_estimated, np.zeros(3), disturbance=None)
    h = 1e-6
    for j in range(6):
        xp = x_estimated.copy(); xp[j] += h
        f1 = quad.dynamics(xp, np.zeros(3), disturbance=None)
        A[:,j] = (f1 - f0)/h

    # SDRE: resolve Riccati e obtém ganho
    P = solve_continuous_are(A, B, Q, R)
    K = np.linalg.inv(R) @ (B.T @ P)
    try:
        Kr = -np.linalg.inv(C @ np.linalg.inv(A - B @ K) @ B)
    except np.linalg.LinAlgError:
        Kr = np.zeros((3,3))

    # Lei de controle baseada no estado estimado
    u = -K @ x_estimated + Kr @ r_ref[:,k]
    
    # Amostra distúrbio de vento (ruído gaussiano)
    disturbance = wind_std * np.random.randn(3)
    
    # Aplica comando com ruído ao sistema real
    quad.step(u, disturbance=disturbance)

    # Salva históricos
    # Estados reais
    phi_hist.append(quad.x[0]); theta_hist.append(quad.x[1]); psi_hist.append(quad.x[2])
    p_hist.append(quad.x[3]); q_hist.append(quad.x[4]); r_hist.append(quad.x[5])
    
    # Estados estimados
    phi_est_hist.append(ekf.x_hat[0]); theta_est_hist.append(ekf.x_hat[1]); psi_est_hist.append(ekf.x_hat[2])
    p_est_hist.append(ekf.x_hat[3]); q_est_hist.append(ekf.x_hat[4]); r_est_hist.append(ekf.x_hat[5])
    
    # Medições ruidosas
    phi_meas_hist.append(y_measured[0]); theta_meas_hist.append(y_measured[1]); psi_meas_hist.append(y_measured[2])
    
    u_hist.append(u)
    dist_hist.append(disturbance)

# Converte para arrays
phi_hist = np.array(phi_hist)
theta_hist = np.array(theta_hist)
psi_hist = np.array(psi_hist)
p_hist = np.array(p_hist)
q_hist = np.array(q_hist)
r_hist = np.array(r_hist)

phi_est_hist = np.array(phi_est_hist)
theta_est_hist = np.array(theta_est_hist)
psi_est_hist = np.array(psi_est_hist)
p_est_hist = np.array(p_est_hist)
q_est_hist = np.array(q_est_hist)
r_est_hist = np.array(r_est_hist)

phi_meas_hist = np.array(phi_meas_hist)
theta_meas_hist = np.array(theta_meas_hist)
psi_meas_hist = np.array(psi_meas_hist)

u_hist = np.array(u_hist)
dist_hist = np.array(dist_hist)

# Plot dos ângulos, estimativas, medições, torques de controle e distúrbio
fig_2d, axs = plt.subplots(6, 1, figsize=(12, 18), sharex=True)

# Subplot 1: Phi
axs[0].plot(t, np.rad2deg(phi_hist), 'b-', label='φ real (°)', linewidth=2)
axs[0].plot(t, np.rad2deg(phi_est_hist), 'r--', label='φ estimado (°)', linewidth=2)
axs[0].plot(t, np.rad2deg(phi_meas_hist), 'g.', alpha=0.3, markersize=1, label='φ medido (°)')
axs[0].plot(t, np.rad2deg(r_phi), 'k--', alpha=0.6, label='ref φ')
axs[0].set_ylabel('φ (°)')
axs[0].legend()
axs[0].grid(True)
axs[0].set_title('Simulação de Atitude com SDRE, Filtro de Kalman e Distúrbios')

# Subplot 2: Theta
axs[1].plot(t, np.rad2deg(theta_hist), 'b-', label='θ real (°)', linewidth=2)
axs[1].plot(t, np.rad2deg(theta_est_hist), 'r--', label='θ estimado (°)', linewidth=2)
axs[1].plot(t, np.rad2deg(theta_meas_hist), 'g.', alpha=0.3, markersize=1, label='θ medido (°)')
axs[1].plot(t, np.rad2deg(r_theta), 'k--', alpha=0.6, label='ref θ')
axs[1].set_ylabel('θ (°)')
axs[1].legend()
axs[1].grid(True)

# Subplot 3: Psi
axs[2].plot(t, np.rad2deg(psi_hist), 'b-', label='ψ real (°)', linewidth=2)
axs[2].plot(t, np.rad2deg(psi_est_hist), 'r--', label='ψ estimado (°)', linewidth=2)
axs[2].plot(t, np.rad2deg(psi_meas_hist), 'g.', alpha=0.3, markersize=1, label='ψ medido (°)')
axs[2].plot(t, np.rad2deg(r_psi), 'k--', alpha=0.6, label='ref ψ')
axs[2].set_ylabel('ψ (°)')
axs[2].legend()
axs[2].grid(True)

# Subplot 4: Velocidades angulares (p, q)
axs[3].plot(t, p_hist, 'b-', label='p real (rad/s)', linewidth=2)
axs[3].plot(t, p_est_hist, 'r--', label='p estimado (rad/s)', linewidth=2)
axs[3].plot(t, q_hist, 'b-', alpha=0.7, label='q real (rad/s)', linewidth=2)
axs[3].plot(t, q_est_hist, 'r--', alpha=0.7, label='q estimado (rad/s)', linewidth=2)
axs[3].set_ylabel('p, q (rad/s)')
axs[3].legend()
axs[3].grid(True)

# Subplot 5: Torques de Controle e Distúrbio (Roll e Pitch)
axs[4].plot(t, u_hist[:,0], label='τφ controle')
axs[4].plot(t, dist_hist[:,0], '--', label='τφ distúrbio')
axs[4].plot(t, u_hist[:,1], label='τθ controle')
axs[4].plot(t, dist_hist[:,1], '--', label='τθ distúrbio')
axs[4].set_ylabel('Torques φ, θ (Nm)')
axs[4].legend()
axs[4].grid(True)

# Subplot 6: Torque de Controle e Distúrbio (Yaw)
axs[5].plot(t, u_hist[:,2], label='τψ controle')
axs[5].plot(t, dist_hist[:,2], '--', label='τψ distúrbio')
axs[5].set_ylabel('Torque ψ (Nm)')
axs[5].set_xlabel('Tempo (s)')
axs[5].legend()
axs[5].grid(True)

plt.tight_layout()
plt.show()

# Animação 3D (usando estados reais)
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

ani = FuncAnimation(fig, update, frames=len(t), interval=1)
plt.show()
