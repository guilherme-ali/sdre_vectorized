import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint
from scipy.linalg import solve_continuous_are

class DroneDynamics:
    def __init__(self, m, g, Ixx, Iyy, Izz, Jr=0.0):
        """
        Inicializa os parâmetros físicos do drone.
        
        Args:
            m (float): Massa do drone (kg)
            g (float): Aceleração gravitacional (m/s²)
            Ixx (float): Momento de inércia no eixo x (kg·m²)
            Iyy (float): Momento de inércia no eixo y (kg·m²)
            Izz (float): Momento de inércia no eixo z (kg·m²)
            Jr (float, optional): Momento de inércia do rotor (kg·m²). Default = 0.0
        """
        self.m = m
        self.g = g
        self.Ixx = Ixx
        self.Iyy = Iyy
        self.Izz = Izz
        self.Jr = Jr

    def model(self, state, t, u, Omega_r):
        """
        Modelo do drone para integração com odeint.
        
        Args:
            state (np.array[12]): Vetor de estado
            t (float): Tempo atual (não usado, necessário para odeint)
            u (list[4]): Vetor de controle
            Omega_r (float): Velocidade angular residual do rotor
            
        Returns:
            np.array[12]: Derivadas temporais do estado
        """
        x, y, z, vx, vy, vz, phi, theta, psi, p, q, r = state
        
        # Calcula funções trigonométricas
        c_phi = np.cos(phi)
        s_phi = np.sin(phi)
        c_theta = np.cos(theta)
        s_theta = np.sin(theta)
        c_psi = np.cos(psi)
        s_psi = np.sin(psi)
        t_theta = np.tan(theta)
        
        # Dinâmica translacional (acelerações lineares)
        ax = (c_phi * c_psi * s_theta + s_phi * s_psi) * u[0] / self.m
        ay = (c_phi * s_psi * s_theta - c_psi * s_phi) * u[0] / self.m
        az = -self.g + (c_phi * c_theta) * u[0] / self.m
        
        # Dinâmica rotacional (acelerações angulares)
        dp = ((self.Iyy - self.Izz) * q * r - self.Jr * Omega_r * q + u[1]) / self.Ixx
        dq = ((self.Izz - self.Ixx) * p * r + self.Jr * Omega_r * p + u[2]) / self.Iyy
        dr = ((self.Ixx - self.Iyy) * p * q + u[3]) / self.Izz
        
        # Cinemática angular (derivadas dos ângulos de Euler)
        dphi = p + (q * s_phi + r * c_phi) * t_theta
        dtheta = q * c_phi - r * s_phi
        dpsi = (q * s_phi + r * c_phi) / c_theta
        
        return [vx, vy, vz, ax, ay, az, dphi, dtheta, dpsi, dp, dq, dr]

def simulate_drone():
    # Parâmetros do drone (quadrotor típico)
    params = {
        'm': 1,      # kg
        'g': 9.81,     # m/s²
        'Ixx': 0.034,  # kg·m²
        'Iyy': 0.034,  # kg·m²
        'Izz': 0.097,  # kg·m²
        'Jr': 0.0    # kg·m²
    }
    
    # Criar instância do modelo do drone
    drone = DroneDynamics(**params)
    
    # Condições iniciais [x, y, z, vx, vy, vz, phi, theta, psi, p, q, r]
    state0 = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    
    # Tempo de simulação: 10 segundos com 1000 pontos
    t = np.linspace(0, 10, 1000)
    
    # Velocidade angular residual do rotor
    Omega_r = 0.0  # rad/s
    
    # Integrar as equações diferenciais
    states = np.zeros((len(t), len(state0)))
    states[0] = state0
    integral_error_z = 0
    
    # --- Parâmetros do controlador SDRE ---
    # Matrizes de ponderação para o controlador de atitude (ajustáveis)
    Q_att = np.diag([100, 100, 100, 1, 1, 1])  # Pondera os erros de [phi, theta, psi, p, q, r]
    R_att = np.diag([0.1, 0.1, 0.1])      # Pondera o custo do controle [u2, u3, u4]

    # Matriz de entrada B para o subsistema de atitude (constante)
    B_att = np.zeros((6, 3))
    B_att[3, 0] = 1 / params['Ixx']
    B_att[4, 1] = 1 / params['Iyy']
    B_att[5, 2] = 1 / params['Izz']
    R_inv = np.linalg.inv(R_att)

    # Vetor para armazenar o histórico de controles para plotagem
    u_history = np.zeros((len(t), 4))

    for i in range(1, len(t)):
        # --- Controle PID para u1 (Empuxo/Altitude) ---
        current_z = states[i-1, 2]
        current_vz = states[i-1, 5]
        
        # Cálculo do erro e seus termos
        error_z = 5 - current_z # Setpoint de altitude = 5m
        dt = t[i] - t[i-1]
        integral_error_z += error_z * dt
        derivative_error_z = -current_vz # Termo derivativo é -d(z)/dt = -vz
        
        # Saída do controlador PID + compensação da gravidade
        Kp_z = 2.0  # Ganho proporcional
        Ki_z = 0.5  # Ganho integral
        Kd_z = 1.5  # Ganho derivativo
        u1_pid = Kp_z * error_z + Ki_z * integral_error_z + Kd_z * derivative_error_z
        u1_val = (u1_pid + params['m'] * params['g']) / (np.cos(states[i-1, 6]) * np.cos(states[i-1, 7])) # Compensação do ângulo

        # --- Controle SDRE para u2, u3, u4 (Atitude) ---
        # Estado de atitude atual [phi, theta, psi, p, q, r]
        phi, theta, psi, p, q, r = states[i-1, 6:]
        
        # Trajetória de referência para atitude (substitui os comandos manuais)
        phi_des, theta_des, psi_des = 0.0, 0.0, 0.0
        ti = t[i]
        if 2.0 < ti <= 5.0:
            theta_des = 0.1 # rad, inclina para frente
        elif 5.0 < ti <= 8.0:
            theta_des = -0.1 # rad, gira
        
        # Vetor de erro de atitude
        e_att = np.array([phi - phi_des, theta - theta_des, psi - psi_des, p - 0, q - 0, r - 0])

        # Construção da matriz A(x) dependente do estado
        A_att = np.zeros((6, 6))
        c_phi, s_phi = np.cos(phi), np.sin(phi)
        c_theta, s_theta = np.cos(theta), np.sin(theta)
        
        # Evita divisão por zero em tan(theta) e 1/cos(theta)
        if abs(c_theta) < 1e-6:
            c_theta = 1e-6 * np.sign(c_theta)
        t_theta = s_theta / c_theta

        # Linhas da Cinemática Angular
        A_att[0, 3], A_att[0, 4], A_att[0, 5] = 1, s_phi * t_theta, c_phi * t_theta
        A_att[1, 4], A_att[1, 5] = c_phi, -s_phi
        A_att[2, 4], A_att[2, 5] = s_phi / c_theta, c_phi / c_theta

        # Linhas da Dinâmica Rotacional (uma possível parametrização SDC)
        A_att[3, 4] = ((params['Iyy'] - params['Izz']) / (2 * params['Ixx'])) * r - (params['Jr'] / (params['Ixx'])) * Omega_r
        A_att[3, 5] = ((params['Iyy'] - params['Izz']) / (2 * params['Ixx'])) * q

        A_att[4, 3] = ((params['Izz'] - params['Ixx']) / (2 * params['Iyy'])) * r + (params['Jr'] / (params['Iyy'])) * Omega_r
        A_att[4, 5] = ((params['Izz'] - params['Ixx']) / (2 * params['Iyy'])) * p
        
        A_att[5, 3] = ((params['Ixx'] - params['Iyy']) / (2 * params['Izz'])) * q
        A_att[5, 4] = ((params['Ixx'] - params['Iyy']) / (2 * params['Izz'])) * p

        # Resolver a Equação de Riccati em tempo contínuo
        P = solve_continuous_are(A_att, B_att, Q_att, R_att)

        # Calcular o ganho K(x)
        K_att = R_inv @ B_att.T @ P

        # Calcular o controle de atitude u = -K*e
        u_att = -K_att @ e_att
        u2_val, u3_val, u4_val = u_att

        # Monta o vetor de controle final e armazena
        u = [u1_val, u2_val, u3_val, u4_val]
        u_history[i] = u
        
        # Integra para o próximo passo
        t_interval = [t[i-1], t[i]]
        result = odeint(drone.model, states[i-1], t_interval, args=(u, Omega_r))
        states[i] = result[-1]
    
    # Plotar resultados
    plt.figure(figsize=(15, 12))
    
    # Posição
    plt.subplot(3, 2, 1)
    plt.plot(t, states[:, 0], label='x')
    plt.plot(t, states[:, 1], label='y')
    plt.plot(t, states[:, 2], label='z')
    plt.axhline(y=5, color='r', linestyle='--', label='Setpoint Z=5m')
    plt.title('Posição do Drone')
    plt.xlabel('Tempo (s)')
    plt.ylabel('Posição (m)')
    plt.legend()
    plt.grid(True)
    
    # Velocidade
    plt.subplot(3, 2, 2)
    plt.plot(t, states[:, 3], label='vx')
    plt.plot(t, states[:, 4], label='vy')
    plt.plot(t, states[:, 5], label='vz')
    plt.title('Velocidade do Drone')
    plt.xlabel('Tempo (s)')
    plt.ylabel('Velocidade (m/s)')
    plt.legend()
    plt.grid(True)
    
    # Ângulos de Euler
    plt.subplot(3, 2, 3)
    plt.plot(t, np.rad2deg(states[:, 6]), label='Roll (φ)')
    plt.plot(t, np.rad2deg(states[:, 7]), label='Pitch (θ)')
    plt.plot(t, np.rad2deg(states[:, 8]), label='Yaw (ψ)')
    plt.title('Ângulos de Euler')
    plt.xlabel('Tempo (s)')
    plt.ylabel('Ângulo (graus)')
    plt.legend()
    plt.grid(True)
    
    # Entradas de Controle
    plt.subplot(3, 2, 4)
    plt.plot(t, u_history[:, 0], label='u1 (Empuxo)')
    plt.plot(t, u_history[:, 1], label='u2 (Roll)')
    plt.plot(t, u_history[:, 2], label='u3 (Pitch)')
    plt.plot(t, u_history[:, 3], label='u4 (Yaw)')
    plt.title('Entradas de Controle')
    plt.xlabel('Tempo (s)')
    plt.ylabel('Controle')
    plt.legend()
    plt.grid(True)
    
    # Trajetória 3D
    ax = plt.subplot(3, 2, (5, 6), projection='3d')
    ax.plot(states[:, 0], states[:, 1], states[:, 2])
    ax.set_title('Trajetória 3D do Drone')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.grid(True)
    
    plt.tight_layout()
    plt.show()

# Executar a simulação
if __name__ == "__main__":
    simulate_drone()