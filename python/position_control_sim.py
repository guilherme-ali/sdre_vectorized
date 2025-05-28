import numpy as np
import scipy.linalg
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D

# ... (Parâmetros do Quadrotor, Simulação e Ruído como antes) ...
# Quadrotor Parameters
MASS = 1.0
GRAVITY = 9.81
L_ARM = 0.25
I_X = 0.01
I_Y = 0.01
I_Z = 0.02
I1 = (I_Y - I_Z) / I_X
I2 = (I_Z - I_X) / I_Y
I3 = (I_X - I_Y) / I_Z

# Simulation parameters
DT = 0.01
SIM_TIME = 20.0 # Aumentado para dar tempo ao controle de posição

# Noise Parameters
VELOCITY_NOISE_STD = 0.05
ANGLE_NOISE_STD = np.deg2rad(0.5)
ANGULAR_RATE_NOISE_STD = np.deg2rad(1.0)
# Para simular imperfeição na estimativa de aceleração, podemos adicionar ruído aqui
# Se não, a estimativa de posição usará acelerações "perfeitas" da dinâmica.
ACCELERATION_ESTIMATION_NOISE_STD = 0.05 # m/s^2 (opcional)


class PIDController:
    def __init__(self, kp, ki, kd, dt, output_limits=(-np.inf, np.inf), anti_windup_gain=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.integral = 0
        self.previous_error = 0
        self.output_limits = output_limits
        self.anti_windup_gain = anti_windup_gain # Para realimentar erro de saturação

    def compute(self, setpoint, current_value):
        error = setpoint - current_value
        
        # Termo Proporcional
        p_term = self.kp * error
        
        # Termo Integral (com anti-windup simples se configurado)
        # Se anti_windup_gain > 0, supomos que o erro de saturação será passado para 'reset_integral_ zbog_saturation'
        i_term_candidate = self.integral + self.ki * error * self.dt
        
        # Termo Derivativo
        d_term = self.kd * (error - self.previous_error) / self.dt
        
        output_candidate = p_term + i_term_candidate + d_term
        
        # Aplica limites de saída
        output_actual = np.clip(output_candidate, self.output_limits[0], self.output_limits[1])
        
        # Anti-windup: Ajusta o integral com base na saturação da saída
        # Se anti_windup_gain (geralmente 1/Ti ou similar) > 0
        if self.ki != 0 and self.anti_windup_gain > 0: # Evita divisão por zero se ki for zero
             saturation_error = output_actual - output_candidate # Negativo se saturou em max, Positivo se min
             self.integral = i_term_candidate + (self.anti_windup_gain / self.ki) * saturation_error
        else:
            self.integral = i_term_candidate

        # Limita o integral acumulado (outra forma de anti-windup)
        # if self.output_limits != (-np.inf, np.inf) and self.ki != 0:
        #     max_integral_contribution = max(abs(self.output_limits[0]), abs(self.output_limits[1])) / 2
        #     self.integral = np.clip(self.integral, -max_integral_contribution / self.ki, max_integral_contribution / self.ki)


        self.previous_error = error
        return output_actual


class Quadrotor:
    def __init__(self):
        # Estado Verdadeiro: X = [x, x_dot, y, y_dot, z, z_dot, phi, phi_dot, theta, theta_dot, psi, psi_dot]
        self.state = np.zeros(12)
        self.state[4] = 1.0 # Altitude inicial z = 1.0m

        # Estado Estimado (para controle de posição X, Y)
        self.est_x = 0.0
        self.est_y = 0.0
        self.est_vx = 0.0
        self.est_vy = 0.0
        # self.est_z e self.est_vz podem ser adicionados se o BMP280 for modelado para Z

        # PIDs de Velocidade
        # Ganhos podem precisar de reajuste com a malha de posição externa
        pid_kp_vxy = 2.0; pid_ki_vxy = 0.2; pid_kd_vxy = 0.15; out_lim_v = 2.0 # Saída dos PIDs de pos são vel desejadas
        pid_kp_vz  = 2.5; pid_ki_vz  = 0.3; pid_kd_vz  = 0.20

        self.pid_vx = PIDController(pid_kp_vxy, pid_ki_vxy, pid_kd_vxy, DT, output_limits=(-out_lim_v, out_lim_v))
        self.pid_vy = PIDController(pid_kp_vxy, pid_ki_vxy, pid_kd_vxy, DT, output_limits=(-out_lim_v, out_lim_v))
        self.pid_vz = PIDController(pid_kp_vz,  pid_ki_vz,  pid_kd_vz,  DT, output_limits=(-out_lim_v, out_lim_v))
        
        # Novos PIDs de Posição (X, Y) - Ganhos iniciais, precisam de ajuste!
        # A saída desses PIDs será a velocidade desejada para os PIDs de velocidade.
        # Limites de saída razoáveis para velocidade desejada (e.g., +/- 2 m/s)
        pos_kp_xy = 1.0; pos_ki_xy = 0.05; pos_kd_xy = 0.5; vel_limit_from_pos = 1.5 # m/s
        self.pid_pos_x = PIDController(pos_kp_xy, pos_ki_xy, pos_kd_xy, DT, output_limits=(-vel_limit_from_pos, vel_limit_from_pos))
        self.pid_pos_y = PIDController(pos_kp_xy, pos_ki_xy, pos_kd_xy, DT, output_limits=(-vel_limit_from_pos, vel_limit_from_pos))


        # SDRE
        self.Q_att = np.diag([10, 1, 10, 1, 10, 1]) * 50
        self.R_att = np.diag([0.1, 0.1, 0.1]) * 0.5

    # ... (get_attitude_state_from_full_state, get_translational_velocities_from_full_state como antes) ...
    def get_attitude_state_from_full_state(self, full_state):
        return full_state[6:12]

    def get_translational_velocities_from_full_state(self, full_state):
        return full_state[1], full_state[3], full_state[5]


    def velocity_controller(self, des_vx, des_vy, des_vz, current_vx_noisy, current_vy_noisy, current_vz_noisy):
        u_tilde_1 = self.pid_vx.compute(des_vx, current_vx_noisy)
        u_tilde_2 = self.pid_vy.compute(des_vy, current_vy_noisy)
        u_tilde_3 = self.pid_vz.compute(des_vz, current_vz_noisy)
        u1 = MASS * (u_tilde_3 + GRAVITY)
        u1 = np.clip(u1, 0, MASS * GRAVITY * 3)
        denominator = u_tilde_3 + GRAVITY
        if np.abs(denominator) < 1e-6: phi_d, theta_d = 0.0, 0.0
        else:
            phi_d = -u_tilde_2 / denominator
            theta_d = u_tilde_1 / denominator
        psi_d = 0.0
        angle_limit = np.deg2rad(35)
        phi_d, theta_d = np.clip(phi_d, -angle_limit, angle_limit), np.clip(theta_d, -angle_limit, angle_limit)
        x_I_d = np.array([phi_d, 0, theta_d, 0, psi_d, 0])
        return u1, x_I_d

    def sdre_attitude_controller(self, x_I_current_noisy, x_I_desired):
        phi_dot_noisy, theta_dot_noisy, psi_dot_noisy = x_I_current_noisy[1], x_I_current_noisy[3], x_I_current_noisy[5]
        A_xI = np.zeros((6, 6)); A_xI[0,1],A_xI[2,3],A_xI[4,5]=1,1,1
        A_xI[1,3]=psi_dot_noisy*I1; A_xI[3,1]=psi_dot_noisy*I2; A_xI[5,1]=theta_dot_noisy*I3
        B_att = np.zeros((6,3)); B_att[1,0],B_att[3,1],B_att[5,2]=1/I_X,1/I_Y,1/I_Z
        try: P = scipy.linalg.solve_continuous_are(A_xI,B_att,self.Q_att,self.R_att)
        except (np.linalg.LinAlgError, ValueError): return np.zeros(3)
        K_sdre = np.linalg.inv(self.R_att)@B_att.T@P
        u_torques = -K_sdre@(x_I_current_noisy-x_I_desired)
        max_trq_rp,max_trq_y=0.5,0.2
        u_torques[0]=np.clip(u_torques[0],-max_trq_rp,max_trq_rp)
        u_torques[1]=np.clip(u_torques[1],-max_trq_rp,max_trq_rp)
        u_torques[2]=np.clip(u_torques[2],-max_trq_y,max_trq_y)
        return u_torques

    def update_dynamics_and_estimates(self, u1, u2, u3, u4, dt):
        s = self.state
        phi,phi_dot,theta,theta_dot,psi,psi_dot = s[6],s[7],s[8],s[9],s[10],s[11]
        cphi,sphi,cth,sth,cpsi,spsi = np.cos(phi),np.sin(phi),np.cos(theta),np.sin(theta),np.cos(psi),np.sin(psi)
        
        # Acelerações inerciais verdadeiras (calculadas pela dinâmica)
        x_ddot_true = (cphi*sth*cpsi + sphi*spsi)*u1/MASS
        y_ddot_true = (cphi*sth*spsi - sphi*cpsi)*u1/MASS
        z_ddot_true = -GRAVITY + (cphi*cth)*u1/MASS
        
        phi_ddot=theta_dot*psi_dot*I1+u2/I_X
        theta_ddot=phi_dot*psi_dot*I2+u3/I_Y
        psi_ddot=phi_dot*theta_dot*I3+u4/I_Z
        
        s_dot=np.zeros(12)
        s_dot[0],s_dot[1]=s[1],x_ddot_true
        s_dot[2],s_dot[3]=s[3],y_ddot_true
        s_dot[4],s_dot[5]=s[5],z_ddot_true
        s_dot[6],s_dot[7]=s[7],phi_ddot
        s_dot[8],s_dot[9]=s[9],theta_ddot
        s_dot[10],s_dot[11]=s[11],psi_ddot
        
        self.state += s_dot * dt
        self.state[10]=(self.state[10]+np.pi)%(2*np.pi)-np.pi

        # --- Atualiza estimativas de posição e velocidade X, Y ---
        # Usando as acelerações verdadeiras calculadas (x_ddot_true, y_ddot_true)
        # Para simular um acelerômetro ruidoso, adicione ruído aqui:
        # x_ddot_for_estimation = x_ddot_true + np.random.normal(0, ACCELERATION_ESTIMATION_NOISE_STD)
        # y_ddot_for_estimation = y_ddot_true + np.random.normal(0, ACCELERATION_ESTIMATION_NOISE_STD)
        x_ddot_for_estimation = x_ddot_true # Sem ruído adicional na aceleração para estimativa (melhor caso)
        y_ddot_for_estimation = y_ddot_true

        self.est_vx += x_ddot_for_estimation * dt
        self.est_vy += y_ddot_for_estimation * dt
        self.est_x += self.est_vx * dt
        self.est_y += self.est_vy * dt


def get_rotation_matrix(phi, theta, psi):
    cphi,sphi,cth,sth,cpsi,spsi=np.cos(phi),np.sin(phi),np.cos(theta),np.sin(theta),np.cos(psi),np.sin(psi)
    R=np.array([[cpsi*cth,cpsi*sth*sphi-spsi*cphi,cpsi*sth*cphi+spsi*sphi],
                  [spsi*cth,spsi*sth*sphi+cpsi*cphi,spsi*sth*cphi-cpsi*sphi],
                  [-sth,cth*sphi,cth*cphi]])
    return R

# --- Simulação ---
quad = Quadrotor()
# Condições iniciais podem ser mantidas ou ajustadas
# quad.state[6]=np.deg2rad(2); quad.state[8]=np.deg2rad(-2); quad.state[7]=np.deg2rad(1)

# Alvos de Posição X, Y (exemplo: mover para x=0.5, y=0.3 e depois parar)
target_x_profile = [(0, 0.0), (5, 0.5), (10, 0.5), (15, 0.0)] # (tempo, valor_x)
target_y_profile = [(0, 0.0), (5, 0.3), (10, 0.3), (15, 0.0)] # (tempo, valor_y)

# Função para obter o alvo de posição atual com base no tempo
def get_target_position(time, profile):
    # Interpolação linear simples ou step
    current_target = profile[0][1] # Default to first value
    for i in range(len(profile) - 1):
        if profile[i][0] <= time < profile[i+1][0]:
            # Interpolação linear (opcional, pode usar step)
            # t0, v0 = profile[i]
            # t1, v1 = profile[i+1]
            # current_target = v0 + (v1 - v0) * (time - t0) / (t1 - t0)
            current_target = profile[i][1] # Step change no início do intervalo
            break
    if time >= profile[-1][0]:
        current_target = profile[-1][1]
    return current_target


time_log, state_log, desired_att_log, u_torques_log, u1_log = [],[],[],[],[]
state_noisy_log, est_pos_vel_log, desired_vel_log = [], [], []

num_steps = int(SIM_TIME / DT)
animation_speed_factor = 5
animation_frames_step = max(1, int(animation_speed_factor / (DT * 1000) * 1000 * DT))


print(f"Rodando simulação por {SIM_TIME}s com DT={DT}s...")
for i in range(num_steps):
    current_time = i * DT
    time_log.append(current_time)

    # --- Alvos de Posição X, Y e Velocidade Z ---
    target_x = get_target_position(current_time, target_x_profile)
    target_y = get_target_position(current_time, target_y_profile)
    
    # Controle de Posição X, Y -> Saída é Vx_des, Vy_des
    current_desired_vx_from_pos = quad.pid_pos_x.compute(target_x, quad.est_x)
    current_desired_vy_from_pos = quad.pid_pos_y.compute(target_y, quad.est_y)

    # Velocidades lineares desejadas para a malha de velocidade
    # Vx e Vy vêm dos PIDs de posição. Vz é para hover (ou pulsos de teste, se reativados).
    current_desired_vx = current_desired_vx_from_pos
    current_desired_vy = current_desired_vy_from_pos
    
    current_desired_vz = 0.0 # Manter Vz = 0
    # Para os pulsos de teste de Vz como antes:
    # if 3.0 <= current_time <= 5.0:
    #     current_desired_vz = 1.0
    
    desired_vel_log.append([current_desired_vx, current_desired_vy, current_desired_vz, target_x, target_y])

    # --- Simulação de Sensores Ruidosos ---
    current_true_state = quad.state.copy()
    vx_true, vy_true, vz_true = quad.get_translational_velocities_from_full_state(current_true_state)
    vx_noisy = vx_true + np.random.normal(0, VELOCITY_NOISE_STD)
    vy_noisy = vy_true + np.random.normal(0, VELOCITY_NOISE_STD)
    vz_noisy = vz_true + np.random.normal(0, VELOCITY_NOISE_STD)

    x_I_true = quad.get_attitude_state_from_full_state(current_true_state)
    x_I_noisy = x_I_true.copy()
    x_I_noisy[0::2] += np.random.normal(0, ANGLE_NOISE_STD, 3)
    x_I_noisy[1::2] += np.random.normal(0, ANGULAR_RATE_NOISE_STD, 3)
    
    noisy_state_for_log = current_true_state.copy()
    noisy_state_for_log[1], noisy_state_for_log[3], noisy_state_for_log[5] = vx_noisy,vy_noisy,vz_noisy
    noisy_state_for_log[6:12] = x_I_noisy
    state_noisy_log.append(noisy_state_for_log)

    # --- Controladores ---
    u1, x_I_d = quad.velocity_controller(current_desired_vx, current_desired_vy, current_desired_vz,
                                         vx_noisy, vy_noisy, vz_noisy)
    desired_att_log.append(x_I_d); u1_log.append(u1)
    
    u_torques = quad.sdre_attitude_controller(x_I_noisy, x_I_d)
    u2,u3,u4 = u_torques[0],u_torques[1],u_torques[2]; u_torques_log.append(u_torques)

    # --- Dinâmica e Estimativas ---
    quad.update_dynamics_and_estimates(u1, u2, u3, u4, DT) # Método modificado
    state_log.append(quad.state.copy())
    est_pos_vel_log.append([quad.est_x, quad.est_y, quad.est_vx, quad.est_vy])


    if i % (num_steps // 20) == 0 and i > 0:
        print(f"Sim: {100*i/num_steps:.0f}%, T:{current_time:.1f}, Xest:{quad.est_x:.2f}, Yest:{quad.est_y:.2f}, X:{quad.state[0]:.2f}, Y:{quad.state[2]:.2f}")

print("Simulação completa.")
# ... (Conversões para np.array)
state_log = np.array(state_log); state_noisy_log = np.array(state_noisy_log)
desired_att_log = np.array(desired_att_log); u_torques_log = np.array(u_torques_log)
u1_log = np.array(u1_log); est_pos_vel_log = np.array(est_pos_vel_log)
desired_vel_log = np.array(desired_vel_log)


# --- Plots 2D Estáticos ---
print("Gerando plots 2D estáticos...")
fig_2d, axs_2d = plt.subplots(5, 2, figsize=(15, 16)) # Aumentado para 5 linhas
fig_2d.canvas.manager.set_window_title('Resultados da Simulação com Controle de Posição XY')

# Posição X, Y, Z (Real)
axs_2d[0, 0].plot(time_log, state_log[:, 0], label='X Real (m)')
axs_2d[0, 0].plot(time_log, est_pos_vel_log[:, 0], '--', label='X Estimado (m)')
axs_2d[0, 0].plot(time_log, desired_vel_log[:, 3], ':', color='red', label='X Alvo (m)')
axs_2d[0, 0].plot(time_log, state_log[:, 2], label='Y Real (m)')
axs_2d[0, 0].plot(time_log, est_pos_vel_log[:, 1], '--', label='Y Estimado (m)')
axs_2d[0, 0].plot(time_log, desired_vel_log[:, 4], ':', color='blue', label='Y Alvo (m)')
axs_2d[0, 0].plot(time_log, state_log[:, 4], label='Z Real (m)')
axs_2d[0, 0].set_title('Posição XYZ (Real vs. Estimado vs. Alvo)')
axs_2d[0, 0].set_xlabel('Tempo (s)'), axs_2d[0, 0].set_ylabel('Posição (m)'), axs_2d[0, 0].legend(), axs_2d[0, 0].grid(True)

# Velocidades Lineares (Real vs. Desejada pelos PIDs de Posição)
axs_2d[0, 1].plot(time_log, state_log[:, 1], label='Vx Real (m/s)')
axs_2d[0, 1].plot(time_log, desired_vel_log[:, 0], '--', label='Vx Desejada (m/s)')
axs_2d[0, 1].plot(time_log, state_log[:, 3], label='Vy Real (m/s)')
axs_2d[0, 1].plot(time_log, desired_vel_log[:, 1], '--', label='Vy Desejada (m/s)')
axs_2d[0, 1].plot(time_log, state_log[:, 5], label='Vz Real (m/s)')
axs_2d[0, 1].plot(time_log, desired_vel_log[:, 2], '--', label='Vz Desejada (m/s)')
axs_2d[0, 1].set_title('Velocidades Lineares (Real vs. Desejada pela Malha de Posição)')
axs_2d[0, 1].set_xlabel('Tempo (s)'), axs_2d[0, 1].set_ylabel('Velocidade (m/s)'), axs_2d[0, 1].legend(), axs_2d[0, 1].grid(True)

# ... (Plots de Ângulos, Taxas Angulares, Torques, Empuxo como antes, usando axs_2d[1,0] até axs_2d[3,1]) ...
# Plot 1,0: Ângulos de Euler
axs_2d[1, 0].plot(time_log, np.rad2deg(state_log[:, 6]), label='Roll (real)')
axs_2d[1, 0].plot(time_log, np.rad2deg(state_log[:, 8]), label='Pitch (real)')
axs_2d[1, 0].plot(time_log, np.rad2deg(state_log[:, 10]), label='Yaw (real)')
axs_2d[1, 0].plot(time_log, np.rad2deg(desired_att_log[:, 0]), '--', label='Roll Desejado')
axs_2d[1, 0].plot(time_log, np.rad2deg(desired_att_log[:, 2]), '--', label='Pitch Desejado')
axs_2d[1, 0].set_title('Ângulos de Euler')
axs_2d[1, 0].set_xlabel('Tempo (s)'), axs_2d[1, 0].set_ylabel('Ângulo (graus)'), axs_2d[1, 0].legend(), axs_2d[1, 0].grid(True)

# Plot 1,1: Velocidades Angulares
axs_2d[1, 1].plot(time_log, np.rad2deg(state_log[:, 7]), label='Roll rate (real)')
axs_2d[1, 1].plot(time_log, np.rad2deg(state_log[:, 9]), label='Pitch rate (real)')
axs_2d[1, 1].plot(time_log, np.rad2deg(state_log[:, 11]), label='Yaw rate (real)')
axs_2d[1, 1].set_title('Velocidades Angulares')
axs_2d[1, 1].set_xlabel('Tempo (s)'), axs_2d[1, 1].set_ylabel('Velocidade Angular (graus/s)'), axs_2d[1, 1].legend(), axs_2d[1, 1].grid(True)

# Plot 2,0: Torques de Controle
axs_2d[2, 0].plot(time_log, u_torques_log[:, 0], label='u2 (Torque Roll)')
axs_2d[2, 0].plot(time_log, u_torques_log[:, 1], label='u3 (Torque Pitch)')
axs_2d[2, 0].plot(time_log, u_torques_log[:, 2], label='u4 (Torque Yaw)')
axs_2d[2, 0].set_title('Torques de Controle (u2, u3, u4)')
axs_2d[2, 0].set_xlabel('Tempo (s)'), axs_2d[2, 0].set_ylabel('Torque (Nm)'), axs_2d[2, 0].legend(), axs_2d[2, 0].grid(True)

# Plot 2,1: Empuxo Total
axs_2d[2, 1].plot(time_log, u1_log, label='u1 (Empuxo Total)')
axs_2d[2, 1].axhline(MASS * GRAVITY, color='r', linestyle='--', label='Empuxo Hover Teórico (mg)')
axs_2d[2, 1].set_title('Empuxo Total (u1)')
axs_2d[2, 1].set_xlabel('Tempo (s)'), axs_2d[2, 1].set_ylabel('Força (N)'), axs_2d[2, 1].legend(), axs_2d[2, 1].grid(True)

# Plot 3,0: Velocidades X Estimadas vs Reais
axs_2d[3, 0].plot(time_log, state_log[:, 1], label='Vx Real (m/s)')
axs_2d[3, 0].plot(time_log, est_pos_vel_log[:, 2], '--', label='Vx Estimada (m/s)')
axs_2d[3, 0].plot(time_log, state_noisy_log[:, 1], alpha=0.5, linestyle=':', label='Vx "Sensor" (m/s)')
axs_2d[3, 0].set_title('Velocidade X (Real vs. Estimada vs. Sensor)')
axs_2d[3, 0].set_xlabel('Tempo (s)'), axs_2d[3, 0].set_ylabel('Vx (m/s)'), axs_2d[3, 0].legend(), axs_2d[3, 0].grid(True)

# Plot 3,1: Velocidades Y Estimadas vs Reais
axs_2d[3, 1].plot(time_log, state_log[:, 3], label='Vy Real (m/s)')
axs_2d[3, 1].plot(time_log, est_pos_vel_log[:, 3], '--', label='Vy Estimada (m/s)')
axs_2d[3, 1].plot(time_log, state_noisy_log[:, 3], alpha=0.5, linestyle=':', label='Vy "Sensor" (m/s)')
axs_2d[3, 1].set_title('Velocidade Y (Real vs. Estimada vs. Sensor)')
axs_2d[3, 1].set_xlabel('Tempo (s)'), axs_2d[3, 1].set_ylabel('Vy (m/s)'), axs_2d[3, 1].legend(), axs_2d[3, 1].grid(True)

# Plot 4,0: Posição X (Real vs. Estimada vs. Alvo)
axs_2d[4, 0].plot(time_log, state_log[:, 0], label='X Real (m)')
axs_2d[4, 0].plot(time_log, est_pos_vel_log[:, 0], '--', label='X Estimado (m)')
axs_2d[4, 0].plot(time_log, desired_vel_log[:, 3], ':', label='X Alvo (m)')
axs_2d[4, 0].set_title('Posição X (Real vs. Estimado vs. Alvo)')
axs_2d[4, 0].set_xlabel('Tempo (s)'), axs_2d[4, 0].set_ylabel('X (m)'), axs_2d[4, 0].legend(), axs_2d[4, 0].grid(True)

# Plot 4,1: Posição Y (Real vs. Estimada vs. Alvo)
axs_2d[4, 1].plot(time_log, state_log[:, 2], label='Y Real (m)')
axs_2d[4, 1].plot(time_log, est_pos_vel_log[:, 1], '--', label='Y Estimado (m)')
axs_2d[4, 1].plot(time_log, desired_vel_log[:, 4], ':', label='Y Alvo (m)')
axs_2d[4, 1].set_title('Posição Y (Real vs. Estimado vs. Alvo)')
axs_2d[4, 1].set_xlabel('Tempo (s)'), axs_2d[4, 1].set_ylabel('Y (m)'), axs_2d[4, 1].legend(), axs_2d[4, 1].grid(True)


fig_2d.tight_layout()
plt.show(block=False)
plt.savefig('python/outputs/position_control_sim_results_2d_static.png', dpi=300)

# --- Animação 3D (como antes) ---
# ... (código da animação 3D permanece o mesmo da resposta anterior) ...
print("Preparando animação 3D com atitude ajustada...")
fig_anim = plt.figure(figsize=(10, 8))
fig_anim.canvas.manager.set_window_title('Animação da Trajetória 3D do Drone com Atitude (Ajustada)')
ax_anim = fig_anim.add_subplot(111, projection='3d')
x_min, x_max = np.min(state_log[:, 0]), np.max(state_log[:, 0])
y_min, y_max = np.min(state_log[:, 2]), np.max(state_log[:, 2])
z_min, z_max = np.min(state_log[:, 4]), np.max(state_log[:, 4])
# Adiciona também as posições estimadas para garantir que os limites da animação as incluam
if est_pos_vel_log.shape[0] > 0:
    x_min = min(x_min, np.min(est_pos_vel_log[:,0])); x_max = max(x_max, np.max(est_pos_vel_log[:,0]))
    y_min = min(y_min, np.min(est_pos_vel_log[:,1])); y_max = max(y_max, np.max(est_pos_vel_log[:,1]))

margin_factor = 0.5 
margin_x = max((x_max - x_min) * margin_factor, 0.5) 
margin_y = max((y_max - y_min) * margin_factor, 0.5)
margin_z = max((z_max - z_min) * margin_factor, 0.5)
ax_anim.set_xlim([x_min - margin_x, x_max + margin_x])
ax_anim.set_ylim([y_min - margin_y, y_max + margin_y])
ax_anim.set_zlim([max(0, z_min - margin_z), z_max + margin_z])
ax_anim.set_xlabel('X (m)'), ax_anim.set_ylabel('Y (m)'), ax_anim.set_zlabel('Z (m)')
ax_anim.set_title('Animação da Trajetória 3D com Atitude')
ax_anim.grid(True)
line_traj, = ax_anim.plot([], [], [], lw=2, label='Trajetória Real')
# Linha para trajetória estimada (opcional)
# line_traj_est, = ax_anim.plot([], [], [], lw=2, linestyle='--', color='gray', label='Trajetória Estimada XY')
drone_marker, = ax_anim.plot([], [], [], marker='o', markersize=6, color='black')
start_point = ax_anim.scatter(state_log[0, 0], state_log[0, 2], state_log[0, 4], c='g', marker='D', s=60, label='Início')
body_axis_length = 0.3
half_axis_len = body_axis_length / 1.0
body_x_line, = ax_anim.plot([], [], [], color='red', lw=3) 
body_y_line, = ax_anim.plot([], [], [], color='green', lw=3)
body_z_line, = ax_anim.plot([], [], [], color='blue', lw=3)
target_marker, = ax_anim.plot([], [], [], marker='x', markersize=10, color='magenta', label='Alvo XY') # Marcador para o alvo XY
ax_anim.legend(loc='best')

def init_animation():
    line_traj.set_data_3d([], [], [])
    # line_traj_est.set_data_3d([], [], [])
    drone_marker.set_data_3d([], [], [])
    body_x_line.set_data_3d([], [], [])
    body_y_line.set_data_3d([], [], [])
    body_z_line.set_data_3d([], [], [])
    target_marker.set_data_3d([], [], [])
    return line_traj, drone_marker, body_x_line, body_y_line, body_z_line, target_marker #, line_traj_est

num_animation_frames = len(time_log) // animation_frames_step

def update_animation(frame_idx):
    sim_idx = min(frame_idx * animation_frames_step, len(time_log) - 1)
    
    # Trajetória Real
    line_traj.set_data_3d(state_log[:sim_idx+1, 0], state_log[:sim_idx+1, 2], state_log[:sim_idx+1, 4])
    # Trajetória Estimada XY (plotada na altitude Z real do drone para visualização)
    # line_traj_est.set_data_3d(est_pos_vel_log[:sim_idx+1, 0], est_pos_vel_log[:sim_idx+1, 1], state_log[:sim_idx+1, 4])

    current_pos = state_log[sim_idx, 0:6:2]
    drone_marker.set_data_3d([current_pos[0]], [current_pos[1]], [current_pos[2]])
    
    # Marcador do Alvo XY (na altitude Z=0 ou Z do drone para melhor visualização)
    current_target_x = desired_vel_log[sim_idx, 3]
    current_target_y = desired_vel_log[sim_idx, 4]
    target_marker.set_data_3d([current_target_x], [current_target_y], [state_log[sim_idx, 4]]) # Plota na altitude Z atual do drone

    phi, theta, psi = state_log[sim_idx, 6], state_log[sim_idx, 8], state_log[sim_idx, 10]
    R_body_to_inertial = get_rotation_matrix(phi, theta, psi)
    body_x_axis_vec_body = np.array([1,0,0]); body_y_axis_vec_body = np.array([0,1,0]); body_z_axis_vec_body = np.array([0,0,1])
    tip_x =R_body_to_inertial@(body_x_axis_vec_body*half_axis_len)
    tip_y =R_body_to_inertial@(body_y_axis_vec_body*half_axis_len)
    tip_z =R_body_to_inertial@(body_z_axis_vec_body*half_axis_len)
    body_x_line.set_data_3d([current_pos[0]-tip_x[0],current_pos[0]+tip_x[0]],[current_pos[1]-tip_x[1],current_pos[1]+tip_x[1]],[current_pos[2]-tip_x[2],current_pos[2]+tip_x[2]])
    body_y_line.set_data_3d([current_pos[0]-tip_y[0],current_pos[0]+tip_y[0]],[current_pos[1]-tip_y[1],current_pos[1]+tip_y[1]],[current_pos[2]-tip_y[2],current_pos[2]+tip_y[2]])
    body_z_line.set_data_3d([current_pos[0]-tip_z[0],current_pos[0]+tip_z[0]],[current_pos[1]-tip_z[1],current_pos[1]+tip_z[1]],[current_pos[2]-tip_z[2],current_pos[2]+tip_z[2]])
    ax_anim.set_title(f'Animação 3D (T:{time_log[sim_idx]:.1f}s) Alvo:[{current_target_x:.1f},{current_target_y:.1f}] Est:[{quad.est_x:.1f},{quad.est_y:.1f}]')
    return line_traj, drone_marker, body_x_line, body_y_line, body_z_line, target_marker #, line_traj_est

anim_interval_ms = 40
ani = animation.FuncAnimation(fig_anim, update_animation, frames=num_animation_frames,
                              init_func=init_animation, blit=True, interval=anim_interval_ms, repeat=False)
plt.show()

print("Salvando animação 3D...")
ani.save('python/outputs/position_control_sim_animation_3d.mp4', writer='ffmpeg', fps=20, dpi=300)
print("Fim do script.")