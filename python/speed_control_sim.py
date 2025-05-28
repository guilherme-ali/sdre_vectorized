import numpy as np
import scipy.linalg
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D

# ... (Mantém todo o código das classes PIDController e Quadrotor como antes) ...
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
SIM_TIME = 15.0 # Tempo suficiente para observar os pulsos de velocidade

# Noise Parameters
VELOCITY_NOISE_STD = 0.05
ANGLE_NOISE_STD = np.deg2rad(0.5)
ANGULAR_RATE_NOISE_STD = np.deg2rad(1.0)


class PIDController:
    def __init__(self, kp, ki, kd, dt, output_limits=(-np.inf, np.inf)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.integral = 0
        self.previous_error = 0
        self.output_limits = output_limits

    def compute(self, setpoint, current_value):
        error = setpoint - current_value
        self.integral += error * self.dt
        derivative = (error - self.previous_error) / self.dt
        self.previous_error = error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        return np.clip(output, self.output_limits[0], self.output_limits[1])

class Quadrotor:
    def __init__(self):
        self.state = np.zeros(12)
        self.state[4] = 1.0

        pid_kp_xy = 2.0
        pid_ki_xy = 0.001
        pid_kd_xy = 0.1
        pid_kp_z = 2.0
        pid_ki_z = 0.001
        pid_kd_z = 0.1

        self.pid_vx = PIDController(pid_kp_xy, pid_ki_xy, pid_kd_xy, DT)
        self.pid_vy = PIDController(pid_kp_xy, pid_ki_xy, pid_kd_xy, DT)
        self.pid_vz = PIDController(pid_kp_z, pid_ki_z, pid_kd_z, DT)

        self.Q_att = np.diag([10, 1, 10, 1, 10, 1]) * 50
        self.R_att = np.diag([0.1, 0.1, 0.1]) * 0.5

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
        if np.abs(denominator) < 1e-6:
            phi_d = 0.0
            theta_d = 0.0
        else:
            phi_d = -u_tilde_2 / denominator
            theta_d = u_tilde_1 / denominator
        psi_d = 0.0
        angle_limit = np.deg2rad(35)
        phi_d = np.clip(phi_d, -angle_limit, angle_limit)
        theta_d = np.clip(theta_d, -angle_limit, angle_limit)
        x_I_d = np.array([phi_d, 0, theta_d, 0, psi_d, 0])
        return u1, x_I_d

    def sdre_attitude_controller(self, x_I_current_noisy, x_I_desired):
        phi_dot_noisy, theta_dot_noisy, psi_dot_noisy = x_I_current_noisy[1], x_I_current_noisy[3], x_I_current_noisy[5]
        A_xI = np.zeros((6, 6))
        A_xI[0, 1], A_xI[2, 3], A_xI[4, 5] = 1, 1, 1
        A_xI[1, 3] = psi_dot_noisy * I1
        A_xI[3, 1] = psi_dot_noisy * I2
        A_xI[5, 1] = theta_dot_noisy * I3
        B_att = np.zeros((6, 3))
        B_att[1, 0], B_att[3, 1], B_att[5, 2] = 1 / I_X, 1 / I_Y, 1 / I_Z
        try:
            P = scipy.linalg.solve_continuous_are(A_xI, B_att, self.Q_att, self.R_att)
        except (np.linalg.LinAlgError, ValueError):
            return np.zeros(3)
        K_sdre = np.linalg.inv(self.R_att) @ B_att.T @ P
        error_att = x_I_current_noisy - x_I_desired
        u_torques = -K_sdre @ error_att
        max_torque_roll_pitch, max_torque_yaw = 0.5, 0.2
        u_torques[0] = np.clip(u_torques[0], -max_torque_roll_pitch, max_torque_roll_pitch)
        u_torques[1] = np.clip(u_torques[1], -max_torque_roll_pitch, max_torque_roll_pitch)
        u_torques[2] = np.clip(u_torques[2], -max_torque_yaw, max_torque_yaw)
        return u_torques

    def update_dynamics(self, u1, u2, u3, u4, dt):
        s = self.state
        phi, phi_dot, theta, theta_dot, psi, psi_dot = s[6], s[7], s[8], s[9], s[10], s[11]
        cphi, sphi, cth, sth, cpsi, spsi = np.cos(phi), np.sin(phi), np.cos(theta), np.sin(theta), np.cos(psi), np.sin(psi)
        x_ddot = (cphi * sth * cpsi + sphi * spsi) * u1 / MASS
        y_ddot = (cphi * sth * spsi - sphi * cpsi) * u1 / MASS
        z_ddot = -GRAVITY + (cphi * cth) * u1 / MASS
        phi_ddot = theta_dot * psi_dot * I1 + u2 / I_X
        theta_ddot = phi_dot * psi_dot * I2 + u3 / I_Y
        psi_ddot = phi_dot * theta_dot * I3 + u4 / I_Z
        s_dot = np.zeros(12)
        s_dot[0], s_dot[1] = s[1], x_ddot
        s_dot[2], s_dot[3] = s[3], y_ddot
        s_dot[4], s_dot[5] = s[5], z_ddot
        s_dot[6], s_dot[7] = s[7], phi_ddot
        s_dot[8], s_dot[9] = s[9], theta_ddot
        s_dot[10], s_dot[11] = s[11], psi_ddot
        self.state += s_dot * dt
        self.state[10] = (self.state[10] + np.pi) % (2 * np.pi) - np.pi

def get_rotation_matrix(phi, theta, psi):
    cphi, sphi, cth, sth, cpsi, spsi = np.cos(phi), np.sin(phi), np.cos(theta), np.sin(theta), np.cos(psi), np.sin(psi)
    R = np.array([
        [cpsi*cth, cpsi*sth*sphi - spsi*cphi, cpsi*sth*cphi + spsi*sphi],
        [spsi*cth, spsi*sth*sphi + cpsi*cphi, spsi*sth*cphi - cpsi*sphi],
        [-sth,     cth*sphi,                  cth*cphi]
    ])
    return R

# --- Simulação ---
quad = Quadrotor()
quad.state[6] = np.deg2rad(2)
quad.state[8] = np.deg2rad(-2)
quad.state[7] = np.deg2rad(1)

time_log, state_log, desired_att_log, u_torques_log, u1_log = [], [], [], [], []
# Log para as velocidades desejadas para plotagem
desired_velocities_log = []
state_noisy_log = []

num_steps = int(SIM_TIME / DT)
animation_speed_factor = 5
animation_frames_step = max(1, int(animation_speed_factor / (DT * 1000) * 1000 * DT))

print(f"Rodando simulação por {SIM_TIME}s com DT={DT}s...")
for i in range(num_steps):
    current_time = i * DT
    time_log.append(current_time)

    # --- Definição das velocidades desejadas com base no tempo ---
    current_desired_vx = 0.0
    if 1.0 <= current_time <= 2.0:
        current_desired_vx = 1.0

    current_desired_vy = 0.0
    if 6.0 <= current_time <= 7.0:
        current_desired_vy = 1.0

    current_desired_vz = 0.0
    if 3.0 <= current_time <= 5.0:
        current_desired_vz = 1.0

    desired_velocities_log.append([current_desired_vx, current_desired_vy, current_desired_vz])
    # --- Fim da definição das velocidades desejadas ---

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
    noisy_state_for_log[1], noisy_state_for_log[3], noisy_state_for_log[5] = vx_noisy, vy_noisy, vz_noisy
    noisy_state_for_log[6:12] = x_I_noisy
    state_noisy_log.append(noisy_state_for_log)

    u1, x_I_d = quad.velocity_controller(current_desired_vx, current_desired_vy, current_desired_vz,
                                         vx_noisy, vy_noisy, vz_noisy)
    desired_att_log.append(x_I_d)
    u1_log.append(u1)
    u_torques = quad.sdre_attitude_controller(x_I_noisy, x_I_d)
    u2, u3, u4 = u_torques[0], u_torques[1], u_torques[2]
    u_torques_log.append(u_torques)
    quad.update_dynamics(u1, u2, u3, u4, DT)
    state_log.append(quad.state.copy())
    if i % (num_steps // 20) == 0 and i > 0:
        print(f"Simulação: {100 * i / num_steps:.0f}% completa. Tempo: {current_time:.2f}s")

print("Simulação completa.")
state_log = np.array(state_log)
state_noisy_log = np.array(state_noisy_log)
desired_att_log = np.array(desired_att_log)
u_torques_log = np.array(u_torques_log)
u1_log = np.array(u1_log)
desired_velocities_log = np.array(desired_velocities_log)


# --- Plots 2D Estáticos ---
print("Gerando plots 2D estáticos...")
fig_2d, axs_2d = plt.subplots(4, 2, figsize=(15, 12))
fig_2d.canvas.manager.set_window_title('Resultados da Simulação 2D (Estático)')

axs_2d[0, 0].plot(time_log, state_log[:, 0], label='x (m)')
axs_2d[0, 0].plot(time_log, state_log[:, 2], label='y (m)')
axs_2d[0, 0].plot(time_log, state_log[:, 4], label='z (m)')
axs_2d[0, 0].set_title('Posição (Estado Real)')
axs_2d[0, 0].set_xlabel('Tempo (s)'), axs_2d[0, 0].set_ylabel('Posição (m)'), axs_2d[0, 0].legend(), axs_2d[0, 0].grid(True)

# Modificado para mostrar Vx, Vy, Vz desejadas e reais
axs_2d[0, 1].plot(time_log, state_log[:, 1], label='Vx Real (m/s)')
axs_2d[0, 1].plot(time_log, desired_velocities_log[:, 0], '--', label='Vx Desejada (m/s)')
axs_2d[0, 1].plot(time_log, state_log[:, 3], label='Vy Real (m/s)')
axs_2d[0, 1].plot(time_log, desired_velocities_log[:, 1], '--', label='Vy Desejada (m/s)')
axs_2d[0, 1].plot(time_log, state_log[:, 5], label='Vz Real (m/s)')
axs_2d[0, 1].plot(time_log, desired_velocities_log[:, 2], '--', label='Vz Desejada (m/s)')
axs_2d[0, 1].set_title('Velocidades Lineares (Real vs. Desejada)')
axs_2d[0, 1].set_xlabel('Tempo (s)'), axs_2d[0, 1].set_ylabel('Velocidade (m/s)'), axs_2d[0, 1].legend(), axs_2d[0, 1].grid(True)


axs_2d[1, 0].plot(time_log, np.rad2deg(state_log[:, 6]), label='Roll (real)')
axs_2d[1, 0].plot(time_log, np.rad2deg(state_log[:, 8]), label='Pitch (real)')
axs_2d[1, 0].plot(time_log, np.rad2deg(state_log[:, 10]), label='Yaw (real)')
axs_2d[1, 0].plot(time_log, np.rad2deg(desired_att_log[:, 0]), '--', label='Roll Desejado')
axs_2d[1, 0].plot(time_log, np.rad2deg(desired_att_log[:, 2]), '--', label='Pitch Desejado')
axs_2d[1, 0].set_title('Ângulos de Euler')
axs_2d[1, 0].set_xlabel('Tempo (s)'), axs_2d[1, 0].set_ylabel('Ângulo (graus)'), axs_2d[1, 0].legend(), axs_2d[1, 0].grid(True)

axs_2d[1, 1].plot(time_log, np.rad2deg(state_log[:, 7]), label='Roll rate (real)')
axs_2d[1, 1].plot(time_log, np.rad2deg(state_log[:, 9]), label='Pitch rate (real)')
axs_2d[1, 1].plot(time_log, np.rad2deg(state_log[:, 11]), label='Yaw rate (real)')
axs_2d[1, 1].set_title('Velocidades Angulares')
axs_2d[1, 1].set_xlabel('Tempo (s)'), axs_2d[1, 1].set_ylabel('Velocidade Angular (graus/s)'), axs_2d[1, 1].legend(), axs_2d[1, 1].grid(True)

axs_2d[2, 0].plot(time_log, u_torques_log[:, 0], label='u2 (Torque Roll)')
axs_2d[2, 0].plot(time_log, u_torques_log[:, 1], label='u3 (Torque Pitch)')
axs_2d[2, 0].plot(time_log, u_torques_log[:, 2], label='u4 (Torque Yaw)')
axs_2d[2, 0].set_title('Torques de Controle (u2, u3, u4)')
axs_2d[2, 0].set_xlabel('Tempo (s)'), axs_2d[2, 0].set_ylabel('Torque (Nm)'), axs_2d[2, 0].legend(), axs_2d[2, 0].grid(True)

axs_2d[2, 1].plot(time_log, u1_log, label='u1 (Empuxo Total)')
axs_2d[2, 1].axhline(MASS * GRAVITY, color='r', linestyle='--', label='Empuxo Hover Teórico (mg)')
axs_2d[2, 1].set_title('Empuxo Total (u1)')
axs_2d[2, 1].set_xlabel('Tempo (s)'), axs_2d[2, 1].set_ylabel('Força (N)'), axs_2d[2, 1].legend(), axs_2d[2, 1].grid(True)

axs_2d[3, 0].plot(time_log, state_log[:, 4], label='Altitude Z Real (m)')
axs_2d[3, 0].plot(time_log, state_noisy_log[:, 4], alpha=0.7, linestyle='--', label='Altitude Z Ruidosa (m)')
axs_2d[3, 0].set_xlabel('Tempo (s)'), axs_2d[3, 0].set_ylabel('Altitude Z (m)')
axs_2d[3, 0].set_title('Altitude Z: Real vs. Leitura Ruidosa')
axs_2d[3, 0].legend(), axs_2d[3, 0].grid(True)

axs_2d[3, 1].plot(time_log, np.rad2deg(state_log[:, 6]), label='Roll Real (graus)')
axs_2d[3, 1].plot(time_log, np.rad2deg(state_noisy_log[:, 6]), alpha=0.7, linestyle='--', label='Roll Ruidoso (graus)')
axs_2d[3, 1].set_xlabel('Tempo (s)'), axs_2d[3, 1].set_ylabel('Roll (graus)')
axs_2d[3, 1].set_title('Roll: Real vs. Leitura Ruidosa')
axs_2d[3, 1].legend(), axs_2d[3, 1].grid(True)

fig_2d.tight_layout()
plt.show(block=False)
plt.savefig('python/outputs/speed_control_sim_results_2d_static.png', dpi=300)

# --- Animação 3D Separada com Representação de Atitude Ajustada ---
print("Preparando animação 3D com atitude ajustada...")
fig_anim = plt.figure(figsize=(10, 8))
fig_anim.canvas.manager.set_window_title('Animação da Trajetória 3D do Drone com Atitude (Ajustada)')
ax_anim = fig_anim.add_subplot(111, projection='3d')

x_min, x_max = np.min(state_log[:, 0]), np.max(state_log[:, 0])
y_min, y_max = np.min(state_log[:, 2]), np.max(state_log[:, 2])
z_min, z_max = np.min(state_log[:, 4]), np.max(state_log[:, 4])
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

line_traj, = ax_anim.plot([], [], [], lw=2, label='Trajetória')
drone_marker, = ax_anim.plot([], [], [], marker='o', markersize=6, color='black', label='Drone CoM')
start_point = ax_anim.scatter(state_log[0, 0], state_log[0, 2], state_log[0, 4], c='g', marker='D', s=60, label='Início')

body_axis_length = 0.5
half_axis_len = body_axis_length / 2.0

body_x_line, = ax_anim.plot([], [], [], color='red', lw=3, label='Corpo X (Frente)') 
body_y_line, = ax_anim.plot([], [], [], color='green', lw=3, label='Corpo Y (Direita)')
body_z_line, = ax_anim.plot([], [], [], color='blue', lw=3, label='Corpo Z (Cima)')
ax_anim.legend(loc='best')

def init_animation():
    line_traj.set_data_3d([], [], [])
    drone_marker.set_data_3d([], [], [])
    body_x_line.set_data_3d([], [], [])
    body_y_line.set_data_3d([], [], [])
    body_z_line.set_data_3d([], [], [])
    return line_traj, drone_marker, body_x_line, body_y_line, body_z_line

num_animation_frames = len(time_log) // animation_frames_step

def update_animation(frame_idx):
    sim_idx = min(frame_idx * animation_frames_step, len(time_log) - 1)
    line_traj.set_data_3d(state_log[:sim_idx+1, 0], state_log[:sim_idx+1, 2], state_log[:sim_idx+1, 4])
    current_pos = state_log[sim_idx, 0:6:2]
    drone_marker.set_data_3d([current_pos[0]], [current_pos[1]], [current_pos[2]])
    phi, theta, psi = state_log[sim_idx, 6], state_log[sim_idx, 8], state_log[sim_idx, 10]
    R_body_to_inertial = get_rotation_matrix(phi, theta, psi)
    body_x_axis_vec_body = np.array([1, 0, 0])
    body_y_axis_vec_body = np.array([0, 1, 0])
    body_z_axis_vec_body = np.array([0, 0, 1])
    tip_x_inertial = R_body_to_inertial @ (body_x_axis_vec_body * half_axis_len)
    tip_y_inertial = R_body_to_inertial @ (body_y_axis_vec_body * half_axis_len)
    tip_z_inertial = R_body_to_inertial @ (body_z_axis_vec_body * half_axis_len)
    body_x_line.set_data_3d([current_pos[0] - tip_x_inertial[0], current_pos[0] + tip_x_inertial[0]],
                            [current_pos[1] - tip_x_inertial[1], current_pos[1] + tip_x_inertial[1]],
                            [current_pos[2] - tip_x_inertial[2], current_pos[2] + tip_x_inertial[2]])
    body_y_line.set_data_3d([current_pos[0] - tip_y_inertial[0], current_pos[0] + tip_y_inertial[0]],
                            [current_pos[1] - tip_y_inertial[1], current_pos[1] + tip_y_inertial[1]],
                            [current_pos[2] - tip_y_inertial[2], current_pos[2] + tip_y_inertial[2]])
    body_z_line.set_data_3d([current_pos[0] - tip_z_inertial[0], current_pos[0] + tip_z_inertial[0]],
                            [current_pos[1] - tip_z_inertial[1], current_pos[1] + tip_z_inertial[1]],
                            [current_pos[2] - tip_z_inertial[2], current_pos[2] + tip_z_inertial[2]])
    ax_anim.set_title(f'Animação 3D com Atitude (Tempo: {time_log[sim_idx]:.2f}s)')
    return line_traj, drone_marker, body_x_line, body_y_line, body_z_line

anim_interval_ms = 40
ani = animation.FuncAnimation(fig_anim, update_animation, frames=num_animation_frames,
                              init_func=init_animation, blit=True, interval=anim_interval_ms, repeat=False)
plt.show()

print("Salvando animação 3D...")
ani.save('python/outputs/speed_control_sim_animation_3d.mp4', writer='ffmpeg', fps=20, dpi=300)
print("Fim do script.")