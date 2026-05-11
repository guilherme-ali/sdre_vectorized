import numpy as np
import scipy.linalg
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# ==========================================
# 1. PARÂMETROS DO DRONE E LIMITES FÍSICOS
# ==========================================
Ixx = 16.57e-6
Iyy = 16.57e-6
Izz = 29.80e-6
Ir = 1.02e-7
m = 0.0469  # 46.9g
g = 9.80665
dt_sim = 0.01  # Simulação base (100Hz)
dt_sdre = 0.02  # SDRE a 50Hz
dt_pid = 0.05  # PID a 10Hz

# Parâmetros Físicos dos Motores (Do main.cpp)
b_coeff = 1.77e-8  # Coeficiente de Empuxo
d_coeff = 0.05 * b_coeff  # Coeficiente de Arrasto
L_arm = 0.060  # 60mm
L_eff = L_arm * np.sin(np.pi / 4)  # Braço efetivo para config em 'X'

# LIMITES DE RPM (Saturação)
MAX_RPM = 31086.0
MAX_OMEGA = MAX_RPM * (2.0 * np.pi) / 60.0
MAX_OMEGA_SQ = MAX_OMEGA**2

# Matriz de Mistura (Mixer Matrix) para Quadricóptero em 'X'
M_mixer = np.array(
    [
        [b_coeff, b_coeff, b_coeff, b_coeff],  # Empuxo Total (Z)
        [
            -b_coeff * L_eff,
            -b_coeff * L_eff,
            b_coeff * L_eff,
            b_coeff * L_eff,
        ],  # Roll (X)
        [
            -b_coeff * L_eff,
            b_coeff * L_eff,
            b_coeff * L_eff,
            -b_coeff * L_eff,
        ],  # Pitch (Y)
        [-d_coeff, d_coeff, -d_coeff, d_coeff],  # Yaw (Z)
    ]
)
M_inv = np.linalg.inv(M_mixer)

# Matriz de Mistura REAIS (simulando desbalanceamento de até 5% nos motores)
error_percent = 0.05
np.random.seed(42)  # Para resultados reprodutíveis
b_coeffs_real = b_coeff * np.random.uniform(1 - error_percent, 1 + error_percent, 4)
d_coeffs_real = d_coeff * np.random.uniform(1 - error_percent, 1 + error_percent, 4)

M_mixer_real = np.array(
    [
        [b_coeffs_real[0], b_coeffs_real[1], b_coeffs_real[2], b_coeffs_real[3]],
        [
            -b_coeffs_real[0] * L_eff,
            -b_coeffs_real[1] * L_eff,
            b_coeffs_real[2] * L_eff,
            b_coeffs_real[3] * L_eff,
        ],
        [
            -b_coeffs_real[0] * L_eff,
            b_coeffs_real[1] * L_eff,
            b_coeffs_real[2] * L_eff,
            -b_coeffs_real[3] * L_eff,
        ],
        [-d_coeffs_real[0], d_coeffs_real[1], -d_coeffs_real[2], d_coeffs_real[3]],
    ]
)

# Matrizes de Peso do SDRE (Q e R)
roll_max_rad = 15.0 * np.pi / 180.0
pitch_max_rad = 15.0 * np.pi / 180.0
yaw_max_rad = 30.0 * np.pi / 180.0
p_max = 15.0 * np.pi / 180.0
q_max = 15.0 * np.pi / 180.0
r_max = 45.0 * np.pi / 180.0

Q = np.diag(
    [
        1.0 / (roll_max_rad**2),
        1.0 / (pitch_max_rad**2),
        1.0 / (yaw_max_rad**2),
        1.0 / (p_max**2),
        1.0 / (q_max**2),
        1.0 / (r_max**2),
    ]
)
R_mat = np.eye(3)

max_tau_roll = 2 * b_coeff * L_eff * MAX_OMEGA * MAX_OMEGA
max_tau_pitch = 2 * b_coeff * L_eff * MAX_OMEGA * MAX_OMEGA
max_tau_yaw = 4 * d_coeff * MAX_OMEGA * MAX_OMEGA

R_mat = np.diag(
    [
        1.0 / (max_tau_roll**2),
        1.0 / (max_tau_pitch**2),
        1.0 / (max_tau_yaw**2),
    ]
)


# ==========================================
# 1B. SENSORES (MPU6050 + QMC5883L) E MADGWICK
# ==========================================
# Ruídos típicos do MPU6050 com DLPF @ ~98Hz
GYRO_NOISE_STD = 0.01  # rad/s (~0.57 °/s)
ACCEL_NOISE_STD = 0.10  # m/s²
GYRO_BIAS = np.array([0.003, -0.002, 0.004])  # rad/s
ACCEL_BIAS = np.array([0.05, -0.03, 0.08])  # m/s²

# Ruído do QMC5883L (saída normalizada)
MAG_NOISE_STD = 0.015

# Inclinação magnética típica (Brasil ≈ -30°). ENU/Z-up: bz = -sin(inc)
MAG_INCLINATION = np.deg2rad(-30.0)
MAG_WORLD = np.array([np.cos(MAG_INCLINATION), 0.0, -np.sin(MAG_INCLINATION)])
MAG_WORLD /= np.linalg.norm(MAG_WORLD)

# Ganho β do filtro Madgwick (compromisso giroscópio × accel/mag)
MADGWICK_BETA = 0.04


def madgwick_marg_update(q, gx, gy, gz, ax, ay, az, mx, my, mz, beta, dt):
    """Filtro Madgwick MARG (Magnetic, Angular Rate, Gravity).
    Convenção Z-up coerente com a simulação. q = [w, x, y, z].
    """
    q1, q2, q3, q4 = q  # q1=w, q2=x, q3=y, q4=z

    norm_a = np.sqrt(ax * ax + ay * ay + az * az)
    if norm_a < 1e-9:
        return q
    ax, ay, az = ax / norm_a, ay / norm_a, az / norm_a

    norm_m = np.sqrt(mx * mx + my * my + mz * mz)
    if norm_m < 1e-9:
        return q
    mx, my, mz = mx / norm_m, my / norm_m, mz / norm_m

    # Direção de referência do campo magnético terrestre (descontando inclinação atual)
    _2q1mx = 2 * q1 * mx
    _2q1my = 2 * q1 * my
    _2q1mz = 2 * q1 * mz
    _2q2mx = 2 * q2 * mx
    hx = (
        mx * q1 * q1
        - _2q1my * q4
        + _2q1mz * q3
        + mx * q2 * q2
        + 2 * q2 * my * q3
        + 2 * q2 * mz * q4
        - mx * q3 * q3
        - mx * q4 * q4
    )
    hy = (
        _2q1mx * q4
        + my * q1 * q1
        - _2q1mz * q2
        + _2q2mx * q3
        - my * q2 * q2
        + my * q3 * q3
        + 2 * q3 * mz * q4
        - my * q4 * q4
    )
    _2bx = np.sqrt(hx * hx + hy * hy)
    _2bz = (
        -_2q1mx * q3
        + _2q1my * q2
        + mz * q1 * q1
        + _2q2mx * q4
        - mz * q2 * q2
        + 2 * q3 * my * q4
        - mz * q3 * q3
        + mz * q4 * q4
    )
    _4bx = 2 * _2bx
    _4bz = 2 * _2bz

    # Gradiente da função objetivo (gravidade + bússola)
    s1 = (
        -2 * q3 * (2 * q2 * q4 - 2 * q1 * q3 - ax)
        + 2 * q2 * (2 * q1 * q2 + 2 * q3 * q4 - ay)
        - _2bz
        * q3
        * (_2bx * (0.5 - q3 * q3 - q4 * q4) + _2bz * (q2 * q4 - q1 * q3) - mx)
        + (-_2bx * q4 + _2bz * q2)
        * (_2bx * (q2 * q3 - q1 * q4) + _2bz * (q1 * q2 + q3 * q4) - my)
        + _2bx
        * q3
        * (_2bx * (q1 * q3 + q2 * q4) + _2bz * (0.5 - q2 * q2 - q3 * q3) - mz)
    )
    s2 = (
        2 * q4 * (2 * q2 * q4 - 2 * q1 * q3 - ax)
        + 2 * q1 * (2 * q1 * q2 + 2 * q3 * q4 - ay)
        - 4 * q2 * (1 - 2 * q2 * q2 - 2 * q3 * q3 - az)
        + _2bz
        * q4
        * (_2bx * (0.5 - q3 * q3 - q4 * q4) + _2bz * (q2 * q4 - q1 * q3) - mx)
        + (_2bx * q3 + _2bz * q1)
        * (_2bx * (q2 * q3 - q1 * q4) + _2bz * (q1 * q2 + q3 * q4) - my)
        + (_2bx * q4 - _4bz * q2)
        * (_2bx * (q1 * q3 + q2 * q4) + _2bz * (0.5 - q2 * q2 - q3 * q3) - mz)
    )
    s3 = (
        -2 * q1 * (2 * q2 * q4 - 2 * q1 * q3 - ax)
        + 2 * q4 * (2 * q1 * q2 + 2 * q3 * q4 - ay)
        - 4 * q3 * (1 - 2 * q2 * q2 - 2 * q3 * q3 - az)
        + (-_4bx * q3 - _2bz * q1)
        * (_2bx * (0.5 - q3 * q3 - q4 * q4) + _2bz * (q2 * q4 - q1 * q3) - mx)
        + (_2bx * q2 + _2bz * q4)
        * (_2bx * (q2 * q3 - q1 * q4) + _2bz * (q1 * q2 + q3 * q4) - my)
        + (_2bx * q1 - _4bz * q3)
        * (_2bx * (q1 * q3 + q2 * q4) + _2bz * (0.5 - q2 * q2 - q3 * q3) - mz)
    )
    s4 = (
        2 * q2 * (2 * q2 * q4 - 2 * q1 * q3 - ax)
        + 2 * q3 * (2 * q1 * q2 + 2 * q3 * q4 - ay)
        + (-_4bx * q4 + _2bz * q2)
        * (_2bx * (0.5 - q3 * q3 - q4 * q4) + _2bz * (q2 * q4 - q1 * q3) - mx)
        + (-_2bx * q1 + _2bz * q3)
        * (_2bx * (q2 * q3 - q1 * q4) + _2bz * (q1 * q2 + q3 * q4) - my)
        + _2bx
        * q2
        * (_2bx * (q1 * q3 + q2 * q4) + _2bz * (0.5 - q2 * q2 - q3 * q3) - mz)
    )
    norm_s = np.sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4)
    if norm_s < 1e-12:
        return q
    s1 /= norm_s
    s2 /= norm_s
    s3 /= norm_s
    s4 /= norm_s

    # Taxa de variação do quaternion (giroscópio + correção pelo gradiente)
    qDot1 = 0.5 * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1
    qDot2 = 0.5 * (q1 * gx + q3 * gz - q4 * gy) - beta * s2
    qDot3 = 0.5 * (q1 * gy - q2 * gz + q4 * gx) - beta * s3
    qDot4 = 0.5 * (q1 * gz + q2 * gy - q3 * gx) - beta * s4

    q1 += qDot1 * dt
    q2 += qDot2 * dt
    q3 += qDot3 * dt
    q4 += qDot4 * dt
    n = np.sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4)
    return np.array([q1, q2, q3, q4]) / n


def quat_to_euler(quat):
    """Quaternion [w,x,y,z] → (phi, theta, psi) na convenção ZYX."""
    w, x, y, z = quat
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    phi = np.arctan2(sinr_cosp, cosr_cosp)
    sinp = np.clip(2 * (w * y - z * x), -1.0, 1.0)
    theta = np.arcsin(sinp)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    psi = np.arctan2(siny_cosp, cosy_cosp)
    return phi, theta, psi


def simulate_sensors(state, T_real, w_x, w_y, w_z, R_wb):
    """Gera leituras simuladas do MPU6050 (acel + giro) e QMC5883L (mag).

    Acelerômetro mede força específica em corpo (sem o termo gravitacional puro):
        f_world = (T/m) · R · ẑ_corpo + vento
        a_corpo = R^T · f_world
    """
    p_b, q_b, r_b = state[9], state[10], state[11]

    f_world = (T_real / m) * R_wb @ np.array([0.0, 0.0, 1.0]) + np.array(
        [w_x, w_y, w_z]
    )
    accel_body = R_wb.T @ f_world
    accel_meas = accel_body + ACCEL_BIAS + np.random.normal(0.0, ACCEL_NOISE_STD, 3)

    gyro_meas = (
        np.array([p_b, q_b, r_b]) + GYRO_BIAS + np.random.normal(0.0, GYRO_NOISE_STD, 3)
    )

    mag_body = R_wb.T @ MAG_WORLD
    mag_meas = mag_body + np.random.normal(0.0, MAG_NOISE_STD, 3)

    return accel_meas, gyro_meas, mag_meas


# ==========================================
# 2. CLASSES E FUNÇÕES AUXILIARES
# ==========================================
class PIDController:
    def __init__(self, kp, ki, kd, dt, limit):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.limit = limit
        self.integral = 0.0
        self.prev_error = 0.0

    def update(self, error):
        self.integral += error * self.dt
        self.integral = np.clip(self.integral, -self.limit, self.limit)

        derivative = (error - self.prev_error) / self.dt
        self.prev_error = error

        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        return np.clip(output, -self.limit, self.limit)


def get_sdre_matrices(roll, pitch, yaw, p, q, r, omega_r=0.0):
    alpha_1, alpha_2, alpha_3 = 0.5, 0.5, 0.5
    A = np.zeros((6, 6))

    sr, cr = np.sin(roll), np.cos(roll)
    cp, tp = np.cos(pitch), np.tan(pitch)

    if abs(cp) < 1e-5:
        cp = np.sign(cp) * 1e-5
    inv_cp = 1.0 / cp

    A[0, 3] = 1.0
    A[0, 4] = sr * tp
    A[0, 5] = cr * tp
    A[1, 4] = cr
    A[1, 5] = -sr
    A[2, 4] = sr * inv_cp
    A[2, 5] = cr * inv_cp

    A[3, 4] = alpha_1 * ((Iyy - Izz) / Ixx) * r - Ir * omega_r / Ixx
    A[3, 5] = (1 - alpha_1) * ((Iyy - Izz) / Ixx) * q

    A[4, 3] = alpha_2 * ((Izz - Ixx) / Iyy) * r + Ir * omega_r / Iyy
    A[4, 5] = (1 - alpha_2) * ((Izz - Ixx) / Iyy) * p

    A[5, 3] = alpha_3 * ((Ixx - Iyy) / Izz) * q
    A[5, 4] = (1 - alpha_3) * ((Ixx - Iyy) / Izz) * p

    B = np.zeros((6, 3))
    B[3, 0] = 1.0 / Ixx
    B[4, 1] = 1.0 / Iyy
    B[5, 2] = 1.0 / Izz

    A2 = A @ A
    Ad = np.eye(6) + A * dt_sdre + A2 * (dt_sdre**2 * 0.5)
    Bd = B * dt_sdre + (A @ B) * (dt_sdre**2 * 0.5)

    return Ad, Bd


def get_rotation_matrix(phi, theta, psi):
    cphi, sphi = np.cos(phi), np.sin(phi)
    cth, sth = np.cos(theta), np.sin(theta)
    cpsi, spsi = np.cos(psi), np.sin(psi)

    Rx = np.array([[1, 0, 0], [0, cphi, -sphi], [0, sphi, cphi]])
    Ry = np.array([[cth, 0, sth], [0, 1, 0], [-sth, 0, cth]])
    Rz = np.array([[cpsi, -spsi, 0], [spsi, cpsi, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


# ==========================================
# 3. SIMULAÇÃO DO SISTEMA (com sensores + Madgwick)
# ==========================================
def simulate():
    time_sim = 20.0
    steps = int(time_sim / dt_sim)
    state = np.zeros(12)

    u_max = b_coeff * MAX_OMEGA_SQ * 4
    erro_max = 0.5
    kp_z = (u_max - g * m) / (erro_max * m) * 0.8

    # kd >= 2*sqrt(kp) para garantir amortecimento crítico ou superamortecido
    kd_z = 2 * np.sqrt(kp_z)

    pid_vx = PIDController(kp=2.0, ki=0.1, kd=1.5, dt=dt_pid, limit=15.0)
    pid_vy = PIDController(kp=2.0, ki=0.1, kd=1.5, dt=dt_pid, limit=15.0)
    pid_z = PIDController(kp=kp_z, ki=0.1, kd=kd_z, dt=dt_pid, limit=15.0)

    pid_vx = PIDController(kp=0.0, ki=0.0, kd=0 * 1.5, dt=dt_pid, limit=15.0)
    pid_vy = PIDController(kp=0.0, ki=0.0, kd=0 * 1.5, dt=dt_pid, limit=15.0)
    pid_z = PIDController(kp=0, ki=0, kd=0, dt=dt_pid, limit=15.0)

    history = {
        "t": [],
        "x": [],
        "y": [],
        "z": [],
        "vx": [],
        "vy": [],
        "vz": [],
        "phi": [],
        "theta": [],
        "psi": [],
        "phi_est": [],
        "theta_est": [],
        "psi_est": [],
        "wind_x": [],
        "wind_y": [],
        "wind_z": [],
        "rpm1": [],
        "rpm2": [],
        "rpm3": [],
        "rpm4": [],
        "ax_meas": [],
        "ay_meas": [],
        "az_meas": [],
        "gx_meas": [],
        "gy_meas": [],
        "gz_meas": [],
        "mx_meas": [],
        "my_meas": [],
        "mz_meas": [],
    }

    K_prev = np.zeros((3, 6))

    phi_ref, theta_ref = 0.0, 0.0
    T_ideal = m * g
    tau_ideal = np.zeros(3)

    # Estado do filtro Madgwick (quaternion identidade) e empuxo aplicado anterior
    quat = np.array([1.0, 0.0, 0.0, 0.0])
    T_real_prev = m * g

    for step in range(steps):
        t = step * dt_sim
        vx, vy, vz = state[3], state[4], state[5]
        phi_t, theta_t, psi_t = state[6], state[7], state[8]
        p_t, q_t, r_t = state[9], state[10], state[11]
        z = state[2]

        Vx_ref, Vy_ref, Z_ref = 0.0, 0.0, 1.0

        # Perturbação de Vento
        w_x = np.random.normal(0.0, 0.2)
        w_y = np.random.normal(0.0, 0.2)
        w_z = np.random.normal(0.0, 0.1)

        if 2.0 <= t <= 3.5:
            # Aumentei um pouco a rajada para forçar os motores a baterem no teto de 31k
            w_x += np.random.normal(5.0, 1.5) * 0.2
            w_y += np.random.normal(-4.0, 1.0) * 0.2
            w_z += np.random.normal(-1.0, 0.5) * 0.2

        # ====================================================
        # SENSORES + FILTRO MADGWICK (executa a cada passo, 100Hz)
        # ====================================================
        R_true = get_rotation_matrix(phi_t, theta_t, psi_t)
        accel_meas, gyro_meas, mag_meas = simulate_sensors(
            state, T_real_prev, w_x, w_y, w_z, R_true
        )
        quat = madgwick_marg_update(
            quat,
            gyro_meas[0],
            gyro_meas[1],
            gyro_meas[2],
            accel_meas[0],
            accel_meas[1],
            accel_meas[2],
            mag_meas[0],
            mag_meas[1],
            mag_meas[2],
            MADGWICK_BETA,
            dt_sim,
        )
        phi_est, theta_est, psi_est = quat_to_euler(quat)
        # Taxas estimadas vêm direto do giroscópio (Madgwick não estima bias do giro)
        p_est, q_est, r_est = gyro_meas[0], gyro_meas[1], gyro_meas[2]
        # ====================================================

        # PID -> Referências Ideais (10Hz)
        if step % int(round(dt_pid / dt_sim)) == 0:
            ux = pid_vx.update(Vx_ref - vx)
            uy = pid_vy.update(Vy_ref - vy)
            uz = pid_z.update(Z_ref - z)

            theta_ref = np.arctan2(ux, uz + g)
            phi_ref = np.arctan2(-uy * np.cos(theta_ref), uz + g)

            T_ideal = m * (uz + g) / (np.cos(theta_ref) * np.cos(phi_ref))

        # SDRE -> Torques Ideais (50Hz) — usa estado ESTIMADO
        if step % int(round(dt_sdre / dt_sim)) == 0:
            Ad, Bd = get_sdre_matrices(phi_est, theta_est, psi_est, p_est, q_est, r_est)
            try:
                P = scipy.linalg.solve_discrete_are(Ad, Bd, Q, R_mat)
                K = np.linalg.inv(R_mat + Bd.T @ P @ Bd) @ (Bd.T @ P @ Ad)
                K_prev = K
            except np.linalg.LinAlgError:
                K = K_prev

            x_att = np.array([phi_est, theta_est, psi_est, p_est, q_est, r_est])
            x_ref = np.array([phi_ref, theta_ref, 0.0, 0, 0, 0])
            tau_ideal = -K @ (x_att - x_ref)

        # ========================================================
        # O "CLAMP" - LIMITANDO RPM E EXTRAINDO AS FORÇAS REAIS
        # ========================================================
        efforts_ideal = np.array([T_ideal, tau_ideal[0], tau_ideal[1], tau_ideal[2]])

        # Converte para omega quadrado ideal
        omega_sq_ideal = M_inv @ efforts_ideal

        # Satura as velocidades dos motores: mínimo 0, máximo MAX_OMEGA_SQ (31086 RPM)
        omega_sq_real = np.clip(omega_sq_ideal, 0.0, MAX_OMEGA_SQ)

        # Converte de volta para RPM apenas para plotagem
        rpms = np.sqrt(omega_sq_real) * (60.0 / (2 * np.pi))

        # Recalcula as forças REAIS que o drone conseguiu gerar (Motor Saturation)
        efforts_real = M_mixer_real @ omega_sq_real
        T_real = efforts_real[0]
        tau_real = efforts_real[1:4]
        T_real_prev = T_real
        # ========================================================

        # Dinâmica Translacional + Vento (Usando T_real e estado verdadeiro)
        dvx = (T_real / m) * (np.sin(theta_t) * np.cos(phi_t)) + w_x
        dvy = (T_real / m) * (-np.sin(phi_t)) + w_y
        dvz = (T_real / m) * (np.cos(theta_t) * np.cos(phi_t)) - g + w_z

        # Dinâmica Rotacional (Usando tau_real)
        dp = tau_real[0] / Ixx + ((Iyy - Izz) / Ixx) * q_t * r_t
        dq = tau_real[1] / Iyy + ((Izz - Ixx) / Iyy) * p_t * r_t
        dr = tau_real[2] / Izz + ((Ixx - Iyy) / Izz) * p_t * q_t

        dphi = (
            p_t
            + q_t * np.sin(phi_t) * np.tan(theta_t)
            + r_t * np.cos(phi_t) * np.tan(theta_t)
        )
        dtheta = q_t * np.cos(phi_t) - r_t * np.sin(phi_t)
        dpsi = q_t * np.sin(phi_t) / np.cos(theta_t) + r_t * np.cos(phi_t) / np.cos(
            theta_t
        )

        # Integração
        state[0] += state[3] * dt_sim
        state[1] += state[4] * dt_sim
        state[2] += state[5] * dt_sim
        state[3] += dvx * dt_sim
        state[4] += dvy * dt_sim
        state[5] += dvz * dt_sim
        state[6] += dphi * dt_sim
        state[7] += dtheta * dt_sim
        state[8] += dpsi * dt_sim
        state[9] += dp * dt_sim
        state[10] += dq * dt_sim
        state[11] += dr * dt_sim

        # Histórico
        history["t"].append(t)
        history["x"].append(state[0])
        history["y"].append(state[1])
        history["z"].append(state[2])
        history["vx"].append(state[3])
        history["vy"].append(state[4])
        history["vz"].append(state[5])
        history["phi"].append(state[6])
        history["theta"].append(state[7])
        history["psi"].append(state[8])
        history["phi_est"].append(phi_est)
        history["theta_est"].append(theta_est)
        history["psi_est"].append(psi_est)
        history["wind_x"].append(w_x)
        history["wind_y"].append(w_y)
        history["wind_z"].append(w_z)
        history["rpm1"].append(rpms[0])
        history["rpm2"].append(rpms[1])
        history["rpm3"].append(rpms[2])
        history["rpm4"].append(rpms[3])
        history["ax_meas"].append(accel_meas[0])
        history["ay_meas"].append(accel_meas[1])
        history["az_meas"].append(accel_meas[2])
        history["gx_meas"].append(gyro_meas[0])
        history["gy_meas"].append(gyro_meas[1])
        history["gz_meas"].append(gyro_meas[2])
        history["mx_meas"].append(mag_meas[0])
        history["my_meas"].append(mag_meas[1])
        history["mz_meas"].append(mag_meas[2])

    return history


history = simulate()

# ==========================================
# 4. GRÁFICOS — DESEMPENHO DE CONTROLE
# ==========================================
fig1 = plt.figure(figsize=(14, 14))

plt.subplot(4, 1, 1)
plt.plot(
    history["t"], history["wind_x"], label="Vento em X", color="darkred", alpha=0.7
)
plt.plot(
    history["t"], history["wind_y"], label="Vento em Y", color="darkgreen", alpha=0.7
)
plt.plot(
    history["t"], history["wind_z"], label="Vento em Z", color="darkblue", alpha=0.7
)
plt.axvspan(2.0, 3.5, color="gray", alpha=0.2, label="Rajada Forte")
plt.title("Perturbação de Vento Injetada")
plt.ylabel("Aceleração (m/s²)")
plt.legend(loc="upper right")
plt.grid(True)

plt.subplot(4, 1, 2)
plt.plot(history["t"], history["vx"], label="Vx Real", color="r")
plt.plot(history["t"], history["vy"], label="Vy Real", color="g")
plt.plot(history["t"], history["z"], label="Z Real (Altitude)", color="b")
plt.axhline(1.0, color="black", linestyle="--", linewidth=1, label="Ref Z (1m)")
plt.axvspan(2.0, 3.5, color="gray", alpha=0.2)
plt.title("Efeito do Vento nas Velocidades e Altitude")
plt.ylabel("m/s e m")
plt.legend(loc="upper right")
plt.grid(True)

plt.subplot(4, 1, 3)
plt.plot(history["t"], np.degrees(history["phi"]), label="Roll ($\\phi$)", color="g")
plt.plot(
    history["t"], np.degrees(history["theta"]), label="Pitch ($\\theta$)", color="r"
)
plt.plot(history["t"], np.degrees(history["psi"]), label="Yaw ($\\psi$)", color="b")
plt.axhline(0, color="black", linestyle="--", linewidth=1)
plt.axvspan(2.0, 3.5, color="gray", alpha=0.2)
plt.title("Reação de Atitude Comandada (Graus) — Estado Real")
plt.ylabel("Ângulo (graus)")
plt.legend(loc="upper right")
plt.grid(True)

plt.subplot(4, 1, 4)
plt.plot(history["t"], history["rpm1"], label="Motor 1 (Front-Dir CW)", color="orange")
plt.plot(history["t"], history["rpm2"], label="Motor 2 (Tras-Dir CCW)", color="purple")
plt.plot(history["t"], history["rpm3"], label="Motor 3 (Tras-Esq CW)", color="cyan")
plt.plot(
    history["t"], history["rpm4"], label="Motor 4 (Front-Esq CCW)", color="magenta"
)
plt.axhline(
    31086, color="red", linestyle="--", linewidth=2, label="Limite Físico (31.086 RPM)"
)
plt.axvspan(2.0, 3.5, color="gray", alpha=0.2)
plt.title("Atuação Física: Saturação de Rotação dos Motores")
plt.xlabel("Tempo (s)")
plt.ylabel("Rotação (RPM)")
plt.legend(loc="lower right")
plt.grid(True)

plt.tight_layout()


# ==========================================
# 4B. SENSORES BRUTOS (MPU6050 + QMC5883L)
# ==========================================
fig_sensors = plt.figure(figsize=(14, 12))

plt.subplot(3, 1, 1)
plt.plot(history["t"], history["ax_meas"], label="Acel X", color="r", alpha=0.7)
plt.plot(history["t"], history["ay_meas"], label="Acel Y", color="g", alpha=0.7)
plt.plot(history["t"], history["az_meas"], label="Acel Z", color="b", alpha=0.7)
plt.axhline(g, color="black", linestyle="--", linewidth=1, label="g = 9.81 m/s²")
plt.axvspan(2.0, 3.5, color="gray", alpha=0.2)
plt.title("MPU6050 — Acelerômetro (força específica, com ruído + bias)")
plt.ylabel("m/s²")
plt.legend(loc="upper right")
plt.grid(True)

plt.subplot(3, 1, 2)
plt.plot(
    history["t"],
    np.degrees(history["gx_meas"]),
    label="Giro X (p)",
    color="r",
    alpha=0.7,
)
plt.plot(
    history["t"],
    np.degrees(history["gy_meas"]),
    label="Giro Y (q)",
    color="g",
    alpha=0.7,
)
plt.plot(
    history["t"],
    np.degrees(history["gz_meas"]),
    label="Giro Z (r)",
    color="b",
    alpha=0.7,
)
plt.axvspan(2.0, 3.5, color="gray", alpha=0.2)
plt.title("MPU6050 — Giroscópio (com ruído + bias)")
plt.ylabel("°/s")
plt.legend(loc="upper right")
plt.grid(True)

plt.subplot(3, 1, 3)
plt.plot(history["t"], history["mx_meas"], label="Mag X", color="r", alpha=0.7)
plt.plot(history["t"], history["my_meas"], label="Mag Y", color="g", alpha=0.7)
plt.plot(history["t"], history["mz_meas"], label="Mag Z", color="b", alpha=0.7)
plt.axvspan(2.0, 3.5, color="gray", alpha=0.2)
plt.title("QMC5883L — Magnetômetro (normalizado, com ruído)")
plt.xlabel("Tempo (s)")
plt.ylabel("Campo (norm.)")
plt.legend(loc="upper right")
plt.grid(True)

plt.tight_layout()


# ==========================================
# 4C. ESTIMATIVA MADGWICK × ESTADO REAL
# ==========================================
fig_est = plt.figure(figsize=(14, 10))

ang_pairs = [
    ("phi", "Roll ($\\phi$)"),
    ("theta", "Pitch ($\\theta$)"),
    ("psi", "Yaw ($\\psi$)"),
]
for i, (key, label) in enumerate(ang_pairs):
    plt.subplot(3, 1, i + 1)
    real = np.degrees(np.array(history[key]))
    est = np.degrees(np.array(history[key + "_est"]))
    plt.plot(history["t"], real, label=f"{label} Real", linewidth=2)
    plt.plot(
        history["t"],
        est,
        label=f"{label} Estimado (Madgwick)",
        linewidth=1.2,
        linestyle="--",
    )
    plt.plot(
        history["t"],
        est - real,
        label="Erro (Est − Real)",
        color="orange",
        alpha=0.6,
    )
    plt.axvspan(2.0, 3.5, color="gray", alpha=0.2)
    plt.title(f"{label}: Real vs Madgwick")
    plt.ylabel("Graus")
    plt.legend(loc="upper right")
    plt.grid(True)

plt.xlabel("Tempo (s)")
plt.tight_layout()


# ==========================================
# 5. ANIMAÇÃO 3D
# ==========================================
fig2 = plt.figure(figsize=(10, 8))
ax = fig2.add_subplot(111, projection="3d")
ax.set_title("Visualização 3D - Drone Quadricóptero")

hx, hy, hz = np.array(history["x"]), np.array(history["y"]), np.array(history["z"])
hphi, htheta, hpsi = (
    np.array(history["phi"]),
    np.array(history["theta"]),
    np.array(history["psi"]),
)

lim = max(np.max(np.abs(hx)), np.max(np.abs(hy)), 1.0)
ax.set_xlim([-lim, lim])
ax.set_ylim([-lim, lim])
ax.set_zlim([-lim, lim])
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.set_zlabel("Z (m)")

L = lim * 0.15
arm1 = np.array([L, 0, 0])
arm2 = np.array([-L, 0, 0])
arm3 = np.array([0, L, 0])
arm4 = np.array([0, -L, 0])

(line_x,) = ax.plot([], [], [], "r-", lw=4, label="Eixo X (Pitch)")
(line_y,) = ax.plot([], [], [], "b-", lw=4, label="Eixo Y (Roll)")
(path,) = ax.plot([], [], [], "k--", alpha=0.5, label="Trajetória")
ax.legend()


def update_anim(frame):
    path.set_data(hx[:frame], hy[:frame])
    path.set_3d_properties(hz[:frame])

    R = get_rotation_matrix(hphi[frame], htheta[frame], hpsi[frame])
    p_center = np.array([hx[frame], hy[frame], hz[frame]])

    p1 = p_center + R @ arm1
    p2 = p_center + R @ arm2
    p3 = p_center + R @ arm3
    p4 = p_center + R @ arm4

    line_x.set_data([p1[0], p2[0]], [p1[1], p2[1]])
    line_x.set_3d_properties([p1[2], p2[2]])

    line_y.set_data([p3[0], p4[0]], [p3[1], p4[1]])
    line_y.set_3d_properties([p3[2], p4[2]])

    return line_x, line_y, path


ani = animation.FuncAnimation(
    fig2, update_anim, frames=np.arange(0, len(hx), 2), interval=20, blit=False
)
plt.show()
