import numpy as np
from scipy.linalg import solve_discrete_are
import matplotlib.pyplot as plt
from tqdm import tqdm
import itertools
import pandas as pd
import sympy as sp
import time

# --- Parâmetros do Sistema ---
# Parâmetros físicos do VANT
Ixx = 0.00184  # Momento de inércia em x
Iyy = 0.00225  # Momento de inércia em y  
Izz = 0.00338  # Momento de inércia em z
Ir = 0.00001   # Momento de inércia do rotor
Omega_r = 0.0  # Velocidade angular do rotor

# Condições iniciais [p, q, r, phi, theta, psi]
x0 = [0.0, 0.0, 0.0, 1, 0.0, 0.0]

# Matrizes de ponderação para a função de custo
Q = np.diag([1, 1, 1, 50.0, 50.0, 50.0])  # 6x6
R = np.eye(3)*0.1  # 3x3

# Matriz de entrada do sistema B (6x3)
B = np.array([[1.0/Ixx, 0.0, 0.0],
              [0.0, 1.0/Iyy, 0.0],
              [0.0, 0.0, 1.0/Izz],
              [0.0, 0.0, 0.0],
              [1.0, 0.0, 0.0],
              [0.0, 0.0, 0.0]])

# Parâmetros de simulação
t_span = [0, 1]
Ts = 0.01  # Período de amostragem (100 amostras por segundo)
num_steps = int((t_span[1] - t_span[0]) / Ts) + 1
t_eval = np.linspace(t_span[0], t_span[1], num_steps)

# Valores dos parâmetros para testar
amostras = 20
param1_values = np.linspace(110, 120, amostras)  # Parâmetro alpha1
param2_values = np.linspace(-220, -200, amostras)  # Parâmetro alpha2
param3_values = np.linspace(60, 70, amostras)  # Parâmetro alpha3

# --- Geração do ruído para simular vento ---
# Configuração do ruído
np.random.seed(42)  # Define uma semente para reprodutibilidade
ruido_magnitude = 0.01  # Magnitude do ruído (ajuste conforme necessário)

# Gerar ruído correlacionado temporalmente para simular vento
# Vamos usar um processo AR(1) para gerar ruído correlacionado
def gerar_ruido_vento(passos, magnitude, correlacao=0.95):
    """
    Gera ruído correlacionado temporalmente para simular vento
    passos: número de passos de tempo
    magnitude: escala do ruído
    correlacao: fator de correlação temporal (0 a 1)
    """
    ruido = np.zeros((6, passos))
    
    # Componentes do ruído (p, q, r são afetados pelo vento)
    for i in range(3):  # Aplica nas taxas angulares p, q, r
        ruido_atual = 0
        for t in range(passos):
            ruido_atual = correlacao * ruido_atual + (1 - correlacao) * np.random.normal(0, 1)
            ruido[i, t] = ruido_atual * magnitude
    
    # Adiciona uma componente senoidal de baixa frequência para simular rajadas
    t = np.linspace(0, 2*np.pi, passos)
    frequencias = [0.5, 0.7, 0.3]  # Diferentes frequências para cada eixo
    for i in range(3):
        ruido[i, :] += magnitude * 0.5 * np.sin(frequencias[i] * t)
    
    return ruido

# Gera o ruído para toda a simulação (será o mesmo para todas as combinações de parâmetros)
ruido_vento = gerar_ruido_vento(num_steps, ruido_magnitude)

# Visualizar o ruído gerado
plt.figure(figsize=(10, 6))
plt.plot(t_eval, ruido_vento[0, :], label='Ruído em p')
plt.plot(t_eval, ruido_vento[1, :], label='Ruído em q')
plt.plot(t_eval, ruido_vento[2, :], label='Ruído em r')
plt.title('Ruído simulando vento aplicado ao sistema')
plt.xlabel('Tempo (s)')
plt.ylabel('Magnitude')
plt.legend()
plt.grid(True)
plt.savefig('python\\outputs\\ruido_vento.png')
plt.close()

print(f"Ruído de vento gerado com magnitude: {ruido_magnitude}")

# --- Otimização: Pré-cálculo Simbólico das Matrizes Ad e Bd ---
print("Iniciando pré-cálculo simbólico das matrizes Ad e Bd...")
setup_start_time = time.time()

# Definir variáveis simbólicas para estados, parâmetros e alphas
p, q, r, phi, theta = sp.symbols('p q r phi theta')
_Ixx, _Iyy, _Izz, _Ir, _Omega_r, _Ts = sp.symbols('Ixx Iyy Izz Ir Omega_r Ts')
_alpha1, _alpha2, _alpha3 = sp.symbols('alpha1 alpha2 alpha3')

# Elementos da matriz A(x) simbólica
a12_sym = _alpha1 * ((_Iyy - _Izz) / _Ixx) * r - (_Ir * _Omega_r / _Ixx)
a13_sym = (1 - _alpha1) * ((_Iyy - _Izz) / _Ixx) * q
a21_sym = _alpha2 * ((_Izz - _Ixx) / _Iyy) * r + (_Ir * _Omega_r / _Iyy)
a23_sym = (1 - _alpha2) * ((_Izz - _Ixx) / _Iyy) * p
a31_sym = _alpha3 * ((_Ixx - _Iyy) / _Izz) * q
a32_sym = (1 - _alpha3) * ((_Ixx - _Iyy) / _Izz) * p

# Matriz A(x) simbólica
A_sym = sp.Matrix([
    [0, a12_sym, a13_sym, 0, 0, 0],
    [a21_sym, 0, a23_sym, 0, 0, 0],
    [a31_sym, a32_sym, 0, 0, 0, 0],
    [1, sp.sin(phi) * sp.tan(theta), sp.cos(phi) * sp.tan(theta), 0, 0, 0],
    [0, sp.cos(phi), -sp.sin(phi), 0, 0, 0],
    [0, sp.sin(phi) / sp.cos(theta), sp.cos(phi) / sp.cos(theta), 0, 0, 0]
])

# Matrizes simbólicas para discretização
B_sym = sp.Matrix(B)
I_sym = sp.eye(6)

# Discretização simbólica (aproximação de Taylor de 2ª ordem)
Ad_sym = I_sym + A_sym * _Ts + (A_sym @ A_sym * _Ts**2) / 2
Bd_sym = (I_sym * _Ts + (A_sym * _Ts**2) / 2) @ B_sym

# Lista de variáveis na ordem correta para as funções
vars_list = [p, q, r, phi, theta, _alpha1, _alpha2, _alpha3, _Ixx, _Iyy, _Izz, _Ir, _Omega_r, _Ts]

# Criar funções numéricas rápidas a partir das expressões simbólicas
Ad_func = sp.lambdify(vars_list, Ad_sym, 'numpy')
Bd_func = sp.lambdify(vars_list, Bd_sym, 'numpy')

setup_time = time.time() - setup_start_time
print(f"Pré-cálculo concluído em {setup_time:.4f} segundos.")

# --- Simulação e Geração de Gráficos ---
plt.style.use('seaborn-v0_8-whitegrid')
fig, axes = plt.subplots(2, 3, figsize=(18, 12))
axes = axes.flatten()
results = {}

total_simulations = len(param1_values) * len(param2_values) * len(param3_values)
print("Iniciando simulações do controle SDRE Discreto para sistema 6x6...")
print(f"Total de simulações: {total_simulations}")
print(f"Período de amostragem: {Ts} segundos")

param_combinations = list(itertools.product(param1_values, param2_values, param3_values))

with tqdm(param_combinations, desc="Simulações", unit="sim", position=0) as pbar:
    for param1, param2, param3 in pbar:
        pbar.set_description(f"α₁={param1:.2f}, α₂={param2:.2f}, α₃={param3:.2f}")
        
        max_real_eigenvalue_for_sim = [-np.inf]
        eigenvalues_over_time = []
        
        states = np.zeros((7, num_steps))
        states[:6, 0] = x0
        states[6, 0] = 0.0
        
        # Parâmetros constantes para esta simulação
        const_params = (Ixx, Iyy, Izz, Ir, Omega_r, Ts)

        for k in range(1, num_steps):
            x = states[:6, k-1]
            pk, qk, rk, phik, thetak, psik = x
            
            # Evitar instabilidade em tan(theta) e 1/cos(theta)
            if abs(np.cos(thetak)) < 1e-6:
                thetak = np.sign(thetak) * (np.pi/2 - 1e-6)

            # Argumentos para as funções pré-calculadas
            func_args = (pk, qk, rk, phik, thetak, param1, param2, param3, *const_params)

            # Discretização usando as funções pré-calculadas (MUITO MAIS RÁPIDO)
            A_d = Ad_func(*func_args)
            B_d = Bd_func(*func_args)
            
            Q_d = Q * Ts
            R_d = R
            
            try:
                P = solve_discrete_are(A_d, B_d, Q_d, R_d)
            except (np.linalg.LinAlgError, ValueError):
                eigenvalues_over_time.append(0.0)
                states[:, k] = states[:, k-1]
                continue
                
            K = np.linalg.inv(B_d.T @ P @ B_d + R_d) @ B_d.T @ P @ A_d
            u = -K @ x.reshape(-1, 1)
            
            eigenvalues = np.linalg.eigvals(A_d - B_d @ K)
            current_max_eig = np.max(np.abs(eigenvalues))
            if current_max_eig > max_real_eigenvalue_for_sim[0]:
                max_real_eigenvalue_for_sim[0] = current_max_eig
            
            eigenvalues_over_time.append(current_max_eig)
            
            x_next = A_d @ x.reshape(-1, 1) + B_d @ u + 0*ruido_vento[:, k].reshape(-1, 1)
            states[:6, k] = x_next.flatten()
            
            cost_increment = (x.T @ Q @ x + u.T @ R @ u.reshape(-1,1))[0, 0] * Ts
            states[6, k] = states[6, k-1] + cost_increment
        
        final_cost = states[6, -1]
        key = (param1, param2, param3)
        results[key] = {
            't': t_eval,
            'states': states[:6, :],
            'cost': final_cost,
            'max_eig': max_real_eigenvalue_for_sim[0],
            'eigenvalues_time': np.array(eigenvalues_over_time)
        }
        
        pbar.set_postfix(J=f"{final_cost:.8f}", λ_max=f"{max_real_eigenvalue_for_sim[0]:.4f}")

# --- Exportação para Excel e Geração de Gráficos ---
if results:
    print("\n--- Processando resultados e exportando para Excel ---")
    data = []
    for params, result in results.items():
        data.append({
            'param1_alpha1': params[0],
            'param2_alpha2': params[1], 
            'param3_alpha3': params[2],
            'custo_total': result['cost'],
            'maior_autovalor_abs': result['max_eig']
        })
    df = pd.DataFrame(data)
    df = df.sort_values('custo_total')
    excel_filename = 'python\\outputs\\resultados_simulacao_sdre_discreto.xlsx'
    try:
        df.to_excel(excel_filename, index=False, sheet_name='Resultados')
        print(f"Arquivo Excel salvo como: {excel_filename}")
    except Exception as e:
        print(f"Erro ao exportar para Excel: {e}")
        csv_filename = 'resultados_simulacao_sdre_discreto.csv'
        df.to_csv(csv_filename, index=False)
        print(f"Arquivo CSV salvo como alternativa: {csv_filename}")

    cost_list = [results[key]['cost'] for key in results.keys()]
    min_cost = min(cost_list)
    max_cost = max(cost_list)
    best_params = None
    worst_params = None
    for params, result in results.items():
        if result['cost'] == min_cost:
            best_params = params
        if result['cost'] == max_cost:
            worst_params = params

    print("\n--- Resumo dos Custos Finais ---")
    print(f"Melhor caso: α₁={best_params[0]:.2f}, α₂={best_params[1]:.2f}, α₃={best_params[2]:.2f}, J={min_cost:.8f}, |λ|_max={results[best_params]['max_eig']:.6f}")
    print(f"Pior caso: α₁={worst_params[0]:.2f}, α₂={worst_params[1]:.2f}, α₃={worst_params[2]:.2f}, J={max_cost:.8f}, |λ|_max={results[worst_params]['max_eig']:.6f}")
    
    print("\n--- Gerando gráficos ---")
    state_names = ['p(t)', 'q(t)', 'r(t)', 'φ(t)', 'θ(t)', 'ψ(t)']
    fig2, axes2 = plt.subplots(2, 3, figsize=(18, 12))
    axes2 = axes2.flatten()
    for i in range(6):
        axes2[i].plot(results[best_params]['t'], results[best_params]['states'][i], 
                     'b-', linewidth=2, 
                     label=f'Melhor: α_1={best_params[0]:.2f}, α_2={best_params[1]:.2f}, α_3={best_params[2]:.2f} (J={min_cost:.8f}, |λ|_max={results[best_params]["max_eig"]:.4f})')
        axes2[i].plot(results[worst_params]['t'], results[worst_params]['states'][i], 
                     'r-', linewidth=2,
                     label=f'Pior: α_1={worst_params[0]:.2f}, α_2={worst_params[1]:.2f}, α_3={worst_params[2]:.2f} (J={max_cost:.8f}, |λ|_max={results[worst_params]["max_eig"]:.4f})')
        axes2[i].set_title(f'Comparação - {state_names[i]}', fontsize=12)
        axes2[i].set_xlabel('Tempo (s)', fontsize=10)
        axes2[i].legend(fontsize=8)
        axes2[i].grid(True)
    plt.tight_layout()
    plt.show()
else:
    print("Nenhum resultado válido obtido nas simulações.")
    # Cria um DataFrame vazio mesmo sem resultados
    df_empty = pd.DataFrame(columns=['param1_alpha1', 'param2_alpha2', 'param3_alpha3', 'custo_total'])
    df_empty.to_excel('resultados_simulacao_sdre_discreto_vazio.xlsx', index=False)
    print("Arquivo Excel vazio criado: resultados_simulacao_sdre_discreto_vazio.xlsx")