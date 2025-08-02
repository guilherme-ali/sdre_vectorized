import numpy as np
from scipy.linalg import solve_discrete_are
import matplotlib.pyplot as plt
from tqdm import tqdm
import itertools
import pandas as pd

# --- Parâmetros do Sistema ---
# Parâmetros físicos do VANT
Ixx = 0.00184  # Momento de inércia em x
Iyy = 0.00225  # Momento de inércia em y  
Izz = 0.00338  # Momento de inércia em z
Ir = 0.00001   # Momento de inércia do rotor
Omega_r = 1000.0  # Velocidade angular do rotor

# Funções trigonométricas
def C_phi(phi):
    return np.cos(phi)

def S_phi(phi):
    return np.sin(phi)

def C_theta(theta):
    return np.cos(theta)

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
amostras = 5
param1_values = np.linspace(0, 1, amostras)  # Parâmetro 1 (para p)
param2_values = np.linspace(0, 1, amostras)  # Parâmetro 2 (para q) 
param3_values = np.linspace(0, 1, amostras)  # Parâmetro 3 (para r)

# Calcula o total de simulações
total_simulations = len(param1_values) * len(param2_values) * len(param3_values)

# --- Simulação e Geração de Gráficos ---
plt.style.use('seaborn-v0_8-whitegrid')
fig, axes = plt.subplots(2, 3, figsize=(18, 12))
axes = axes.flatten()
results = {}

print("Iniciando simulações do controle SDRE Discreto para sistema 6x6...")
print(f"Total de simulações: {total_simulations}")
print(f"Período de amostragem: {Ts} segundos")

# Cria todas as combinações de parâmetros
param_combinations = list(itertools.product(param1_values, param2_values, param3_values))

# Loop através dos três parâmetros com barra de progresso
with tqdm(param_combinations, desc="Simulações", unit="sim", position=0) as pbar:
    for param1, param2, param3 in pbar:
        # Atualiza a descrição da barra de progresso
        pbar.set_description(f"α₁={param1:.2f}, α₂={param2:.2f}, α₃={param3:.2f}")
        
        # Variável para rastrear o maior autovalor durante esta simulação específica
        max_real_eigenvalue_for_sim = [-np.inf]
        # Lista para armazenar todos os autovalores ao longo do tempo
        eigenvalues_over_time = []
        
        # Inicializa vetor de estados e custo
        states = np.zeros((7, num_steps))  # 6 estados + custo
        states[:6, 0] = x0
        states[6, 0] = 0.0  # Custo inicial
        
        # Simulação discretizada
        for k in range(1, num_steps):
            # Estado atual
            x = states[:6, k-1].reshape(-1, 1)
            p, q, r, phi, theta, psi = x.flatten()
            
            # Calcula as funções trigonométricas
            s_phi = S_phi(phi)
            c_phi = C_phi(phi)
            c_theta = C_theta(theta)
            
            # Evitar divisão por zero para tan(theta) e 1/cos(theta)
            if abs(c_theta) < 1e-6:
                c_theta = 1e-6 * np.sign(c_theta)
            t_theta = np.sin(theta) / c_theta
            
            # Matriz A(x) contínua
            # Linha 1: p_dot
            a12 = param1 * ((Iyy - Izz) / Ixx) * r - (Ir * Omega_r / Ixx)
            a13 = (1 - param1) * ((Iyy - Izz) / Ixx) * q

            # Linha 2: q_dot
            a21 = param2 * ((Izz - Ixx) / Iyy) * r + (Ir * Omega_r / Iyy)
            a23 = (1 - param2) * ((Izz - Ixx) / Iyy) * p

            # Linha 3: r_dot
            a31 = param3 * ((Ixx - Iyy) / Izz) * q
            a32 = (1 - param3) * ((Ixx - Iyy) / Izz) * p
            
            A_c = np.array([
                [0.0, a12, a13, 0.0, 0.0, 0.0],
                [a21, 0.0, a23, 0.0, 0.0, 0.0],
                [a31, a32, 0.0, 0.0, 0.0, 0.0],
                [1.0, s_phi * t_theta, c_phi * t_theta, 0.0, 0.0, 0.0],
                [0.0, c_phi, -s_phi, 0.0, 0.0, 0.0],
                [0.0, s_phi / c_theta, c_phi / c_theta, 0.0, 0.0, 0.0]
            ])
            
            # Discretização do sistema (aproximação de segunda ordem - Taylor)
            A_d = np.eye(6) + A_c * Ts + (A_c @ A_c * Ts**2) / 2.0
            B_d = (np.eye(6) * Ts + A_c * Ts**2 / 2.0) @ B
            
            # Ajuste das matrizes de custo para o sistema discreto
            Q_d = Q * Ts
            R_d = R
            
            # Tenta resolver a Equação de Riccati discreta
            try:
                P = solve_discrete_are(A_d, B_d, Q_d, R_d)
            except (np.linalg.LinAlgError, ValueError):
                eigenvalues_over_time.append(0.0)  # Adiciona 0 em caso de erro
                states[:, k] = states[:, k-1]  # Mantém o estado anterior
                continue
                
            # Calcula a entrada de controle SDRE discreta
            K = np.linalg.inv(B_d.T @ P @ B_d + R_d) @ B_d.T @ P @ A_d
            u = -K @ x
            
            # Calcula os autovalores de A(x) - B @ K e atualiza o máximo da simulação
            eigenvalues = np.linalg.eigvals(A_d - B_d @ K)
            current_max_eig = np.max(np.abs(eigenvalues))  # Maior valor absoluto para sistemas discretos
            if current_max_eig > max_real_eigenvalue_for_sim[0]:
                max_real_eigenvalue_for_sim[0] = current_max_eig
            
            # Armazena o maior autovalor atual
            eigenvalues_over_time.append(current_max_eig)
            
            # Próximo estado usando o modelo discreto
            x_next = A_d @ x + B_d @ u
            states[:6, k] = x_next.flatten()
            
            # Calcula o custo incremental
            cost_increment = (x.T @ Q @ x + u.T @ R @ u)[0, 0] * Ts
            states[6, k] = states[6, k-1] + cost_increment
        
        # Armazena os resultados
        final_cost = states[6, -1]
        key = (param1, param2, param3)
        results[key] = {
            't': t_eval,
            'states': states[:6, :],
            'cost': final_cost,
            'max_eig': max_real_eigenvalue_for_sim[0],
            'eigenvalues_time': np.array(eigenvalues_over_time)
        }
        
        # Atualiza o sufixo da barra de progresso com o custo e o maior autovalor
        pbar.set_postfix(J=f"{final_cost:.8f}", λ_max=f"{max_real_eigenvalue_for_sim[0]:.4f}")

# --- Exportação para Excel ---
if results:
    print("\n--- Processando resultados e exportando para Excel ---")
    
    # Prepara os dados para o DataFrame
    data = []
    for params, result in results.items():
        data.append({
            'param1_alpha1': params[0],
            'param2_alpha2': params[1], 
            'param3_alpha3': params[2],
            'custo_total': result['cost'],
            'maior_autovalor_abs': result['max_eig']
        })
    
    # Cria o DataFrame
    df = pd.DataFrame(data)
    
    # Ordena por custo total (menor para maior)
    df = df.sort_values('custo_total')
    
    # Define o nome do arquivo Excel
    excel_filename = 'python\\outputs\\resultados_simulacao_sdre_discreto.xlsx'
    
    try:
        # Exporta para Excel
        df.to_excel(excel_filename, index=False, sheet_name='Resultados')
        print(f"Arquivo Excel salvo como: {excel_filename}")
        print(f"Total de registros exportados: {len(df)}")
    except Exception as e:
        print(f"Erro ao exportar para Excel: {e}")
        print("Verifique se você tem a biblioteca openpyxl instalada: pip install openpyxl")
        
        # Como alternativa, salva em CSV
        csv_filename = 'resultados_simulacao_sdre_discreto.csv'
        df.to_csv(csv_filename, index=False)
        print(f"Arquivo CSV salvo como alternativa: {csv_filename}")

    # Encontra os melhores e piores parâmetros
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

    # Resumo dos resultados
    print("\n--- Resumo dos Custos Finais ---")
    print(f"Melhor caso: α₁={best_params[0]:.2f}, α₂={best_params[1]:.2f}, α₃={best_params[2]:.2f}, J={min_cost:.8f}, |λ|_max={results[best_params]['max_eig']:.6f}")
    print(f"Pior caso: α₁={worst_params[0]:.2f}, α₂={worst_params[1]:.2f}, α₃={worst_params[2]:.2f}, J={max_cost:.8f}, |λ|_max={results[worst_params]['max_eig']:.6f}")
    
    print("\nTodos os resultados:")
    for params in sorted(results.keys()):
        cost = results[params]['cost']
        max_eig = results[params]['max_eig']
        print(f"α₁={params[0]:.2f}, α₂={params[1]:.2f}, α₃={params[2]:.2f}: J={cost:.8f}, |λ|_max={max_eig:.6f}")

else:
    print("Nenhum resultado válido obtido nas simulações.")
    # Cria um DataFrame vazio mesmo sem resultados
    df_empty = pd.DataFrame(columns=['param1_alpha1', 'param2_alpha2', 'param3_alpha3', 'custo_total'])
    df_empty.to_excel('resultados_simulacao_sdre_discreto_vazio.xlsx', index=False)
    print("Arquivo Excel vazio criado: resultados_simulacao_sdre_discreto_vazio.xlsx")

# --- Geração dos Gráficos ---
print("\n--- Gerando gráficos ---")

if results:
    # Plota cada estado em um subplot
    state_names = ['p(t)', 'q(t)', 'r(t)', 'φ(t)', 'θ(t)', 'ψ(t)']
    for params, result in results.items():
        for i in range(6):
            axes[i].plot(result['t'], result['states'][i], alpha=0.7,
                       label=f'α_1={params[0]:.1f}, α_2={params[1]:.1f}, α_3={params[2]:.1f}')
            axes[i].set_title(f'Estado {state_names[i]}', fontsize=12)
            axes[i].set_xlabel('Tempo (s)', fontsize=10)
            axes[i].grid(True)

    # Finaliza os gráficos
    for ax in axes:
        if len(results) <= 10:  # Só mostra legenda se não houver muitas curvas
            ax.legend(fontsize=8)

    plt.tight_layout()
    plt.show()

    # Gráfico de comparação dos melhores vs piores resultados
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