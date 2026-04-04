# SDRE_VECTORIZED

Sistema de controle SDRE (State-Dependent Riccati Equation) vetorizado para estabilização de atitude de VANTs (Veículos Aéreos Não Tripulados) em tempo real, implementado em ESP32.

## 📋 Descrição

Este projeto implementa um controlador LQR adaptativo baseado na técnica SDRE para controle de atitude de quadricópteros. O sistema resolve a equação algébrica de Riccati discreta (DARE) em tempo real utilizando múltiplos algoritmos otimizados para microcontroladores.

### Características Principais

- **Controle SDRE em tempo real** - Recálculo dos ganhos a cada ciclo de controle
- **Múltiplos solvers DARE** - 7 algoritmos diferentes para resolver a equação de Riccati
- **Filtro de Kalman** - Estimação de estados com fusão sensorial (IMU + Magnetômetro)
- **Comunicação WiFi** - Telemetria e ajuste de parâmetros em tempo real
- **Otimizado para ESP32** - Uso eficiente de memória e processamento

## 🔧 Hardware Suportado

- **Microcontrolador**: ESP32 (240MHz dual-core)
- **IMU**: MPU6050 / MPU9250
- **Magnetômetro**: HMC5883L / QMC5883L (opcional)
- **ESCs**: PWM padrão ou OneShot125
- **Frame**: Quadricóptero em configuração X

## 📁 Estrutura do Projeto

```
SDRE_VECTORIZED/
├── src/
│   └── main.cpp              # Benchmark dos solvers DARE
├── lib/
│   ├── AUTOLQR/              # Biblioteca principal de controle LQR
│   │   ├── AutoLQR.cpp/h     # Classe principal com solvers DARE
│   │   ├── KalmanFilter.cpp/h # Filtro de Kalman para estimação
│   │   └── MatrixOperations.cpp/h # Operações matriciais otimizadas
│   ├── MotorControl/         # Controle de motores e ESCs
│   ├── WiFiComm/             # Comunicação WiFi para telemetria
│   └── utils/                # Utilitários (LED, sensores, etc.)
├── python/                   # Scripts de simulação e análise
│   ├── sim_control.py        # Simulação do sistema de controle
│   ├── compara_solvers.py    # Comparação de solvers em Python
│   └── outputs/              # Gráficos e resultados
└── test/                     # Testes e benchmarks
```

## 🚀 Instalação

### Pré-requisitos

- [PlatformIO](https://platformio.org/) (VS Code Extension recomendada)
- ESP32 DevKit ou similar
- Python 3.8+ (para simulações)

### Compilação

```bash
# Clone o repositório
git clone https://github.com/seu-usuario/SDRE_VECTORIZED.git
cd SDRE_VECTORIZED

# Compile e envie para o ESP32
pio run --target upload

# Monitor serial
pio device monitor
```

## 📊 Benchmark dos Solvers DARE

O projeto inclui implementações de 7 métodos para resolver a DARE, com benchmark integrado:

| Método | Tempo Médio (μs) | Desvio Padrão | Erro RMS vs Referência |
|--------|------------------|---------------|------------------------|
| **SDA** | 8303.78 | 12.12 | 4.99e-05 |
| **SDA-ss** | 8503.93 | 12.91 | 4.98e-05 |
| **ASDA** | 9020.53 | 11.46 | 1.69e-05 |
| **SDA Scaled** | 8666.85 | 12.05 | 4.69e-05 |
| **ADDA** | 8277.27 | 13.36 | 4.99e-05 |
| **Van Dooren** | 41915.83 | 5825.84 | 4.05e-05 |
| **Iterativo** | 13333.36 | 2125.54 | Referência |

*Testes realizados em ESP32 @ 240MHz, sistema 6 estados x 3 controles, 100 iterações*

### Recomendações de Uso

- **ADDA**: Melhor relação velocidade/precisão para uso em tempo real
- **SDA**: Boa opção com menor variância
- **Iterativo**: Alta precisão, ideal como referência ou com warm-start

## 🎮 Uso Básico

```cpp
#include <AutoLQR.h>

// Criar controlador (6 estados, 3 controles)
AutoLQR lqr(6, 3);

// Configurar matrizes de custo
lqr.setCostMatrices(Q, R);

// Loop de controle
void loop() {
    // Atualizar matrizes do sistema (SDRE)
    lqr.setStateMatrix(Ad);
    lqr.setInputMatrix(Bd);
    
    // Calcular ganhos (escolher método)
    lqr.computeGains("ADDA");  // ou "SDA", "ITERATIVE", etc.
    
    // Atualizar estado e calcular controle
    lqr.updateState(x);
    lqr.updateReference(ref);
    lqr.calculateControl(u);
}
```

## 📚 Documentação

- [AUTOLQR Library](lib/AUTOLQR/README.md) - Documentação da biblioteca de controle
- [Motor Calibration](lib/MotorControl/MOTOR_CALIBRATION_GUIDE.md) - Guia de calibração dos motores
- [WiFi Integration](WIFI_INTEGRATION.md) - Configuração da comunicação WiFi
- [LED Guide](LED_BATTERY_GUIDE.md) - Indicadores LED e bateria

## 🧮 Modelo do Sistema

O sistema de atitude do quadricóptero é modelado como:

**Estados** (6): `[φ, θ, ψ, p, q, r]` (ângulos de Euler + velocidades angulares)

**Controles** (3): `[τx, τy, τz]` (torques nos eixos do corpo)

A matriz A é dependente do estado atual (SDRE), recalculada a cada ciclo.

## 📈 Simulações Python

```bash
cd python

# Simulação do controle de atitude
python atitude_sim.py

# Comparação dos solvers
python compara_solvers.py

# Busca de parâmetros ótimos
python matriz_otima/busca_parametros_otimos_sdre.py
```

## 🤝 Contribuições

Contribuições são bem-vindas! Por favor, abra uma issue para discussão antes de submeter PRs significativos.

## 📞 Contato

Para dúvidas ou sugestões, abra uma issue no repositório.

---

*Desenvolvido para pesquisa em controle de VANTs - SDRE em tempo real para sistemas embarcados*