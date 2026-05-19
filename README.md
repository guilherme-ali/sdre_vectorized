# SDRE_VECTORIZED

Controle de atitude de quadricóptero em **ESP32-S2** com **SDRE-LQR** (State-Dependent Riccati Equation) recalculado em tempo real. O sistema resolve a DARE a cada ciclo via uma das implementações otimizadas disponíveis (SDA, ADDA, etc.), com PID como alternativa selecionável.

## Visão geral

- **Controle SDRE em tempo real** — ganhos K recalculados ciclo a ciclo a partir de A(x)
- **Multiplos solvers DARE** — 7 algoritmos implementados em `lib/AUTOLQR/` (benchmark abaixo)
- **PID alternativo** — controlador PID compatível com a mesma interface, selecionável via flag
- **Madgwick (AHRS)** — estimação de orientação a partir de MPU6050 (+ QMC5883L opcional)
- **Butterworth digital** — filtro passa-baixa por eixo antes do AHRS (anti-aliasing em 200 Hz)
- **Riccati em FreeRTOS task** — DARE desacoplada do loop principal (5 ms) via mutex
- **Telemetria em RAM + LittleFS** — buffer circular de 1000 amostras, persistido em flash ao desarmar
- **WiFi/UDP (CRTP)** — protocolo Crazyflie, compatível com o app **ESP-Drone** (Espressif)
- **Failsafe de tilt** — desarma e trava motores em inclinação > 60° (zona singular `1/cos(pitch)`)

## Hardware

| Componente       | Modelo                        | Observação                       |
|------------------|-------------------------------|----------------------------------|
| Microcontrolador | ESP32-S2 Saola-1              | 240 MHz, single-core             |
| IMU              | MPU6050                       | I2C (SDA=GPIO11, SCL=GPIO10)     |
| Magnetômetro     | QMC5883L (opcional, 9-DOF)    | Mesmo barramento I2C             |
| Motores          | 4× brushed, hélice 55 mm      | PWM 25 kHz, 10-bit               |
| Bateria          | LiPo 1S 3.7 V                 | Monitorada por ADC (GPIO2)       |
| LEDs status      | RGB + branco (always-on HW)   | GPIO 7/8/9                       |

## Estrutura do projeto

```
SDRE_VECTORIZED/
├── platformio.ini            # Configuração PlatformIO (ESP32-S2, lib_deps)
├── src/
│   └── main.cpp              # Loop principal + SDRETask + montagem das matrizes
├── lib/
│   ├── AUTOLQR/              # 7 solvers DARE (SDA, ADDA, ASDA, ...) + operações matriciais
│   ├── KalmanFilter/         # Filtro de Kalman linear (opcional — alternativa ao Madgwick)
│   ├── PIDController/        # PID compatível com interface SDRE (controlador alternativo)
│   ├── BiquadFilter/         # Butterworth de 2ª ordem para sinais da IMU
│   ├── Telemetry/            # Buffer circular em RAM + persistência em LittleFS
│   ├── MotorControl/         # PWM dos 4 ESCs + armar/desarmar + mapeamento ω² → throttle
│   ├── WiFiComm/             # Servidor UDP CRTP (compatível com app ESP-Drone)
│   └── utils/                # Drivers MPU6050/QMC5883L, LEDs/bateria, alocação X-quad
├── python/
│   ├── atitude_sim.py            # Simulação do controle de atitude
│   ├── compara_solvers.py        # Comparação dos solvers DARE
│   ├── plot_telemetry.py         # Plota CSV exportado pela telemetria do drone
│   ├── simulador/                # Notebook (Jupyter) com simulações exploratórias
│   ├── matriz_otima/             # Busca/análise dos parâmetros ótimos de Q,R
│   ├── execucao_otima/           # Execução da matriz ótima encontrada
│   └── outputs/                  # Resultados (PNG, MP4, XLSX, CSV)
├── test/                     # Calibração de sensores, benchmarks, testes unitários
└── docs/                     # Guias auxiliares (LED, calibração, WiFi, quick start)
```

## Configuração rápida (flags em `src/main.cpp`)

```cpp
const bool DEBUG_MODE       = false; // true: prints detalhados; false: Serial Plotter
const bool PRINT_TELEMETRY  = false; // stream contínuo roll/pitch/yaw/p/q/r
const bool USE_MAGNETOMETER = false; // 9-DOF (QMC5883L) ou 6-DOF (só accel+gyro)
const int  CONTROLLER_TYPE  = 0;     // 0 = SDRE, 1 = PID
const bool USE_ASYNC_SDRE   = true;  // true: Riccati em FreeRTOS task; false: síncrono
```

## Compilação

Pré-requisitos: [PlatformIO](https://platformio.org/) (extensão VS Code recomendada), Python 3.8+ para simulações.

```bash
pio run                    # compila
pio run --target upload    # envia para o ESP32-S2
pio device monitor         # monitor serial (115200 baud)
```

## Benchmark dos solvers DARE

ESP32-S2 @ 240 MHz, sistema 6 estados × 3 controles, **800 000 execuções** sob dinâmica real de quadricóptero (resultados publicados em CBA 2026):

### Desempenho temporal

| Método         | Média (μs)       | σ (μs)   | Pior caso (μs) | Falhas / 800k | Iterações (méd ± σ) | Iter. (pior) |
|----------------|------------------|----------|----------------|---------------|---------------------|--------------|
| SDA-SS         | **8 413,26**     | 541,55   | 10 902         | 55 349 (6,9 %) | 7,79 ± 0,52         | 10           |
| **SDA (base)** | **8 663,59**     | **146,49** | **8 750**    | **0**          | 7,99 ± 0,13         | 8            |
| SDA-Scaled     | 8 854,91         | 327,95   | 11 024         | 48 302 (6,0 %) | 8,08 ± 0,31         | 10           |
| ASDA           | 9 114,64         | **24,64** | 9 180         | 0              | 8,00 ± 0,00         | 8            |
| SDA-ADDA       | 10 754,63        | 556,55   | 13 654         | 40 314 (5,0 %) | 7,96 ± 0,42         | 10           |
| Iterativo      | 11 912,00        | 2 868,74 | 16 884         | 0              | 22,49 ± 5,64        | 32           |
| Van Dooren     | 39 281,13        | 3 637,45 | 126 877        | 0              | 1,00 ± 0,00         | 1            |

### Precisão (erro RMS dos ganhos K vs. método iterativo de referência)

| Método         | Erro RMS                  |
|----------------|---------------------------|
| **SDA (base)** | **9,36 × 10⁻⁷**           |
| ASDA           | 1,92 × 10⁻⁵               |
| Van Dooren     | 5,53 × 10⁻⁵               |
| SDA-ADDA       | 1,85 × 10⁻⁴               |
| SDA-SS         | 3,22 × 10⁻⁴               |
| SDA-Scaled     | 3,43 × 10⁻⁴               |
| Iterativo      | — (referência)            |

> **Recomendação:** **SDA (base)** — melhor balanço velocidade × precisão × robustez. Zero falhas em 800 k execuções, menor erro RMS (~10⁻⁷), pior caso a 8 750 μs cabe folgado no ciclo de controle a 80 Hz (12 500 μs).
>
> **ASDA** como alternativa quando previsibilidade temporal é crítica (σ de apenas 24,64 μs).
>
> **Evitar** SDA-SS, SDA-Scaled e SDA-ADDA em malha de tempo real: ~5–7 % de falha sob excitações estocásticas do voo. **Van Dooren** descartado por custo: ~4,5× mais lento que o SDA base.

Detalhes, derivações e API em [`lib/AUTOLQR/README.md`](lib/AUTOLQR/README.md). Referências bibliográficas dos algoritmos em [`docs/REFERENCES.md`](docs/REFERENCES.md#2-solvers-dare-equação-algébrica-de-riccati-discreta).

## Modelo do sistema

**Estado** (6): $x = [\phi,\ \theta,\ \psi,\ p,\ q,\ r]^T$ — ângulos de Euler + taxas no corpo

**Controle** (3): $u = [\tau_x,\ \tau_y,\ \tau_z]^T$ — torques nos eixos do corpo

Dinâmica não-linear linearizada por SDRE:

$$\dot{x} = A(x)\,x + B\,u$$

com $A(x)$ recalculada a cada ciclo. `updateSystemMatrix()` em `main.cpp` monta $A_d, B_d, Q_d, R_d$ analiticamente (Taylor 2ª/3ª ordem) explorando a esparsidade do problema — ~14× mais rápido que multiplicação matricial genérica.

Controle aplicado:

$$u = -K\,x + K_r\,r,\quad K_r = -K[:,\,0\!:\!m]$$

## Simulações Python

```bash
cd python

python atitude_sim.py                                # Simulação do controle de atitude
python compara_solvers.py                            # Comparação dos solvers DARE
python plot_telemetry.py outputs/telem.csv           # Plota CSV exportado pelo drone
python matriz_otima/busca_parametros_otimos_sdre.py  # Busca Q,R ótimos
```

## Documentação

- [`docs/REFERENCES.md`](docs/REFERENCES.md) — **referências bibliográficas de todos os métodos**
- [`lib/AUTOLQR/README.md`](lib/AUTOLQR/README.md) — solvers DARE, API, benchmarks
- [`lib/KalmanFilter/README.md`](lib/KalmanFilter/README.md) — filtro de Kalman linear (opcional)
- [`lib/PIDController/README.md`](lib/PIDController/README.md) — controlador PID alternativo
- [`lib/WiFiComm/README.md`](lib/WiFiComm/README.md) — protocolo CRTP e ESP-Drone
- [`docs/QUICK_TEST.md`](docs/QUICK_TEST.md) — teste rápido de conexão e controle
- [`docs/MOTOR_CALIBRATION_GUIDE.md`](docs/MOTOR_CALIBRATION_GUIDE.md) — calibração de coeficiente de empuxo
- [`docs/WIFI_INTEGRATION.md`](docs/WIFI_INTEGRATION.md) — setup do app ESP-Drone
- [`docs/LED_BATTERY_GUIDE.md`](docs/LED_BATTERY_GUIDE.md) — indicadores LED e thresholds de bateria

## Contribuições

Issues e PRs bem-vindos. Para PRs significativos, abra uma issue antes para discussão.

---

*Pesquisa em controle adaptativo de VANTs — SDRE em tempo real para sistemas embarcados.*
