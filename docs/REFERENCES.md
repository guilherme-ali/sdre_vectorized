# Referências bibliográficas

Referências consolidadas para cada método/algoritmo implementado no projeto. Organizadas por área para facilitar consulta.

---

## 1. Controle SDRE (State-Dependent Riccati Equation)

A técnica SDRE generaliza o LQR linear para sistemas não-lineares parametrizando a planta como $\dot{x} = A(x)\,x + B(x)\,u$ e resolvendo a equação de Riccati a cada ciclo com $A$ avaliado no estado atual.

1. **Çimen, T.** (2008). *State-Dependent Riccati Equation (SDRE) Control: A Survey*. IFAC Proceedings Volumes, 41(2), 3761–3775.
   — Survey de referência. Cobre formulação, propriedades de estabilidade e armadilhas práticas (ambiguidade de fatoração $A(x)$).

2. **Mracek, C. P., & Cloutier, J. R.** (1998). *Control designs for the nonlinear benchmark problem via the state-dependent Riccati equation method*. International Journal of Robust and Nonlinear Control, 8(4-5), 401–433.
   — Aplicação clássica de SDRE; benchmarks padrão da literatura.

3. **Pearson, J. D.** (1962). *Approximation Methods in Optimal Control*. Journal of Electronics and Control, 13(5), 453–469.
   — Trabalho seminal que motivou o uso de SDRE.

4. **Cloutier, J. R.** (1997). *State-dependent Riccati equation techniques: an overview*. Proceedings of the American Control Conference, 932–936.
   — Visão geral introdutória.

**Aplicação a quadricópteros (referência direta deste projeto):**

5. **Voos, H.** (2009). *Nonlinear control of a quadrotor micro-UAV using feedback-linearization*. IEEE International Conference on Mechatronics.

6. **Babaei, A. R., & Malekzadeh, M.** (2019). *Robust backstepping control of a quadrotor UAV using extended Kalman filter*. Mechanical Engineering Journal.

---

## 2. Solvers DARE (Equação Algébrica de Riccati Discreta)

A DARE é o núcleo computacional do SDRE: $A^T P A - P - A^T P B (R + B^T P B)^{-1} B^T P A + Q = 0$. O projeto implementa 7 métodos:

### SDA / ADDA / ASDA / SDA-ss / SDA Scaled (família de doubling algorithms)

1. **Chu, E. K.-W., Fan, H.-Y., Lin, W.-W., & Wang, C.-S.** (2004). *Structure-preserving algorithms for periodic discrete-time algebraic Riccati equations*. International Journal of Control, 77(8), 767–788.
   — Base do SDA (Structure-preserving Doubling Algorithm) e da variante com single shift (SDA-ss).

2. **Chu, E. K.-W., Fan, H.-Y., & Lin, W.-W.** (2005). *A structure-preserving doubling algorithm for continuous-time algebraic Riccati equations*. Linear Algebra and its Applications, 396, 55–80.
   — Análise de convergência quadrática do SDA.

3. **Lin, W.-W., & Xu, S.-F.** (2006). *Convergence analysis of structure-preserving doubling algorithms for Riccati-type matrix equations*. SIAM Journal on Matrix Analysis and Applications, 28(1), 26–39.
   — Prova rigorosa de convergência; base teórica do ADDA.

4. **Huang, T.-M., Li, R.-C., & Lin, W.-W.** (2018). *Structure-Preserving Doubling Algorithms for Nonlinear Matrix Equations*. SIAM (livro).
   — Tratado moderno cobrindo SDA, ADDA, ASDA e variantes escaladas.

### Van Dooren (pencil simpléctico estendido)

5. **Van Dooren, P.** (1981). *A generalized eigenvalue approach for solving Riccati equations*. SIAM Journal on Scientific and Statistical Computing, 2(2), 121–135.
   — Método via decomposição QZ generalizada do pencil Hamiltoniano. Mais lento, mas numericamente robusto.

6. **Laub, A. J.** (1979). *A Schur method for solving algebraic Riccati equations*. IEEE Transactions on Automatic Control, 24(6), 913–921.
   — Variante clássica usando Schur ordinária; alternativa ao Van Dooren.

### Iterativo

7. **Hewer, G. A.** (1971). *An iterative technique for the computation of the steady state gains for the discrete optimal regulator*. IEEE Transactions on Automatic Control, 16(4), 382–384.
   — Iteração de Newton sobre a DARE; converge quadraticamente perto da solução.

### Regra de Bryson (escolha de Q e R)

8. **Bryson, A. E., & Ho, Y.-C.** (1975). *Applied Optimal Control: Optimization, Estimation, and Control*. Hemisphere Publishing.
   — Regra prática: $Q_{ii} = 1/(\text{máx aceitável de } x_i)^2$, idem $R_{ii}$. Usada em `main.cpp`.

---

## 3. Discretização de matrizes contínuas

Em `updateSystemMatrix()` o projeto usa expansão de Taylor de 2ª/3ª ordem em $\Delta t$ para discretizar $A, B$ e os termos cruzados de $Q, R$, explorando a esparsidade analítica do problema.

1. **Van Loan, C. F.** (1978). *Computing integrals involving the matrix exponential*. IEEE Transactions on Automatic Control, 23(3), 395–404.
   — Método canônico (matriz exponencial). Inviável aqui pelo custo; Taylor truncado é justificável dado o $\Delta t = 5$ ms pequeno.

2. **Franklin, G. F., Powell, J. D., & Workman, M.** (1998). *Digital Control of Dynamic Systems*, 3ª ed. Addison-Wesley.
   — Cap. 4: discretização de modelos contínuos para controle digital, incluindo $Q_d, R_d$ a partir de $Q, R$ contínuos.

---

## 4. AHRS — Estimação de Orientação

### Filtro Madgwick (usado em `main.cpp`)

1. **Madgwick, S. O. H.** (2010). *An efficient orientation filter for inertial and inertial/magnetic sensor arrays*. Internal Report, University of Bristol.
   — Algoritmo original. Disponível em <https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/>

2. **Madgwick, S. O. H., Harrison, A. J. L., & Vaidyanathan, R.** (2011). *Estimation of IMU and MARG orientation using a gradient descent algorithm*. IEEE International Conference on Rehabilitation Robotics, 1–7.
   — Versão publicada e revisada.

### Filtro de Kalman (lib/KalmanFilter/, alternativa)

Ver [`lib/KalmanFilter/README.md`](../lib/KalmanFilter/README.md) — referências detalhadas (Kalman 1960, Welch & Bishop, Simon 2006, etc.).

### Madgwick vs Mahony vs EKF

3. **Mahony, R., Hamel, T., & Pflimlin, J.-M.** (2008). *Nonlinear complementary filters on the special orthogonal group*. IEEE Transactions on Automatic Control, 53(5), 1203–1218.
   — Filtro complementar não-linear, principal alternativa ao Madgwick.

---

## 5. Filtragem de sinais — Butterworth digital (BiquadFilter)

1. **Butterworth, S.** (1930). *On the Theory of Filter Amplifiers*. Wireless Engineer, 7, 536–541.
   — Filtro original (analógico).

2. **Oppenheim, A. V., & Schafer, R. W.** (2009). *Discrete-Time Signal Processing*, 3ª ed. Prentice Hall.
   — Cap. 7: projeto de filtros IIR (transformação bilinear, Butterworth digital). Forma de implementação direta II transposta usada em `BiquadFilter.cpp`.

3. **Smith, J. O.** (2007). *Introduction to Digital Filters with Audio Applications*. W3K Publishing. Disponível online: <https://ccrma.stanford.edu/~jos/filters/>
   — Tutorial gratuito sobre seções biquad.

---

## 6. Modelagem dinâmica do quadricóptero

Dinâmica de corpo rígido + acoplamento giroscópico + alocação X-quad:

1. **Bouabdallah, S.** (2007). *Design and Control of Quadrotors with Application to Autonomous Flying*. PhD Thesis, EPFL.
   — Referência canônica. Cap. 4: modelo dinâmico não-linear completo (Euler-Lagrange e Newton-Euler).

2. **Mahony, R., Kumar, V., & Corke, P.** (2012). *Multirotor aerial vehicles: Modeling, estimation, and control of quadrotor*. IEEE Robotics & Automation Magazine, 19(3), 20–32.
   — Modelo simplificado para controle, alocação de torques, escolha de configuração X vs +.

3. **Beard, R. W.** (2008). *Quadrotor Dynamics and Control Rev 0.1*. Brigham Young University.
   — Notas didáticas gratuitas detalhando a derivação de $A(x)$ usada em `updateSystemMatrix()`.

---

## 7. Calibração de sensores

### Magnetômetro (hard-iron + soft-iron)

1. **Gebre-Egziabher, D., Elkaim, G. H., Powell, J. D., & Parkinson, B. W.** (2001). *Calibration of strapdown magnetometers in magnetic field domain*. ASCE Journal of Aerospace Engineering, 19(2), 87–102.
   — Método de ajuste por elipsoide implementado em `test/calibrate_magnetometer.cpp`.

2. **Renaudin, V., Afzal, M. H., & Lachapelle, G.** (2010). *Complete triaxis magnetometer calibration in the magnetic domain*. Journal of Sensors, vol. 2010.
   — Calibração 3-D completa (rotação + escala + offset).

### MPU6050 (offsets de accel/gyro)

3. **InvenSense** (2013). *MPU-6000 and MPU-6050 Product Specification, Rev. 3.4*. Datasheet.
   — Bandwidth do DLPF, escalas full-range, procedimento de zero-rate calibration.

---

## 8. Comunicação WiFi — Protocolo CRTP

1. **Bitcraze AB** (2020). *Crazyflie Real-Time Protocol (CRTP) specification*. <https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/functional-areas/crtp/>
   — Especificação do protocolo; tabela de ports (0x03 = Commander, 0x07 = Link Control) implementada em `lib/WiFiComm/WiFiComm.cpp`.

2. **Espressif Systems** (2023). *ESP-Drone — Quadrotor solution based on Espressif chips*. <https://github.com/espressif/esp-drone>
   — App oficial usado no projeto (compatível com CRTP via UDP).

---

## 9. ESC / PWM para motores brushed

1. **Brescianini, D., Hehn, M., & D'Andrea, R.** (2013). *Nonlinear quadrocopter attitude control*. ETH Zurich Technical Report.
   — Modelo de empuxo $T = b\,\omega^2$ e torque de drag $Q = d\,\omega^2$ usados na alocação `calculateMotorOmegaSq`.

2. **Pounds, P., Mahony, R., & Corke, P.** (2010). *Modelling and control of a large quadrotor robot*. Control Engineering Practice, 18(7), 691–699.
   — Coeficientes de empuxo/drag para diversas hélices; metodologia de calibração (base de `test/motor_calibration_test.cpp`).

---

## 10. Implementação em ESP32 / sistemas embarcados

1. **Espressif Systems** (2024). *ESP32-S2 Technical Reference Manual*.
   — LEDC PWM (`MotorControl`), ADC (`led_control`), I2C, FreeRTOS tasks.

2. **Vandersypen, L. M. K.** (2018). *Concurrent Real-Time Programming for FreeRTOS*. Apress.
   — Padrão produtor-consumidor com mutex (usado entre `loop()` e `SDRETask`).

---

## Mapeamento método → seção

| Onde aparece                             | Seção(ões) relevantes |
|-------------------------------------------|------------------------|
| `lib/AUTOLQR/` — solvers DARE             | §2                     |
| `src/main.cpp` — `updateSystemMatrix`     | §1, §3, §6             |
| `src/main.cpp` — pesos Q, R               | §2 (Bryson)            |
| `lib/Madgwick` (externa, lib_deps)        | §4                     |
| `lib/KalmanFilter/`                       | §4 (Kalman, lib próprio README) |
| `lib/BiquadFilter/`                       | §5                     |
| `lib/utils/` — `calculateMotorOmegaSq`    | §6, §9                 |
| `lib/utils/` — drivers MPU/QMC            | §7                     |
| `lib/WiFiComm/`                           | §8                     |
| `lib/MotorControl/`                       | §9                     |
| `test/calibrate_*.cpp`                    | §7                     |
| `test/motor_calibration_test.cpp`         | §9                     |
