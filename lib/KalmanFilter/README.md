# KalmanFilter

Filtro de Kalman linear (LKF) genérico para sistemas em tempo discreto. Encapsula as etapas de **predição** e **atualização** em uma classe enxuta, com toda a álgebra delegada a `MatrixOperations` (de `lib/AUTOLQR/`).

> **Status:** opcional. Não é usado pelo `main.cpp` atual — o projeto fundiu IMU+AHRS via **Madgwick** (`lib/Madgwick` externa) por ser ~10× mais barato em CPU para estimação de orientação. O Kalman fica disponível para quem precisar de fusão de sensores adicionais (ex.: GPS, barômetro, encoders) ou estimação de estados não diretamente medidos.

## Para que serve

Estimação ótima (no sentido de mínima variância) do estado de um sistema linear estocástico:

$$x_{k+1} = A\,x_k + B\,u_k + w_k,\quad w_k \sim \mathcal{N}(0, Q)$$
$$z_k = C\,x_k + v_k,\quad v_k \sim \mathcal{N}(0, R)$$

onde $w_k$ é ruído de processo e $v_k$ é ruído de medição (ambos brancos, gaussianos, não correlacionados).

**Casos de uso típicos no contexto deste projeto:**

- Fusão IMU + magnetômetro + GPS para estimar posição/velocidade/orientação simultaneamente
- Estimação de viés do giroscópio em tempo real (estado aumentado)
- Reconstrução de torques aerodinâmicos não medidos a partir das taxas
- Suavização de medições ruidosas de bateria/encoder para feedback de controle

## Equações implementadas

### Predição

$$\hat{x}_{k|k-1} = A\,\hat{x}_{k-1|k-1} + B\,u_{k-1}$$
$$P_{k|k-1} = A\,P_{k-1|k-1}\,A^T + Q$$

### Atualização (correção pela medição $z_k$)

**Inovação e sua covariância:**

$$y_k = z_k - C\,\hat{x}_{k|k-1}$$
$$S_k = C\,P_{k|k-1}\,C^T + R$$

**Ganho de Kalman:**

$$K_k = P_{k|k-1}\,C^T\,S_k^{-1}$$

**Atualização do estado e covariância:**

$$\hat{x}_{k|k} = \hat{x}_{k|k-1} + K_k\,y_k$$
$$P_{k|k} = (I - K_k\,C)\,P_{k|k-1}$$

> **Nota numérica:** a implementação usa a forma padrão $(I - KC)P$, não a forma de **Joseph** $(I-KC)P(I-KC)^T + KRK^T$. A forma padrão é mais barata mas pode perder simetria/positivo-definidade em precisão `float` após muitas iterações. Se observar divergência, troque por Joseph (ver Simon, 2006, §6.1).

## API

```cpp
#include <KalmanFilter.h>

// (n=estado, p=controle, m=medicao)
KalmanFilter kf(/*state_dim=*/6, /*control_dim=*/3, /*measurement_dim=*/6);

void setup() {
    // Modelo do sistema
    float A[36] = { /* ... */ };
    float B[18] = { /* ... */ };
    float C[36] = { /* ... */ };  // identidade se mede todo o estado

    // Covariancias de ruido
    float Q[36] = { /* diag — ajustar empiricamente */ };
    float R[36] = { /* diag — variancia dos sensores */ };

    // Condicao inicial
    float x0[6] = {0};
    float P0[36] = { /* inicialmente grande se incerto: I * 1.0 */ };

    kf.init(A, B, C, Q, R, x0, P0);
}

void loop() {
    float u[3] = { /* controle aplicado */ };
    float z[6] = { /* medicoes dos sensores */ };

    kf.predict(u);   // pode passar nullptr se nao houver entrada de controle
    kf.update(z);

    const float* x_est = kf.getState();        // estado estimado (6×1)
    const float* P_est = kf.getCovariance();   // covariancia (6×6)
}
```

## Custo computacional

Para sistema $n=6$, $p=3$, $m=6$ no ESP32-S2 @ 240 MHz:

- `predict()`: ~300 µs (1 mat·vec + 2 mat·mat de 6×6)
- `update()`: ~800 µs (3 mat·mat 6×6 + 1 inversa 6×6 + soma)
- **Total/ciclo:** ~1.1 ms (cabe folgado no loop de 5 ms)

A memória total alocada (todas as matrizes temporárias) para $n=6,m=6$ é ~1.5 KB.

## Sintonização de Q e R

- **R:** começa pela variância medida do sensor (gire o sensor parado, calcule $\sigma^2$ por eixo).
- **Q:** começa pequeno (ex.: $10^{-4} I$) e aumenta se o filtro "fica preso" no modelo (lento para responder a mudanças); diminui se a estimativa for ruidosa demais.
- **Razão $Q/R$** controla o trade-off:
  - $Q/R \gg 1$ → confia mais nas medições (resposta rápida, ruidoso)
  - $Q/R \ll 1$ → confia mais no modelo (suave, mas com atraso)

Procedimento prático: ver Brown & Hwang, *Introduction to Random Signals and Applied Kalman Filtering*, cap. 5.

## Arquivos

```
KalmanFilter/
├── KalmanFilter.cpp / .h
├── library.json    # dependência: AUTOLQR (para MatrixOperations)
└── README.md
```

## Referências

1. **Kalman, R. E.** (1960). *A New Approach to Linear Filtering and Prediction Problems*. Transactions of the ASME — Journal of Basic Engineering, 82(D), 35–45.
   — Paper original. Leitura obrigatória para entender a derivação por estimação MMSE.

2. **Welch, G., & Bishop, G.** (2006). *An Introduction to the Kalman Filter*. UNC-Chapel Hill, TR 95-041.
   — Tutorial canônico, gratuito. Boa primeira leitura: <https://www.cs.unc.edu/~welch/kalman/>

3. **Simon, D.** (2006). *Optimal State Estimation: Kalman, H∞, and Nonlinear Approaches*. Wiley.
   — Texto-livro completo. Cap. 5 (LKF), cap. 6 (formas numericamente estáveis incl. Joseph), cap. 13 (EKF), cap. 14 (UKF) para extensões não-lineares.

4. **Brown, R. G., & Hwang, P. Y. C.** (2012). *Introduction to Random Signals and Applied Kalman Filtering*, 4ª ed. Wiley.
   — Foco em aplicações práticas e tuning empírico de Q e R.

5. **Grewal, M. S., & Andrews, A. P.** (2014). *Kalman Filtering: Theory and Practice with MATLAB*, 4ª ed. Wiley.
   — Inclui implementações de referência em MATLAB, útil para validar este código.

6. **Madgwick, S. O. H.** (2010). *An efficient orientation filter for inertial and inertial/magnetic sensor arrays*. Internal report, University of Bristol.
   — Justifica por que o projeto adota Madgwick em vez de EKF para AHRS no loop de 200 Hz. Disponível em <https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/>
