# PIDController Library

Biblioteca de controlador PID para controle de atitude de drones, compatível com a interface do AutoLQR/SDRE.

## Características

- **3 controladores PID independentes**: Roll, Pitch e Yaw
- **Anti-windup**: Limites configuráveis para os termos integrais
- **Filtro de derivada**: Filtro passa-baixa opcional no termo derivativo para reduzir ruído
- **Interface compatível**: Mesmos métodos que AutoLQR para facilitar troca de controladores
- **Uso direto de taxa angular**: Usa as taxas do giroscópio (p, q, r) ao invés da derivada do erro para melhor estabilidade

## Uso Básico

```cpp
#include "PIDController.h"

#define STATE_SIZE 6
#define CONTROL_SIZE 3

// Criar controlador
PIDController pid(STATE_SIZE, CONTROL_SIZE);

void setup() {
    // Configurar ganhos PID
    pid.setRollGains(2.0, 0.5, 0.1);   // Kp, Ki, Kd para Roll
    pid.setPitchGains(2.0, 0.5, 0.1);  // Kp, Ki, Kd para Pitch
    pid.setYawGains(1.5, 0.2, 0.05);   // Kp, Ki, Kd para Yaw
    
    // Ou configurar todos de uma vez
    pid.setAllGains(2.0, 0.5, 0.1,     // Roll: Kp, Ki, Kd
                    2.0, 0.5, 0.1,     // Pitch: Kp, Ki, Kd
                    1.5, 0.2, 0.05);   // Yaw: Kp, Ki, Kd
    
    // Configurar limites de anti-windup
    pid.setIntegralLimits(1.0, 1.0, 0.5);
    
    // Configurar limites de saída
    pid.setOutputLimits(-10.0, 10.0);
    
    // Configurar tempo de amostragem
    pid.setSamplingTime(0.012);  // 12ms
}

void loop() {
    // Estado atual: [roll, pitch, yaw, p, q, r]
    float state[6] = {roll, pitch, yaw, p, q, r};
    pid.updateState(state);
    
    // Referência: [phi_desired, theta_desired, psi_desired]
    float ref[3] = {phi_desired, theta_desired, yaw_desired};
    pid.updateReference(ref);
    
    // Calcular controle
    float u[3];  // [tau_roll, tau_pitch, tau_yaw]
    pid.calculateControl(u);
}
```

## Compatibilidade com AutoLQR

A biblioteca implementa os mesmos métodos que AutoLQR para facilitar a troca:

```cpp
// Estes métodos existem para compatibilidade, mas não fazem nada no PID:
pid.setStateMatrix(Ad);      // Ignorado - PID não usa
pid.setInputMatrix(Bd);      // Ignorado - PID não usa
pid.setCostMatrices(Q, R);   // Ignorado - PID não usa
pid.computeGains();          // Retorna true imediatamente
pid.getRicattiSolution();    // Retorna nullptr
```

## Métodos Principais

### Configuração de Ganhos

| Método | Descrição |
|--------|-----------|
| `setRollGains(kp, ki, kd)` | Define ganhos para o eixo roll |
| `setPitchGains(kp, ki, kd)` | Define ganhos para o eixo pitch |
| `setYawGains(kp, ki, kd)` | Define ganhos para o eixo yaw |
| `setAllGains(...)` | Define todos os ganhos de uma vez |

### Configuração de Limites

| Método | Descrição |
|--------|-----------|
| `setIntegralLimits(r, p, y)` | Define limites anti-windup |
| `setOutputLimits(min, max)` | Define limites de saída do controle |

### Operação

| Método | Descrição |
|--------|-----------|
| `updateState(state)` | Atualiza o estado atual |
| `updateReference(ref)` | Atualiza a referência |
| `calculateControl(u)` | Calcula os sinais de controle |
| `resetIntegrals()` | Zera os termos integrais |

### Debug

| Método | Descrição |
|--------|-----------|
| `getIntegrals(&r, &p, &y)` | Obtém valores integrais atuais |
| `getErrors(&r, &p, &y)` | Obtém erros atuais |
| `exportGains(K)` | Exporta ganhos em formato matriz |

## Tuning

### Valores Iniciais Recomendados (Drone pequeno ~40g)

```cpp
// Conservador - mais estável, menos agressivo
pid.setAllGains(1.5, 0.3, 0.08,   // Roll
                1.5, 0.3, 0.08,   // Pitch
                1.0, 0.1, 0.03);  // Yaw

// Moderado - equilíbrio
pid.setAllGains(2.5, 0.5, 0.15,   // Roll
                2.5, 0.5, 0.15,   // Pitch
                2.0, 0.3, 0.08);  // Yaw

// Agressivo - mais responsivo
pid.setAllGains(4.0, 1.0, 0.25,   // Roll
                4.0, 1.0, 0.25,   // Pitch
                3.0, 0.5, 0.12);  // Yaw
```

### Processo de Tuning

1. **Comece com Ki = 0, Kd = 0**
2. **Ajuste Kp** até obter oscilação consistente
3. **Ajuste Kd** para amortecer a oscilação
4. **Ajuste Ki** para eliminar erro em regime permanente
5. **Repita** para cada eixo (Roll, Pitch, Yaw)

## Diferenças do SDRE

| Aspecto | PID | SDRE |
|---------|-----|------|
| Ganhos | Fixos (manual) | Calculados em tempo real |
| Não-linearidades | Não considera | Considera via matriz A(x) |
| Custo computacional | Muito baixo | Mais alto |
| Estabilidade garantida | Não (depende do tuning) | Sim (se solução existe) |
| Facilidade de tuning | Mais intuitivo | Requer conhecimento do sistema |
