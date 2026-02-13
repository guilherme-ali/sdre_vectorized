# 🔍 Análise da Cadeia de Cálculos: SDRE → Duty Cycle

## 📊 Problema Identificado

O usuário questiona se a implementação da cadeia de cálculos está correta perante as medições reais do motor:

```
SDRE → u (torques) → ω² → duty cycle → motor
```

A teoria diz que **T = b × ω²**, mas é preciso verificar se a conversão para duty cycle está implementada corretamente.

---

## 🔗 Cadeia Completa de Cálculos

### 1️⃣ SDRE Controller → Torques de Controle (u)

**Localização:** `main_backup.cpp:475-485`

```cpp
float u[CONTROL_SIZE];
sdreController.calculateControl(u);
```

**Saída:**
- `u[0]` = Torque de roll (τx) [N·m]
- `u[1]` = Torque de pitch (τy) [N·m]
- `u[2]` = Torque de yaw (τz) [N·m]

✅ **Status:** OK - Controlador SDRE calcula torques corretamente

---

### 2️⃣ Torques → ω² (Omega Quadrado)

**Localização:** `utils.cpp:310-349`

**Função:** `calculateMotorOmegaSq()`

#### Matriz de Alocação de Controle (X-quad):

```
[ω1²]   [1/(4b)  -1/(2bL)   1/(2bL)  -1/(4d)] [thrust]
[ω2²] = [1/(4b)  -1/(2bL)  -1/(2bL)   1/(4d)] [  τx  ]
[ω3²]   [1/(4b)   1/(2bL)  -1/(2bL)  -1/(4d)] [  τy  ]
[ω4²]   [1/(4b)   1/(2bL)   1/(2bL)   1/(4d)] [  τz  ]
```

**Código (linhas 323-342):**
```cpp
float inv_4b = 1.0f / (4.0f * b_coeff);
float inv_2bL = 1.0f / (2.0f * b_coeff * L_arm);
float inv_4d = 1.0f / (4.0f * d_coeff);

w1_sq = u1 * inv_4b - u2 * inv_2bL + u3 * inv_2bL - u4 * inv_4d;
w2_sq = u1 * inv_4b - u2 * inv_2bL - u3 * inv_2bL + u4 * inv_4d;
w3_sq = u1 * inv_4b + u2 * inv_2bL - u3 * inv_2bL - u4 * inv_4d;
w4_sq = u1 * inv_4b + u2 * inv_2bL + u3 * inv_2bL + u4 * inv_4d;
```

**Parâmetros usados (`main_backup.cpp:73-74`):**
```cpp
const float MOTOR_B_COEFF = 9.68e-9;   // N/(rad/s)²
const float MOTOR_D_COEFF = 0.05 * MOTOR_B_COEFF;  // N·m/(rad/s)²
const float L_ARM = 0.060f;  // 60mm
```

✅ **Status:** OK - Física e matemática corretas

**Verificação dimensional:**
- `thrust [N] / (4 × b [N/(rad/s)²])` = `[rad²/s²]` ✓
- `τ [N·m] / (2 × b × L [N/(rad/s)² × m])` = `[rad²/s²]` ✓

---

### 3️⃣ ω² → Throttle (%)

**Localização:** `MotorControl.cpp:258-276`

**Função:** `omegaSqToThrottle()`

```cpp
float MotorControl::omegaSqToThrottle(float omega_sq) {
    // Garante não negativo
    if (omega_sq < 0) {
        omega_sq = 0;
    }

    // Limita ao máximo configurado
    if (omega_sq > max_omega_sq) {
        omega_sq = max_omega_sq;
    }

    // Mapeia omega² para throttle (0-100%)
    // Relação LINEAR: throttle = (omega_sq / max_omega_sq) * 100
    float throttle = (omega_sq / max_omega_sq) * (max_throttle - min_throttle) + min_throttle;

    return constrainThrottle(throttle);
}
```

**Parâmetros (`main_backup.cpp:70-71, 319`):**
```cpp
const float MAX_RPM = 51000.0f;
const float MAX_OMEGA = (MAX_RPM * 2π) / 60 = 5340.7 rad/s
max_omega_sq = MAX_OMEGA² = 28,523,099 rad²/s²
```

**Mapeamento:**
```
throttle = (ω² / 28,523,099) × 100%
```

---

## 🚨 PROBLEMA IDENTIFICADO: Relação Linear vs. Quadrática

### ⚠️ Inconsistência Teórica

A teoria nos diz que:
```
T = b × ω²
```

Portanto, a relação entre **throttle e força** deveria ser:
```
ω ∝ throttle        (assumindo ESC linear)
T ∝ ω²
T ∝ throttle²       (relação QUADRÁTICA!)
```

Mas o código atual faz:
```cpp
throttle = (ω² / max_ω²) × 100%
```

Isso implica que:
```
throttle ∝ ω²
ω ∝ √throttle       (relação de RAIZ QUADRADA!)
```

### 🔬 O que deveria ser?

Se o ESC mapeia linearmente throttle → RPM (comportamento típico):
```
RPM = throttle × MAX_RPM
ω = throttle × MAX_OMEGA
ω² = throttle² × MAX_OMEGA²
```

Portanto:
```
throttle = ω / MAX_OMEGA          (relação LINEAR com ω)
throttle = √(ω² / MAX_OMEGA²)     (relação de RAIZ QUADRADA com ω²)
```

---

## 🔧 Correção Proposta

### Opção 1: ESC com Mapeamento Linear (RPM ∝ throttle)

**Mais comum em ESCs de drone**

```cpp
float MotorControl::omegaSqToThrottle(float omega_sq) {
    if (omega_sq < 0) {
        omega_sq = 0;
    }

    // Calcula omega (raiz quadrada)
    float omega = sqrt(omega_sq);

    // Limita ao máximo
    float max_omega = sqrt(max_omega_sq);
    if (omega > max_omega) {
        omega = max_omega;
    }

    // Relação LINEAR: throttle = (ω / ω_max) × 100
    float throttle = (omega / max_omega) * (max_throttle - min_throttle) + min_throttle;

    return constrainThrottle(throttle);
}
```

### Opção 2: ESC com Mapeamento Quadrático (RPM ∝ √throttle)

**Menos comum, mas alguns ESCs fazem correção de curva**

```cpp
// Código atual está correto se ESC já faz a linearização de thrust!
float throttle = (omega_sq / max_omega_sq) * (max_throttle - min_throttle) + min_throttle;
```

---

## 🧪 Como Verificar Qual Está Correto?

### Teste Experimental

1. **Fixe o drone numa balança**
2. **Aplique throttle de 0% a 100%**
3. **Meça a força produzida**

**Se força ∝ throttle²:** ESC é linear → Use Opção 1
**Se força ∝ throttle:** ESC já lineariza → Código atual OK (Opção 2)

### Análise dos Dados de Calibração

De acordo com `motor_calibration_test.cpp` e o guia de calibração:

```cpp
// Linha 81-92 de motor_calibration_test.cpp
float throttleToOmegaSq(int throttle_percent) {
    float throttle_fraction = throttle_percent / 100.0f;
    float omega = throttle_fraction * MAX_OMEGA;  // ← RELAÇÃO LINEAR!
    return omega * omega;
}
```

O código de calibração **assume que throttle → ω é linear**, o que sugere que:
- ESC mapeia linearmente throttle → RPM
- Portanto, a **Opção 1 é a correta**!

---

## 📐 Análise da Conversão Throttle → PWM

**Localização:** `MotorControl.cpp:252-256`

```cpp
int MotorControl::throttleToPWM(float throttle) {
    // Mapeia throttle (0-100%) para PWM (0-1023)
    return map(throttle * 100, 0, 10000, 0, PWM_MAX_VALUE);
}
```

✅ **Status:** OK - Conversão linear de throttle → duty cycle

**Duty cycle = (throttle / 100) × 100% = throttle%**

---

## 🎯 Conclusão e Recomendações

### ❌ Problema Encontrado

A função `omegaSqToThrottle()` em `MotorControl.cpp:258-276` está **incorreta**.

**Código atual (ERRADO):**
```cpp
throttle = (omega_sq / max_omega_sq) * 100
```

**Deveria ser:**
```cpp
throttle = (sqrt(omega_sq) / sqrt(max_omega_sq)) * 100
throttle = sqrt(omega_sq / max_omega_sq) * 100
```

### ✅ Impacto no Sistema

Com a implementação atual:
- **Throttle pedido é MAIOR** do que o necessário
- **Resposta será muito agressiva** em baixos valores de ω
- **Pode saturar motores prematuramente**

Exemplo numérico:
```
Desejado: ω = 2670 rad/s (50% do máximo)
ω² = 7,128,900

ATUAL (ERRADO):
throttle = 7,128,900 / 28,523,099 × 100 = 25%

CORRETO:
throttle = sqrt(7,128,900 / 28,523,099) × 100 = 50%
```

**Erro:** O sistema está comandando apenas **metade do throttle necessário**!

### 🔄 Correção Necessária

**Arquivo:** `lib/MotorControl/MotorControl.cpp`
**Função:** `omegaSqToThrottle()` (linha 258)

**Substituir:**
```cpp
float throttle = (omega_sq / max_omega_sq) * (max_throttle - min_throttle) + min_throttle;
```

**Por:**
```cpp
float omega = sqrt(omega_sq);
float max_omega = sqrt(max_omega_sq);
float throttle = (omega / max_omega) * (max_throttle - min_throttle) + min_throttle;
```

---

## 📊 Tabela Comparativa (Exemplo com MAX_OMEGA = 5340.7 rad/s)

| ω² desejado | ω (rad/s) | Throttle ATUAL (ERRADO) | Throttle CORRETO | Erro (%) |
|-------------|-----------|-------------------------|------------------|----------|
| 0 | 0 | 0% | 0% | 0% |
| 7,130,775 | 2,670 | 25% | 50% | -50% |
| 14,261,550 | 3,778 | 50% | 70.7% | -29% |
| 21,392,324 | 4,625 | 75% | 86.6% | -13% |
| 28,523,099 | 5,341 | 100% | 100% | 0% |

**Observação:** O erro é MAIOR em valores médios de throttle!

---

## ✅ Validação da Teoria T = b × ω²

A teoria **T = b × ω²** está **CORRETA** e é bem implementada em:
- `calculateMotorOmegaSq()` ✓
- Guia de calibração ✓
- Coeficientes MOTOR_B_COEFF e MOTOR_D_COEFF ✓

O **único problema** está na conversão `ω² → throttle`.

---

## 📝 Próximos Passos

1. ✅ Corrigir `MotorControl::omegaSqToThrottle()`
2. ✅ Recompilar e testar
3. ✅ Verificar comportamento do drone em voo
4. ✅ Validar com dados de calibração existentes

---

**Documento gerado em:** 2026-02-13
**Versão:** 1.0
