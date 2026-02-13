# Verificação da Implementação do Controle de Motores

## 🔍 Análise da Cadeia de Cálculos

### Fluxo Completo (src/main.cpp)

1. **SDRE/PID** calcula sinais de controle:
   - `u[0]` = Torque de Roll (N·m)
   - `u[1]` = Torque de Pitch (N·m)
   - `u[2]` = Torque de Yaw (N·m)
   - `thrust` = Empuxo total (N)

2. **calculateMotorOmegaSq()** (utils.cpp:310-349) converte para ω²:
   ```cpp
   // Matriz de alocação X-quad (CORRETO ✅)
   w1_sq = thrust/(4b) - u_roll/(2bL) + u_pitch/(2bL) - u_yaw/(4d)
   w2_sq = thrust/(4b) - u_roll/(2bL) - u_pitch/(2bL) + u_yaw/(4d)
   w3_sq = thrust/(4b) + u_roll/(2bL) - u_pitch/(2bL) - u_yaw/(4d)
   w4_sq = thrust/(4b) + u_roll/(2bL) + u_pitch/(2bL) + u_yaw/(4d)
   ```
   ✅ **Esta etapa está CORRETA** - aplica corretamente T = b·ω²

3. **setMotorsFromOmegaSq()** (MotorControl.cpp:100-109):
   ```cpp
   float throttle1 = omegaSqToThrottle(w1_sq);
   // ...
   ```

4. **omegaSqToThrottle()** (MotorControl.cpp:258-276):
   ```cpp
   float throttle = (omega_sq / max_omega_sq) * 100;  // ❌ PROBLEMA!
   ```

5. **throttleToPWM()** (MotorControl.cpp:252-256):
   ```cpp
   int pwm = map(throttle * 100, 0, 10000, 0, 1023);  // ✅ CORRETO
   ```

---

## ❌ PROBLEMA IDENTIFICADO

### Localização
**Arquivo**: `lib/MotorControl/MotorControl.cpp`
**Função**: `omegaSqToThrottle()` (linha 258-276)
**Linha Problemática**: 271

### Código Atual (INCORRETO)
```cpp
float MotorControl::omegaSqToThrottle(float omega_sq)
{
    // Mapeia omega² para throttle (0-100%) usando float
    // Relação linear: throttle = (omega_sq / max_omega_sq) * 100
    // ❌ Não usa sqrt para evitar não-linearidade e simplificar
    float throttle = (omega_sq / max_omega_sq) * (max_throttle - min_throttle) + min_throttle;

    return constrainThrottle(throttle);
}
```

### Por que está errado?

A implementação assume: **ω² ∝ throttle** (relação linear)

Mas a física real do sistema motor+ESC é:

1. **Throttle → Voltage**: ESC converte duty cycle PWM em tensão
   - `V_motor ≈ V_battery × (PWM / PWM_max)`
   - Relação aproximadamente linear

2. **Voltage → RPM**: Motor brushless responde à tensão
   - `RPM ∝ V_motor × KV`
   - Para motores brushless, RPM é aproximadamente proporcional à tensão

3. **RPM → ω**:
   - `ω = RPM × (2π/60)`
   - Conversão linear

4. **Conclusão**:
   - `ω ∝ throttle`
   - `ω² ∝ throttle²`

Portanto, para converter `ω²` para `throttle`, precisamos de **raiz quadrada**!

---

## ✅ SOLUÇÃO

### Código Correto
```cpp
float MotorControl::omegaSqToThrottle(float omega_sq)
{
    // Garante que omega_sq não seja negativo
    if (omega_sq < 0) {
        omega_sq = 0;
    }

    // Limita ao máximo configurado
    if (omega_sq > max_omega_sq) {
        omega_sq = max_omega_sq;
    }

    // CORRETO: throttle ∝ ω, então throttle ∝ √(ω²)
    // Para motores brushless: RPM ∝ Voltage ∝ PWM/Throttle
    // Portanto: ω ∝ throttle → ω² ∝ throttle²
    // Logo: throttle = √(ω² / ω²_max) × 100%
    float normalized_omega = sqrt(omega_sq / max_omega_sq);
    float throttle = normalized_omega * (max_throttle - min_throttle) + min_throttle;

    return constrainThrottle(throttle);
}
```

---

## 🧮 Exemplo Numérico

### Parâmetros do Sistema (src/main.cpp)
```cpp
MAX_RPM = 51000 RPM
MAX_OMEGA = 5340.7 rad/s
MAX_OMEGA_SQ = 28523077 rad²/s²
```

### Exemplo: Motor precisa girar a 50% da velocidade máxima

**Valores Esperados**:
- ω = 2670.35 rad/s (50% de MAX_OMEGA)
- ω² = 7130769 rad²/s² (25% de MAX_OMEGA_SQ)
- **Throttle esperado**: 50%

**Implementação Atual (INCORRETA)**:
```
throttle = (7130769 / 28523077) × 100 = 25% ❌
```
Resultado: Motor gira apenas a 25% → Muito lento!

**Implementação Correta**:
```
throttle = √(7130769 / 28523077) × 100 = √(0.25) × 100 = 50% ✅
```
Resultado: Motor gira a 50% → Correto!

---

## 📊 Tabela de Comparação

| ω (% de MAX) | ω² (% de MAX) | Throttle ERRADO | Throttle CORRETO |
|--------------|---------------|-----------------|------------------|
| 0%           | 0%            | 0%              | 0%               |
| 25%          | 6.25%         | 6.25%           | 25%              |
| 50%          | 25%           | 25%             | 50%              |
| 75%          | 56.25%        | 56.25%          | 75%              |
| 100%         | 100%          | 100%            | 100%             |

**Observação**: A implementação atual aplica **sub-throttle** significativo!
- Motor a 50% da velocidade → apenas 25% de throttle aplicado
- Resultado: Drone não consegue gerar empuxo suficiente

---

## 🔬 Verificação com Teoria T = b·ω²

### Cadeia Completa Correta

1. **Controlador SDRE** pede thrust = 0.3 N
2. **calculateMotorOmegaSq()**: `ω² = T / (4b) = 0.3 / (4 × 2.798e-7) = 267918 rad²/s²`
3. **omegaSqToThrottle() CORRIGIDO**:
   ```
   throttle = √(267918 / 28523077) × 100 = 3.06%
   ```
4. **Motor gira** com throttle = 3.06%
   - `ω_real = throttle × MAX_OMEGA = 0.0306 × 5340.7 = 163.4 rad/s`
   - `ω²_real = 26699 rad²/s²` ... ❌ Não bate!

### PROBLEMA ADICIONAL DESCOBERTO!

Espera, refazendo:
```
ω² = 267918 rad²/s²
ω = √(267918) = 517.8 rad/s
ω / MAX_OMEGA = 517.8 / 5340.7 = 9.7%
throttle = 9.7%  ✅
```

**Verificação**:
- Throttle = 9.7% → ω = 9.7% × 5340.7 = 517.8 rad/s ✅
- T = 4 × b × ω² = 4 × 2.798e-7 × 267918 = 0.3 N ✅

---

## ⚠️ IMPACTO DO BUG

### Sintomas Esperados no Drone:
1. ✅ **Motores giram muito devagar** (sub-throttle)
2. ✅ **Drone não decola** (empuxo insuficiente)
3. ✅ **Controlador aumenta comando** para compensar
4. ✅ **Possível instabilidade** (resposta não-linear inesperada)
5. ✅ **Saturação dos motores** prematura

### Por que o código menciona "evitar não-linearidade"?

O comentário no código diz:
```cpp
// Não usa sqrt para evitar não-linearidade e simplificar
```

**ISSO ESTÁ INVERTIDO!**
- **Usar sqrt é necessário** para ter resposta LINEAR do sistema
- **Não usar sqrt** introduz não-linearidade (quadrática)

A relação física real é:
- `T = b·ω²` (teoria do motor - não-linear em ω)
- `ω ∝ throttle` (motor brushless - linear)
- Portanto: `T ∝ throttle²` no nível do motor físico

O controlador SDRE já compensa a não-linearidade `T = b·ω²` ao calcular ω².
A conversão ω² → throttle deve usar sqrt para manter linearidade!

---

## 🎯 CONCLUSÃO

### Verificação da Implementação

| Componente | Status | Nota |
|------------|--------|------|
| Teoria T = b·ω² | ✅ CORRETO | Aplicada corretamente em calculateMotorOmegaSq() |
| Matriz de alocação | ✅ CORRETO | X-quad configuration correta |
| calculateMotorOmegaSq() | ✅ CORRETO | Conversão thrust+torques → ω² correta |
| **omegaSqToThrottle()** | ❌ **ERRADO** | **Falta sqrt() - bug crítico!** |
| throttleToPWM() | ✅ CORRETO | Mapeamento linear correto |
| Coeficientes b e d | ⚠️ VERIFICAR | Precisam calibração experimental |

### Correção Necessária

**Arquivo**: `lib/MotorControl/MotorControl.cpp`
**Linha**: 271
**Mudança**: Adicionar `sqrt()` no cálculo do throttle

---

## 📝 Próximos Passos

1. ✅ **Aplicar correção** no omegaSqToThrottle()
2. ⚠️ **Recalibrar MOTOR_B_COEFF** seguindo MOTOR_CALIBRATION_GUIDE.md
3. ⚠️ **Testar em bancada** antes de voar
4. ⚠️ **Validar resposta** dos motores com dados reais

---

## 🔗 Referências

- **Teoria**: Brushless Motor Physics (T = Kt·I, RPM = Kv·V)
- **Código**: lib/MotorControl/MotorControl.cpp:258-276
- **Calibração**: lib/MotorControl/MOTOR_CALIBRATION_GUIDE.md
- **SDRE**: src/main.cpp:489-496

---

**Data da Análise**: 2026-02-13
**Autor**: Claude Code (Análise Automatizada)
