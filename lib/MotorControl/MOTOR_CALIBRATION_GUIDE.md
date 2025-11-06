# 🎯 Guia de Calibração do MOTOR_B_COEFF

## 📋 Material Necessário

1. **Balança digital** (precisão mínima de 0.1g, ideal 0.01g)
2. **Suporte fixo** para prender o drone (fita dupla-face, braçadeira, etc.)
3. **Bateria carregada**
4. **Computador** com Serial Monitor
5. **Superfície estável** (sem vibração)

---

## 🔧 Preparação

### 1. Configure seu motor no código

Antes de compilar, ajuste os parâmetros do seu motor específico em `test/motor_calibration_test.cpp`:

```cpp
const float KV_RATING = 8500.0f;   // KV do seu motor (verifique na especificação)
const float VOLTAGE = 3.7f;        // Tensão da bateria (3.7V = 1S LiPo, 7.4V = 2S, etc.)
```

**Como descobrir o KV do motor:**
- Verifique a caixa ou especificação do motor
- Exemplos comuns:
  - Tiny Whoop: 8500-19000 KV
  - Mini quad 2-3": 5000-8000 KV  
  - Racing 5": 2300-2700 KV

### 2. Compile o programa de teste

No PlatformIO:
```
1. Abra test/motor_calibration_test.cpp
2. Build (Ctrl+Alt+B)
3. Upload (Ctrl+Alt+U)
```

---

## 🧪 Procedimento de Teste

### Passo 1: Setup Físico

1. **Fixe o drone na balança**
   - Use fita dupla-face ou suporte
   - Certifique-se que está BEM FIXO (não pode voar!)
   - Hélices devem estar instaladas

2. **Zere a balança (TARA)**
   - Com o drone fixo em cima
   - A balança deve mostrar 0.0g

3. **Posicionamento**
   - Drone nivelado (horizontal)
   - Afastado de paredes (mínimo 50cm)
   - Sem objetos acima que possam refletir o fluxo de ar

### Passo 2: Execução do Teste

1. **Conecte ao Serial Monitor** (115200 baud)

2. **Pressione ENTER** para iniciar

3. **Para cada nível de throttle:**
   ```
   O programa irá:
   ├─ Acelerar os motores para X%
   ├─ Aguardar 3 segundos (estabilização)
   └─ Solicitar leitura da balança
   
   Você deve:
   ├─ LER o valor na balança (em gramas)
   ├─ DIGITAR o valor no Serial Monitor
   └─ PRESSIONAR Enter
   ```

4. **Exemplo de execução:**
   ```
   ╔════════════════════════════════════════════╗
   ║  MEDIÇÃO - Throttle:  20%                  ║
   ╚════════════════════════════════════════════╝
   
     ► Motores acelerando...
     ► Estabilizado!
   
     📊 LEIA O VALOR DA BALANÇA (em gramas)
        Digite o valor e pressione ENTER:
        > 15.3           <--- VOCÊ DIGITA AQUI
   
     ✅ Medição registrada:
        Força: 15.3 g = 0.1501 N
        ω² total: 284467.52 (rad/s)²
        B_coeff calculado: 0.0000005276 N/(rad/s)²
   ```

5. **Repita** até completar todas as medições (0% a 100%)

### Passo 3: Resultados

Ao final, você verá uma tabela completa:

```
┌─────────┬───────────┬────────────┬─────────────┬──────────────────┐
│Throttle │  Força    │   Força    │   ω² total  │   B_coeff        │
│   (%)   │   (g)     │    (N)     │  (rad/s)²   │  N/(rad/s)²      │
├─────────┼───────────┼────────────┼─────────────┼──────────────────┤
│    0    │      0.0  │    0.0000  │         0.00│  0.0000000000    │
│    5    │      2.1  │    0.0206  │     35558.44│  0.0000005794    │
│   10    │      8.5  │    0.0834  │    142233.76│  0.0000005863    │
│   15    │     19.2  │    0.1884  │    319975.44│  0.0000005889    │
│   20    │     34.1  │    0.3345  │    568950.24│  0.0000005878    │
...
└─────────┴───────────┴────────────┴─────────────┴──────────────────┘

╔═══════════════════════════════════════════════════════════════╗
║              VALOR RECOMENDADO PARA MOTOR_B_COEFF             ║
╚═══════════════════════════════════════════════════════════════╝

  📌 MOTOR_B_COEFF = 0.0000005856

     (Média de 20 medições válidas)

  📊 Estatísticas:
     Desvio padrão: 0.0000000043
     Coef. variação: 0.73%
     ✅ Excelente consistência!
```

---

## 🔬 Entendendo a Teoria

### Fórmula Fundamental

$$F_{thrust} = b \cdot \sum_{i=1}^{4} \omega_i^2$$

Onde:
- $F_{thrust}$ = Força total (empuxo) em Newtons [N]
- $b$ = `MOTOR_B_COEFF` em [N/(rad/s)²]
- $\omega_i$ = Velocidade angular do motor i em [rad/s]

### Conversões

1. **Gramas → Newtons:**
   $$F_N = \frac{massa_{gramas}}{1000} \times 9.81$$

2. **Throttle → RPM → ω:**
   ```
   RPM = (Throttle_%) × KV × Voltage / 100
   ω = RPM × (2π/60)  [rad/s]
   ```

3. **Cálculo do coeficiente:**
   $$b = \frac{F_{thrust}}{4 \times \omega^2}$$
   
   (Dividimos por 4 pois temos 4 motores iguais)

### Exemplo Prático

Medição: Throttle 50%, Balança marca 120.5g

```
1. Converter força:
   F = (120.5/1000) × 9.81 = 1.182 N

2. Calcular ω (exemplo: motor 8500KV, bateria 3.7V):
   RPM = 50% × 8500 × 3.7 / 100 = 15,725 RPM
   ω = 15,725 × (2π/60) = 1,646.4 rad/s
   ω² = 2,710,632 (rad/s)²

3. Calcular coeficiente:
   b = 1.182 / (4 × 2,710,632)
   b = 0.000000109 N/(rad/s)²
```

---

## ⚙️ Aplicando o Valor

### 1. Copie o valor recomendado

Exemplo: `MOTOR_B_COEFF = 0.0000005856`

### 2. Edite `src/main.cpp`

```cpp
// ANTES:
const float MOTOR_B_COEFF = 0.0001;  // ❌ Valor estimado

// DEPOIS:
const float MOTOR_B_COEFF = 0.0000005856;  // ✅ Valor calibrado
```

### 3. Recompile e faça upload

```
PlatformIO: Build (Ctrl+Alt+B)
PlatformIO: Upload (Ctrl+Alt+U)
```

---

## ❗ Troubleshooting

### Problema: Balança mostra valores negativos
**Causa:** Fluxo de ar empurrando a balança para baixo  
**Solução:** 
- Coloque um anteparo ao redor (caixa sem tampa)
- Use balança com maior capacidade
- Inverta o drone (hélices para baixo)

### Problema: Valores muito inconsistentes (CV > 10%)
**Causas possíveis:**
- Vibração da superfície
- Bateria fraca (tensão caindo)
- Motor/ESC com problema
- Tempo de estabilização muito curto

**Soluções:**
- Aumente `STABILIZATION_TIME` para 5000ms
- Troque a bateria
- Teste em superfície mais estável
- Verifique conexões dos motores

### Problema: Motores não aceleram
**Verifique:**
- ESCs calibrados? (descomentar `motors.calibrateESCs()`)
- Bateria conectada?
- Motores armados corretamente?
- Valores de limite em `MotorControl`

---

## 📊 Valores de Referência

### Motores Típicos de Drone

| Tipo | KV | Hélice | B_coeff (aprox.) |
|------|-------|--------|------------------|
| Tiny Whoop | 19000 | 31mm | 0.0000001 - 0.0000003 |
| Mini 2" | 8500 | 2" | 0.0000003 - 0.0000006 |
| Racing 5" | 2400 | 5" | 0.000001 - 0.000003 |
| Camera 7" | 1600 | 7" | 0.000003 - 0.000008 |

**⚠️ ATENÇÃO:** Estes são valores aproximados! Sempre calibre com seu setup específico!

---

## 🎓 Relação com MOTOR_D_COEFF

O coeficiente `MOTOR_D_COEFF` (drag/torque) está relacionado com `MOTOR_B_COEFF`:

$$d \approx k \times b$$

Onde $k$ é um fator que depende do desenho da hélice (geralmente 0.1 a 0.2).

**Regra prática:**
```cpp
MOTOR_D_COEFF = MOTOR_B_COEFF × 0.15  // Valor típico
```

Para calibrar precisamente o `MOTOR_D_COEFF`, seria necessário um sensor de torque, o que é mais complexo. Na prática, a relação acima funciona bem.

---

## 📝 Checklist Final

- [ ] Motor KV e tensão configurados corretamente
- [ ] Balança zerada (tara) com drone fixo
- [ ] Drone bem fixo (não pode voar)
- [ ] ESCs calibrados previamente
- [ ] Bateria carregada
- [ ] Serial Monitor em 115200 baud
- [ ] Anotou todas as medições
- [ ] Coeficiente de variação < 10%
- [ ] Valor aplicado em `main.cpp`
- [ ] Código recompilado e enviado

---

## 🎯 Resultado Esperado

Com o `MOTOR_B_COEFF` calibrado corretamente:

✅ **Controle mais preciso** da altitude  
✅ **Thrust calculado** corresponde ao thrust real  
✅ **Melhor estabilidade** em modo hover  
✅ **Resposta mais linear** aos comandos  

---

**Boa calibração! 🚁**
