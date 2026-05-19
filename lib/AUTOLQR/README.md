# AutoLQR - Biblioteca de Controle LQR Adaptativo

Biblioteca otimizada para cálculo de ganhos LQR em tempo real, projetada para sistemas embarcados como ESP32. Implementa múltiplos algoritmos para resolver a Equação Algébrica de Riccati Discreta (DARE).

## 📋 Características

- **7 métodos de solução DARE** implementados (SDA, SDA-ss, ASDA, SDA Scaled, ADDA, Van Dooren, Iterativo)
- **Operações matriciais otimizadas** para sistemas de pequeno/médio porte (`MatrixOperations`)
- **Warm-start** automático para o método iterativo (P anterior como inicial)
- **Baixo consumo de memória** — alocação única no construtor, sem `new` em runtime

## 🔧 Métodos Disponíveis

### Comparação de Performance (ESP32-S2 @ 240MHz)

Sistema de teste: **6 estados × 3 controles**, **800 000 execuções** sob dinâmica real de quadricóptero (CBA 2026).

#### Tempo de execução

| Método | Identificador | Média (μs) | σ (μs) | Pior caso (μs) | Falhas / 800k | Iter. (méd ± σ) | Iter. (pior) |
|--------|---------------|------------|--------|----------------|---------------|------------------|--------------|
| **SDA (base)** | `"SDA"` | **8663,59** | **146,49** | **8750** | **0** | 7,99 ± 0,13 | 8 |
| SDA Single-Shift | `"SDA_SS"` | 8413,26 | 541,55 | 10902 | 55 349 | 7,79 ± 0,52 | 10 |
| ASDA (Adaptativo) | `"ASDA"` | 9114,64 | **24,64** | 9180 | 0 | 8,00 ± 0,00 | 8 |
| SDA Scaled | `"SDA_SCALED"` | 8854,91 | 327,95 | 11024 | 48 302 | 8,08 ± 0,31 | 10 |
| SDA-ADDA | `"ADDA"` | 10754,63 | 556,55 | 13654 | 40 314 | 7,96 ± 0,42 | 10 |
| Van Dooren | `"VAN_DOOREN"` | 39281,13 | 3637,45 | 126877 | 0 | 1,00 ± 0,00 | 1 |
| Iterativo | `"ITERATIVE"` | 11912,00 | 2868,74 | 16884 | 0 | 22,49 ± 5,64 | 32 |

#### Precisão (erro RMS dos ganhos K vs. método iterativo)

| Método | Erro RMS |
|--------|----------|
| **SDA (base)** | **9,36 × 10⁻⁷** |
| ASDA | 1,92 × 10⁻⁵ |
| Van Dooren | 5,53 × 10⁻⁵ |
| SDA-ADDA | 1,85 × 10⁻⁴ |
| SDA-SS | 3,22 × 10⁻⁴ |
| SDA-Scaled | 3,43 × 10⁻⁴ |
| Iterativo | — (referência) |

### Descrição dos Métodos

#### 1. SDA (Structure-preserving Doubling Algorithm) ⭐ **RECOMENDADO**
```cpp
lqr.computeGains("SDA");
```
- **Melhor para**: Uso geral em tempo real — **escolha padrão para o projeto**
- **Características**: Convergência quadrática, preserva estrutura simpléctica
- **Complexidade**: O(n³) por iteração, ~8 iterações em regime
- **Robustez comprovada**: 0 falhas em 800 000 execuções; menor erro RMS (9,36×10⁻⁷)

#### 2. SDA-ss (SDA com Single Shift)
```cpp
lqr.computeGains("SDA_SS");
```
- **Melhor para**: Sistemas com autovalores próximos de 1
- **Características**: Parâmetro de shift γ melhora convergência em casos difíceis
- **Trade-off**: Ligeiramente mais lento, melhor robustez

#### 3. ASDA (Adaptive SDA)
```cpp
lqr.computeGains("ASDA");
```
- **Melhor para**: Precisão máxima
- **Características**: Escalonamento adaptativo durante iterações
- **Vantagem**: Menor erro RMS entre os métodos SDA

#### 4. SDA Scaled
```cpp
lqr.computeGains("SDA_SCALED");
```
- **Melhor para**: Sistemas mal-condicionados
- **Características**: Pré-escalonamento do pencil Hamiltoniano
- **Uso**: Quando matriz A tem normas de linha muito diferentes

#### 5. ADDA (Alternating-Directional Doubling Algorithm)
```cpp
lqr.computeGains("ADDA");
```
- **Características**: Usa duas matrizes auxiliares V e W simétricas
- **⚠️ Atenção**: ~5 % de taxa de falha sob excitações estocásticas do voo (40 314 / 800 k testes). Apenas para sistemas bem-condicionados e suaves.

#### 6. Van Dooren (Extended Symplectic Pencil)
```cpp
lqr.computeGains("VAN_DOOREN");
```
- **Melhor para**: Robustez numérica
- **Características**: Usa pencil estendido (2n+m)×(2n+m) com deflação QR
- **Trade-off**: Significativamente mais lento, usa decomposição QZ

#### 7. Iterativo (Riccati Iteration)
```cpp
lqr.computeGains("ITERATIVE");
```
- **Melhor para**: Alta precisão, warm-start
- **Características**: Iteração direta da equação de Riccati
- **Vantagem**: Excelente com warm-start (solução anterior como inicial)

## 📊 Exemplo de Matriz K Resultante

Para o sistema de atitude de quadricóptero (6×3):

```
K [3 x 6]:
  [   0.012991,    0.000003,    0.000479,    0.001459,   -0.000000,   -0.000002]
  [   0.000034,    0.012779,   -0.001173,    0.000001,    0.001458,   -0.000002]
  [  -0.000775,    0.002094,    0.023238,   -0.000005,   -0.000000,    0.002624]
```

## 🚀 Uso Básico

### Inicialização

```cpp
#include <AutoLQR.h>

// Criar controlador: (número de estados, número de controles)
AutoLQR lqr(6, 3);

// Definir matrizes de custo Q e R
float Q[36] = {
    100, 0, 0, 0, 0, 0,
    0, 100, 0, 0, 0, 0,
    0, 0, 100, 0, 0, 0,
    0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 1
};

float R[9] = {
    1, 0, 0,
    0, 1, 0,
    0, 0, 1
};

lqr.setCostMatrices(Q, R);
```

### Loop de Controle SDRE

```cpp
void loop() {
    // 1. Atualizar matrizes do sistema baseado no estado atual
    updateSystemMatrix(roll, pitch, yaw, p, q, r);
    
    // 2. Discretizar e configurar
    lqr.setStateMatrix(Ad);   // Matriz de estados discretizada
    lqr.setInputMatrix(Bd);   // Matriz de entrada discretizada
    
    // 3. Calcular ganhos (SDA base = padrão do projeto)
    lqr.computeGains("SDA");
    
    // 4. Calcular ação de controle
    lqr.updateState(current_state);
    lqr.updateReference(reference);
    
    float control[3];
    lqr.calculateControl(control);
    
    // 5. Aplicar aos atuadores
    applyControl(control);
}
```

### Exportar Ganhos

```cpp
float K[18];  // 3 controles × 6 estados
lqr.exportGains(K);

// Obter solução de Riccati P
const float* P = lqr.getRicattiSolution();
```

## 📐 API Completa

### Classe AutoLQR

```cpp
class AutoLQR {
public:
    // Construtor/Destrutor
    AutoLQR(int stateSize, int controlSize);
    ~AutoLQR();
    
    // Configuração do sistema
    bool setStateMatrix(const float* A);      // Matriz de estados (n×n)
    bool setInputMatrix(const float* B);      // Matriz de entrada (n×m)
    bool setCostMatrices(const float* Q, const float* R);  // Matrizes de custo
    
    // Cálculo de ganhos
    bool computeGains(const char* method);    // "SDA", "ADDA", "ITERATIVE", etc.
    void setGains(const float* K);            // Definir ganhos manualmente
    
    // Controle
    void updateState(const float* state);     // Atualizar estado atual
    void updateReference(const float* ref);   // Atualizar referência
    void calculateControl(float* u);          // Calcular u = -Kx + Kr*r
    
    // Exportação
    bool exportGains(float* K);               // Exportar matriz K
    bool exportKr(float* Kr);                 // Exportar ganho de referência
    const float* getRicattiSolution() const;  // Obter matriz P
    
    // Utilitários
    bool isSystemControllable();              // Verificar controlabilidade
    float calculateExpectedCost();            // Calcular custo x'Px
    float estimateConvergenceTime(float threshold);
};
```

## ⚙️ Recomendações de Uso

### Para controle em tempo real (escolha padrão do projeto)
```cpp
// SDA base: 0 falhas em 800k execuções, pior caso 8750 us, erro RMS 9.36e-7
lqr.computeGains("SDA");
```

### Quando previsibilidade temporal é crítica (jitter mínimo)
```cpp
// ASDA: desvio padrão de apenas 24.64 us (vs 146.49 us do SDA base)
lqr.computeGains("ASDA");
```

### Para validação offline / referência de precisão
```cpp
// Iterativo com warm-start: alta precisão, lento e jitter alto.
// NÃO recomendado em malha de tempo real.
lqr.computeGains("ITERATIVE");
```

## 🔬 Detalhes de Implementação

### Equação DARE Resolvida

$$A^T P A - P - A^T P B (R + B^T P B)^{-1} B^T P A + Q = 0$$

### Cálculo do Ganho K

$$K = (R + B^T P B)^{-1} B^T P A$$

### Controle Aplicado

$$u = -K x + K_r r$$

onde $K_r$ é o ganho de referência para tracking.

## 📁 Arquivos

```
AUTOLQR/
├── AutoLQR.cpp / .h            # Classe principal (estende MatrixOperations)
├── MatrixOperations.cpp / .h   # Operações lineares (inv, mul, transp, QR, etc.)
└── README.md                   # Esta documentação
```

## 📚 Referências

1. **SDA/ADDA**: Chu, E. K.-W., et al. "Structure-preserving doubling algorithms for Riccati equations." Numerical Linear Algebra with Applications, 2005.

2. **Van Dooren**: P. van Dooren, "A Generalized Eigenvalue Approach For Solving Riccati Equations", SIAM J. Sci. Stat. Comput., Vol.2(2), 1981.

3. **SDRE**: Çimen, T. "State-Dependent Riccati Equation (SDRE) Control: A Survey." IFAC Proceedings Volumes, 2008.

## 📄 Licença

MIT License - Veja o arquivo LICENSE no diretório raiz do projeto.
