# AutoLQR - Biblioteca de Controle LQR Adaptativo

Biblioteca otimizada para cálculo de ganhos LQR em tempo real, projetada para sistemas embarcados como ESP32. Implementa múltiplos algoritmos para resolver a Equação Algébrica de Riccati Discreta (DARE).

## 📋 Características

- **7 métodos de solução DARE** implementados
- **Operações matriciais otimizadas** para sistemas de pequeno/médio porte
- **Filtro de Kalman** integrado para estimação de estados
- **Warm-start** para métodos iterativos
- **Baixo consumo de memória** com alocação dinâmica controlada

## 🔧 Métodos Disponíveis

### Comparação de Performance (ESP32 @ 240MHz)

Sistema de teste: **6 estados × 3 controles**, 100 iterações

| Método | Identificador | Tempo Médio (μs) | σ (μs) | σ da Média | Erro RMS |
|--------|---------------|------------------|--------|------------|----------|
| **SDA** | `"SDA"` | 8303.78 | 12.12 | 1.2117 | 4.99e-05 |
| **SDA Single-Shift** | `"SDA_SS"` | 8503.93 | 12.91 | 1.2913 | 4.98e-05 |
| **ASDA (Adaptativo)** | `"ASDA"` | 9020.53 | 11.46 | 1.1462 | 1.69e-05 |
| **SDA Scaled** | `"SDA_SCALED"` | 8666.85 | 12.05 | 1.2054 | 4.69e-05 |
| **ADDA** | `"ADDA"` | 8277.27 | 13.36 | 1.3356 | 4.99e-05 |
| **Van Dooren** | `"VAN_DOOREN"` | 41915.83 | 5825.84 | 582.58 | 4.05e-05 |
| **Iterativo** | `"ITERATIVE"` | 13333.36 | 2125.54 | 212.55 | Referência |

### Descrição dos Métodos

#### 1. SDA (Structure-preserving Doubling Algorithm)
```cpp
lqr.computeGains("SDA");
```
- **Melhor para**: Uso geral em tempo real
- **Características**: Convergência quadrática, preserva estrutura simpléctica
- **Complexidade**: O(n³) por iteração, ~15-20 iterações típicas

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
- **Melhor para**: Melhor velocidade com boa precisão ⭐ **RECOMENDADO**
- **Características**: Usa duas matrizes auxiliares V e W simétricas
- **Vantagem**: Mais rápido que SDA padrão

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
    
    // 3. Calcular ganhos (escolher método)
    lqr.computeGains("ADDA");
    
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

### Para Controle em Tempo Real (< 10ms ciclo)
```cpp
// Usar ADDA - melhor velocidade com precisão adequada
lqr.computeGains("ADDA");
```

### Para Máxima Precisão
```cpp
// Usar ITERATIVE com warm-start
// A matriz P da iteração anterior é mantida automaticamente
lqr.computeGains("ITERATIVE");
```

### Para Sistemas Mal-Condicionados
```cpp
// Usar SDA_SCALED ou ASDA
lqr.computeGains("SDA_SCALED");
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
├── AutoLQR.cpp          # Implementação principal
├── AutoLQR.h            # Interface da classe
├── KalmanFilter.cpp/h   # Filtro de Kalman
├── MatrixOperations.cpp/h # Operações matriciais
└── README.md            # Esta documentação
```

## 📚 Referências

1. **SDA/ADDA**: Chu, E. K.-W., et al. "Structure-preserving doubling algorithms for Riccati equations." Numerical Linear Algebra with Applications, 2005.

2. **Van Dooren**: P. van Dooren, "A Generalized Eigenvalue Approach For Solving Riccati Equations", SIAM J. Sci. Stat. Comput., Vol.2(2), 1981.

3. **SDRE**: Çimen, T. "State-Dependent Riccati Equation (SDRE) Control: A Survey." IFAC Proceedings Volumes, 2008.

## 📄 Licença

MIT License - Veja o arquivo LICENSE no diretório raiz do projeto.
