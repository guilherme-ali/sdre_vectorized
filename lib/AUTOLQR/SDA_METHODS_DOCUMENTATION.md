# Métodos SDA Melhorados para Solução da Equação de Riccati (DARE)

Este documento descreve os métodos SDA (Structure-preserving Doubling Algorithm) implementados para resolver a Equação Algébrica de Riccati Discreta (DARE).

## Visão Geral

A DARE é fundamental para o controle LQR discreto:

```
A'·P·A - P - A'·P·B·(R + B'·P·B)^(-1)·B'·P·A + Q = 0
```

O SDA é um algoritmo de dobramento que preserva a estrutura Hamiltoniana do problema, oferecendo convergência quadrática.

## Resultados do Benchmark (ESP32-S2, Sistema 6x3)

| Método | Tempo Médio (µs) | Observação |
|--------|-----------------|------------|
| **SDA (orig)** | **~8300** | **Mais rápido para sistemas bem condicionados** |
| SDA_SS | ~8500 | +2% overhead do shift adaptativo |
| SDA_SCALED | ~8600 | +4% overhead do escalonamento |
| ASDA | ~9000 | +8% overhead do balanceamento |
| ITERATIVE | ~13300 | Com warm start |
| SCHUR | ~39000 | Usa decomposição QZ (Eigen) |
| VAN_DOOREN | ~55800 | Pencil estendido (Eigen) |

**Conclusão:** Para sistemas bem condicionados (como controle de atitude de drones), o SDA original é a melhor escolha. Os métodos melhorados são mais úteis em cenários específicos.

## Métodos Implementados

### 1. SDA Original (`SDA`) - **RECOMENDADO PARA USO GERAL**

O método SDA básico usa a seguinte iteração:

```
Ak+1 = Ak·(I + Gk·Hk)^(-1)·Ak
Gk+1 = Gk + Ak·(I + Gk·Hk)^(-1)·Gk·Ak'
Hk+1 = Hk + Ak'·Hk·(I + Gk·Hk)^(-1)·Ak
```

**Inicialização:**
- A₀ = A
- G₀ = B·R^(-1)·B'
- H₀ = Q

**Uso:** `lqr.computeGains("SDA")`

---

### 2. SDA com Single Shift (`SDA_SS`)

Versão que usa parâmetro de shift γ para melhorar convergência em casos difíceis.

**Quando usar:**
- ✅ Sistemas com autovalores muito próximos ao círculo unitário (|λ| ≈ 1)
- ✅ Quando o SDA original não converge
- ❌ **NÃO usar** para sistemas bem condicionados (adiciona overhead)

**Uso:** `lqr.computeGains("SDA_SS")`

---

### 3. SDA Adaptativo (`ASDA`)

Aplica escalonamento adaptativo a cada iteração para balancear Gk e Hk.

**Quando usar:**
- ✅ Sistemas com grande diferença entre pesos Q e R (ex: Q[0,0] = 10000, R[0,0] = 0.001)
- ✅ Problemas de overflow/underflow numérico
- ❌ **NÃO usar** para sistemas balanceados (adiciona overhead)

**Uso:** `lqr.computeGains("ASDA")`

---

### 4. SDA Escalonado (`SDA_SCALED`)

Pré-condiciona o sistema antes das iterações.

**Quando usar:**
- ✅ Matriz A com elementos de magnitudes muito diferentes
- ✅ Linhas de A com normas muito desiguais
- ❌ **NÃO usar** se A já está bem balanceada (adiciona overhead)

**Uso:** `lqr.computeGains("SDA_SCALED")`

---

## Guia de Seleção de Método

```
Sistema bem condicionado? (controle típico de drones, robôs)
├── SIM → Use SDA (original)
└── NÃO → Continue...

Autovalores próximos de 1?
├── SIM → Use SDA_SS
└── NÃO → Continue...

Grande diferença Q/R?
├── SIM → Use ASDA
└── NÃO → Continue...

Matriz A desbalanceada?
├── SIM → Use SDA_SCALED
└── NÃO → Use SDA (original)
```

## Uso no Código

```cpp
#include <AutoLQR.h>

AutoLQR lqr(6, 3);  // 6 estados, 3 controles

lqr.setStateMatrix(A);
lqr.setInputMatrix(B);
lqr.setCostMatrices(Q, R);

// Para a maioria dos casos:
lqr.computeGains("SDA");

// Para casos especiais:
lqr.computeGains("SDA_SS");      // Autovalores ~1
lqr.computeGains("ASDA");        // Q/R desbalanceados
lqr.computeGains("SDA_SCALED");  // A desbalanceada
```

## Referências

1. Chu, E.K.-W., Fan, H.-Y., Lin, W.-W., Wang, C.-S. "Structure-preserving algorithms for periodic discrete-time algebraic Riccati equations"
2. Lin, W.-W., Xu, S.-F. "Convergence analysis of structure-preserving doubling algorithms for Riccati-type matrix equations"
3. Guo, C.-H. "Convergence rate of an iterative method for a nonlinear matrix equation"
