# BiquadFilter

Filtro Butterworth digital de 2ª ordem (forma direta II transposta) para amostragem de sinais de IMU. Usado em `main.cpp` por eixo (`ax,ay,az,gx,gy,gz`) como anti-aliasing digital antes do filtro de fusão Madgwick.

## API

```cpp
#include <BiquadFilter.h>

BiquadFilter bq;
bq.begin(cutoff_hz, sample_rate_hz); // calcula coeficientes a partir do cutoff
float y = bq.update(x);              // chamar a cada amostra
```

## Notas

- **Cutoff < Nyquist:** a frequência de corte precisa ficar abaixo de `sample_rate / 2`. No projeto roda a 200 Hz com cutoff de 80 Hz.
- **Inicialização:** os estados internos partem de zero; espere alguns ciclos para o filtro estabilizar antes de usar a saída para controle.
- **Sem alocação dinâmica:** cada instância carrega apenas coeficientes + 2 floats de estado.

## Arquivos

```
BiquadFilter/
├── BiquadFilter.cpp / .h
└── README.md
```
