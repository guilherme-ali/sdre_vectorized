# Telemetry

Buffer circular em RAM para telemetria de voo com persistência opcional em LittleFS. Implementado como header-only (template-free, sem alocação dinâmica). Default: 1000 amostras × ~68 B/amostra ≈ 68 KB em RAM.

## Por que persistir em flash

O ESP32 reseta quando o Serial Monitor abre. Sem persistência, o buffer de voo se perde antes de podermos ler. A solução: chamar `saveToFile()` ao desarmar os motores (operação de ~100–300 ms, ok com drone parado) e `loadFromFile()` no `setup()` após `LittleFS.begin()`.

## API

```cpp
#include <Telemetry.h>

Telemetry telemetry;

// No loop de controle (custo ~1 us — apenas writes em RAM):
telemetry.log(millis(),
              roll, pitch, yaw,
              roll_ref, pitch_ref, yaw_ref,
              p, q, r,
              u0, u1, u2,                // torques [N·m]
              w1_sq, w2_sq, w3_sq, w4_sq); // ω² dos motores

// Ao desarmar:
telemetry.saveToFile();          // /telem.bin (binary raw struct)

// No setup, após LittleFS.begin():
if (telemetry.loadFromFile()) { /* restaurado */ }

// Sob demanda (drone desarmado):
telemetry.dumpCSV(Serial);       // CSV no Serial Monitor
telemetry.reset();
```

## Schema da amostra (`Sample`)

```cpp
struct Sample {
    uint32_t t_ms;
    float roll, pitch, yaw;                 // rad (medido pelo Madgwick)
    float roll_ref, pitch_ref, yaw_ref;     // rad (setpoint do controle remoto)
    float p, q, r;                          // rad/s (taxas no corpo)
    float u0, u1, u2;                       // torques [N·m]: roll, pitch, yaw
    float w1_sq, w2_sq, w3_sq, w4_sq;       // rad²/s² (entrada dos ESCs)
};
```

## Workflow típico

1. Drone armado: `log()` a cada ciclo de controle (200 Hz × 5 s ≈ 1000 amostras).
2. Drone desarma → `saveToFile()` automático nos callbacks `onClientDisconnected` e no failsafe de tilt.
3. PC abre Serial Monitor → ESP32 reseta → `setup()` chama `loadFromFile()`.
4. Usuário envia `'D'` no Serial → `dumpCSV()` despeja o CSV.
5. Importar CSV no `python/plot_telemetry.py` para análise.

## Arquivos

```
Telemetry/
├── Telemetry.h    # Header-only, ~130 linhas
└── README.md
```
