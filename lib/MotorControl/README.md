# MotorControl

Controle PWM dos 4 motores brushed do drone, com sequência de armar/desarmar (ESC),
limites configuráveis e mapeamento ω² → throttle.

## Configuração de hardware (fixa em `MotorControl.h`)

| Motor | GPIO | Canal LEDC | Posição (X-quad)   | Sentido |
|-------|------|------------|--------------------|---------|
| M1    | 5    | 0          | Frente direita (FR) | CW      |
| M2    | 4    | 1          | Trás direita (RR)   | CCW     |
| M3    | 3    | 2          | Trás esquerda (RL)  | CW      |
| M4    | 6    | 3          | Frente esquerda (FL) | CCW    |

- **PWM:** 25 kHz, 10-bit (0–1023)
- **Throttle range:** 0–100 % configurável via `setThrottleLimits(min, max)`

## API

```cpp
#include <MotorControl.h>

MotorControl motors;

void setup() {
    motors.begin();
    motors.setThrottleLimits(0, 100);
    motors.setOmegaSqLimits(0, MAX_OMEGA * MAX_OMEGA);
    // motors.calibrateESCs(); // descomentar uma única vez (veja docs/MOTOR_CALIBRATION_GUIDE.md)
}

void loop() {
    if (received_first_command) motors.armMotors();

    // Após calcular ω² dos 4 motores pela alocação X-quad:
    motors.setMotorsFromOmegaSq(w1_sq, w2_sq, w3_sq, w4_sq);

    // Segurança:
    motors.stopAllMotors();   // zera throttle imediatamente
    motors.disarmMotors();    // desarma (retorna ao standby)
}
```

## Calibração de ESC

Procedimento descrito em [`docs/MOTOR_CALIBRATION_GUIDE.md`](../../docs/MOTOR_CALIBRATION_GUIDE.md).

Para medir `MOTOR_B_COEFF` (empuxo): rodar `test/motor_calibration_test.cpp`.

## Arquivos

```
MotorControl/
├── MotorControl.cpp / .h
└── README.md
```
