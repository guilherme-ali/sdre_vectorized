# utils

Helpers compartilhados pelo `main.cpp`: drivers de sensores (MPU6050, QMC5883L), controle de LEDs/bateria e a alocação de controle X-quad.

## Conteúdo

| Arquivo                  | Responsabilidade                                                                 |
|--------------------------|----------------------------------------------------------------------------------|
| `utils.h` / `utils.cpp`  | Inicialização, calibração e leitura do MPU6050; driver bare-metal do QMC5883L (I2C direto); alocação X-quad (torques → ω²); helpers de display no Serial (incluindo `displayGains` para K e Kr do SDRE). |
| `led_control.h` / `.cpp` | Classe `LEDControl` — gerencia LEDs (blue/green/red), modos blink, leitura de bateria via ADC e thresholds (low/critical). |

## Conexões I2C

- **SDA** = GPIO11, **SCL** = GPIO10, **clock** = 400 kHz
- MPU6050 em `0x68`
- QMC5883L em `0x0D` (compartilha barramento; `start_QMC5883L` reusa o `Wire` já inicializado pelo MPU)

## API principal

```cpp
// Sensores
void start_IMU_MPU6050(Adafruit_MPU6050& mpu);
void read_MPU6050(...);                 // saída: ax,ay,az em g; gx,gy,gz em rad/s
void start_QMC5883L(TwoWire& wire);
void setQMC5883LCalibration(...);       // hard-iron offsets + soft-iron scales
void read_QMC5883L(float& mx, float& my, float& mz);  // saída em μT

// Alocação de controle (X-quad)
void calculateMotorOmegaSq(float thrust, float u_torques[3],
                           float b, float d, float L_arm,
                           float& w1_sq, float& w2_sq, float& w3_sq, float& w4_sq);

// Display (Serial Monitor, modo DEBUG)
void displayStates(float states[]);
void displayControlSignals(float u[], float thrust);
void displayMotorOmegaSq(...);
void displayMotorOmegaSqDetailed(float w1_sq, ...);
```

## LEDs e bateria

Detalhes em [`docs/LED_BATTERY_GUIDE.md`](../../docs/LED_BATTERY_GUIDE.md).

```cpp
LEDControl leds;
leds.begin();
leds.setSystemReady(true);     // azul piscando
leds.setUDPReceiving(true);    // verde piscando
leds.setLowPower(true);        // vermelho aceso

if (leds.isCriticalBattery()) { /* halt */ }
```

## Calibração

Antes do primeiro voo, rodar:

- `test/calibrate_mpu.cpp` — offsets do MPU6050 (cole os valores em `main.cpp`)
- `test/calibrate_magnetometer.cpp` — offsets hard/soft-iron do QMC5883L (idem)
