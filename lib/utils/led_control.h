#ifndef LED_CONTROL_H
#define LED_CONTROL_H

#include <Arduino.h>

// ============= DEFINIÇÃO DOS PINOS DOS LEDS (conforme tabela) =============
// LED_WHITE não é controlável por software (sempre ligado quando há energia)
#define LED_BLUE_PIN    7     // GPIO7 - LED_BLUE (SENSORS_CALIBRATION / SYSTEM_READY)
#define LED_GREEN_PIN   9     // GPIO9 - LED_GREEN (UDP_RX)
#define LED_RED_PIN     8     // GPIO8 - LED_RED (LOW_POWER)

// ============= PINO ADC PARA LEITURA DE BATERIA =============
#define BATTERY_PIN     2     // GPIO2 - ADC_7_BAT (VBAT/2)

// ============= THRESHOLDS DE BATERIA (LiPo 1S - 3.7V Nominal) =============
#define BATTERY_LOW_VOLTAGE   3.4f    // Tensão considerada baixa (~20% restante)
#define BATTERY_CRITICAL_VOLTAGE 3.2f // Tensão crítica (mínimo seguro para LiPo)
#define BATTERY_FULL_VOLTAGE  4.2f    // Tensão máxima carregada (1S LiPo)
#define BATTERY_NOMINAL_VOLTAGE 3.7f  // Tensão nominal da bateria
#define VOLTAGE_DIVIDER_RATIO 2.0f    // Divisor de tensão (VBAT/2)

// ============= ESTADOS DOS LEDS =============
enum LEDState {
    LED_OFF,
    LED_FULLY_LIT,
    LED_BLINKING,
    LED_BLINKING_SLOWLY
};

class LEDControl {
private:
    // Estados atuais dos LEDs (LED branco não é controlável)
    LEDState blue_state;
    LEDState green_state;
    LEDState red_state;
    
    // Controle de blinking
    unsigned long last_blink_time;
    bool blink_state;
    unsigned long blink_interval;
    
    // Bateria
    float battery_voltage;
    bool low_battery_detected;
    unsigned long last_battery_check;
    
    // Métodos privados
    void updateLED(int pin, LEDState state);
    float readBatteryVoltage();
    
public:
    LEDControl();
    void begin();
    void update();
    
    // Controle individual dos LEDs (LED branco não é controlável)
    void setSensorsCalibration(bool active);
    void setSystemReady(bool ready);
    void setUDPReceiving(bool receiving);
    void setLowPower(bool low);
    
    // Bateria
    float getBatteryVoltage();
    float getBatteryPercentage();
    bool isLowBattery();
    bool isCriticalBattery();
    
    // Debug
    void printStatus();
};

#endif // LED_CONTROL_H
