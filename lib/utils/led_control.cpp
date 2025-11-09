#include "led_control.h"

LEDControl::LEDControl() {
    blue_state = LED_OFF;
    green_state = LED_OFF;
    red_state = LED_OFF;
    
    last_blink_time = 0;
    blink_state = false;
    blink_interval = 500; // 500ms para blinking normal
    
    battery_voltage = 0.0f;
    low_battery_detected = false;
    last_battery_check = 0;
}

void LEDControl::begin() {
    // Configura os pinos dos LEDs como saída (LED branco não é controlável)
    pinMode(LED_BLUE_PIN, OUTPUT);
    pinMode(LED_GREEN_PIN, OUTPUT);
    pinMode(LED_RED_PIN, OUTPUT);
    
    // Configura o pino de leitura da bateria
    pinMode(BATTERY_PIN, INPUT);
    
    // Inicializa todos os LEDs apagados
    digitalWrite(LED_BLUE_PIN, LOW);
    digitalWrite(LED_GREEN_PIN, LOW);
    digitalWrite(LED_RED_PIN, LOW);
    
    // Lê tensão inicial da bateria
    battery_voltage = readBatteryVoltage();
}

void LEDControl::update() {
    unsigned long current_time = millis();
    
    // Atualiza estado de blinking a cada intervalo
    if (current_time - last_blink_time >= blink_interval) {
        blink_state = !blink_state;
        last_blink_time = current_time;
    }
    
    // Atualiza cada LED (LED branco não é controlável)
    updateLED(LED_BLUE_PIN, blue_state);
    updateLED(LED_GREEN_PIN, green_state);
    updateLED(LED_RED_PIN, red_state);
    
    // Verifica bateria a cada 1 segundo
    if (current_time - last_battery_check >= 1000) {
        battery_voltage = readBatteryVoltage();
        last_battery_check = current_time;
        
        // Atualiza estado de bateria baixa
        if (battery_voltage < BATTERY_LOW_VOLTAGE && battery_voltage > 0.1f) {
            low_battery_detected = true;
        } else if (battery_voltage > BATTERY_LOW_VOLTAGE + 0.2f) {
            low_battery_detected = false;
        }
    }
}

void LEDControl::updateLED(int pin, LEDState state) {
    if (pin < 0) return; // LED não configurado
    
    switch (state) {
        case LED_OFF:
            digitalWrite(pin, LOW);
            break;
            
        case LED_FULLY_LIT:
            digitalWrite(pin, HIGH);
            break;
            
        case LED_BLINKING:
            // Blinking rápido (500ms on/off)
            digitalWrite(pin, blink_state ? HIGH : LOW);
            break;
            
        case LED_BLINKING_SLOWLY:
            // Blinking lento (1000ms on/off) - usa blink_state mas com intervalo maior
            // Para simplicidade, usa o mesmo blink_state mas poderia ter lógica separada
            digitalWrite(pin, blink_state ? HIGH : LOW);
            break;
    }
}

float LEDControl::readBatteryVoltage() {
    // Lê o ADC (0-4095 para ESP32, 12-bit ADC)
    int adc_value = analogRead(BATTERY_PIN);
    
    // Converte para tensão (0-3.3V no ADC)
    float adc_voltage = (adc_value / 4095.0f) * 3.3f;
    
    // Aplica o divisor de tensão (VBAT/2)
    float battery_volts = adc_voltage * VOLTAGE_DIVIDER_RATIO;
    
    return battery_volts;
}

// ============= CONTROLE DOS LEDS CONFORME TABELA =============
// LED branco não é controlável (sempre aceso quando há energia)

void LEDControl::setSensorsCalibration(bool active) {
    if (active) {
        blue_state = LED_BLINKING_SLOWLY;
        blink_interval = 1000; // Blinking lento: 1 segundo
    }
}

void LEDControl::setSystemReady(bool ready) {
    if (ready) {
        blue_state = LED_BLINKING;
        blink_interval = 500; // Blinking rápido: 500ms
    } else {
        blue_state = LED_OFF;
    }
}

void LEDControl::setUDPReceiving(bool receiving) {
    green_state = receiving ? LED_BLINKING : LED_OFF;
}

void LEDControl::setLowPower(bool low) {
    red_state = low ? LED_FULLY_LIT : LED_OFF;
}

// ============= FUNÇÕES DE BATERIA =============

float LEDControl::getBatteryVoltage() {
    return battery_voltage;
}

float LEDControl::getBatteryPercentage() {
    // Calcula porcentagem baseado em LiPo 1S (3.0V a 4.2V)
    if (battery_voltage <= BATTERY_CRITICAL_VOLTAGE) return 0.0f;
    if (battery_voltage >= BATTERY_FULL_VOLTAGE) return 100.0f;
    
    float percentage = ((battery_voltage - BATTERY_CRITICAL_VOLTAGE) / 
                       (BATTERY_FULL_VOLTAGE - BATTERY_CRITICAL_VOLTAGE)) * 100.0f;
    
    return constrain(percentage, 0.0f, 100.0f);
}

bool LEDControl::isLowBattery() {
    return (battery_voltage < BATTERY_LOW_VOLTAGE && battery_voltage > 0.1f);
}

bool LEDControl::isCriticalBattery() {
    return (battery_voltage < BATTERY_CRITICAL_VOLTAGE && battery_voltage > 0.1f);
}

void LEDControl::printStatus() {
    Serial.println("╔════════════════════════════════════════════════════════╗");
    Serial.println("║                   STATUS DOS LEDS                      ║");
    Serial.println("╚════════════════════════════════════════════════════════╝");
    
    // Status dos LEDs (LED branco não é controlável)
    const char* state_names[] = {"OFF", "FULLY_LIT", "BLINKING", "BLINKING_SLOWLY"};
    
    Serial.println("⚪ WHITE (Power):       Sempre aceso (hardware)");
    Serial.printf("🔵 BLUE (System):      %s\n", state_names[blue_state]);
    Serial.printf("🟢 GREEN (UDP):        %s\n", state_names[green_state]);
    Serial.printf("🔴 RED (Low Battery):  %s\n", state_names[red_state]);
    
    Serial.println();
    
    // Status da bateria
    Serial.println("╔════════════════════════════════════════════════════════╗");
    Serial.println("║                  STATUS DA BATERIA                     ║");
    Serial.println("╚════════════════════════════════════════════════════════╝");
    Serial.printf("🔋 Tensão:      %.2fV\n", battery_voltage);
    Serial.printf("📊 Carga:       %.1f%%\n", getBatteryPercentage());
    
    if (isCriticalBattery()) {
        Serial.println("⚠️  Status:      CRÍTICA! POUPE IMEDIATAMENTE!");
    } else if (isLowBattery()) {
        Serial.println("⚠️  Status:      BAIXA - Considere recarregar");
    } else if (battery_voltage > BATTERY_LOW_VOLTAGE + 0.5f) {
        Serial.println("✅ Status:      OK");
    } else {
        Serial.println("ℹ️  Status:      Normal");
    }
    
    Serial.println("════════════════════════════════════════════════════════\n");
}
