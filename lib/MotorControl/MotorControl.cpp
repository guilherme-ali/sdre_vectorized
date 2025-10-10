#include "MotorControl.h"

MotorControl::MotorControl()
    : motor1_throttle(0)
    , motor2_throttle(0)
    , motor3_throttle(0)
    , motor4_throttle(0)
    , armed(false)
    , min_throttle(MIN_THROTTLE)
    , max_throttle(MAX_THROTTLE)
    , min_omega_sq(0)
    , max_omega_sq(10000)
{
}

MotorControl::~MotorControl()
{
    stopAllMotors();
}

bool MotorControl::begin()
{
    // Configura os canais PWM
    ledcSetup(MOTOR_1_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcSetup(MOTOR_2_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcSetup(MOTOR_3_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcSetup(MOTOR_4_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
    
    // Associa os pinos aos canais PWM
    ledcAttachPin(MOTOR_1_PIN, MOTOR_1_CHANNEL);
    ledcAttachPin(MOTOR_2_PIN, MOTOR_2_CHANNEL);
    ledcAttachPin(MOTOR_3_PIN, MOTOR_3_CHANNEL);
    ledcAttachPin(MOTOR_4_PIN, MOTOR_4_CHANNEL);
    
    // Inicializa todos os motores em 0
    stopAllMotors();
    
    Serial.println("=========================================");
    Serial.println("Sistema de controle de motores inicializado!");
    Serial.print("PWM Frequência: "); Serial.print(PWM_FREQUENCY); Serial.println(" Hz");
    Serial.print("PWM Resolução: "); Serial.print(PWM_RESOLUTION); Serial.println(" bits");
    Serial.println("\nMapeamento de motores:");
    Serial.print("Motor 1 (GPIO"); Serial.print(MOTOR_1_PIN); Serial.println(") - Frente-Direita");
    Serial.print("Motor 2 (GPIO"); Serial.print(MOTOR_2_PIN); Serial.println(") - Frente-Esquerda");
    Serial.print("Motor 3 (GPIO"); Serial.print(MOTOR_3_PIN); Serial.println(") - Trás-Direita");
    Serial.print("Motor 4 (GPIO"); Serial.print(MOTOR_4_PIN); Serial.println(") - Trás-Esquerda");
    Serial.println("=========================================");
    
    return true;
}

void MotorControl::setMotor1(float throttle)
{
    motor1_throttle = constrainThrottle(throttle);
    if (armed) {
        writeMotorPWM(MOTOR_1_CHANNEL, motor1_throttle);
    }
}

void MotorControl::setMotor2(float throttle)
{
    motor2_throttle = constrainThrottle(throttle);
    if (armed) {
        writeMotorPWM(MOTOR_2_CHANNEL, motor2_throttle);
    }
}

void MotorControl::setMotor3(float throttle)
{
    motor3_throttle = constrainThrottle(throttle);
    if (armed) {
        writeMotorPWM(MOTOR_3_CHANNEL, motor3_throttle);
    }
}

void MotorControl::setMotor4(float throttle)
{
    motor4_throttle = constrainThrottle(throttle);
    if (armed) {
        writeMotorPWM(MOTOR_4_CHANNEL, motor4_throttle);
    }
}

void MotorControl::setAllMotors(float throttle)
{
    setMotor1(throttle);
    setMotor2(throttle);
    setMotor3(throttle);
    setMotor4(throttle);
}

void MotorControl::setAllMotors(float motor1, float motor2, float motor3, float motor4)
{
    setMotor1(motor1);
    setMotor2(motor2);
    setMotor3(motor3);
    setMotor4(motor4);
}

void MotorControl::setMotorsFromOmegaSq(float w1_sq, float w2_sq, float w3_sq, float w4_sq)
{
    // Converte omega² para throttle (0-100%)
    float throttle1 = omegaSqToThrottle(w1_sq);
    float throttle2 = omegaSqToThrottle(w2_sq);
    float throttle3 = omegaSqToThrottle(w3_sq);
    float throttle4 = omegaSqToThrottle(w4_sq);
    
    setAllMotors(throttle1, throttle2, throttle3, throttle4);
}

void MotorControl::stopAllMotors()
{
    motor1_throttle = 0;
    motor2_throttle = 0;
    motor3_throttle = 0;
    motor4_throttle = 0;
    
    ledcWrite(MOTOR_1_CHANNEL, 0);
    ledcWrite(MOTOR_2_CHANNEL, 0);
    ledcWrite(MOTOR_3_CHANNEL, 0);
    ledcWrite(MOTOR_4_CHANNEL, 0);
    
    armed = false;
}

void MotorControl::armMotors()
{
    Serial.println("=========================================");
    Serial.println("Iniciando sequência de ARM dos motores...");
    Serial.println("ATENÇÃO: Motores vão começar a girar!");
    Serial.println("=========================================");
    
    // Sequência de armar: enviar sinal mínimo por 2 segundos
    for(int i = 3; i > 0; i--) {
        Serial.print("Armando em ");
        Serial.print(i);
        Serial.println(" segundos...");
        delay(1000);
    }
    
    // Envia sinal mínimo (idle) para todos os ESCs
    ledcWrite(MOTOR_1_CHANNEL, throttleToPWM(IDLE_THROTTLE));
    ledcWrite(MOTOR_2_CHANNEL, throttleToPWM(IDLE_THROTTLE));
    ledcWrite(MOTOR_3_CHANNEL, throttleToPWM(IDLE_THROTTLE));
    ledcWrite(MOTOR_4_CHANNEL, throttleToPWM(IDLE_THROTTLE));
    
    delay(2000);
    
    armed = true;
    Serial.println("Motores ARMADOS!");
    Serial.println("=========================================");
}

void MotorControl::disarmMotors()
{
    Serial.println("Desarmando motores...");
    stopAllMotors();
    Serial.println("Motores desarmados.");
}

bool MotorControl::isArmed() const
{
    return armed;
}

void MotorControl::calibrateESCs()
{
    Serial.println("=========================================");
    Serial.println("CALIBRAÇÃO DE ESCs");
    Serial.println("ATENÇÃO: Desconecte as hélices!");
    Serial.println("=========================================");
    
    for(int i = 5; i > 0; i--) {
        Serial.print("Iniciando calibração em ");
        Serial.print(i);
        Serial.println(" segundos...");
        delay(1000);
    }
    
    // Passo 1: Enviar sinal máximo
    Serial.println("Enviando sinal MÁXIMO...");
    ledcWrite(MOTOR_1_CHANNEL, PWM_MAX_VALUE);
    ledcWrite(MOTOR_2_CHANNEL, PWM_MAX_VALUE);
    ledcWrite(MOTOR_3_CHANNEL, PWM_MAX_VALUE);
    ledcWrite(MOTOR_4_CHANNEL, PWM_MAX_VALUE);
    
    Serial.println("Ligue a bateria AGORA!");
    delay(5000);
    
    // Passo 2: Enviar sinal mínimo
    Serial.println("Enviando sinal MÍNIMO...");
    ledcWrite(MOTOR_1_CHANNEL, 0);
    ledcWrite(MOTOR_2_CHANNEL, 0);
    ledcWrite(MOTOR_3_CHANNEL, 0);
    ledcWrite(MOTOR_4_CHANNEL, 0);
    
    delay(5000);
    
    Serial.println("Calibração concluída!");
    Serial.println("Os ESCs devem ter emitido bipes confirmando a calibração.");
    Serial.println("=========================================");
}

void MotorControl::setThrottleLimits(float min, float max)
{
    min_throttle = constrain(min, 0, 100);
    max_throttle = constrain(max, min_throttle, 100);
    
    Serial.print("Limites de throttle atualizados: ");
    Serial.print(min_throttle);
    Serial.print("% - ");
    Serial.print(max_throttle);
    Serial.println("%");
}

void MotorControl::setOmegaSqLimits(float min, float max)
{
    min_omega_sq = min;
    max_omega_sq = max;
    
    Serial.print("Limites de omega² atualizados: ");
    Serial.print(min_omega_sq);
    Serial.print(" - ");
    Serial.println(max_omega_sq);
}

void MotorControl::printMotorValues()
{
    Serial.print("GPIO"); Serial.print(MOTOR_1_PIN); Serial.print(": ");
    Serial.print(motor1_throttle, 2); Serial.print("%");
    
    Serial.print(" | GPIO"); Serial.print(MOTOR_2_PIN); Serial.print(": ");
    Serial.print(motor2_throttle, 2); Serial.print("%");
    
    Serial.print(" | GPIO"); Serial.print(MOTOR_3_PIN); Serial.print(": ");
    Serial.print(motor3_throttle, 2); Serial.print("%");
    
    Serial.print(" | GPIO"); Serial.print(MOTOR_4_PIN); Serial.print(": ");
    Serial.print(motor4_throttle, 2); Serial.print("%");
    
    Serial.print(" | Armed: "); Serial.println(armed ? "YES" : "NO");
}

// ============= FUNÇÕES PRIVADAS =============

float MotorControl::constrainThrottle(float throttle)
{
    return constrain(throttle, min_throttle, max_throttle);
}

int MotorControl::throttleToPWM(float throttle)
{
    // Mapeia throttle (0-100%) para PWM (0-1023)
    return map(throttle * 100, 0, 10000, 0, PWM_MAX_VALUE);
}

float MotorControl::omegaSqToThrottle(float omega_sq)
{
    // Garante que omega_sq não seja negativo
    if (omega_sq < 0) {
        omega_sq = 0;
    }
    
    // Mapeia omega² para throttle (0-100%)
    // Relação: throttle ∝ √(omega²) = omega
    float omega = sqrt(omega_sq);
    
    // Mapeia linearmente entre os limites
    float throttle = map(omega * 1000, 
                         sqrt(min_omega_sq) * 1000, 
                         sqrt(max_omega_sq) * 1000, 
                         min_throttle * 10, 
                         max_throttle * 10) / 10.0;
    
    return constrainThrottle(throttle);
}

void MotorControl::writeMotorPWM(int channel, float throttle)
{
    int pwmValue = throttleToPWM(throttle);
    ledcWrite(channel, pwmValue);
}