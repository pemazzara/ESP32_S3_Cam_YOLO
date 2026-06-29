// SpeedController.cpp
#include "speed_controller.h"
#include "motor_control.h"

// Constantes del PID
#define KP 0.5f
#define KI 0.1f
#define KD 0.0f
#define CONTROL_PERIOD_S 0.02f  // 20ms = 50Hz
#define MAX_INTEGRAL 200.0f

extern MotorControl motorController;
extern std::atomic<uint32_t> pulsesLeft;
extern std::atomic<uint32_t> pulsesRight;

void SpeedController::begin() {
    xSpeedControllerMutex = xSemaphoreCreateMutex();
    reset();
}

void SpeedController::setTarget(float angle, float pwm, uint16_t xCent, uint16_t yCent) {
    if (xSemaphoreTake(xSpeedControllerMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        base_angle = angle;
        target_pwm = pwm;
        x_centroid = xCent;
        y_centroid = yCent;
        xSemaphoreGive(xSpeedControllerMutex);
    }
}

void SpeedController::updateControl() {
    if (xSemaphoreTake(xSpeedControllerMutex, pdMS_TO_TICKS(5)) != pdTRUE) {
        return;  // No pudimos obtener el mutex
    }
    
    // 1. Captura atómica de pulsos (solo una vez)
    uint32_t pLeft = pulsesLeft.exchange(0, std::memory_order_relaxed);
    uint32_t pRight = pulsesRight.exchange(0, std::memory_order_relaxed);
    
    // 2. Si el target es 0, detener todo
    if (target_pwm < 1.0f) {
        motorController.applyHardwarePWM(0, 0);
        resetIntegral();
        last_pwm_left = 0;
        last_pwm_right = 0;
        xSemaphoreGive(xSpeedControllerMutex);
        return;
    }
    
    // 3. Convertir pulsos a RPM
    // En 20ms: RPM = (pulsos * 60) / (PPR * 0.02) = pulsos * 3000 / PPR
    float currentRPMLeft = (pLeft * 3000.0f) / ENCODER_PPR;
    float currentRPMRight = (pRight * 3000.0f) / ENCODER_PPR;
    
    // 4. Cinemática vectorial
    //float rad = (base_angle * PI) / 180.0f;
    float adjustedRad = (base_angle - 90.0f) * M_PI / 180.0;
    //float giro = cos(rad);
    float giro = sin(adjustedRad); // +1=izquierda, -1=derecha
    //float avance = sin(rad);
    float avance = cos(adjustedRad); // +1=adelante, -1=atrás
    
    // Distribución de PWM deseado para cada rueda
    float desiredPWMLeft = (avance + giro) * target_pwm;
    float desiredPWMRight = (avance - giro) * target_pwm;
    
    // 5. Convertir PWM deseado a RPM objetivo
    // Asumiendo 300 RPM máximo a 1023 PWM
    const float PWM_TO_RPM = 300.0f / 1023.0f;
    float targetRPMLeft = abs(desiredPWMLeft) * PWM_TO_RPM;
    float targetRPMRight = abs(desiredPWMRight) * PWM_TO_RPM;
    
    // 6. Control PID para rueda izquierda
    float errorLeft = targetRPMLeft - currentRPMLeft;
    
    // Anti-windup: solo integrar si el PWM no está saturado
    if (abs(last_pwm_left) < 950 && abs(last_pwm_left) > 50) {
        integral_left += errorLeft * CONTROL_PERIOD_S;
        integral_left = constrain(integral_left, -MAX_INTEGRAL, MAX_INTEGRAL);
    }
    
    // PID: Feedforward (85%) + P + I
    float outputLeft = (abs(desiredPWMLeft) * 0.85f) + 
                       (errorLeft * KP) + 
                       (integral_left * KI);
    
    // 7. Control PID para rueda derecha
    float errorRight = targetRPMRight - currentRPMRight;
    
    if (abs(last_pwm_right) < 950 && abs(last_pwm_right) > 50) {
        integral_right += errorRight * CONTROL_PERIOD_S;
        integral_right = constrain(integral_right, -MAX_INTEGRAL, MAX_INTEGRAL);
    }
    
    float outputRight = (abs(desiredPWMRight) * 0.85f) + 
                        (errorRight * KP) + 
                        (integral_right * KI);
    
    // 8. Aplicar signos y límites
    int16_t finalPWMLeft = constrain((int16_t)outputLeft, 0, 1023);
    int16_t finalPWMRight = constrain((int16_t)outputRight, 0, 1023);
    
    if (desiredPWMLeft < 0) finalPWMLeft = -finalPWMLeft;
    if (desiredPWMRight < 0) finalPWMRight = -finalPWMRight;
    
    // 9. Enviar al hardware
    motorController.applyHardwarePWM(finalPWMLeft, finalPWMRight);
    
    // Guardar para debug y anti-windup
    last_pwm_left = finalPWMLeft;
    last_pwm_right = finalPWMRight;
    last_rpm_left = currentRPMLeft;
    last_rpm_right = currentRPMRight;
    last_target_rpm_left = targetRPMLeft;
    last_target_rpm_right = targetRPMRight;
    
    xSemaphoreGive(xSpeedControllerMutex);
}

void SpeedController::stop() {
    if (xSemaphoreTake(xSpeedControllerMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        motorController.applyHardwarePWM(0, 0);
        resetIntegral();
        target_pwm = 0;
        last_pwm_left = 0;
        last_pwm_right = 0;
        xSemaphoreGive(xSpeedControllerMutex);
    }
}

void SpeedController::printDebug() {
    Serial.printf("🎯 RPM L: %.1f/%.1f | RPM R: %.1f/%.1f | PWM: [%d, %d]\n",
                  last_target_rpm_left, last_rpm_left,
                  last_target_rpm_right, last_rpm_right,
                  last_pwm_left, last_pwm_right);
}

void SpeedController::reset() {
    resetIntegral();
    target_pwm = 0;
    base_angle = 90;
    last_pwm_left = 0;
    last_pwm_right = 0;
}

void SpeedController::resetIntegral() {
    integral_left = 0;
    integral_right = 0;
}


