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
    float rad = (base_angle * PI) / 180.0f;
    float giro = cos(rad);
    float avance = sin(rad);
    
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


/*

#include "speed_controller.h"
#include "sonar_integration.h"
#include "motor_control.h"



extern UltraSonicMeasure sonar;
extern MotorControl motorController;
//extern CalibrationParams calibParams;
extern SensorData_t globalSensorData;
SemaphoreHandle_t xSpeedControllerMutex;

void SpeedController::begin() {
    
    xSpeedControllerMutex = xSemaphoreCreateMutex();
    if (!xSpeedControllerMutex) {
        Serial.println("❌ SpeedController: Error creando mutex");
        return;
    }
}
// O un método unificado:
void SpeedController::setCalibration(float K, float tau) {
        // Calcular tau_deseado (puede ser fijo o configurable)
        const float tau_deseado = tau / 2.0;
        Kp = tau / (K * tau_deseado);
        Ki = (1.0 / (K * tau_deseado)) * CONTROL_PERIOD_S;  // T debe ser conocido
        K_ff = K;    
}
void SpeedController::loadCalibration(float K, float tau) {
    setCalibration(K, tau);
}
void SpeedController::updateControl() {
    if (xSemaphoreTake(xSpeedControllerMutex, pdMS_TO_TICKS(5)) != pdTRUE) {
        return;
    }
    
    // El JS ya hizo todo el trabajo de seguimiento inteligente
    // Solo ejecutamos los comandos recibidos
    float pwm_target = target_avel; 
    float angle_target = base_angle; 
    
    // Debug para verificar
    static uint32_t lastTrackDebug = 0;
    if (millis() - lastTrackDebug > 500) {
        Serial.printf("🎯 Ejecutando: Angulo=%.1f° | PWM_Target=%.0f\n", angle_target, pwm_target);
        lastTrackDebug = millis();
    }
    
    // Resto del código PID...
    avel_current = current_pwm * (MAX_VELOCITY_MM_S / 1023.0f);
    float error = pwm_target - avel_current;
    
    // Zona muerta
    if (abs(error) < 5.0f) {
        error = 0;
    }
    
    // Término integral con anti-windup
    if (current_pwm > 50 && current_pwm < 973) {
        error_integral += error * CONTROL_PERIOD_S;
    }
    error_integral = constrain(error_integral, -500, 500);
    
    // PID
    float p_term = Kp * error;
    float i_term = Ki * error_integral;
    float correction = p_term + i_term;
    
    // PWM final
    int new_pwm = (int)(pwm_target + correction);
    new_pwm = constrain(new_pwm, 0, 1023);
    
    if (pwm_target < 1.0f) {
        new_pwm = 0;
        error_integral = 0;
    }
    
    current_pwm = new_pwm;
    motorController.setPWM(current_pwm, angle_target, false);
    
    last_error = error;
    xSemaphoreGive(xSpeedControllerMutex);
}

/*
// ==================== CONTROLADOR DE VELOCIDAD CON ENCODERS REALES ====================   
// 1. Asegúrate de tener los contadores ISR declarados (pines 33 y 34 como vimos antes)
// 2. En tu setup(), configura los pines de los encoders y las interrupciones
// 3. Crea una tarea de control de motor que se ejecute a 20ms (50Hz) para leer los encoders y ajustar el PWM con un PID incremental
void SpeedController::updateControl() {
    if (xSemaphoreTake(xSpeedControllerMutex, pdMS_TO_TICKS(5)) != pdTRUE) {
        return;
    }
    
    float pwm_target = target_avel; 
    float angle_target = base_angle; 
    
    // 1. Si el target es 0 (STOP), matamos todo inmediatamente
    if (pwm_target < 1.0f) {
        motorController.applyHardwarePWM(0, 0); // Apaga el puente H directo
        error_integral_left = 0;
        error_integral_right = 0;
        xSemaphoreGive(xSpeedControllerMutex);
        return;
    }

    // 2. Captura física de los encoders reales (Frecuencia: 50Hz / 20ms)
    // 🛞 CAPTURA ATÓMICA PERFECTA: Lee y resetea en un solo paso indivisible
    uint32_t pLeft  = pulsesLeft.exchange(0, std::memory_order_relaxed);
    uint32_t pRight = pulsesRight.exchange(0, std::memory_order_relaxed);
    

    // Convertir pulsos a RPM (Multiplicador 150.0f para disco de 20 ranuras a 50Hz)
    float currentSpeedLeft = pLeft * 150.0f;
    float currentSpeedRight = pRight * 150.0f;

    // 3. Cinemática Vectorial (Giro y Avance)
    float rad = (angle_target * M_PI) / 180.0;
    float giro = cos(rad);
    float avance = sin(rad);

    // Distribución del PWM base esperado para cada rueda [-1023, 1023]
    float desiredPWMLeft = (avance + giro) * pwm_target;
    float desiredPWMRight = (avance - giro) * pwm_target;

    // Convertir ese PWM esperado a RPM ideales (asumiendo aprox 300 RPM max a 1023 PWM)
    const float PWM_TO_RPM = 300.0f / 1023.0f;
    float targetRPMLeft = abs(desiredPWMLeft) * PWM_TO_RPM;
    float targetRPMRight = abs(desiredPWMRight) * PWM_TO_RPM;

    // 4. Lazo Cerrado PID - RUEDA IZQUIERDA
    float errorLeft = targetRPMLeft - currentSpeedLeft;
    if (pwm_target > 50 && pwm_target < 950) { // Anti-windup
        error_integral_left += errorLeft * CONTROL_PERIOD_S;
    }
    error_integral_left = constrain(error_integral_left, -200, 200);
    float outLeft = (abs(desiredPWMLeft) * 0.85f) + (errorLeft * Kp) + (error_integral_left * Ki);

    // 5. Lazo Cerrado PID - RUEDA DERECHA
    float errorRight = targetRPMRight - currentSpeedRight;
    if (pwm_target > 50 && pwm_target < 950) {
        error_integral_right += errorRight * CONTROL_PERIOD_S;
    }
    error_integral_right = constrain(error_integral_right, -200, 200);
    float outRight = (abs(desiredPWMRight) * 0.85f) + (errorRight * Kp) + (error_integral_right * Ki);

    // 6. Aplicar signos de dirección y enviar al puente H
    int16_t finalPWMLeft = constrain((int16_t)outLeft, 0, 1023) * (desiredPWMLeft >= 0 ? 1 : -1);
    int16_t finalPWMRight = constrain((int16_t)outRight, 0, 1023) * (desiredPWMRight >= 0 ? 1 : -1);

    // Despachamos al puente H (Sustituye aquí por tu comando real de hardware, ej: motorController.setPWM)
    motorController.applyHardwarePWM(finalPWMLeft, finalPWMRight);
    
    // Telemetría de diagnóstico cada 500ms
    static uint32_t lastTrackDebug = 0;
    if (millis() - lastTrackDebug > 500) {
        Serial.printf("🎯 RPM L: Target=%0.1f Real=%0.1f | RPM R: Target=%0.1f Real=%0.1f | OutPWM: [%d, %d]\n", 
                      targetRPMLeft, currentSpeedLeft, targetRPMRight, currentSpeedRight, finalPWMLeft, finalPWMRight);
        lastTrackDebug = millis();
    }
    
    xSemaphoreGive(xSpeedControllerMutex);
}


    int16_t SpeedController::getCurrentPWM() {
        return current_pwm;
    }
    float SpeedController::getCurrentAvel() {
        if (xSemaphoreTake(xSpeedControllerMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            float avel = avel_current;
            xSemaphoreGive(xSpeedControllerMutex);
            return avel;
        }
        return 0; // o algún valor de error
    }

void SpeedController::setTarget(float angle, float pwm, uint16_t xCent, uint16_t yCent) {
    if (xSemaphoreTake(xSpeedControllerMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        base_angle = angle;
        target_avel = pwm;
        targetXCentroid = xCent;
        targetYCentroid = yCent;
        xSemaphoreGive(xSpeedControllerMutex);
    }
}
void SpeedController::setReferencePoint(uint16_t x, uint16_t y) {
    referenceXCentroid = x;
    referenceYCentroid = y;
}
    float SpeedController::getLastError() {
        if (xSemaphoreTake(xSpeedControllerMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            float error = last_error;
            xSemaphoreGive(xSpeedControllerMutex);
            return error;
        }
        return 0; // o algún valor de error
    }

*/