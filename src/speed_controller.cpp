
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
void SpeedController::updateControl() {
    if (xSemaphoreTake(xSpeedControllerMutex, pdMS_TO_TICKS(5)) != pdTRUE) {
        return;
    }
    
    // ==================== MODO DE CONTROL ====================
    // Si hay centroides válidos, usar seguimiento visual
    bool usarSeguimiento = (targetXCentroid > 0 || targetYCentroid > 0);
    
    float pwm_target = target_avel;
    float angle_target = base_angle;
    
    if (usarSeguimiento) {
        // ==================== CONTROL DE ÁNGULO (error en X) ====================
        int16_t errorX = (int16_t)targetXCentroid - (int16_t)referenceXCentroid;
        
        // PID para ángulo
        angleError = errorX;
        
        // Término integral con anti-windup
        if (abs(current_pwm) < 900) {  // Solo integrar si no está saturado
            angleIntegral += angleError * CONTROL_PERIOD_S;
            angleIntegral = constrain(angleIntegral, -200, 200);
        }
        
        // Término derivativo
        float angleDeriv = (angleError - lastAngleError) / CONTROL_PERIOD_S;
        
        // Calcular corrección de ángulo
        angleCorrection = (Kp_angle * angleError) + 
                          (Ki_angle * angleIntegral) + 
                          (Kd_angle * angleDeriv);
        
        // Aplicar corrección al ángulo base (90° es recto)
        angle_target = 90.0f + angleCorrection;
        angle_target = constrain(angle_target, 0.0f, 180.0f);
        
        // ==================== CONTROL DE VELOCIDAD (error en Y y distancia) ====================
        int16_t errorY = (int16_t)targetYCentroid - (int16_t)referenceYCentroid;
        uint16_t distancia = sqrt(errorX*errorX + errorY*errorY);
        
        // Velocidad proporcional a la distancia (más lejos = más rápido)
        float velocidad_normalizada = constrain(distancia / 100.0f, 0.0f, 1.0f);
        pwm_target = velocidad_normalizada * 1023.0f;
        
        // Zona muerta: si el objeto está cerca del centro, detener
        if (distancia < 15) {
            pwm_target = 0;
            angleIntegral = 0;  // Resetear integral
        } else if (pwm_target < 80) {
            pwm_target = 80;  // Velocidad mínima para vencer fricción
        }
        
        // Debug de seguimiento
        static uint32_t lastTrackDebug = 0;
        if (millis() - lastTrackDebug > 500) {
            Serial.printf("🎯 Seguimiento: Centro=(%d,%d) Err=(%d,%d) Dist=%u Ang=%.1f PWM=%.0f\n",
                          targetXCentroid, targetYCentroid, errorX, errorY, 
                          distancia, angle_target, pwm_target);
            lastTrackDebug = millis();
        }
        
        lastAngleError = angleError;
    }
    
    // ==================== 1. VELOCIDAD ACTUAL ESTIMADA ====================
    avel_current = current_pwm * (MAX_VELOCITY_MM_S / 1023.0f);
    
    // ==================== 2. CÁLCULO DEL ERROR ====================
    float error = pwm_target - avel_current;
    
    if (abs(error) < 5.0f) {
        error = 0;
    }
    
    // ==================== 3. TÉRMINO INTEGRAL ====================
    if (current_pwm > 50 && current_pwm < 973) {
        error_integral += error * CONTROL_PERIOD_S;
    }
    error_integral = constrain(error_integral, -500, 500);
    
    // ==================== 4. PID DE VELOCIDAD ====================
    float p_term = Kp * error;
    float i_term = Ki * error_integral;
    float correction = p_term + i_term;
    
    // ==================== 5. PWM FINAL ====================
    int new_pwm = (int)(pwm_target + correction);
    new_pwm = constrain(new_pwm, 0, 1023);
    
    if (pwm_target < 1.0f) {
        new_pwm = 0;
        error_integral = 0;
    }
    
    // ==================== 6. APLICAR A LOS MOTORES ====================
    current_pwm = new_pwm;
    motorController.setPWM(current_pwm, angle_target, false);
    
    // ==================== 7. DEBUG ====================
    static uint32_t last_debug = 0;
    if (millis() - last_debug > 500) {
        Serial.printf("PID: target=%.0f actual=%.0f error=%.1f PWM=%d Ang=%.1f\n",
                      pwm_target, avel_current, error, current_pwm, angle_target);
        last_debug = millis();
    }
    
    last_error = error;
    xSemaphoreGive(xSpeedControllerMutex);
}*/

/*
void SpeedController::updateControl() {
    if (xSemaphoreTake(xSpeedControllerMutex, pdMS_TO_TICKS(5)) != pdTRUE) {
        return; // Si el mutex está ocupado, saltar este ciclo
    }
    
    // ==================== 1. VELOCIDAD ACTUAL ESTIMADA ====================
    // Sin encoders, estimamos la velocidad a partir del PWM anterior
    avel_current = current_pwm * (MAX_VELOCITY_MM_S / 1023.0f);
    
    // ==================== 2. CÁLCULO DEL ERROR ====================
    float error = target_avel - avel_current;
    
    // Zona muerta: si el error es muy pequeño, no corregir (evita oscilaciones)
    if (abs(error) < 5.0f) {
        error = 0;
    }
    
    // ==================== 3. TÉRMINO INTEGRAL (anti-windup) ====================
    // Solo acumular si el PWM no está saturado
    if (current_pwm > 50 && current_pwm < 973) { // No saturar si está en los extremos
        error_integral += error * CONTROL_PERIOD_S;
    }
    error_integral = constrain(error_integral, -500, 500);
    
    // ==================== 4. TÉRMINO DERIVATIVO (opcional, desactivado por ahora) ====================
    // float error_deriv = (error - last_error) / CONTROL_PERIOD_S;
    // float d_term = Kd * error_deriv;
    
    // ==================== 5. PID ====================
    float p_term = Kp * error;
    float i_term = Ki * error_integral;
    // float d_term = Kd * error_deriv;  // Desactivado
    float correction = p_term + i_term; // + d_term;
    
    // ==================== 6. FEEDFORWARD ====================
    // El feedforward es directamente el target (ya que target_avel es PWM)
    float pwm_ff = target_avel;
    
    // ==================== 7. PWM FINAL ====================
    int new_pwm = (int)(pwm_ff + correction);
    
    // Limitar PWM al rango válido
    new_pwm = constrain(new_pwm, 0, 1023);
    
    // Si el target es 0, forzar PWM a 0 inmediatamente (sin PID)
    if (target_avel < 1.0f) {
        new_pwm = 0;
        error_integral = 0; // Resetear integral al frenar
    }
    
    // ==================== 8. APLICAR A LOS MOTORES ====================
    current_pwm = new_pwm;
    motorController.setPWM(current_pwm, base_angle, false);
    
    // ==================== 9. GUARDAR ERROR PARA EL SIGUIENTE CICLO ====================
    last_error = error;
    
    // ==================== 10. DEBUG (descomentar para calibrar) ====================
    
    static uint32_t last_debug = 0;
    if (millis() - last_debug > 500) {
        Serial.printf("PID: target=%.0f actual=%.0f error=%.1f P=%.1f I=%.1f PWM=%d\n",
                      target_avel, avel_current, error, p_term, i_term, current_pwm);
        last_debug = millis();
    }
    
    
    xSemaphoreGive(xSpeedControllerMutex);
}*/

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



