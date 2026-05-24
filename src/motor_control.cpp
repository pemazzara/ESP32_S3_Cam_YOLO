// motor_control.cpp

#include "motor_control.h"


void MotorControl::begin() {
    Serial.println("🚗 Inicializando MotorControl L298N...");
    
    // 1. Configurar pines de dirección como outputs
    gpio_config_t dir_pins = {};
    dir_pins.pin_bit_mask = (1ULL << IN1) | (1ULL << IN2) | 
                            (1ULL << IN3) | (1ULL << IN4);
    dir_pins.mode = GPIO_MODE_OUTPUT;
    dir_pins.pull_up_en = GPIO_PULLUP_DISABLE;
    dir_pins.pull_down_en = GPIO_PULLDOWN_DISABLE;
    dir_pins.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&dir_pins);
    
    // Estado inicial seguro
    gpio_set_level((gpio_num_t)IN1, 0);
    gpio_set_level((gpio_num_t)IN2, 0);
    gpio_set_level((gpio_num_t)IN3, 0);
    gpio_set_level((gpio_num_t)IN4, 0);
    
    ledc_timer_config_t ledc_timer = {
    .speed_mode       = LEDC_LOW_SPEED_MODE,
    .duty_resolution  = (ledc_timer_bit_t)MOTOR_PWM_RES, // <--- Cast aquí
    .timer_num        = LEDC_TIMER_0,
    .freq_hz          = MOTOR_PWM_FREQ,
    .clk_cfg          = LEDC_AUTO_CLK
};
    
    esp_err_t ret = ledc_timer_config(&ledc_timer);
    if (ret != ESP_OK) {
        Serial.printf("❌ Error timer LEDC: 0x%X\n", ret);
        ledc_initialized = false;
        return;
    }
    
    // 3. Configurar canal A (Motor izquierdo)
    ledc_channel_a.gpio_num = (gpio_num_t)ENA;
    ledc_channel_a.speed_mode = LEDC_LOW_SPEED_MODE;
    ledc_channel_a.channel = LEDC_CHANNEL_0;
    ledc_channel_a.intr_type = LEDC_INTR_DISABLE;
    ledc_channel_a.timer_sel = LEDC_TIMER_0;
    ledc_channel_a.duty = 0;
    ledc_channel_a.hpoint = 0;
    ledc_channel_a.flags.output_invert = 0;
    
    ret = ledc_channel_config(&ledc_channel_a);
    if (ret != ESP_OK) {
        Serial.printf("❌ Error canal LEDC A: 0x%X\n", ret);
        ledc_initialized = false;
        return;
    }
    
    // 4. Configurar canal B (Motor derecho)
    ledc_channel_b.gpio_num = (gpio_num_t)ENB;
    ledc_channel_b.speed_mode = LEDC_LOW_SPEED_MODE;
    ledc_channel_b.channel = LEDC_CHANNEL_1;
    ledc_channel_b.intr_type = LEDC_INTR_DISABLE;
    ledc_channel_b.timer_sel = LEDC_TIMER_0;
    ledc_channel_b.duty = 0;
    ledc_channel_b.hpoint = 0;
    ledc_channel_b.flags.output_invert = 0;
    
    ret = ledc_channel_config(&ledc_channel_b);
    if (ret != ESP_OK) {
        Serial.printf("❌ Error canal LEDC B: 0x%X\n", ret);
        ledc_initialized = false;
        return;
    }
    
    // 5. Establecer duty cycle inicial
    ledc_set_duty(ledc_channel_a.speed_mode, ledc_channel_a.channel, 0);
    ledc_set_duty(ledc_channel_b.speed_mode, ledc_channel_b.channel, 0);
    ledc_update_duty(ledc_channel_a.speed_mode, ledc_channel_a.channel);
    ledc_update_duty(ledc_channel_b.speed_mode, ledc_channel_b.channel);
    
    // 6. Inicializar variables de estado
    targetLeftSpeed = 0;
    targetRightSpeed = 0;
    currentLeftSpeed = 0;
    currentRightSpeed = 0;
    lastUpdateTime = millis();
    ledc_initialized = true;
    applyHardwarePWM(0, 0);
    Serial.println("✅ MotorControl inicializado correctamente");
    Serial.printf("   PWM: %d Hz, %d bits, Canales: %d y %d\n", 
                  MOTOR_PWM_FREQ, 
                  (MOTOR_PWM_RES == LEDC_TIMER_10_BIT) ? 10 : 8,
                  ledc_channel_a.channel, 
                  ledc_channel_b.channel);
}
void MotorControl::setTargetAngle(int16_t angle) {
    if (abs(angle) > 360) return;
    targetAngle = angle;
}

void MotorControl::applyHardwarePWM(int16_t left, int16_t right) {
    if (!ledc_initialized) {
        Serial.println("❌ ERROR: LEDC no inicializado en applyHardwarePWM");
        return;
    }
    
    uint32_t dutyLeft = abs(left);
    uint32_t dutyRight = abs(right);

    if (dutyLeft > MAX_DUTY_CYCLE) dutyLeft = MAX_DUTY_CYCLE;
    if (dutyRight > MAX_DUTY_CYCLE) dutyRight = MAX_DUTY_CYCLE;

    // ==================== Polaridad INDEPENDIENTE ====================
    
    // MOTOR IZQUIERDO
    if (left > 0) { 
        gpio_set_level((gpio_num_t)IN1, 1);  // Adelante
        gpio_set_level((gpio_num_t)IN2, 0);
    } else if (left < 0) { 
        gpio_set_level((gpio_num_t)IN1, 0);  // Atrás
        gpio_set_level((gpio_num_t)IN2, 1);
    } else { 
        gpio_set_level((gpio_num_t)IN1, 0);  // Parada Izquierdo
        gpio_set_level((gpio_num_t)IN2, 0);
    }

    // MOTOR DERECHO
    if (right > 0) { 
        gpio_set_level((gpio_num_t)IN3, 0);  // Adelante
        gpio_set_level((gpio_num_t)IN4, 1);
    } else if (right < 0) { 
        gpio_set_level((gpio_num_t)IN3, 1);  // Atrás
        gpio_set_level((gpio_num_t)IN4, 0);
    } else { 
        gpio_set_level((gpio_num_t)IN3, 0);  // Parada Derecho
        gpio_set_level((gpio_num_t)IN4, 0);
    }
    
    // ==================== Aplicar PWM ====================
    ledc_set_duty(ledc_channel_a.speed_mode, ledc_channel_a.channel, dutyLeft);
    ledc_update_duty(ledc_channel_a.speed_mode, ledc_channel_a.channel);
    
    ledc_set_duty(ledc_channel_b.speed_mode, ledc_channel_b.channel, dutyRight);
    ledc_update_duty(ledc_channel_b.speed_mode, ledc_channel_b.channel);

    // ... código de debug ...
}


void MotorControl::setPWM(int16_t pwm, int16_t targetAngle, bool immediate) {
    //Serial.printf("🔧 setPWM llamado con pwm=%d, targetAngle=%d, immediate=%d\n", 
    //          pwm, targetAngle, immediate);
    if (!ledc_initialized) return;
    if (abs(targetAngle) > 360) return;
    
    // Suavizado del ángulo (Interpolación Lineal hacia el target)
    // 1. Suavizar el ángulo (Slew Rate)
    const float ANGLE_STEP = 3.0f;
    if (abs(targetAngle - currentInterpolatedAngle) > 0.5f) {
        if (currentInterpolatedAngle < targetAngle) currentInterpolatedAngle += ANGLE_STEP;
        else currentInterpolatedAngle -= ANGLE_STEP;
    }
    // Aplicar la cinemática con el ángulo suavizado
    float rad = (currentInterpolatedAngle * M_PI) / 180.0;
    float giro = cos(rad);
    float avance = sin(rad);
    
    int16_t rawLeft = (avance + giro) * pwm;
    int16_t rawRight = (avance - giro) * pwm;
    
    const int MAX_VAL = 1023;
    rawLeft = constrain(rawLeft, -MAX_VAL, MAX_VAL);
    rawRight = constrain(rawRight, -MAX_VAL, MAX_VAL);
    
    if (immediate && pwm == 0) {
        targetLeftSpeed = 0;
        targetRightSpeed = 0;
        currentLeftSpeed = 0;
        currentRightSpeed = 0;
    } else if (immediate  && pwm != 0) {
         targetLeftSpeed = rawLeft;
         targetRightSpeed = rawRight;
         currentLeftSpeed = rawLeft;
         currentRightSpeed = rawRight;
         lastUpdateTime = millis();
    } else {
        targetLeftSpeed = rawLeft;
        targetRightSpeed = rawRight;
        lastUpdateTime = millis();
    }
    applyHardwarePWM(targetLeftSpeed, targetRightSpeed);
}
/*
void MotorControl::updateRamping() {
    if (!ledc_initialized) return;
        
    static uint32_t lastRampTime = 0;
    uint32_t ahora = millis();

    if (ahora - lastRampTime < 20) return;
    lastRampTime = ahora;

    // Heartbeat timeout
    if (ahora - lastUpdateTime > HEARTBEAT_TIMEOUT) {
        if (targetLeftSpeed != 0 || targetRightSpeed != 0) {
            Serial.println("🛑 Heartbeat timeout - Deteniendo motores");
        }
        targetLeftSpeed = 0;
        targetRightSpeed = 0;
    }

    // Lógica de rampa
    const int16_t ACCEL_STEP = 10;
    const int16_t DECEL_STEP = 20;

    auto calculateRamp = [&](int16_t current, int16_t target) -> int16_t {
        if (current == target) return current;
        
        int16_t step = (abs(target) > abs(current)) ? ACCEL_STEP : DECEL_STEP;
        
        if (target > current) return min((int16_t)(current + step), target);
        else return max((int16_t)(current - step), target);
    };

    currentLeftSpeed = calculateRamp(currentLeftSpeed, targetLeftSpeed);
    currentRightSpeed = calculateRamp(currentRightSpeed, targetRightSpeed);

    // Enviar al hardware
    this->applyHardwarePWM(currentLeftSpeed, currentRightSpeed);
}
*/
void MotorControl::emergencyStop() {
    targetLeftSpeed = 0;
    targetRightSpeed = 0;
    currentLeftSpeed = 0;
    currentRightSpeed = 0;
    
    if (ledc_initialized) {
        gpio_set_level((gpio_num_t)IN1, 0);
        gpio_set_level((gpio_num_t)IN2, 0);
        gpio_set_level((gpio_num_t)IN3, 0);
        gpio_set_level((gpio_num_t)IN4, 0);
        
        ledc_set_duty(ledc_channel_a.speed_mode, ledc_channel_a.channel, 0);
        ledc_set_duty(ledc_channel_b.speed_mode, ledc_channel_b.channel, 0);
        ledc_update_duty(ledc_channel_a.speed_mode, ledc_channel_a.channel);
        ledc_update_duty(ledc_channel_b.speed_mode, ledc_channel_b.channel);
    }
    
    Serial.println("🛑 EMERGENCY STOP ejecutado");
}
void MotorControl::setSpeed(int speedA, int speedB) {
    // Limitar valores entre -1023 y 1023
    speedA = constrain(speedA, -1023, 1023);
    speedB = constrain(speedB, -1023, 1023);
    
    // Control Motor A
    if (speedA > 0) {
        // Adelante
        gpio_set_level((gpio_num_t)IN1, 1);
        gpio_set_level((gpio_num_t)IN2, 0);
        ledc_set_duty(ledc_channel_a.speed_mode, 
                      ledc_channel_a.channel, 
                      abs(speedA));
    } 
    else if (speedA < 0) {
        // Atrás
        gpio_set_level((gpio_num_t)IN1, 0);
        gpio_set_level((gpio_num_t)IN2, 1);
        ledc_set_duty(ledc_channel_a.speed_mode, 
                      ledc_channel_a.channel, 
                      abs(speedA));
    } 
    else {
        // Parar
        gpio_set_level((gpio_num_t)IN1, 0);
        gpio_set_level((gpio_num_t)IN2, 0);
        ledc_set_duty(ledc_channel_a.speed_mode, 
                      ledc_channel_a.channel, 
                      0);
    }
    
    // Control Motor B (similar)
    if (speedB > 0) {
        gpio_set_level((gpio_num_t)IN3, 1);
        gpio_set_level((gpio_num_t)IN4, 0);
        ledc_set_duty(ledc_channel_b.speed_mode, 
                      ledc_channel_b.channel, 
                      abs(speedB));
    } 
    else if (speedB < 0) {
        gpio_set_level((gpio_num_t)IN3, 0);
        gpio_set_level((gpio_num_t)IN4, 1);
        ledc_set_duty(ledc_channel_b.speed_mode, 
                      ledc_channel_b.channel, 
                      abs(speedB));
    } 
    else {
        gpio_set_level((gpio_num_t)IN3, 0);
        gpio_set_level((gpio_num_t)IN4, 0);
        ledc_set_duty(ledc_channel_b.speed_mode, 
                      ledc_channel_b.channel, 
                      0);
    }
    
    // ACTUALIZAR DUTY (crítico!)
    ledc_update_duty(ledc_channel_a.speed_mode, ledc_channel_a.channel);
    ledc_update_duty(ledc_channel_b.speed_mode, ledc_channel_b.channel);
}



uint16_t MotorControl::getBatteryVoltage() {
    return 12000; // Simulación de 12.0V (12000 mV)
}
    
uint8_t MotorControl::getStatusFlags() {
    uint8_t flags = 0x00;
    
    if (emergencyStopActive) {
        flags |= 0x01; // Bit 0: Emergencia activa
    }
    if (isSafetyTimeout()) {
        flags |= 0x02; // Bit 1: Timeout de seguridad
    }
    
    return flags;
}

void MotorControl::updateSafety() {
    if (isSafetyTimeout() && !emergencyStopActive) {
        stop();
    }
}

// Implementación de movimientos (omitiendo la función executeAutonomousCommand por simplicidad)
void MotorControl::moveForward(int speed) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); analogWrite(ENA, speed);
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); analogWrite(ENB, speed);
}
void MotorControl::moveBackward(int speed) {
    // ✅ OPTIMIZADA para HW-095 con regulador 5V
    speed = constrain(speed, 0, MAX_SAFE_SPEED);
    
    Serial.printf("[HW-095] 🔙 ATRÁS solicitado: %d/255\n", speed);
    
    // 1. SI estamos yendo adelante, parar brevemente
    if (currentLeftSpeed > 0 || currentRightSpeed > 0) {
        Serial.println("  ↪️ Transición ADELANTE→ATRÁS: Parada breve");
        stop();
        delay(20);  // ⬅️ HW-095 necesita menos delay
    }
    
    // 2. Configurar dirección (con delay reducido para HW-095)
    // Motor izquierdo: Atrás
    digitalWrite(IN1, LOW);
    delayMicroseconds(100);  // ⬅️ HW-095 es más rápido
    digitalWrite(IN2, HIGH);
    
    // Motor derecho: Atrás
    digitalWrite(IN3, LOW);
    delayMicroseconds(100);
    digitalWrite(IN4, HIGH);
    
    // 3. Aplicar PWM con rampa OPTIMIZADA
    int duty = map(speed, 0, 255, 0, 255); // 8-bit
    
    // Rampa más agresiva (HW-095 lo soporta mejor)
    int step = (duty > 100) ? 15 : 10;
    for (int i = 0; i <= duty; i += step) {
        ledcWrite(0, i);  // Canal 0 = ENA
        ledcWrite(1, i);  // Canal 1 = ENB
        delay(2);
    }
    
    // 4. Actualizar estado
    currentLeftSpeed = -speed;
    currentRightSpeed = -speed;
    
    // 5. MONITOREO (opcional)
    static uint32_t backwardCount = 0;
    backwardCount++;
    if (backwardCount % 5 == 0) {
        Serial.printf("  📊 Stats: %d movimientos atrás ejecutados\n", backwardCount);
    }
}

void MotorControl::turnLeft(int speed) {
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); analogWrite(ENA, speed); // Izq: Atrás
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); analogWrite(ENB, speed); // Der: Adelante
}
void MotorControl::turnRight(int speed) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); analogWrite(ENA, speed); // Izq: Adelante
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); analogWrite(ENB, speed); // Der: Atrás
}
void MotorControl::stop() {
    analogWrite(ENA, 0); analogWrite(ENB, 0);
    digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

void MotorControl::setMotorSpeeds(int leftSpeed, int rightSpeed) {
        // Suavizado de velocidad
        currentLeftSpeed = smoothSpeed(currentLeftSpeed, leftSpeed);
        currentRightSpeed = smoothSpeed(currentRightSpeed, rightSpeed);
        
        analogWrite(ENA, currentLeftSpeed);
        analogWrite(ENB, currentRightSpeed);
    }
 
    int MotorControl::smoothSpeed(int current, int target) {
        if(abs(current - target) <= MAX_ACCELERATION) return target;
        return (current < target) ? current + MAX_ACCELERATION : current - MAX_ACCELERATION;
    }


void MotorControl::resetSafetyTimer() {
    lastUpdateTime = millis();
}

bool MotorControl::isSafetyTimeout() {
    unsigned long elapsed = millis() - lastUpdateTime;
    
    if (elapsed > SAFETY_TIMEOUT_MS) {
        Serial.printf("[Safety] ⏰ Timeout: %lums sin comandos\n", elapsed);
        return true;
    }
    return false;
}
// En MotorControl.cpp del Slave
uint8_t MotorControl::getMotorStatus() {
    uint8_t status = 0x00;
    
    if (emergencyStopActive) {
        status |= 0x01; // Bit 0: Emergencia activa
        Serial.println("[Status] ⚠️ EMERGENCIA ACTIVA - Motores bloqueados");
    }
    if (currentLeftSpeed != 0 || currentRightSpeed != 0) {
        status |= 0x02; // Bit 1: Movimiento
    }
    if (isSafetyTimeout()) {
        status |= 0x04; // Bit 2: Timeout de seguridad
        Serial.println("[Status] ⚠️ TIMEOUT ACTIVO - Sin comandos recientes");
    }
    
    return status;
}




