#include <Arduino.h>

// Pines asignados
#define PIN_ENCODER_LEFT  33
#define PIN_ENCODER_RIGHT 34

// Contadores de pulsos ISR
volatile uint32_t pulsesLeft = 0;
volatile uint32_t pulsesRight = 0;

void IRAM_ATTR isrLeft()  { pulsesLeft++; }
void IRAM_ATTR isrRight() { pulsesRight++; }

// Variables de control compartidas (las que ya actualiza tu WebSocket)
extern std::atomic<float> targetAngle;
extern std::atomic<int16_t> targetSpeed;

// Inicialización en tu setup()
void setupEncoders() {
    pinMode(PIN_ENCODER_LEFT, INPUT);
    pinMode(PIN_ENCODER_RIGHT, INPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_LEFT), isrLeft, RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_RIGHT), isrRight, RISING);

    // Crear la tarea de control de bajo nivel a 20Hz (cada 50ms)
    xTaskCreatePinnedToCore(motorPIDTask, "MotorPID", 4096, NULL, 3, NULL, 1);
}

// Variables globales/externas que ya tenés mapeadas
extern std::atomic<uint32_t> lastCommandTime;
extern std::atomic<float> targetAngle;
extern std::atomic<int16_t> targetSpeed;

// Contadores ISR de tus pines 33 y 34
extern volatile uint32_t pulsesLeft;
extern volatile uint32_t pulsesRight;

void motorPIDTask(void *pvParameters) {
    Serial.printf("⚙️ Lazo Cerrado Bilateral Activo en Core %d\n", xPortCoreID());
    
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(20); // Loop estricto de 20ms (50 Hz)

    // Variables internas del PID para cada rueda
    float errorIntLeft = 0, errorIntRight = 0;
    float lastErrorLeft = 0, lastErrorRight = 0;
    
    // Constantes a calibrar (Al pasar a escala 1023 y RPM, vas a notar que requieren valores sutiles)
    const float Kp = 1.2f;
    const float Ki = 0.15f;
    const float CONTROL_PERIOD_S = 0.02f; // 20ms
    
    float currentInterpolatedAngle = 90.0f;

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        uint32_t ahora = millis();
        uint32_t tUltimoCmd = lastCommandTime.load(std::memory_order_acquire);

        // ==================== 1. ESCUDO FAILSAFE ====================
        if (ahora - tUltimoCmd > 1000) {
            // Si pasa más de 1 segundo sin datos del navegador, PARADA DE EMERGENCIA
            motorController.applyHardwarePWM(0, 0); 
            errorIntLeft = 0; errorIntRight = 0;
            continue;
        }

        // ==================== 2. CAPTURA DE ENCODERS REALES ====================
        // Captura rápida y reseteo inmediato para no perder pulsos en la siguiente vuelta
        uint32_t pLeft = pulsesLeft;   pulsesLeft = 0;
        uint32_t pRight = pulsesRight; pulsesRight = 0;

        // Multiplicador para 20ms con disco de 4 ranuras (4p): 
        // 1 pulso en 20ms = 50 pulsos/seg -> /4 ranuras = 12.5 RPS -> *60 = 750 RPM
        float currentSpeedLeft = pLeft * 750.0f;
        float currentSpeedRight = pRight * 750.0f;

        // ==================== 3. LEER OBJETIVOS DEL NAVEGADOR ====================
        float tAngle = targetAngle.load(std::memory_order_acquire);
        int16_t tSpeed = targetSpeed.load(std::memory_order_acquire);

        if (tSpeed == 0) {
            motorController.applyHardwarePWM(0, 0); 
            errorIntLeft = 0; errorIntRight = 0;
            continue;
        }

        // ==================== 4. CINEMÁTICA VECTORIAL SUAVE ====================
        const float ANGLE_STEP = 3.0f; // Máximo cambio de ángulo por ciclo (suavizado)
        float angleDiff = tAngle - currentInterpolatedAngle;
        while (angleDiff < -180.0f) angleDiff += 360.0f;
        while (angleDiff > 180.0f)  angleDiff -= 360.0f;
        
        if (abs(angleDiff) > 0.5f) {
            currentInterpolatedAngle += (angleDiff > 0) ? ANGLE_STEP : -ANGLE_STEP;
        }

        float rad = (currentInterpolatedAngle * M_PI) / 180.0;
        float giro = cos(rad);
        float avance = sin(rad);

        // Traducimos la entrada del navegador (0-255) a la escala nativa de tus motores (0-1023)
        float targetPWMBase = (tSpeed / 255.0f) * 1023.0f;

        // Distribución diferencial exacta de velocidad deseada para cada rueda [-1023, 1023]
        float desiredSpeedLeft = (avance + giro) * targetPWMBase;
        float desiredSpeedRight = (avance - giro) * targetPWMBase;

        // Supongamos que tus motores a 1023 PWM entregan aprox 300 RPM reales libres
        const float PWM_TO_RPM = 300.0f / 1023.0f;
        float targetRPMLeft = abs(desiredSpeedLeft) * PWM_TO_RPM;
        float targetRPMRight = abs(desiredSpeedRight) * PWM_TO_RPM;

        // ==================== 5. LAZO CERRADO PID PI CON ANT-WINDUP ====================
        
        // --- RUEDA IZQUIERDA ---
        float errorLeft = targetRPMLeft - currentSpeedLeft;
        if (targetPWMBase > 50 && targetPWMBase < 950) { // Anti-windup lógico
            errorIntLeft += errorLeft * CONTROL_PERIOD_S;
        }
        errorIntLeft = constrain(errorIntLeft, -200, 200);
        float outputPWMLeft = (abs(desiredSpeedLeft) * 0.85f) + (errorLeft * Kp) + (errorIntLeft * Ki);

        // --- RUEDA DERECHA ---
        float errorRight = targetRPMRight - currentSpeedRight;
        if (targetPWMBase > 50 && targetPWMBase < 950) {
            errorIntRight += errorRight * CONTROL_PERIOD_S;
        }
        errorIntRight = constrain(errorIntRight, -200, 200);
        float outputPWMRight = (abs(desiredSpeedRight) * 0.85f) + (errorRight * Kp) + (errorIntRight * Ki);

        // ==================== 6. DIRECCIÓN FÍSICA Y SALIDA AL PUENTE H ====================
        // Acotamos a la resolución real de tu temporizador (0 a 1023) y reinyectamos el signo de marcha
        int16_t finalPWMLeft = constrain((int16_t)outputPWMLeft, 0, 1023) * (desiredSpeedLeft >= 0 ? 1 : -1);
        int16_t finalPWMRight = constrain((int16_t)outputPWMRight, 0, 1023) * (desiredSpeedRight >= 0 ? 1 : -1);

        // Envío directo al hardware físico (IN1, IN2, ENA, etc.)
        motorController.applyHardwarePWM(finalPWMLeft, finalPWMRight);

        // Telemetría de diagnóstico cada 500ms
        static uint32_t lastDebug = 0;
        if (ahora - lastDebug > 500) {
            Serial.printf("🛞 RPM L: Deseadas=%0.1f Real=%0.1f (PWM=%d) | RPM R: Deseadas=%0.1f Real=%0.1f (PWM=%d)\n",
                          targetRPMLeft, currentSpeedLeft, finalPWMLeft,
                          targetRPMRight, currentSpeedRight, finalPWMRight);
            lastDebug = ahora;
        }
    }
}