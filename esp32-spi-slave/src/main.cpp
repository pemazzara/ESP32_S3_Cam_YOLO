// main.cpp del ESP32 Slave
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "motor_control.h"
#include "sensor_control.h"
#include "SPISlave.h"
#include "SensorFusion.h"
#include "esp_task_wdt.h"
#include "esp_heap_caps.h"
#include "speed_controller.h"
#include "slave_robot_control.h"


#define CONTROL_PERIOD_MS 20
#define CONTROL_PERIOD_S (CONTROL_PERIOD_MS / 1000.0f)

std::atomic<float> targetAngle{90.0f};
std::atomic<int> targetSpeed{0};
std::atomic<uint16_t> targetXCentroid{0};
std::atomic<uint16_t> targetYCentroid{0};
std::atomic<uint32_t> lastValidCommandTime{0};
// Contadores de pulsos ISR
std::atomic<uint32_t> pulsesLeft{0};
std::atomic<uint32_t> pulsesRight{0};

std::atomic<bool> emergencyStop{false};      // ← Inicializar como false
std::atomic<bool> failsafe{false};           // ← Inicializar como false

// =========================================================
// VARIABLES GLOBALES
// =========================================================
OdometryProcessor odom;
SensorFusion fusion;
RobotState currentState;
ObstacleData currentObstacles;
SensorControl sensors;
SpeedController speedController;
MotorControl motorController;

//CalibrationParams calibParams;
// =========================================================
// Handles de tareas
TaskHandle_t motorTaskHandle = NULL;
TaskHandle_t spiTaskHandle = NULL;
TaskHandle_t xSensorRead_Handle = NULL;
TaskHandle_t fusionTaskHandle = NULL;
// =========================================================
// ==================== PROTOTIPOS DE FUNCIONES ISR ====================
void IRAM_ATTR isrLeft()  { 
    pulsesLeft.fetch_add(1, std::memory_order_relaxed); 
}
void IRAM_ATTR isrRight() { 
    pulsesRight.fetch_add(1, std::memory_order_relaxed); 
}
void setupEncoders() {
    pinMode(PIN_ENCODER_LEFT, INPUT_PULLUP);  // Usa pullup interno
    pinMode(PIN_ENCODER_RIGHT, INPUT_PULLUP);
    
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_LEFT), isrLeft, RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_RIGHT), isrRight, RISING);
    
    Serial.println("✅ Encoders configurados en pines 19 y 20");
}

// =========================================================
// DECLARACIÓN DE TAREAS
// =========================================================
// Tarea de control de motores (Core 1, alta prioridad)
void motorTask(void *pvParameters);
// Tarea de fusión de sensores (Core 1, prioridad media)
void fusionTask(void *pvParameters);
// Tarea de lectura de sensores TOF (Core 0, prioridad baja)
void tofSensorTask(void *pvParameters);
// Tarea de comunicación SPI (Core 0, prioridad media)
void spiTask(void *pvParameters);

void printResetReason();


void setup_tasks() {
    Serial.println("🔧 Configurando tareas...");
   // Tareas en Core 0 (Aislada para radio)
    // Tarea SPI SLAVE (misma prioridad que motor)
    xTaskCreatePinnedToCore(
        spiTask,
        "SPI_Slave",
        6144,        // 6KB stack
        NULL,
        4,       
        &spiTaskHandle,
        0            // Core 0 exclusivo
    );
    // Tarea SENSORES (baja prioridad)
    xTaskCreatePinnedToCore(
        tofSensorTask,
        "TOFSensorRead",
        4096,
        NULL,
        1, // Prioridad baja
        &xSensorRead_Handle,
        0
    );  
    // Tareas en Core 1 (Ordenadas por prioridad real)   
    // Tarea MOTOR (alta prioridad, mucho stack)
    xTaskCreatePinnedToCore(
        motorTask,
        "MotorControl",
        8192,
        NULL,
        4,  // Prioridad media
        &motorTaskHandle,   // Handle
        1   // Core 1
    );
    // Tarea FUSIÓN (prioridad media, menos stack)
    xTaskCreatePinnedToCore(
        fusionTask, 
        "Fusion", 
        4096, 
        NULL, 
        2, 
        &fusionTaskHandle, 
        1);
 
    Serial.println("✅ Sistema inicializado correctamente");
    Serial.println("   - Core 1: Motor + SPI (Alta prioridad)");
    Serial.println("   - Core 0: Sensores + Monitor (Baja prioridad)");
}

void check_heap() {
    size_t free_heap = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
    size_t min_free = heap_caps_get_minimum_free_size(MALLOC_CAP_DEFAULT);
    
    Serial.printf("Heap libre: %d bytes, Min libre: %d bytes\n", 
                  free_heap, min_free);
    
    if (free_heap < 2048) {
        Serial.println("⚠️ ALERTA: Heap bajo!");
    }
}



// =========================================================
// TAREAS
// =========================================================

void motorTask(void *pvParameters) {
    esp_task_wdt_add(NULL);

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(20); // 50 Hz

    float currentSpeed = 0.0f;      // Velocidad suavizada actual
    float currentAngle = 90.0f;     // Ángulo suavizado actual
    uint16_t xCent = 128;
    uint16_t yCent = 128;
    float lastPWM = 0.0f;

    uint32_t lastDebugTime = 0;

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        esp_task_wdt_reset();

        // 1. Captura de encoders
        uint32_t pLeft = pulsesLeft.exchange(0, std::memory_order_relaxed);
        uint32_t pRight = pulsesRight.exchange(0, std::memory_order_relaxed);
        odom.update(pLeft, pRight);

        uint32_t now = millis();
        uint32_t last_valid_command_ms = lastValidCommandTime.load(std::memory_order_acquire);

        // 2. FAILSAFE / EMERGENCY STOP
        if (emergencyStop || (now - last_valid_command_ms > 1000)) {
            // Detener motores inmediatamente
            if (lastPWM != 0.0f || emergencyStop) {
                if (emergencyStop) {
                    Serial.println("🚨 EMERGENCY STOP activo");
                } else {
                    Serial.println("⚠️ FAILSAFE: Timeout, deteniendo motores");
                }
                speedController.setTarget(90.0f, 0.0f, 128, 128);
                speedController.updateControl();
                currentSpeed = 0.0f;
                currentAngle = 90.0f;
                lastPWM = 0.0f;
            }
            // Si fue emergency stop, además forzar el hardware
            if (emergencyStop) {
                motorController.setPWM(0, 90.0f, true);
            }
        }
        // 3. Nuevo comando SPI disponible
        else if (SPISlave::isCommandReady()) {
            // Actualizar timestamp
            lastValidCommandTime.store(now, std::memory_order_release);

            // Leer los nuevos valores atómicos (en variables temporales)
            float newAngle = targetAngle.load(std::memory_order_acquire);
            float newSpeed = targetSpeed.load(std::memory_order_acquire);
            uint16_t newXCent = targetXCentroid.load(std::memory_order_acquire);
            uint16_t newYCent = targetYCentroid.load(std::memory_order_acquire);

            // Debug
            Serial.printf("🎯 Leído de atómicos: Ang=%.1f° PWM=%.0f Centro=(%d,%d)\n",
                          newAngle, newSpeed, newXCent, newYCent);

            // Suavizado simple (evita cambios bruscos)
            currentAngle = currentAngle * 0.7f + newAngle * 0.3f;
            currentSpeed = currentSpeed * 0.7f + newSpeed * 0.3f;
            xCent = newXCent;
            yCent = newYCent;

            // Aplicar control de velocidad
            speedController.setTarget(currentAngle, currentSpeed, xCent, yCent);
            speedController.updateControl();

            lastPWM = currentSpeed;
        }
        // 4. Sin comando nuevo, pero todavía con velocidad activa
        else if (lastPWM > 0.0f) {
            // Re-aplicar el último comando suavizado
            speedController.setTarget(currentAngle, currentSpeed, xCent, yCent);
            speedController.updateControl();

            // Debug periódico
            if (now - lastDebugTime > 1000) {
                Serial.printf("🔄 Manteniendo: Ang=%.1f° Vel=%.0f\n", currentAngle, currentSpeed);
                lastDebugTime = now;
            }
        }
        // 5. Si no hay velocidad y no hay comando, no hacer nada (ya se detuvo)
    }
}


// Task ESPECÍFICA para sensores (alta frecuencia)
void tofSensorTask(void *pvParameters) {
    esp_task_wdt_add(NULL);

    while(1) {
        esp_task_wdt_reset();

        sensors.readAll(); // Lee TOFs locales
        fusion.updateTOF(0, sensors.frontDistance);
        fusion.updateTOF(1, sensors.leftDistance);
        fusion.updateTOF(2, sensors.rightDistance);
            
        //evaluarEmergenciaInmediata(globalSensorData);
        vTaskDelay(pdMS_TO_TICKS(20));
    }  // Tarea crítica: Lectura del sensor
}

// Nueva tarea de fusión sensorial (baja prioridad)
void fusionTask(void *pvParameters) {
    esp_task_wdt_add(NULL);

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(100); // 10 Hz
    
    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        esp_task_wdt_reset();
        
        // Procesar fusión TOF + odometría
        fusion.processFusion();        
        // Actualizar datos para enviar al maestro
        // Actualizar estado del robot (lo usará SPI para responder)
        // Nota: SPISlave accede directamente a fusion y odom, 
        // así que esto es solo para mantener actualizado
    }
}

// En tu tarea de comunicación SPI con el maestro
// Tarea dedicada para SPI (alta prioridad, core 0)
void spiTask(void* pvParameters) {
    esp_task_wdt_add(NULL);

    for (;;) {
        esp_task_wdt_reset();
        SPISlave::processSPICommunication();
        vTaskDelay(pdMS_TO_TICKS(5));  // Pequeña pausa para no saturar
    }
}


void printResetReason() {
    esp_reset_reason_t reason = esp_reset_reason();
    
    Serial.println("\n=== RAZÓN DE REINICIO ===");
    switch(reason) {
        case ESP_RST_POWERON:
            Serial.println("Power-on reset");
            break;
        case ESP_RST_EXT:
            Serial.println("External pin reset");
            break;
        case ESP_RST_SW:
            Serial.println("Software reset");
            break;
        case ESP_RST_PANIC:
            Serial.println("Software panic reset");
            break;
        case ESP_RST_INT_WDT:
            Serial.println("Interrupt watchdog reset");
            break;
        case ESP_RST_TASK_WDT:
            Serial.println("Task watchdog reset");
            break;
        case ESP_RST_WDT:
            Serial.println("Other watchdog reset");
            break;
        case ESP_RST_DEEPSLEEP:
            Serial.println("Deep sleep reset");
            break;
        case ESP_RST_BROWNOUT:
            Serial.println("Brownout reset");
            break;
        case ESP_RST_SDIO:
            Serial.println("SDIO reset");
            break;
        default:
            Serial.printf("Unknown reset reason: %d\n", reason);
    }
    Serial.println("=========================\n");
}

// En setup() tanto del Master como del Slave:
void printStructInfo() {
    Serial.println("=== SENSOR PAYLOAD LAYOUT ===");
    Serial.printf("sizeof(SensorPayload_t): %d bytes\n", sizeof(SensorPayload_t));
    Serial.printf("  pos_x_m:        offset=%d size=%d\n", offsetof(SensorPayload_t, pos_x_m), sizeof(float));
    Serial.printf("  pos_y_m:        offset=%d size=%d\n", offsetof(SensorPayload_t, pos_y_m), sizeof(float));
    Serial.printf("  theta_rad:      offset=%d size=%d\n", offsetof(SensorPayload_t, theta_rad), sizeof(float));
    Serial.printf("  linear_vel:     offset=%d size=%d\n", offsetof(SensorPayload_t, linear_vel_m_s), sizeof(float));
    Serial.printf("  angular_vel:    offset=%d size=%d\n", offsetof(SensorPayload_t, angular_vel_rad_s), sizeof(float));
    Serial.printf("  odom_timestamp: offset=%d size=%d\n", offsetof(SensorPayload_t, odom_timestamp), sizeof(uint32_t));
    Serial.printf("  obstacle_count: offset=%d size=%d\n", offsetof(SensorPayload_t, obstacle_count), sizeof(uint8_t));
    Serial.printf("  obstacle_dist:  offset=%d size=%d\n", offsetof(SensorPayload_t, obstacle_dist), sizeof(uint16_t)*3);
    Serial.printf("  obstacle_angle: offset=%d size=%d\n", offsetof(SensorPayload_t, obstacle_angle), sizeof(int16_t)*3);
    Serial.printf("  obstacle_sensor:offset=%d size=%d\n", offsetof(SensorPayload_t, obstacle_sensor), sizeof(uint8_t)*3);
    Serial.printf("  tof_front_mm:   offset=%d size=%d\n", offsetof(SensorPayload_t, tof_front_mm), sizeof(uint16_t));
    Serial.printf("  tof_left_mm:    offset=%d size=%d\n", offsetof(SensorPayload_t, tof_left_mm), sizeof(uint16_t));
    Serial.printf("  tof_right_mm:   offset=%d size=%d\n", offsetof(SensorPayload_t, tof_right_mm), sizeof(uint16_t));
    Serial.printf("  encoder_left:   offset=%d size=%d\n", offsetof(SensorPayload_t, encoder_left_cnt), sizeof(uint32_t));
    Serial.printf("  encoder_right:  offset=%d size=%d\n", offsetof(SensorPayload_t, encoder_right_cnt), sizeof(uint32_t));
    Serial.printf("  system_flags:   offset=%d size=%d\n", offsetof(SensorPayload_t, system_flags), sizeof(uint8_t));
    Serial.printf("  battery_mv:     offset=%d size=%d\n", offsetof(SensorPayload_t, battery_mv), sizeof(uint16_t));
    Serial.printf("  sensor_timestamp:offset=%d size=%d\n", offsetof(SensorPayload_t, sensor_timestamp), sizeof(uint32_t));
    Serial.println("==============================");
}
// =========================================================
// SETUP
// =========================================================
void setup() {
    Serial.begin(115200);
    delay(2000); // Esperar para estabilidad
    printResetReason();
    Serial.println("\n=== ESP32 SLAVE - CONTROL DE VELOCIDAD CON ENCODERS ===\n");
    // Configurar watchdog
    uint32_t wdt_timeout_s = 5;
    esp_err_t wdt_ret = esp_task_wdt_init(wdt_timeout_s, true);
    if (wdt_ret != ESP_OK) {
        Serial.println("❌ Error inicializando WDT");
    }
    esp_task_wdt_add(NULL);

    // Inicializar componentes
    Serial.println("🔧 Inicializando componentes...");

    // 1. Inicializar componentes
    odom.init();
    fusion.init(&odom);
    motorController.begin();
    speedController.begin();
    
    sensors.begin();
    setupEncoders();

        // Inicializar SPI Slave (¡después de fusion y odom!)
    if (!SPISlave::init(&fusion, &odom)) {
        Serial.println("❌ Error fatal: SPI Slave no inicializado");
        while(1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }

    // 2. Crear primitivas FreeRTOS // Crear tareas
    setup_tasks();

    // Verificar heap
    check_heap();
    printStructInfo();
    Serial.println("✅ Sistema FreeRTOS iniciado correctamente");
    Serial.println("   - SPI Slave en Core 1");
    Serial.println("   - Esperando comandos del Master...");
}

// =========================================================
// LOOP
// =========================================================
void loop() {
    esp_task_wdt_reset(); // RESET CRÍTICO para evitar reinicios
    vTaskDelay(pdMS_TO_TICKS(1000)); 
}
// =========================================================
// FIN DEL ARCHIVO
