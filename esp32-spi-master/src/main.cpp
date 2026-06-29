// main.cpp - ESP32 con FreeRTOS
// https://www.oceanlabz.in/getting-started-with-esp32-s3-wroom-n16r8-cam-dev-board/
// Para activar microfono en Chrome: chrome://flags/#unsafely-treat-insecure-origin-as-secure
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include "config.h"
#include "GradientAligner.h"
#include "esp_task_wdt.h"
#include <atomic>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <SPIFFS.h>
#include "esp_camera.h"
#include "camera_pins.h"
#include "spi_master.h"
#include "robot_control.h"

#define PIN_ENCODER_LEFT  14
#define PIN_ENCODER_RIGHT 39
#define ENCODER_PPR 20  // Pulsos por revolución de tu encoder


// Handles globales de FreeRTOS
TaskHandle_t xSafetyTaskHandle = NULL;
TaskHandle_t xSensorRead_Handle = NULL;
TaskHandle_t xSPITaskHandle = NULL;
TaskHandle_t xNavigationTaskHandle = NULL;
TaskHandle_t motorTaskHandle = NULL;
TaskHandle_t httpTaskHandle = NULL;

//QueueHandle_t xStatusQueue;
//QueueHandle_t xBLECommandQueue;
 

// Definición de las variables (solo UNA vez)
std::atomic<uint16_t> targetXCentroid{128};  // Inicializar en centro
std::atomic<uint16_t> targetYCentroid{128};  // Inicializar en centro
std::atomic<uint16_t> targetSpeed{0};
std::atomic<float> targetAngle{90.0f};
std::atomic<uint32_t> lastCommandTime{0};

// Contadores de pulsos ISR
std::atomic<uint32_t> pulsesLeft{0};
std::atomic<uint32_t> pulsesRight{0};


// ==================== MUTEX ====================
SemaphoreHandle_t sensorMutex;

// ====================ESP32 ESTRUCTURA DE PAQUETE ====================
#pragma pack(push, 1)
struct ControlPacket {
    uint8_t  header1;    // 0xAA
    uint8_t  header2;    // 0x55
    uint8_t  cmdId;
    uint16_t xCentroid;
    uint16_t yCentroid;
    int16_t  angle;      // ángulo * 10
    uint8_t  speed;
    uint8_t  flags;
    uint8_t  crc;
};
#pragma pack(pop)


// CRC-8 con polinomio 0x8C (inverso de 0x31)
uint8_t crc8(const uint8_t *data, size_t len) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) {
        uint8_t byte = data[i];
        for (int8_t bit = 0; bit < 8; bit++) {
            uint8_t mix = (crc ^ byte) & 0x01;
            crc >>= 1;
            if (mix) crc ^= 0x8C;  // Polinomio 0x8C (inverso de 0x31)
            byte >>= 1;
        }
    }
    return crc;
}

void setupFreeRTOS();
void printResetReason();
// ✅ DECLARAR TODAS LAS TASKS
void safetyTask(void *pvParameters);
void tofSensorTask(void *pvParameters);
void httpTask(void *pvParameters);
void spiMasterTask(void *pvParameters);
void navigationTask(void *pvParameters);


void sendTelemetry();
void sendStatusUpdate();
//void checkJTAGPins();
void speedsToSpeedAngle(int16_t left, int16_t right, int& speed, int& angle);


// Instancias globales
//SensorPayload_t globalSensorData;
static SPIMaster spiMaster; // Vive para siempre en el segmento de datos
//CalibrationData_t calibrationData;
//AngleOptimizer angleOptimizer;
// ==================== WI-FI ====================
//const char *ssid = "Mi_ssid";
//const char *password = "Mi_contraseña";
// ==================== WI-FI REAL ====================
const char *ssid = "Hervidero";
const char *password = "lSdS,seemm,slh+gqshielhpdlti";

// ==================== SERVIDORES ====================
WebServer serverHTTP(80);
WebSocketsServer webSocket(81); // Puerto 81 para WebSocket
void setupCamera() {
    // Configuración de la cámara OV2640
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    
    config.xclk_freq_hz = 20000000;
    config.frame_size = FRAMESIZE_QVGA;     // 320x240
    config.pixel_format = PIXFORMAT_JPEG;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.jpeg_quality = 15;               // Más compresión = menos memoria
    config.fb_count = 1;                    // Solo 1 buffer (reduce memoria)
    esp_err_t err = esp_camera_init(&config);
    // Imprimir memoria disponible antes de inicializar
    Serial.printf("📊 PSRAM libre antes de cámara: %d bytes\n", ESP.getFreePsram());
    
    if (err != ESP_OK) {
        Serial.printf("⚠️ Cámara no disponible (error 0x%x).\n", err);
        Serial.println("   Probando con resolución menor (QQVGA)...");
        
        // Segundo intento con resolución más baja
        config.frame_size = FRAMESIZE_QQVGA; // 160x120
        config.jpeg_quality = 20;            // Más compresión aún
        config.fb_count = 1;
        
        err = esp_camera_init(&config);
        if (err != ESP_OK) {
            Serial.printf("❌ Cámara no disponible incluso en QQVGA (error 0x%x).\n", err);
            Serial.println("   El stream MJPEG no funcionará. El resto del sistema sigue operativo.");
            return;
        }
        Serial.println("✅ Cámara OV2640 lista (QQVGA - 160x120)");
    } else {
        Serial.println("✅ Cámara OV2640 lista (QVGA - 320x240)");
    }
    
    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        s->set_vflip(s, 0);    // 1 = voltear verticalmente
        s->set_hmirror(s, 1);  // 1 = voltear horizontalmente
        s->set_brightness(s, 1);
        s->set_contrast(s, 0);
        s->set_saturation(s, 0);
        s->set_whitebal(s, 1);
        s->set_awb_gain(s, 1);
        s->set_vflip(s, 0);
        s->set_hmirror(s, 0);
    }
    
    Serial.printf("📊 PSRAM libre después de cámara: %d bytes\n", ESP.getFreePsram());
}

// ==================== HANDLER MJPEG ====================
void handleMjpeg() {
    // Tomamos el control directo del socket del cliente
    WiFiClient client = serverHTTP.client();
    
    // 1. Verificar de forma segura que la cámara responde
    camera_fb_t *fb_test = esp_camera_fb_get();
    if (!fb_test) {
        client.println("HTTP/1.1 503 Service Unavailable");
        client.println("Content-Type: text/plain");
        client.println("Access-Control-Allow-Origin: *"); // CORS también en errores
        client.println("Connection: close");
        client.println();
        client.println("Camara no disponible");
        return;
    }
    esp_camera_fb_return(fb_test);
    
    // 2. Enviar la cabecera inicial HTTP de forma manual y limpia
    // ¡Aquí inyectamos el CORS directamente en el flujo del socket!
    client.print("HTTP/1.1 200 OK\r\n");
    client.print("Content-Type: multipart/x-mixed-replace; boundary=frame\r\n");
    client.print("Access-Control-Allow-Origin: *\r\n"); // <-- Clave para ONNX Web
    client.print("Connection: keep-alive\r\n");
    client.print("\r\n"); // Fin de las cabeceras principales

    // 3. Bucle de transmisión de fotogramas (Streaming continuo)
    while (client.connected()) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            Serial.println("⚠️ Frame perdido (cámara ocupada)");
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        // Construir el delimitador de parte para multipart
        String header = "--frame\r\n"
                       "Content-Type: image/jpeg\r\n"
                       "Content-Length: " + String(fb->len) + "\r\n\r\n";
                       
        // Enviar cabecera de la imagen, el buffer binario y el salto de línea
        client.write(header.c_str(), header.length());
        client.write(fb->buf, fb->len);
        client.write("\r\n", 2);

        esp_camera_fb_return(fb);
        
        // Un delay prudencial para mantener estables los FPS y no ahogar el procesador
        vTaskDelay(pdMS_TO_TICKS(30)); 
    }
    
    Serial.println("📺 Stream de video finalizado (Cliente desconectado)");
}

// ==================== HANDLER WEBSOCKET ====================
void onWebSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
    switch (type) {
        case WStype_CONNECTED:
            Serial.printf("[%u] Cliente WebSocket conectado\n", num);
            lastCommandTime.store(millis(), std::memory_order_release);
            // Enviar telemetría inmediatamente al conectar
            sendTelemetry();
            break;

        case WStype_DISCONNECTED:
            Serial.printf("[%u] Cliente desconectado\n", num);
            lastCommandTime.store(0, std::memory_order_release);
            break;

        case WStype_BIN:
            if (length == 12 && payload[0] == 0xAA && payload[1] == 0x55) {
                uint8_t calcCrc = crc8(payload + 2, 9);
                if (calcCrc == payload[11]) {
                    ControlPacket* pkt = (ControlPacket*)payload;
                    
                    // Siempre actualizar timestamp para evitar failsafe
                    lastCommandTime.store(millis(), std::memory_order_release);
                    
                    switch (pkt->cmdId) {
                        case 0x00: // STOP
                            targetSpeed.store(0, std::memory_order_release);
                            targetAngle.store(90.0f, std::memory_order_release);
                            Serial.println("📥 WS STOP");
                            break;
                            
                        case 0x01: // DRIVE
                            {
                                float angle = pkt->angle / 10.0f;
                                uint16_t speed = pkt->speed;
                                uint16_t xCent = pkt->xCentroid;
                                uint16_t yCent = pkt->yCentroid;
                                
                                targetAngle.store(angle, std::memory_order_release);
                                // Mapear 0-255 → 0-1023 para PWM 10 bits
                                targetSpeed.store((uint16_t)(speed * 4), std::memory_order_release);
                                targetXCentroid.store(xCent, std::memory_order_release);
                                targetYCentroid.store(yCent, std::memory_order_release);
                                
                                Serial.printf("📥 WS DRIVE: Ang=%.1f° Vel=%d Centro=(%d,%d)\n", 
                                            angle, speed, xCent, yCent);
                            }
                            break;
                            
                        case 0x02: // HEARTBEAT
                            {
                                static uint32_t lastHeartbeatLog = 0;
                                if (millis() - lastHeartbeatLog > 5000) {
                                    Serial.println("💓 Heartbeat recibido");
                                    lastHeartbeatLog = millis();
                                }
                            }
                            break;

                        case 0x03: // RESET EMERGENCY ← NUEVO
                            Serial.println("📥 WS RESET EMERGENCY");
                            // Enviar comando SPI para resetear emergency
                            spiMaster.sendResetEmergency();
                            break;
                            
                        case 0x04: // RESET ODOMETRY ← NUEVO
                            Serial.println("📥 WS RESET ODOMETRY");
                            spiMaster.sendResetOdometry();
                            break;

                        default:
                            Serial.printf("⚠️ Comando desconocido: 0x%02X\n", pkt->cmdId);
                            break;
                    }
                } else {
                    Serial.printf("❌ CRC error: calc=0x%02X recv=0x%02X\n", calcCrc, payload[11]);
                }
            } else {
                Serial.printf("⚠️ Paquete inválido: %d bytes\n", length);
            }
            break;
    }
}

uint8_t mapTo8Bit(uint16_t distance_mm) {
    uint8_t distance_tx;
    
    // 1. Validar si el sensor falló o está fuera de rango
    if (distance_mm > 4000 || distance_mm == 0) { 
        distance_tx = 255; // Código de "zona libre / infinito"
    } else {
        uint16_t distance_cm = distance_mm / 10;    
        
        // 2. Aplicar saturación para evitar el overflow en 8 bits
        if (distance_cm > 254) {
            distance_tx = 254; 
        } else {
            distance_tx = (uint8_t)distance_cm;
        }
    }
    return distance_tx;    
}
// ==================== ENVIAR TELEMETRÍA POR WEBSOCKET ====================
// Ver version anterior
void sendTelemetry() {
    if (webSocket.connectedClients() == 0) return;

    SensorPayload_t sensorData;
    
    if (spiMaster.getLastResponse(&sensorData)) {
        uint8_t telemetry[20] = {0};
        telemetry[0] = 0xBB;
        telemetry[1] = 0x66;
        
        // Batería (simulada o real)
        telemetry[2] = sensorData.battery_mv > 0 ? 
                       map(sensorData.battery_mv, 3300, 4200, 0, 100) : 85;
        
        // Velocidad actual
        telemetry[3] = targetSpeed.load(std::memory_order_relaxed) / 4;  // 0-1020 → 0-255
        
        // Distancias TOF (convertir mm a cm, limitar a 255)
        telemetry[4] = min(255, sensorData.tof_front_mm / 10);
        telemetry[5] = min(255, sensorData.tof_left_mm / 10);
        telemetry[6] = min(255, sensorData.tof_right_mm / 10);
        
        // Estado del sistema
        telemetry[7] = sensorData.system_flags;  // Bit0: emergency, Bit1: failsafe
        
        // Obstáculos (datos adicionales)
        telemetry[8] = sensorData.obstacle_count;
        telemetry[9] = sensorData.obstacle_dist[0] / 10;  // Primer obstáculo en cm
        
        webSocket.broadcastBIN(telemetry, sizeof(telemetry));
        
        // Debug periódico
        static uint32_t lastTelemetryDebug = 0;
        if (millis() - lastTelemetryDebug > 2000) {
            Serial.printf("📡 Telemetría: Bat=%d%% Vel=%d ToF F:%d L:%d R:%d cm Obst=%d Flags=0x%02X\n", 
                         telemetry[2], telemetry[3], telemetry[4], 
                         telemetry[5], telemetry[6], telemetry[8], telemetry[7]);
            lastTelemetryDebug = millis();
        }
    }
}
/*
void sendTelemetry() {
    if (webSocket.connectedClients() == 0) return;

    uint8_t telemetry[20] = {0};
    telemetry[0] = 0xBB;
    telemetry[1] = 0x66;
    telemetry[2] = 85;  // Batería (simulada)
    telemetry[3] = targetSpeed.load(std::memory_order_relaxed);
    spiMaster.getLastResponse(&globalSensorData); 
    telemetry[4] = mapTo8Bit(globalSensorData.tof_front_mm);   // Distancia frontal
    telemetry[5] = mapTo8Bit(globalSensorData.tof_left_mm);    // Distancia izquierda
    telemetry[6] = mapTo8Bit(globalSensorData.tof_right_mm);   // Distancia derecha
//Serial.printf("📡 Enviando Telemetría: Batería=%d%% Vel=%d ToF F:%dcm L:%dcm R:%dcm\n", 
    //          telemetry[2], telemetry[3], telemetry[4], telemetry[5], telemetry[6]);
    webSocket.broadcastBIN(telemetry, sizeof(telemetry));
}
*/

bool autonomousMode = false;

// ✅ SETUP FREERTOS SIMPLIFICADO
void setupFreeRTOS() {
    Serial.println("🔧 Inicializando FreeRTOS...");
    
    // Crear tasks
    // Tasks en Core 0 (Aislada para radio)
    xTaskCreatePinnedToCore(
    httpTask, 
    "HTTP_Task", 
    4096, NULL, 
    1, 
    &httpTaskHandle, 
    0);
    // Tasks en Core 1 (Ordenadas por prioridad real)
    Serial.println("   Creando task SPI...");
    xTaskCreatePinnedToCore(
        spiMasterTask,
        "SPI_Master", 
        8192,  // ✅ Aumentar stack para SPI
        &spiMaster,   // ✅ Sin parámetros complejos
        3,  // Prioridad alta para SPI
        &xSPITaskHandle,
        1
    );

    Serial.println("✅ FreeRTOS inicializado");
    Serial.println("   - Core 1: MotorControl (Prioridad Alta)");
    Serial.println("   - Core 0: Ble (Baja prioridad)");
}


// ==================== TAREA HTTP (Core 0) ====================
void httpTask(void *pvParameters) {
    Serial.printf("🖥️ HTTP Task corriendo en Core %d\n", xPortGetCoreID());
    
    uint32_t lastDebug = 0;
    for (;;) {
        serverHTTP.handleClient();
        
        // Debug cada 5 segundos
        if (millis() - lastDebug > 5000) {
            Serial.printf("📊 HTTP: clientes WS conectados: %d\n", webSocket.connectedClients());
            lastDebug = millis();
        }
        
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}


void speedsToSpeedAngle(int16_t left, int16_t right, int& speed, int& angle) {
    // 1. Si ambos son cero, es STOP
    if (left == 0 && right == 0) {
        speed = 0;
        angle = 90;  // Ángulo por defecto, no importa porque speed=0
        return;
    }

    // 2. Calcular velocidad media (escala 0-1023)
    speed = (abs(left) + abs(right)) / 2;
    if (speed > 1023) speed = 1023;

    // 3. Determinar direcciones individuales
    bool leftForward = left >= 0;
    bool rightForward = right >= 0;

    // 4. Calcular factor de giro normalizado entre -1 y 1
    //    -1 = giro máximo a la izquierda, 1 = giro máximo a la derecha
    float turnFactor;
    float maxSpeed = max(abs(left), abs(right));
    if (maxSpeed == 0) {
        turnFactor = 0;
    } else {
        // La diferencia de velocidades normalizada por la máxima
        float diff = (right - left) / 2.0f;  // media de la diferencia
        turnFactor = diff / maxSpeed;
        // turnFactor queda entre -1 y 1 aproximadamente
    }

    // 5. Mapear turnFactor a ángulo según el caso
    if (leftForward && rightForward) {
        // Ambos adelante: ángulo entre 0° (derecha) y 180° (izquierda)
        // turnFactor = -1 -> giro izquierda (180°)
        // turnFactor = 0  -> recto (90°)
        // turnFactor = 1  -> giro derecha (0°)
        angle = 90 - (int)(turnFactor * 90);
    }
    else if (!leftForward && !rightForward) {
        // Ambos atrás: ángulo entre 180° y 360°
        // turnFactor = -1 -> giro izquierda atrás (180°? cuidado)
        // Realmente queremos que atrás recto sea 270°
        // Para atrás, el factor de giro invierte el sentido
        angle = 270 - (int)(turnFactor * 90);
    }
    else if (leftForward && !rightForward) {
        // Giro izquierda sobre el eje (left adelante, right atrás)
        // Esto debería dar un ángulo cercano a 180°
        // La magnitud del giro depende de la relación de velocidades
        float ratio = (float)abs(right) / (float)abs(left);
        angle = 180 - (int)(90 * ratio);
    }
    else if (!leftForward && rightForward) {
        // Giro derecha sobre el eje (left atrás, right adelante)
        float ratio = (float)abs(left) / (float)abs(right);
        angle = (int)(90 * ratio);
    }

    // 6. Normalizar ángulo a [0, 360)
    angle = (angle + 360) % 360;
}

void spiMasterTask(void *pvParameters) {
    SPIMaster* spiMasterPtr = (SPIMaster*)pvParameters;
    
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(20); // 50 Hz
    uint16_t lastPWM = 0;
    
    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        
        uint32_t ahora = millis();
        uint32_t tUltimoCmd = lastCommandTime.load(std::memory_order_acquire);
        
        if (ahora - tUltimoCmd > 1000) {
            // FAILSAFE: timeout sin comandos
            if (lastPWM != 0) {
                Serial.println("⚠️ FAILSAFE: Timeout, enviando STOP");
                spiMasterPtr->sendStopCommand();
                lastPWM = 0;
                // Resetear centroides al parar
                targetXCentroid.store(128, std::memory_order_release);
                targetYCentroid.store(128, std::memory_order_release);
            } else {
                // Enviar heartbeat esporádico
                static uint32_t lastHeartbeat = 0;
                if (ahora - lastHeartbeat > 500) {
                    spiMasterPtr->sendHeartbeat();
                    lastHeartbeat = ahora;
                }
            }
        } else {
            // Comando activo: leer valores atómicos una sola vez
            float angulo = targetAngle.load(std::memory_order_acquire);
            uint16_t velocidad = targetSpeed.load(std::memory_order_acquire);
            
            // Enviar comando con los centroides actuales
            spiMasterPtr->sendDriveCommand((int)velocidad, angulo, false);
            
            lastPWM = velocidad;
            
            // Debug periódico
            static uint32_t lastDebug = 0;
            if (ahora - lastDebug > 1000) {
                SensorPayload_t data; 
                spiMasterPtr->getLastResponse(&data);
                Serial.printf("📡 Slave: (%.2f,%.2f) θ=%.1f° Obst=%d\n",
                             data.pos_x_m, data.pos_y_m,
                             data.theta_rad * 180.0f / PI,
                             data.obstacle_count);
                
                if (data.obstacle_count > 0) {
                    for (int i = 0; i < data.obstacle_count && i < 3; i++) {
                        Serial.printf("   🚧 Obst %d: %dmm @ %d° (sensor %d)\n",
                                     i, data.obstacle_dist[i], 
                                     data.obstacle_angle[i], data.obstacle_sensor[i]);
                    }
                }
                lastDebug = ahora;
            }
        }
    }
}


void printResetReason() {
    esp_reset_reason_t reason = esp_reset_reason();
    Serial.printf("🔄 Reset Reason: ");
    switch(reason) {
        case ESP_RST_POWERON: Serial.println("Power On"); break;
        case ESP_RST_EXT: Serial.println("External Reset"); break;
        case ESP_RST_SW: Serial.println("Software Reset"); break;
        case ESP_RST_PANIC: Serial.println("Exception/Panic"); break;
        case ESP_RST_INT_WDT: Serial.println("Interrupt Watchdog"); break;
        case ESP_RST_TASK_WDT: Serial.println("Task Watchdog"); break;
        case ESP_RST_WDT: Serial.println("Other Watchdog"); break;
        case ESP_RST_DEEPSLEEP: Serial.println("Deep Sleep"); break;
        case ESP_RST_BROWNOUT: Serial.println("Brownout"); break;
        case ESP_RST_SDIO: Serial.println("SDIO Reset"); break;
        default: Serial.println("Unknown"); break;
    }
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

// 🏁 SETUP - Inicialización
void setup() {
    Serial.begin(115200);
    delay(1000);
    
    printResetReason();
    printStructInfo();
    // Inicializar SPI Master ANTES de crear tareas
    if (!spiMaster.begin()) {
        Serial.println("❌ Error fatal: SPI Master no inicializado");
        while(1) { delay(1000); }
    }
    // Cámara
    setupCamera();
    // 2. INICIALIZAR SPIFFS (¡CRÍTICO! Debe ir antes del servidor)
    // El 'true' mapea y formatea automáticamente si el sistema de archivos está corrupto
    if (!SPIFFS.begin(true)) {
        Serial.println("❌ Error al montar SPIFFS. El servidor no podrá leer los archivos.");
    } else {
        Serial.println("📂 SPIFFS montado con éxito.");
    }
    
    sensorMutex = xSemaphoreCreateMutex();   
    // Wi-Fi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\n✅ Wi-Fi: " + WiFi.localIP().toString());
    
       // Liberar PSRAM antes de inicializar la cámara
    Serial.printf("📊 PSRAM libre antes de liberar: %d bytes\n", ESP.getFreePsram());
    
    // Si hay poca PSRAM, reiniciar para limpiar fragmentación
    if (ESP.getFreePsram() < 2000000) { // Menos de 2MB libres
        Serial.println("⚠️ PSRAM baja, reiniciando para limpiar...");
        delay(1000);
        ESP.restart();
    }
        // 5. CONFIGURACIÓN DEL SERVIDOR HTTP ÚNICO (serverHTTP)
    // Servir el archivo index.html directamente desde SPIFFS
    serverHTTP.serveStatic("/", SPIFFS, "/index.html");
    serverHTTP.on("/stream", handleMjpeg);
    serverHTTP.begin();
      
    // WebSocket
    webSocket.begin();
    webSocket.onEvent(  onWebSocketEvent);
     
     
    setupFreeRTOS();


    Serial.println("✅ Sistema listo:");
    Serial.println("   Web: http://" + WiFi.localIP().toString());
    Serial.println("   Stream: http://" + WiFi.localIP().toString() + "/stream");
    Serial.println("   WS: ws://" + WiFi.localIP().toString() + ":81");
}

// 🔁 LOOP principal (en Core 1) - Puede usarse para tareas de baja prioridad
void loop() {
    // Mantiene vivo el servidor de WebSockets
    // Corre exclusivamente en el Core 1 gestionando la dinámica del robot
    webSocket.loop();
    //server.handleClient();
    // Enviar telemetría HUD al navegador cada 100ms
    static uint32_t lastTelemetry = 0;
    if (millis() - lastTelemetry > 100) {
        sendTelemetry();
        lastTelemetry = millis();
    }
    
    vTaskDelay(pdMS_TO_TICKS(5)); // Cede tiempo de CPU
}
