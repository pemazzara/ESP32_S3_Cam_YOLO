// main.cpp - ESP32 con FreeRTOS
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include "sensor_control.h"
#include "motor_control.h"
#include "sonar_integration.h"
#include "speed_controller.h"
#include "config.h"
#include "GradientAligner.h"
#include "esp_task_wdt.h"
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <SPIFFS.h>
#include <atomic>
#include <freertos/semphr.h>
#include "esp_camera.h"
#include "camera_pins.h"


// Handles globales de FreeRTOS
// ==================== HANDLES DE TAREAS ====================
TaskHandle_t xSafetyTaskHandle = NULL;
TaskHandle_t xSensorRead_Handle = NULL;
TaskHandle_t sonarTaskHandle = NULL;
TaskHandle_t xSPITaskHandle = NULL;
TaskHandle_t xNavigationTaskHandle = NULL;
TaskHandle_t motorTaskHandle = NULL;
TaskHandle_t httpTaskHandle = NULL;

// ==================== VARIABLES DE CONTROL ====================
// ==================== VARIABLES ATÓMICAS ======================
// Variables compartidas (atómicas) para comunicación entre cores
std::atomic<float> targetAngle{90.0f};
std::atomic<int> targetSpeed{0};
std::atomic<uint16_t> targetXCentroid{0};  // Nuevo
std::atomic<uint16_t> targetYCentroid{0};  // Nuevo
std::atomic<uint32_t> lastCommandTime{0};
// Variables globales adicionales
std::atomic<uint32_t> lastValidCommandTime{0};
std::atomic<uint8_t> lastValidCmdId{0};
std::atomic<uint16_t> lastValidXCentroid{0};
std::atomic<uint16_t> lastValidYCentroid{0};
std::atomic<float> lastValidAngle{0};
std::atomic<uint8_t> lastValidSpeed{0};

 
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
void sonarTask(void *pvParameters);
void httpTask(void *pvParameters);
void motorTask(void *pvParameters);
void navigationTask(void *pvParameters);


//void checkJTAGPins();
void speedsToSpeedAngle(int16_t left, int16_t right, int& speed, int& angle);


// Instancias globales
MotorControl motorController;
UltraSonicMeasure sonar;
SpeedController speedController;

SensorControl sensors;
SensorData_t globalSensorData;

//AngleOptimizer angleOptimizer;


// ==================== WI-FI ====================
const char *ssid = "Mi_ssid";
const char *password = "Mi_contraseña";

// ==================== SERVIDORES ====================
WebServer serverHTTP(80);
WebSocketsServer webSocket(81); // Puerto 81 para WebSocket



//bool hasNewCommand = false;

void setupCamera() {
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
            
                    // ACTUALIZAR lastCommandTime SIEMPRE que llegue un paquete válido
                    // independientemente del cmdId
                    lastCommandTime.store(millis(), std::memory_order_release);
            
                    // Para cmdId=0 (STOP) o cmdId=2 (HEARTBEAT), no actualizar centroides
                    if (pkt->cmdId == 0x00 || pkt->cmdId == 0x02) {
                        // No modificar targetXCentroid, targetYCentroid, etc.
                        // Solo actualizar timestamp para evitar failsafe
                        static uint32_t lastIgnoreLog = 0;
                        if (millis() - lastIgnoreLog > 5000) {
                            Serial.printf("❤️ Heartbeat/Stop recibido (Cmd=%d), timestamp actualizado\n", pkt->cmdId);
                            lastIgnoreLog = millis();
                        }
                        return;  // No procesar centroides
                    }
            
                    // Solo para CMD_DRIVE (0x01) actualizar centroides
                    if (pkt->cmdId == 0x01) {
                        targetXCentroid.store(pkt->xCentroid, std::memory_order_release);
                        targetYCentroid.store(pkt->yCentroid, std::memory_order_release);
                        targetAngle.store(pkt->angle / 10.0f, std::memory_order_release);
                        targetSpeed.store(pkt->speed, std::memory_order_release);
                
                        Serial.printf("✅ [%u] Centro=(%d,%d) Ang=%.1f° Vel=%d Cmd=%d\n",
                              num, pkt->xCentroid, pkt->yCentroid,
                              pkt->angle / 10.0f, pkt->speed, pkt->cmdId);
                    }
                }
            }
            break;
    }
}
                    /* Guardar valores usando memory_order_release para consistencia
                    targetXCentroid.store(pkt->xCentroid, std::memory_order_release);
                    targetYCentroid.store(pkt->yCentroid, std::memory_order_release);
                    targetAngle.store(pkt->angle / 10.0f, std::memory_order_release);
                    targetSpeed.store(pkt->speed, std::memory_order_release);
                    lastCommandTime.store(millis(), std::memory_order_release);

                    Serial.printf("✅ [%u] Centro=(%d,%d) Ang=%.1f° Vel=%d Cmd=%d CRC=0x%02X\n",
                                  num, pkt->xCentroid, pkt->yCentroid,
                                  pkt->angle / 10.0f, pkt->speed, pkt->cmdId, calcCrc);
                } else {
                    Serial.printf("❌ CRC error: calc=0x%02X recv=0x%02X (payload[11])\n", 
                                  calcCrc, payload[11]);
                    // Debug: mostrar los primeros bytes para diagnóstico
                    Serial.printf("   Raw: ");
                    for(int i=0; i<12; i++) {
                        Serial.printf("%02X ", payload[i]);
                    }
                    Serial.println();
                }
            } else {
                Serial.printf("⚠️ Paquete inválido: %d bytes, headers: 0x%02X 0x%02X\n", 
                              length, payload[0], payload[1]);
            }
            break;
    }
}
*/

// ==================== ENVIAR TELEMETRÍA POR WEBSOCKET ====================

void sendTelemetry() {
    if (webSocket.connectedClients() == 0) return;

    uint8_t telemetry[20] = {0};
    telemetry[0] = 0xBB;
    telemetry[1] = 0x66;
    telemetry[2] = 85;  // Batería (simulada)
    telemetry[3] = targetSpeed.load(std::memory_order_relaxed);
    // Añadir sensores reales aquí
    telemetry[4] = 0;   // Distancia frontal
    telemetry[5] = 0;   // Distancia izquierda
    telemetry[6] = 0;   // Distancia derecha

    webSocket.broadcastBIN(telemetry, sizeof(telemetry));
}

bool autonomousMode = false;

void motorTask(void *pvParameters) {
    Serial.printf("⚙️ Motor Task en Core %d\n", xPortGetCoreID());
    
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(20); // 50 Hz
    // Variables para seguimiento continuo
    float lastAngle = 90.0f;
    float lastPWM = 0;
    // Configuración de seguimiento visual
    const uint16_t CENTRO_X_REF = 160;  // Depende de tu resolución (ej. 320x240)
    const uint16_t CENTRO_Y_REF = 120;
    const float KP_ANGULAR = 0.5f;      // Ganancia proporcional para ángulo
    const float KP_LINEAL = 0.3f;       // Ganancia para velocidad
    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        
        uint32_t ahora = millis();
        uint32_t tUltimoCmd = lastCommandTime.load(std::memory_order_acquire);
       // FAILSAFE: 1 segundo sin comandos -> parar
        if (ahora - tUltimoCmd > 1000) {
            if (lastPWM != 0) {
                Serial.println("⚠️ FAILSAFE: Timeout, deteniendo");
                speedController.setTarget(90.0f, 0, 0, 0);
                speedController.updateControl();
                lastPWM = 0;
            }
        } else {
            // Leer valores actuales
            float angulo = targetAngle.load(std::memory_order_acquire);
            int velocidad = targetSpeed.load(std::memory_order_acquire);
            uint16_t xCent = targetXCentroid.load(std::memory_order_acquire);
            uint16_t yCent = targetYCentroid.load(std::memory_order_acquire);
            
            // Convertir velocidad (0-255) a PWM (0-1023)
            float pwm_target = (velocidad / 255.0f) * 1023.0f;
            
            // DEBUG: Mostrar cada 1 segundo lo que se está enviando
            static uint32_t lastDebug = 0;
            if (millis() - lastDebug > 1000) {
                Serial.printf("📊 Control: Ang=%.1f PWM=%.0f Centro=(%d,%d)\n", 
                              angulo, pwm_target, xCent, yCent);
                lastDebug = millis();
            }
            
            // Enviar al controlador (siempre, el controlador decide si hay seguimiento)
            speedController.setTarget(angulo, pwm_target, xCent, yCent);
            speedController.updateControl();
            lastPWM = pwm_target;
        }
    }
}
        
/*
        if (ahora - tUltimoCmd > 500) {
            speedController.setTarget(90.0f, 0, 0, 0);
            speedController.updateControl();
        } else {
            float angulo = targetAngle.load(std::memory_order_relaxed);
            int velocidad = targetSpeed.load(std::memory_order_relaxed);
            uint16_t xCent = targetXCentroid.load(std::memory_order_relaxed);
            uint16_t yCent = targetYCentroid.load(std::memory_order_relaxed);
            
            float pwm_target = (velocidad / 255.0f) * 1023.0f;
            
            // Pasar centroides al controlador
            speedController.setTarget(angulo, pwm_target, xCent, yCent);
            speedController.updateControl();
        }
    }
}*/

/* ==================== TAREA MOTOR (Core 1, 50 Hz) ====================
void motorTask(void *pvParameters) {
    Serial.printf("⚙️ Motor Task en Core %d\n", xPortGetCoreID());
    
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(20); // 50 Hz
    
    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        
        uint32_t ahora = millis();
        uint32_t tUltimoCmd = lastCommandTime.load(std::memory_order_acquire);
        // FAILSAFE: Si no recibimos comandos de YOLO en más de 500ms, paramos por seguridad
        if (ahora - tUltimoCmd > 500) {
                speedController.setTarget(90.0f, 0);
                speedController.updateControl();
        } else {
            float angulo = targetAngle.load(std::memory_order_relaxed);
            int velocidad = targetSpeed.load(std::memory_order_relaxed);
            // Convertir velocidad (0-255) a PWM (0-1023)
            float pwm_target = (velocidad / 255.0f) * 1023.0f;
            pwm_target = constrain(pwm_target, 0.0f, 1023.0f);
            
            speedController.setTarget(angulo, pwm_target);
            speedController.updateControl();
        
        }
    }
}
*/
// ✅ SETUP FREERTOS SIMPLIFICADO
void setupFreeRTOS() {
    Serial.println("🔧 Inicializando FreeRTOS..."); 
    // Tareas FreeRTOS
    // Tasks en Core 0 (Aislada para radio)
    xTaskCreatePinnedToCore(
        httpTask, 
        "HTTP_Task", 
        4096, NULL, 
        1, 
        &httpTaskHandle, 
        0);

    // Tasks en Core 1 (Ordenadas por prioridad real)
    // Serial.println("   Creando task MotorControl...");       
    xTaskCreatePinnedToCore(
        motorTask,
        "MotorControl",
        4096,
        NULL,
        2,
        &motorTaskHandle,   // Handle
        1   // Core 1
    );
    /* Tarea TOF SENSORS alta prioridad, mucho stack
    Serial.println("   Creando task tofSensorRead...");
    xTaskCreatePinnedToCore(
        tofSensorTask,     // Función
        "SensorRead",        // Nombre
        8192,               // ← AUMENTA ESTE VALOR (ej: 8192 o 16384)
        NULL,               // Parámetros
        3, //configMAX_PRIORITIES - 2,
        &xSensorRead_Handle,
        0
    );*/

    /* Tasks comunes
    xTaskCreatePinnedToCore(
        sonarTask,
        "Sonar",
        8192,        // 8KB stack
        &sonar,
        1,  // Prioridad baja
        &sonarTaskHandle,
        1   // Core 1
    ); 

    xTaskCreatePinnedToCore(
        safetyTask,
        "Safety",
        4096,
        NULL,
        5, //TASK_PRIORITY_SAFETY,
        &xSafetyTaskHandle,
        1
    );
    
    Serial.println("   Creando task Navigation...");
    xTaskCreatePinnedToCore(
        navigationTask,
        "Navigation",
        4096, 
        NULL,
        2, //TASK_PRIORITY_NAV,
        &xNavigationTaskHandle,
        1
    );*/

    Serial.println("✅ FreeRTOS inicializado");
    Serial.println("   - Core 1: MotorControl (Prioridad Alta)");
    Serial.println("   - Core 0: HTTP Server (Prioridad Media)");   
}
/*
void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n=== PRUEBA DE PSRAM ===");
    Serial.printf("Flash size: %d MB\n", ESP.getFlashChipSize() / (1024 * 1024));
    Serial.printf("PSRAM size: %d bytes", ESP.getPsramSize());
    
    if (ESP.getPsramSize() == 0) {
        Serial.println(" (NO DETECTADA)");
        Serial.println("❌ La PSRAM no está configurada correctamente");
    } else {
        Serial.printf(" (%d MB)\n", ESP.getPsramSize() / (1024 * 1024));
        Serial.printf("PSRAM libre: %d bytes\n", ESP.getFreePsram());
        
        void* ptr = ps_malloc(1000000);
        if (ptr) {
            Serial.println("✅ Asignación de 1MB en PSRAM exitosa");
            free(ptr);
        }
    }
}*/

void setup() {
    Serial.begin(115200);
    
    // SPIFFS
    if (!SPIFFS.begin(true)) {
        Serial.println("❌ Error SPIFFS");
    }
    
    // Mutex
    sensorMutex = xSemaphoreCreateMutex();
    //xSpeedControllerMutex = xSemaphoreCreateMutex();
    //if (xSpeedControllerMutex == NULL) {
    //    Serial.println("❌ Error creando mutex. Reiniciando...");
     //   ESP.restart();
    //}
    
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
    // Cámara
    setupCamera();
        
    //sensors.begin();// Sensores ToF
    motorController.begin();
    
    speedController.begin();
    
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


uint16_t fuseFrontalDistances(SensorData_t &data) {
    uint8_t confidence = 0;
    uint16_t final_dist = 0;

    // 1. Validar rangos individuales (filtros de salud)
    bool sonar_valid = (data.sonarDistance > 20 && data.sonarDistance < 4000);
    bool tof_valid = (data.tofFront > 20 && data.tofFront < 2000);

    // 2. Lógica de decisión
    if (sonar_valid && tof_valid) {
        int16_t diff = abs((int16_t)data.sonarDistance - (int16_t)data.tofFront);
        
        if (diff < 150) { // Si coinciden razonablemente (15cm)
            final_dist = (data.sonarDistance + data.tofFront) / 2; // Fusión promedio
            confidence |= 0x18; // Bits de fuente: 11 (Fused)
        } else {
            // Discrepancia: El objeto es raro (quizás un cristal o una rejilla)
            // Regla de oro: Ante la duda, confía en la distancia MÁS CORTA (Seguridad)
            final_dist = min(data.sonarDistance, data.tofFront);
            confidence |= 0x04; // Bit de discrepancia
            confidence |= (data.sonarDistance < data.tofFront) ? 0x08 : 0x10; 
        }
    } else if (tof_valid) {
        final_dist = data.tofFront;
        confidence |= 0x10; // Fuente: TOF
    } else if (sonar_valid) {
        final_dist = data.sonarDistance;
        confidence |= 0x08; // Fuente: Sonar
    }

    data.sensorStatus = confidence; // Guardamos el veredicto
    return final_dist;
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


// Task ESPECÍFICA para sensores (alta frecuencia)
void tofSensorTask(void *pvParameters) {
    while(1) {

        sensors.readAll(); // Lee TOFs locales
        if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            globalSensorData.tofFront = sensors.frontDistance;
            globalSensorData.tofLeft = sensors.leftDistance;
            globalSensorData.tofRight = sensors.rightDistance;            // ... actualizar resto de TOFs ...
            globalSensorData.lastTofUpdate = millis();
            xSemaphoreGive(sensorMutex);
        }
         
        //spiMaster.evaluarEmergenciaInmediata(globalSensorData);
        vTaskDelay(pdMS_TO_TICKS(20));
    }  // Tarea crítica: Lectura del sensor
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

// 🔁 LOOP principal (en Core 1) - Puede usarse para tareas de baja prioridad
void loop() {
    // Mantiene vivo el servidor de WebSockets
    // Corre exclusivamente en el Core 1 gestionando la dinámica del robot
    webSocket.loop();
    // Enviar telemetría HUD al navegador cada 100ms
    static uint32_t lastTelemetry = 0;
    if (millis() - lastTelemetry > 100) {
        sendTelemetry();
        lastTelemetry = millis();
    }
    
    vTaskDelay(pdMS_TO_TICKS(5)); // Cede tiempo de CPU
}
