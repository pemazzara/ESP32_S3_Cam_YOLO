// main.cpp - ESP32 con FreeRTOS
// https://www.oceanlabz.in/getting-started-with-esp32-s3-wroom-n16r8-cam-dev-board/
// Para activar microfono en Chrome: chrome://flags/#unsafely-treat-insecure-origin-as-secure
// chrome://flags/#unsafely-treat-insecure-origin-as-secure
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include "sensor_control.h"
#include "motor_control.h"
#include "sonar_integration.h"
#include "speed_controller.h"
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
TaskHandle_t xSensorRead_Handle = NULL;
TaskHandle_t sonarTaskHandle = NULL;
TaskHandle_t motorTaskHandle = NULL;
TaskHandle_t httpTaskHandle = NULL;

// ==================== VARIABLES ATÓMICAS DE CONTROL ====================
// Para comunicación entre cores
// Variables de control compartidas (las que ya actualiza tu WebSocket)


std::atomic<uint32_t> lastCommandTime{0};
// Variables globales adicionales
std::atomic<uint32_t> lastValidCommandTime{0};
std::atomic<uint8_t> lastValidCmdId{0};
std::atomic<uint16_t> lastValidXCentroid{0};
std::atomic<uint16_t> lastValidYCentroid{0};
std::atomic<float> lastValidAngle{0};
std::atomic<uint8_t> lastValidSpeed{0};
std::atomic<float> targetAngle{90.0f};
std::atomic<int> targetSpeed{0};
std::atomic<uint16_t> targetXCentroid{0};  // Nuevo
std::atomic<uint16_t> targetYCentroid{0};  // Nuevo
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
void tofSensorTask(void *pvParameters);
void httpTask(void *pvParameters);
void motorTask(void *pvParameters);
// ==================== PROTOTIPOS DE FUNCIONES ISR ====================
void IRAM_ATTR isrLeft()  { 
    pulsesLeft.fetch_add(1, std::memory_order_relaxed); 
}
void IRAM_ATTR isrRight() { 
    pulsesRight.fetch_add(1, std::memory_order_relaxed); 
}


void checkJTAGPins();
void speedsToSpeedAngle(int16_t left, int16_t right, int& speed, int& angle);


// Instancias globales
MotorControl motorController;
SensorControl sensors;
SpeedController speedController;
SensorData_t globalSensorData;


// ==================== WI-FI ====================
//const char *ssid = "Mi_ssid";
//const char *password = "Mi_contraseña";
// ==================== WI-FI REAL ====================
const char *ssid = "Hervidero";
const char *password = "lSdS,seemm,slh+gqshielhpdlti";

// ==================== SERVIDORES ====================
WebServer serverHTTP(80);
WebSocketsServer webSocket(81); // Puerto 81 para WebSocket

void setupEncoders() {
    pinMode(PIN_ENCODER_LEFT, INPUT_PULLUP);  // Usa pullup interno
    pinMode(PIN_ENCODER_RIGHT, INPUT_PULLUP);
    
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_LEFT), isrLeft, RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_RIGHT), isrRight, RISING);
    
    Serial.println("✅ Encoders configurados en pines 14 y 47");
}



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
            
            // Siempre actualizar timestamp para evitar failsafe
            lastCommandTime.store(millis(), std::memory_order_release);
            
            // Según el comando, actuar de forma diferente
            switch (pkt->cmdId) {
                case 0x00: // STOP
                    targetSpeed.store(0, std::memory_order_release);
                    Serial.printf("📥 WS STOP: Velocidad forzada a 0\n");
                    break;
                    
                case 0x01: // DRIVE
                    targetAngle.store(pkt->angle / 10.0f, std::memory_order_relaxed);
                    targetSpeed.store(pkt->speed, std::memory_order_relaxed);
                    Serial.printf("📥 WS DRIVE: Ang=%.1f° Vel=%d\n", pkt->angle / 10.0f, pkt->speed);
                    break;
                    
                case 0x02: // HEARTBEAT
                    // Solo actualizar timestamp, no modificar velocidad ni ángulo
                    // El log se imprime cada 5 segundos para no saturar
                    static uint32_t lastHeartbeatLog = 0;
                    if (millis() - lastHeartbeatLog > 5000) {
                        Serial.println("💓 Heartbeat recibido");
                        lastHeartbeatLog = millis();
                    }
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

void sendTelemetry() {
    if (webSocket.connectedClients() == 0) return;

    uint8_t telemetry[20] = {0};
    telemetry[0] = 0xBB;
    telemetry[1] = 0x66;
    telemetry[2] = 85;  // Batería (simulada)
    telemetry[3] = targetSpeed.load(std::memory_order_relaxed);

    telemetry[4] = mapTo8Bit(globalSensorData.tofFront);   // Distancia frontal
    telemetry[5] = mapTo8Bit(globalSensorData.tofLeft);    // Distancia izquierda
    telemetry[6] = mapTo8Bit(globalSensorData.tofRight);   // Distancia derecha
Serial.printf("📡 Enviando Telemetría: Batería=%d%% Vel=%d ToF F:%dcm L:%dcm R:%dcm\n", 
              telemetry[2], telemetry[3], telemetry[4], telemetry[5], telemetry[6]);
    webSocket.broadcastBIN(telemetry, sizeof(telemetry));
}


// Para el control de velocidad, vamos a implementar un controlador PID incremental 
// que también incorpore corrección angular basada en el seguimiento visual del centroide. 
// Esto permitirá que el robot no solo mantenga la velocidad deseada, sino que 
// también corrija su trayectoria para seguir al objetivo detectado por la cámara.
/*
void motorTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(20); // 50 Hz estricto
    float lastPWM = 0;
    
    // Variables para debug no bloqueante
    uint32_t lastDebugTime = 0;
    uint32_t lastControlDebugTime = 0;

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        
        uint32_t ahora = millis();
        uint32_t tUltimoCmd = lastValidCommandTime.load(std::memory_order_acquire);
        
        // FAILSAFE: 1 segundo sin comandos -> parar
        if (ahora - tUltimoCmd > 1000) {
            if (lastPWM != 0) {
                Serial.println("⚠️ FAILSAFE: Timeout, deteniendo motores");
                speedController.setTarget(90.0f, 0, 0, 0);
                speedController.updateControl(); // Esto apaga motores y resetea integrales
                lastPWM = 0;
            }
        } else {
            // Leer valores actuales de los targets
            float angulo = targetAngle.load(std::memory_order_acquire);
            int velocidad = targetSpeed.load(std::memory_order_acquire);
            uint16_t xCent = targetXCentroid.load(std::memory_order_acquire);
            uint16_t yCent = targetYCentroid.load(std::memory_order_acquire);
            
            // Convertir velocidad (0-255) a PWM (0-1023)
            float pwm_target = (velocidad / 255.0f) * 1023.0f;
            
            // Aplicar zona muerta para evitar vibraciones a bajas velocidades
            if (velocidad < 5) {
                pwm_target = 0;
            }
            
            // Actualizar controlador de velocidad
            speedController.setTarget(angulo, pwm_target, xCent, yCent);
            speedController.updateControl(); // 🛞 ¡Acá ocurre la magia bilateral!
            lastPWM = pwm_target;
            
            // DEBUG: Mostrar información de control cada 1 segundo
            if (ahora - lastControlDebugTime > 1000) {
                Serial.printf("📊 Control: Ang=%.1f° Vel=%d PWM_Base=%.0f Centro=(%d,%d)\n", 
                              angulo, velocidad, pwm_target, xCent, yCent);
                
                // También mostrar el estado actual del controlador
                speedController.printDebug();
                lastControlDebugTime = ahora;
            }
        }
        
        // DEBUG adicional: Mostrar pulsos de encoder cada 2 segundos
        if (ahora - lastDebugTime > 2000) {
            uint32_t pLeft = pulsesLeft.load(std::memory_order_acquire);
            uint32_t pRight = pulsesRight.load(std::memory_order_acquire);
            Serial.printf("🔧 Pulsos encoder (crudos): L=%d R=%d\n", pLeft, pRight);
            lastDebugTime = ahora;
        }
    }
}
*/

void motorTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(20); // 50 Hz estricto
    float lastPWM = 0;

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        
        uint32_t ahora = millis();
        uint32_t tUltimoCmd = lastCommandTime.load(std::memory_order_acquire);
        
        if (ahora - tUltimoCmd > 1000) {
            if (lastPWM != 0) {
                Serial.println("⚠️ FAILSAFE: Timeout, deteniendo");
                speedController.setTarget(90.0f, 0, 0, 0);
                speedController.updateControl(); // Esto va a apagar motores y resetear integrales
                lastPWM = 0;
            }
        } else {
            float angulo = targetAngle.load(std::memory_order_acquire);
            int velocidad = targetSpeed.load(std::memory_order_acquire);
            uint16_t xCent = targetXCentroid.load(std::memory_order_acquire);
            uint16_t yCent = targetYCentroid.load(std::memory_order_acquire);
            
            float pwm_target = (velocidad / 255.0f) * 1023.0f;
            
            speedController.setTarget(angulo, pwm_target, xCent, yCent);
            speedController.updateControl(); // 🛞 ¡Acá ocurre la magia bilateral!
            lastPWM = pwm_target;
        }
    }
}

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
    // Crear la tarea de control de bajo nivel a 20Hz (cada 50ms)
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
    // Tarea TOF SENSORS alta prioridad, mucho stack
    Serial.println("   Creando task tofSensorRead...");
    xTaskCreatePinnedToCore(
        tofSensorTask,     // Función
        "SensorRead",        // Nombre
        8192,               // ← AUMENTA ESTE VALOR (ej: 8192 o 16384)
        NULL,               // Parámetros
        3, //configMAX_PRIORITIES - 2,
        &xSensorRead_Handle,
        0
    );

    Serial.println("✅ FreeRTOS inicializado");
    Serial.println("   - Core 1: MotorControl (Prioridad Alta)");
    Serial.println("   - Core 0: HTTP Server (Prioridad Media)");   
}

/*
void setup() {
    Serial.begin(115200);
    setupEncoders();
    // 2. INICIALIZAR SPIFFS (¡CRÍTICO! Debe ir antes del servidor)
    // El 'true' mapea y formatea automáticamente si el sistema de archivos está corrupto
    if (!SPIFFS.begin(true)) {
        Serial.println("❌ Error al montar SPIFFS. El servidor no podrá leer los archivos.");
    } else {
        Serial.println("📂 SPIFFS montado con éxito.");
    }

    // 3. CONFIGURAR EL SERVIDOR ASÍNCRONO
    // Servimos la raíz, mapeamos a la raíz de SPIFFS y seteamos index.html por defecto
    server.serveStatic("/", SPIFFS, "/")
          .setDefaultFile("index.html"); 

    // Aquí irían tus mapeos de WebSockets (si usas AsyncWebSocket)
    // ws.onEvent(onWsEvent);
    // server.addHandler(&ws);

    // 4. ARRANCAR SERVIDOR HTTP
    server.begin();
    Serial.println("🚀 Servidor HTTP Asíncrono iniciado.");
    // Mutex
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
    // Cámara
    setupCamera();
        
    sensors.begin();// Sensores ToF
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
*/
void setup() {
    Serial.begin(115200);
   
        // SPIFFS
    if (!SPIFFS.begin(true)) {
        Serial.println("❌ Error SPIFFS");
    }
    

    
    // Inicializar Hardware Crítico e Interrupciones

    setupCamera();
    setupEncoders();
    
    sensors.begin(); // Inicializar sensores ToF VL53L0X
    motorController.begin();
    speedController.begin();
    
    // Creación de Semáforos
    sensorMutex = xSemaphoreCreateMutex();

    // 2. Conectar a Wi-Fi primero (Es vital tener IP antes de levantar servicios de red)
    WiFi.begin(ssid, password);
    Serial.print("🌐 Conectando a Wi-Fi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\n✅ Wi-Fi Conectado: " + WiFi.localIP().toString());
    
    // 3. Gestión de Memoria PSRAM y Cámara (Evitamos fragmentación)
    Serial.printf("📊 PSRAM libre antes de la cámara: %d bytes\n", ESP.getFreePsram());
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
    

    // 7. Lanzar el Planificador de FreeRTOS (motorTask, etc.)
    setupFreeRTOS();
    
    // Reporte Final por Consola
    Serial.println("\n🤖 === SISTEMA YOLO ROBOT LISTO ===");
    Serial.println("   Web Panel: http://" + WiFi.localIP().toString());
    Serial.println("   Video Stream: http://" + WiFi.localIP().toString() + "/stream");
    Serial.println("   WebSocket URL: ws://" + WiFi.localIP().toString() + ":81\n");
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
         
        //evaluarEmergenciaInmediata(globalSensorData);
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
