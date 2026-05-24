
#include "sonar_integration.h"
#include <Arduino.h>
#include <driver/gpio.h>

SensorData_t sonar_data; // Variable global para almacenar los datos del sonar

UltraSonicMeasure::UltraSonicMeasure(uint8_t trigPin, uint8_t echoPin) 
    : SONAR_TRIG(trigPin), SONAR_ECHO(echoPin) {
    // Inicializar estructura de datos
    memset(&sonar_data, 0, sizeof(sonar_data));
    sonar_data.sensorStatus = 0;       // Bit 0: Sonar OK, Bit 1: TOFs OK
    sonar_data.sonarDistance = 1200;  // Valor por defecto
    sonar_data.a_vel = 0;
    sonar_data.emergency = false;
    sonar_data.timestamp = 0;
    for(int i = 0; i < FILTER_SIZE; i++) distanceBuffer[i] = 1200;
}


void UltraSonicMeasure::begin() {
    pinMode(SONAR_TRIG, OUTPUT);
    digitalWrite(SONAR_TRIG, LOW);
    pinMode(SONAR_ECHO, INPUT_PULLDOWN);
    
    data_mutex = xSemaphoreCreateMutex();
    if (!data_mutex) {
        Serial.println("❌ Sonar: Error creando mutex");
        return;
    }

    // 1. Si el canal ya estaba instalado, desinstalar primero (por si acaso)
    rmt_driver_uninstall(RMT_RX_CHANNEL);

    // 2. Configurar RMT para recepción
    rmt_config_t rmt_rx = {};
    rmt_rx.channel = RMT_RX_CHANNEL;
    rmt_rx.gpio_num = (gpio_num_t)SONAR_ECHO;
    rmt_rx.clk_div = 80;                     // → 1 tick = 1µs
    rmt_rx.mem_block_num = 1;
    rmt_rx.rmt_mode = RMT_MODE_RX;
    rmt_rx.rx_config.filter_en = true;
    rmt_rx.rx_config.filter_ticks_thresh = 100;   // Ignorar ruido < 100µs
    rmt_rx.rx_config.idle_threshold = 25000;      // Timeout > 25ms

    esp_err_t err = rmt_config(&rmt_rx);
    if (err != ESP_OK) {
        Serial.printf("❌ Error rmt_config: %d\n", err);
        return;
    }

    err = rmt_driver_install(rmt_rx.channel, 1000, 0);
    if (err != ESP_OK) {
        Serial.printf("❌ Error rmt_driver_install: %d\n", err);
        return;
    }

    Serial.printf("🔊 Sonar HC-SR04 con RMT canal %d en pin %d\n", RMT_RX_CHANNEL, SONAR_ECHO);
    initialized = true;
}

void UltraSonicMeasure::sonarUpdate() {
    uint16_t currentDistance = 0; // local
    uint32_t currentRawTime = 0;
    float approachRate = 0;

    // 1. Enviar pulso de trigger de 10µs
    digitalWrite(SONAR_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(SONAR_TRIG, LOW);

    // 2. Preparar el RMT para recibir
    rmt_rx_start(RMT_RX_CHANNEL, true); // true = resetear el reloj interno

    // 3. Esperar y obtener los items del ringbuffer
    RingbufHandle_t rb = NULL;
    rmt_get_ringbuf_handle(RMT_RX_CHANNEL, &rb);
    
    size_t rx_size = 0;
    rmt_item32_t* items = (rmt_item32_t*) xRingbufferReceive(rb, &rx_size, pdMS_TO_TICKS(100)); // Timeout de 100ms


    if (items && rx_size >= 1) {
        // El primer item debería contener la duración del pulso HIGH (el echo)
        // Asumiendo que el pulso empieza con un flanco de subida (level0=1)
        if (items[0].level0 == 1) {
            currentRawTime = items[0].duration0;  // Duración en ticks de 1µs
            if (currentRawTime > 115 && currentRawTime < 23500) {
                // Fórmula: (duración (µs) * 0.343 (mm/µs)) / 2
                // Podemos usar enteros para más velocidad: (duration_us * 343) / 2000
                currentDistance = (currentRawTime * 343) / 2000;
            }
        }
        vRingbufferReturnItem(rb, (void*) items);
    }
    uint32_t currentTime = millis();
    // Calcular velocidad si tenemos una lectura anterior
    /* Solo actualizamos el gradiente si el robot realmente giró
        if (abs(x3_actual - x3_anterior) > 2.0) { 
            x4_gradiente = (x1_filtrada - x1_anterior) / (x3_actual - x3_anterior);
        }*/
    if (lastSonarTime > 0 && currentTime > lastSonarTime) {
        uint32_t timeDiff = currentTime - lastSonarTime; // ms
        if (timeDiff > 20) { // Evitar cálculos erráticos por lecturas muy rápidas
                   float rawApproachRate = (float)(currentRawTime - lastRawTime) * 1000.0f / timeDiff;
            // Filtrar: 80% valor anterior, 20% valor nuevo
            approachRate = (0.8f * approachRate) + (0.2f * rawApproachRate);
            //approachRate = (float)(currentRawTime - lastRawTime) * 1000.0f / timeDiff;
        }
       
    }
    // Actualizar valores para la próxima lectura
    lastSonarTime = currentTime;
    lastRawTime = currentRawTime;
    
    rmt_rx_stop(RMT_RX_CHANNEL);

    // Aplicar filtro a la distancia (media móvil simple)
    distanceBuffer[filterIndex] = currentDistance;
    filterIndex = (filterIndex + 1) % FILTER_SIZE;
    uint32_t sum = 0;
    for (int i = 0; i < FILTER_SIZE; i++) sum += distanceBuffer[i];
    uint16_t filteredDistance = sum / FILTER_SIZE;

        // Actualizar estructura con mutex
    // Actualizar estructura con mutex
    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        // Escalar el valor adimensional por 1000 o 10000 para enviarlo como entero
        sonar_data.a_vel = (int16_t)(approachRate * 1000.0f);
        //sonar_data.a_vel = approachRate;
        sonar_data.sonarDistance = filteredDistance;  // usar distancia filtrada
        sonar_data.sensorStatus |= 0x01; // Marcar el sensor como OK
        if (filteredDistance > 0 && filteredDistance < 200) {
            if (emergency_counter < EMERGENCY_THRESHOLD) emergency_counter++;
        } else {
            emergency_counter = 0;
        }
        sonar_data.emergency = (emergency_counter >= EMERGENCY_THRESHOLD);
        sonar_data.timestamp = currentTime;
        xSemaphoreGive(data_mutex);
    }

    Serial.printf("[Sonar] %dmm/s → %dmm\n", sonar_data.a_vel, sonar_data.sonarDistance); 

}



bool UltraSonicMeasure::getLastSonarData(SensorData_t &data) {
    if (!initialized || !data_mutex) return false;
    
    bool success = false;
    
    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        memcpy(&data, &sonar_data, sizeof(SensorData_t));
        xSemaphoreGive(data_mutex);
        success = true;
    }
    
    return success;
}


bool UltraSonicMeasure::isEmergency() {
    bool emergency = true;
    if (millis() - sonar_data.timestamp > 500) return true;
    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        emergency = sonar_data.emergency;
        xSemaphoreGive(data_mutex);
    }
    
    return emergency;
}
bool UltraSonicMeasure::isSensorOK() {
    bool ok = false;   
    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        ok = sonar_data.sensorStatus & 0x01; // Verificar bit de estado del sonar
        xSemaphoreGive(data_mutex);
    }   
    return ok;
}
