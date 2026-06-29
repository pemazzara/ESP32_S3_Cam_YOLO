/* spi_master.cpp
#include "spi_master.h"
#include <esp_err.h>
#include <esp_log.h>

SPIMaster::SPIMaster() : initialized(false), spi(nullptr), errorCounter(0) {
    spi_mutex = xSemaphoreCreateMutex();
    data_mutex = xSemaphoreCreateMutex();
    
    // Inicializar buffers
    memset(&master_tx_buffer, 0, sizeof(SPICommandFrame_t));
    memset(&master_rx_buffer, 0, sizeof(SPIResponseFrame_t));
    memset(&last_drive_payload, 0, sizeof(ControlPayload_t));
}

SPIMaster::~SPIMaster() {
    if (spi_mutex) vSemaphoreDelete(spi_mutex);
    if (data_mutex) vSemaphoreDelete(data_mutex);
    if (spi) spi_bus_remove_device(spi);
}

bool SPIMaster::begin() {
    Serial.println("[SPI Master] Inicializando...");
    
    if (!spi_mutex || !data_mutex) {
        Serial.println("[SPI Master] ❌ Error creando mutex");
        return false;
    }
    
    // Configurar pines
    pinMode(SPI_MASTER_SS, OUTPUT);
    digitalWrite(SPI_MASTER_SS, HIGH);
    
    // 1. Configuración del bus SPI
    spi_bus_config_t buscfg = {
        .mosi_io_num = SPI_MASTER_MOSI,
        .miso_io_num = SPI_MASTER_MISO,
        .sclk_io_num = SPI_MASTER_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = (int)std::max(sizeof(SPICommandFrame_t), sizeof(SPIResponseFrame_t)),
        .flags = 0,
        .intr_flags = 0
    };
    
    // 2. Configuración del dispositivo SPI
    spi_device_interface_config_t devcfg = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .mode = 0,                          // SPI Mode 0
        .duty_cycle_pos = 128,
        .cs_ena_pretrans = 0,
        .cs_ena_posttrans = 0,
        .clock_speed_hz = 1000000,          // 1 MHz (conservador)
        .input_delay_ns = 0,
        .spics_io_num = SPI_MASTER_SS,
        .flags = 0,     // Half-duplex para evitar colisiones
        .queue_size = 1,
        .pre_cb = NULL,
        .post_cb = NULL,
    };
    
    // 3. Inicializar bus SPI
    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        Serial.printf("[SPI Master] ❌ Error init bus: %s\n", esp_err_to_name(ret));
        return false;
    }
    
    // 4. Añadir dispositivo al bus
    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &spi);
    if (ret != ESP_OK) {
        Serial.printf("[SPI Master] ❌ Error add device: %s\n", esp_err_to_name(ret));
        return false;
    }
    
    initialized = true;
    
    // Enviar comando STOP inicial
    sendStopCommand();
    
    Serial.println("[SPI Master] ✅ Inicializado correctamente");
    return true;
}

bool SPIMaster::testSPI() {
    if (!initialized) {
        Serial.println("❌ SPI no inicializado");
        return false;
    }
    
    SPICommandFrame_t test_cmd;
    memset(&test_cmd, 0, sizeof(test_cmd));
    prepareCommandFrame(&test_cmd);
    
    SPIResponseFrame_t test_resp;
    memset(&test_resp, 0, sizeof(test_resp));
    
    spi_transaction_t trans;
    memset(&trans, 0, sizeof(trans));
    trans.length = sizeof(SPICommandFrame_t) * 8;
    trans.tx_buffer = &test_cmd;
    trans.rx_buffer = &test_resp;
    
    esp_err_t ret = spi_device_transmit(spi, &trans);
    
    if (ret == ESP_OK) {
        Serial.printf("✅ Test SPI exitoso. Magic recibido: 0x%02X\n", test_resp.magic);
        return true;
    } else {
        Serial.printf("❌ Test SPI fallido: %s\n", esp_err_to_name(ret));
        return false;
    }
}

bool SPIMaster::sendSPIFrame(uint8_t* data, size_t len) {
    if (!data || !initialized) return false;
    
    // Limpiar buffer de recepción
    memset(&master_rx_buffer, 0, sizeof(SPIResponseFrame_t));
    
    // Configurar transacción
    spi_transaction_t trans;
    memset(&trans, 0, sizeof(trans));
    trans.length = len * 8;
    trans.tx_buffer = data;
    trans.rx_buffer = &master_rx_buffer;
    
    // Transmitir
    esp_err_t ret = spi_device_transmit(spi, &trans);
    if (ret != ESP_OK) {
        errorCounter++;
        Serial.printf("[SPI] Error transmisión: %s\n", esp_err_to_name(ret));
        return false;
    }
    
    // Validar respuesta
    if (master_rx_buffer.magic == SPI_MAGIC_SLAVE) {
        uint16_t computed_crc = calculateCRC16(
            (const uint8_t*)&master_rx_buffer.payload, 
            sizeof(SensorPayload_t)
        );
        
        if (computed_crc == master_rx_buffer.crc16) {
            errorCounter = 0;
            return true;
        } else {
            errorCounter++;
            Serial.printf("[SPI] Error CRC: esperado=0x%04X recibido=0x%04X\n", 
                         computed_crc, master_rx_buffer.crc16);
            return false;
        }
    } else {
        errorCounter++;
        handleCommunicationErrors(master_rx_buffer.magic);
        return false;
    }
}

void SPIMaster::sendDriveCommand(float speed, float angle_deg, bool tracking) {
    if (!initialized) return;
    
    // Preparar payload
    master_tx_buffer.payload.type = CMD_DRIVE;
    master_tx_buffer.payload.speed = (int16_t)constrain(speed, -1023, 1023);
    master_tx_buffer.payload.angle = angleToPayload(angle_deg);
    master_tx_buffer.payload.xCentroid = 128;
    master_tx_buffer.payload.yCentroid = 128;
    master_tx_buffer.payload.flags = packFlags(tracking, false);
    
    // Guardar último comando
    last_drive_payload = master_tx_buffer.payload;
    
    // Calcular CRC y enviar
    prepareCommandFrame(&master_tx_buffer);
    
    if (sendSPIFrame((uint8_t*)&master_tx_buffer, sizeof(SPICommandFrame_t))) {
        // Éxito - los datos ya están en master_rx_buffer
    }
}

void SPIMaster::sendStopCommand() {
    if (!initialized) return;
    
    master_tx_buffer.payload.type = CMD_STOP;
    master_tx_buffer.payload.speed = 0;
    master_tx_buffer.payload.angle = angleToPayload(90.0f);
    master_tx_buffer.payload.xCentroid = 128;
    master_tx_buffer.payload.yCentroid = 128;
    master_tx_buffer.payload.flags = 0;
    
    last_drive_payload = master_tx_buffer.payload;
    prepareCommandFrame(&master_tx_buffer);
    sendSPIFrame((uint8_t*)&master_tx_buffer, sizeof(SPICommandFrame_t));
}

void SPIMaster::sendHeartbeat() {
    if (!initialized) return;
    
    // Reenviar último comando con tipo HEARTBEAT
    SPICommandFrame_t hb_frame;
    memcpy(&hb_frame.payload, &last_drive_payload, sizeof(ControlPayload_t));
    hb_frame.payload.type = CMD_HEARTBEAT;
    prepareCommandFrame(&hb_frame);
    sendSPIFrame((uint8_t*)&hb_frame, sizeof(SPICommandFrame_t));
}

void SPIMaster::handleCommunicationErrors(uint8_t magic) {
    switch (magic) {
        case 0xFF:
            Serial.println("[SPI] 🚨 Slave desconectado o MISO en HIGH");
            break;
        case 0x00:
            Serial.println("[SPI] ⚠️ MISO en LOW (respuesta vacía)");
            break;
        case SPI_MAGIC_MASTER:  // 0xA5
            Serial.println("[SPI] 🔄 Error de espejo (MOSI=MISO?)");
            break;
        default:
            Serial.printf("[SPI] ❓ Magic inesperado: 0x%02X\n", magic);
            break;
    }
    
    // Si hay muchos errores, intentar reconectar
    if (errorCounter > 10) {
        Serial.println("[SPI] Demasiados errores, reintentando...");
        reconnect();
    }
}

bool SPIMaster::reconnect() {
    Serial.println("[SPI] Re-inicializando bus...");
    if (spi) {
        spi_bus_remove_device(spi);
        spi = nullptr;
    }
    initialized = false;
    errorCounter = 0;
    return begin();
}

SensorPayload_t SPIMaster::getLastResponse(SensorPayload_t* payload) {
    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (payload) {
            memcpy(payload, &master_rx_buffer.payload, sizeof(SensorPayload_t));
        }
        SensorPayload_t result = master_rx_buffer.payload;
        xSemaphoreGive(data_mutex);
        return result;
    }
    return SensorPayload_t{};
}

bool SPIMaster::isSlaveInEmergency() {
    return (master_rx_buffer.payload.system_flags & 0x01) != 0;
}

bool SPIMaster::isSlaveMoving() {
    return (master_rx_buffer.payload.system_flags & 0x02) != 0;
}
*/

// spi_master.cpp - VERSIÓN FINAL
#include "spi_master.h"
#include <algorithm>
#include <esp_err.h>
#include "robot_control.h"



SPIMaster::SPIMaster() : initialized(false), spi(nullptr), errorCounter(0) {
    tx_buffer_dma = (SPICommandFrame_t*) heap_caps_aligned_alloc(
        32, sizeof(SPICommandFrame_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    rx_buffer_a = (SPIResponseFrame_t*) heap_caps_aligned_alloc(
        32, sizeof(SPIResponseFrame_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    rx_buffer_b = (SPIResponseFrame_t*) heap_caps_aligned_alloc(
        32, sizeof(SPIResponseFrame_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);

    rx_dma  = rx_buffer_a;
    rx_safe = rx_buffer_b;

    spi_mutex  = xSemaphoreCreateMutex();
    swap_mutex = xSemaphoreCreateMutex();

    if (tx_buffer_dma) memset(tx_buffer_dma, 0, sizeof(SPICommandFrame_t));
    if (rx_buffer_a)   memset(rx_buffer_a, 0, sizeof(SPIResponseFrame_t));
    if (rx_buffer_b)   memset(rx_buffer_b, 0, sizeof(SPIResponseFrame_t));
}

SPIMaster::~SPIMaster() {
    if (tx_buffer_dma) { heap_caps_free(tx_buffer_dma); tx_buffer_dma = nullptr; }
    if (rx_buffer_a)   { heap_caps_free(rx_buffer_a);   rx_buffer_a = nullptr; }
    if (rx_buffer_b)   { heap_caps_free(rx_buffer_b);   rx_buffer_b = nullptr; }
    if (spi_mutex)     { vSemaphoreDelete(spi_mutex);   spi_mutex = nullptr; }
    if (swap_mutex)    { vSemaphoreDelete(swap_mutex);  swap_mutex = nullptr; }
    if (spi)           { spi_bus_remove_device(spi);    spi = nullptr; }
}

bool SPIMaster::begin() {
    if (!tx_buffer_dma || !rx_buffer_a || !rx_buffer_b) {
        Serial.println("❌ Error: Buffers DMA no asignados");
        return false;
    }

    pinMode(SPI_MASTER_SS, OUTPUT);
    digitalWrite(SPI_MASTER_SS, HIGH);

    spi_bus_config_t buscfg = {
        .mosi_io_num = SPI_MASTER_MOSI,
        .miso_io_num = SPI_MASTER_MISO,
        .sclk_io_num = SPI_MASTER_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = sizeof(SPIResponseFrame_t),
        .flags = 0,
        .intr_flags = 0
    };

    spi_device_interface_config_t devcfg = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .mode = 0,
        .duty_cycle_pos = 128,
        .cs_ena_pretrans = 0,
        .cs_ena_posttrans = 0,
        .clock_speed_hz = 1000000,
        .input_delay_ns = 0,
        .spics_io_num = SPI_MASTER_SS,
        .flags = 0,
        .queue_size = 1,
        .pre_cb = NULL,
        .post_cb = NULL,
    };

    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        Serial.printf("❌ Error bus SPI: %s\n", esp_err_to_name(ret));
        return false;
    }

    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &spi);
    if (ret != ESP_OK) {
        Serial.printf("❌ Error dispositivo SPI: %s\n", esp_err_to_name(ret));
        return false;
    }

    initialized = true;
    Serial.printf("✅ SPI Master listo (%d bytes/transacción)\n", sizeof(SPIResponseFrame_t));
    return true;
}

void SPIMaster::swapBuffers() {
    if (xSemaphoreTake(swap_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
        SPIResponseFrame_t* temp = rx_safe;
        rx_safe = rx_dma;
        rx_dma = temp;
        xSemaphoreGive(swap_mutex);
    }
}

bool SPIMaster::sendSPIFrame(uint8_t* data, size_t len) {
    if (!data || !initialized || !tx_buffer_dma || !rx_dma) return false;
    if (xSemaphoreTake(spi_mutex, pdMS_TO_TICKS(10)) != pdTRUE) return false;

    size_t copy_len = std::min(len, sizeof(SPICommandFrame_t));
    memcpy(tx_buffer_dma, data, copy_len);
    memset(rx_dma, 0, sizeof(SPIResponseFrame_t));

    spi_transaction_t trans = {};
    trans.length = sizeof(SPIResponseFrame_t) * 8;
    trans.tx_buffer = tx_buffer_dma;
    trans.rx_buffer = rx_dma;

    esp_err_t ret = spi_device_transmit(spi, &trans);
    bool success = false;

    if (ret == ESP_OK) {
        swapBuffers();
        if (rx_safe->magic == SPI_MAGIC_SLAVE) {
            uint16_t crc = calculateCRC16(
                (const uint8_t*)&rx_safe->payload, sizeof(SensorPayload_t));
            if (crc == rx_safe->crc16) {
                errorCounter = 0;
                success = true;
            }
        }
    }

    if (!success) errorCounter++;
    xSemaphoreGive(spi_mutex);
    return success;
}

void SPIMaster::sendDriveCommand(int speed, float angle_deg, bool tracking) {
    if (!initialized) return;
    
    SPICommandFrame_t cmd = {};
    cmd.magic = SPI_MAGIC_MASTER;
    cmd.payload.type = CMD_DRIVE;
    
    // Convertir velocidad 0-1023 a rango con signo si es necesario
    cmd.payload.speed = (int16_t)speed;  // 0-1023, siempre positivo
    
    cmd.payload.angle = angleToPayload(angle_deg);
    
    // USAR los centroides almacenados, no hardcodear
    cmd.payload.xCentroid = targetXCentroid.load(std::memory_order_acquire);
    cmd.payload.yCentroid = targetYCentroid.load(std::memory_order_acquire);
    
    cmd.payload.flags = packFlags(tracking, false);
    
    prepareCommandFrame(&cmd);
    sendSPIFrame((uint8_t*)&cmd, sizeof(SPICommandFrame_t));
    
    // Debug detallado
    static uint32_t lastCmdDebug = 0;
    if (millis() - lastCmdDebug > 2000) {
        Serial.printf("📤 SPI CMD: Vel=%d Ang=%.1f° Centro=(%d,%d) Tracking=%d\n",
                     speed, angle_deg, 
                     cmd.payload.xCentroid, 
                     cmd.payload.yCentroid,
                     tracking);
        lastCmdDebug = millis();
    }
}
bool SPIMaster::sendResetEmergency() {
    if (!initialized) return false;
    
    SPICommandFrame_t cmd = {};
    cmd.magic = SPI_MAGIC_MASTER;
    cmd.payload.type = CMD_RESET_EMERGENCY;  // 0x07 según tu enumeración
    cmd.payload.speed = 0;
    cmd.payload.angle = 90.0f * 10;  // Formato *10
    cmd.payload.xCentroid = 128;
    cmd.payload.yCentroid = 128;
    cmd.payload.flags = 0;
    
    prepareCommandFrame(&cmd);
    return sendSPIFrame((uint8_t*)&cmd, sizeof(SPICommandFrame_t));
}

bool SPIMaster::sendResetOdometry() {
    if (!initialized) return false;
    
    SPICommandFrame_t cmd = {};
    cmd.magic = SPI_MAGIC_MASTER;
    cmd.payload.type = CMD_RESET_ODOMETRY;  // 0x08 según tu enumeración
    cmd.payload.speed = 0;
    cmd.payload.angle = 0;
    cmd.payload.xCentroid = 0;
    cmd.payload.yCentroid = 0;
    cmd.payload.flags = 0;
    
    prepareCommandFrame(&cmd);
    return sendSPIFrame((uint8_t*)&cmd, sizeof(SPICommandFrame_t));
}

bool SPIMaster::sendStopCommand() {
    if (!initialized) return false;
    
    SPICommandFrame_t cmd = {};
    cmd.magic = SPI_MAGIC_MASTER;
    cmd.payload.type = CMD_STOP;  // 0x01
    cmd.payload.speed = 0;
    cmd.payload.angle = 90.0f * 10;
    cmd.payload.xCentroid = 128;
    cmd.payload.yCentroid = 128;
    cmd.payload.flags = 0;
    
    prepareCommandFrame(&cmd);
    return sendSPIFrame((uint8_t*)&cmd, sizeof(SPICommandFrame_t));
}


void SPIMaster::sendHeartbeat() {
    if (!initialized) return;
    
    // Reenviar último comando con tipo HEARTBEAT
    SPICommandFrame_t hb_frame;
    memcpy(&hb_frame.payload, &last_drive_payload, sizeof(ControlPayload_t));
    hb_frame.payload.type = CMD_HEARTBEAT;
    prepareCommandFrame(&hb_frame);
    sendSPIFrame((uint8_t*)&hb_frame, sizeof(SPICommandFrame_t));
}

bool SPIMaster::getLastResponse(SensorPayload_t* payload) {
    SensorPayload_t result = {};
    if (xSemaphoreTake(swap_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        if (rx_safe) result = rx_safe->payload;
        xSemaphoreGive(swap_mutex);
    }
    if (payload) *payload = result;
    return true;
}
/*
SensorPayload_t SPIMaster::getLastResponse(SensorPayload_t* payload) {
    SensorPayload_t result = {};
    if (xSemaphoreTake(swap_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        if (rx_safe) result = rx_safe->payload;
        xSemaphoreGive(swap_mutex);
    }
    if (payload) *payload = result;
    return result;
}*/