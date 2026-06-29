// SPISlave.cpp
#include "SPISlave.h"
#include "slave_robot_control.h"

// Inicialización de miembros estáticos
SPICommandFrame_t* SPISlave::spi_rx_buffer = nullptr;
SPIResponseFrame_t* SPISlave::spi_tx_buffer = nullptr;
spi_slave_transaction_t SPISlave::transaction = {};
SemaphoreHandle_t SPISlave::buffer_mutex = nullptr;
SemaphoreHandle_t SPISlave::cmd_ready_sem = nullptr;
bool SPISlave::initialized = false;

ControlPayload_t SPISlave::last_command = {0};
SensorFusion* SPISlave::fusion = nullptr;
OdometryProcessor* SPISlave::odom = nullptr;


RobotState robotState;
ObstacleData obstacleData;


bool SPISlave::init(SensorFusion* fusion_ptr, OdometryProcessor* odom_ptr) {
    if (initialized) {
        Serial.println("⚠️ SPI Slave ya inicializado");
        return true;
    }
    
    // Guardar referencias
    fusion = fusion_ptr;
    odom = odom_ptr;
    
    Serial.println("🚀 Inicializando SPI Slave...");
    
    // Crear semáforos
    buffer_mutex = xSemaphoreCreateMutex();
    cmd_ready_sem = xSemaphoreCreateBinary();
    
    if (!buffer_mutex || !cmd_ready_sem) {
        Serial.println("❌ Error creando semáforos SPI");
        return false;
    }
    
    // Asignar buffers DMA (alineados a 4 bytes)
    size_t rx_size = (sizeof(SPICommandFrame_t) + 3) & ~3;  // Redondear a múltiplo de 4
    size_t tx_size = (sizeof(SPIResponseFrame_t) + 3) & ~3;
    Serial.printf("   SPI frame sizes: cmd=%d resp=%d\n", 
                  sizeof(SPICommandFrame_t), sizeof(SPIResponseFrame_t));

    spi_rx_buffer = (SPICommandFrame_t*) heap_caps_aligned_alloc(
        32, rx_size, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    spi_tx_buffer = (SPIResponseFrame_t*) heap_caps_aligned_alloc(
        32, tx_size, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);

    if (!spi_rx_buffer || !spi_tx_buffer) {
        Serial.println("❌ Error asignando buffers SPI");
        return false;
    }
    
    // ✅ Asegurar que el buffer RX es suficientemente grande
    Serial.printf("   SPI buffers: RX=%d bytes TX=%d bytes\n", rx_size, tx_size);
    
    if (!spi_rx_buffer || !spi_tx_buffer) {
        Serial.println("❌ No hay memoria DMA para buffers SPI");
        if (spi_rx_buffer) free(spi_rx_buffer);
        if (spi_tx_buffer) free(spi_tx_buffer);
        return false;
    }
    
    // Limpiar buffers
    memset(spi_rx_buffer, 0, rx_size);
    memset(spi_tx_buffer, 0, tx_size);
    
    // Configurar bus SPI
    spi_bus_config_t buscfg = {
        .mosi_io_num = SPI_SLAVE_MOSI,
        .miso_io_num = SPI_SLAVE_MISO,
        .sclk_io_num = SPI_SLAVE_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = (int)std::max(rx_size, tx_size)  // Cast explícito
    };
    
    spi_slave_interface_config_t slvcfg = {
        .spics_io_num = SPI_SLAVE_SS,
        .flags = 0,
        .queue_size = 3,
        .mode = 0  // SPI_MODE0: CPOL=0, CPHA=0
    };
    
    // Inicializar SPI Slave
    esp_err_t ret = spi_slave_initialize(spi_host, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        Serial.printf("❌ Error inicializando SPI Slave: 0x%X\n", ret);
        free(spi_rx_buffer);
        free(spi_tx_buffer);
        return false;
    }
    
    // Preparar primera respuesta y encolar transacción
    SPIResponseFrame_t initResponse;
    prepareSensorResponse(initResponse);
    memcpy(spi_tx_buffer, &initResponse, sizeof(SPIResponseFrame_t));
    queueNextTransaction();
    
    initialized = true;
    Serial.println("✅ SPI Slave inicializado correctamente");
    return true;
}

void SPISlave::processSPICommunication() {
    if (!initialized) return;
    
    spi_slave_transaction_t* result = nullptr;
    esp_err_t ret = spi_slave_get_trans_result(spi_host, &result, pdMS_TO_TICKS(10));
    
    if (ret == ESP_OK && result != nullptr) {
        // 1. Hacer copia local del comando recibido (evita acceder a DMA después)
        SPICommandFrame_t rx_local;
        memcpy(&rx_local, spi_rx_buffer, sizeof(SPICommandFrame_t));
        
        // 2. Validar y procesar el comando
        if (rx_local.magic == SPI_MAGIC_MASTER) {
            processReceivedCommand(rx_local);
        } else {
            // Si no es comando válido, al menos actualizar heartbeat
            // (podría ser una transacción de solo lectura)
            SPI_DEBUG_PRINT("Transacción SPI sin comando (solo lectura)");
        }
        
        // 3. Preparar la respuesta para la PRÓXIMA transacción
        SPIResponseFrame_t nextResponse;
        prepareSensorResponse(nextResponse);
        
        // 4. Copiar respuesta al buffer DMA (protegido por mutex)
        if (xSemaphoreTake(buffer_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            memcpy(spi_tx_buffer, &nextResponse, sizeof(SPIResponseFrame_t));
            xSemaphoreGive(buffer_mutex);
        }
        
        // 5. Encolar siguiente transacción
        queueNextTransaction();
    }
}

void SPISlave::processReceivedCommand(const SPICommandFrame_t& rx_frame) {
    last_command = rx_frame.payload;
    
    switch (rx_frame.payload.type) {
        case CMD_DRIVE: {
            // CORRECCIÓN: Convertir correctamente
            float angle = rx_frame.payload.angle / 10.0f;  // ángulo * 10 → grados
            float speed = (float)rx_frame.payload.speed;    // Ya es 0-1023
            
            // Actualizar variables atómicas con tipos correctos
            targetAngle.store(angle, std::memory_order_release);          // float
            targetSpeed.store(speed, std::memory_order_release);          // float
            targetXCentroid.store(rx_frame.payload.xCentroid, std::memory_order_release);
            targetYCentroid.store(rx_frame.payload.yCentroid, std::memory_order_release);
            
            // Actualizar timestamp
            lastValidCommandTime.store(millis(), std::memory_order_release);
            
            if (emergencyStop.load(std::memory_order_acquire)) {
                emergencyStop.store(false, std::memory_order_release);
                Serial.println("✅ Emergency stop liberado por comando DRIVE");
            }
            // Señalizar nuevo comando
            xSemaphoreGive(cmd_ready_sem);
            
            SPI_DEBUG_PRINT("🚗 DRIVE: angle=%.1f° speed=%.0f flags=0x%02X", 
                           angle, speed, rx_frame.payload.flags);
            break;
        }
        
        case CMD_STOP:
        case CMD_SET_EMERGENCY: {
            targetSpeed.store(0.0f, std::memory_order_release);          // float 0
            targetAngle.store(90.0f, std::memory_order_release);         // float 90
            lastValidCommandTime.store(0, std::memory_order_release);     // Forzar failsafe
            emergencyStop.store(true, std::memory_order_release);    // ← USAR .store()
            failsafe.store(false, std::memory_order_release);
            xSemaphoreGive(cmd_ready_sem);
            SPI_DEBUG_PRINT("🛑 EMERGENCY STOP");
            break;
        }
        case CMD_RESET_ODOMETRY: {
            if (odom) {
                odom->resetPosition(0, 0, 0);
                SPI_DEBUG_PRINT("📍 Odometría reseteada");
            }
            break;
        }
        
        case CMD_HEARTBEAT: {
            // Solo actualizar timestamp, sin cambiar movimiento
            lastValidCommandTime.store(millis(), std::memory_order_release);
            break;
        }
        
        default:
            SPI_DEBUG_PRINT("⚠️ Comando desconocido: %d", rx_frame.payload.type);
            break;
        
    }
}
/*
void SPISlave::processReceivedCommand(const SPICommandFrame_t& rx_frame) {
    // Copiar comando para uso externo
    last_command = rx_frame.payload;
    
    // Actualizar targets según el tipo de comando
    switch (rx_frame.payload.type) {
        case CMD_DRIVE: {
            // Convertir ángulo (formato *10) a grados float
            float angle = rx_frame.payload.angle / 10.0f;
            float speed = rx_frame.payload.speed;
            
            // Actualizar variables atómicas (las lee motorTask)
            targetAngle.store(angle, std::memory_order_release);
            targetSpeed.store(speed, std::memory_order_release);
            targetXCentroid.store(rx_frame.payload.xCentroid, std::memory_order_release);
            targetYCentroid.store(rx_frame.payload.yCentroid, std::memory_order_release);
            
            // Actualizar timestamp del último comando (evita failsafe)
            lastValidCommandTime.store(millis(), std::memory_order_release);
            //xSemaphoreGive(cmd_ready_sem);
            SPI_DEBUG_PRINT("🚗 DRIVE: angle=%.1f speed=%.0f flags=0x%02X", 
                           angle, speed, rx_frame.payload.flags);
            break;
        }
        
        case CMD_STOP:
        case CMD_SET_EMERGENCY: {
            targetSpeed.store(0, std::memory_order_release);
            targetAngle.store(90.0f, std::memory_order_release);
            lastValidCommandTime.store(0, std::memory_order_release);  // Forzar failsafe
            emergencyStop = true;
            SPI_DEBUG_PRINT("🛑 EMERGENCY STOP");
            break;
        }
        
        case CMD_RESET_EMERGENCY: {
            emergencyStop = false;
            failsafe = false;
            lastValidCommandTime.store(millis(), std::memory_order_release);
            SPI_DEBUG_PRINT("✅ Emergency reset");
            break;
        }
        
        case CMD_RESET_ODOMETRY: {
            if (odom) {
                odom->resetPosition(0, 0, 0);
                SPI_DEBUG_PRINT("📍 Odometría reseteada");
            }
            break;
        }
        
        case CMD_HEARTBEAT: {
            // Solo actualizar timestamp, sin cambiar movimiento
            lastValidCommandTime.store(millis(), std::memory_order_release);
            break;
        }
        
        default:
            SPI_DEBUG_PRINT("⚠️ Comando desconocido: %d", rx_frame.payload.type);
            break;
    }
    
    // Señalar que hay un nuevo comando (para FSM si la usas)
    xSemaphoreGive(cmd_ready_sem);
}*/

void SPISlave::prepareSensorResponse(SPIResponseFrame_t& txFrame) {
    memset(&txFrame, 0, sizeof(SPIResponseFrame_t));
    
    // Obtener estado del robot
    RobotState state = odom ? odom->getState() : RobotState{};
    ObstacleData obstacles = fusion ? fusion->getObstacles() : ObstacleData{};
    
    // Llenar payload de odometría
    txFrame.payload.pos_x_m = state.x_m;
    txFrame.payload.pos_y_m = state.y_m;
    txFrame.payload.theta_rad = state.theta_rad;
    txFrame.payload.linear_vel_m_s = state.linear_vel_m_s;
    txFrame.payload.angular_vel_rad_s = state.angular_vel_rad_s;
    txFrame.payload.odom_timestamp = state.timestamp_ms;
    
    // Obstáculos
    txFrame.payload.obstacle_count = obstacles.count > 3 ? 3 : obstacles.count;
    for (int i = 0; i < txFrame.payload.obstacle_count; i++) {
        txFrame.payload.obstacle_dist[i] = obstacles.obstacles[i].distance_mm;
        txFrame.payload.obstacle_angle[i] = obstacles.obstacles[i].angle_deg;
        txFrame.payload.obstacle_sensor[i] = obstacles.obstacles[i].sensor_id;
    }
    // Rellenar obstáculos restantes con ceros
    for (int i = txFrame.payload.obstacle_count; i < 3; i++) {
        txFrame.payload.obstacle_dist[i] = 0;
        txFrame.payload.obstacle_angle[i] = 0;
        txFrame.payload.obstacle_sensor[i] = 0;
    }
    
    // ✅ Datos crudos de sensores TOF (usando getters)
    if (fusion) {
        txFrame.payload.tof_front_mm = fusion->getTOFFront();
        txFrame.payload.tof_left_mm  = fusion->getTOFLeft();
        txFrame.payload.tof_right_mm = fusion->getTOFRight();
    } else {
        txFrame.payload.tof_front_mm = 0;
        txFrame.payload.tof_left_mm  = 0;
        txFrame.payload.tof_right_mm = 0;
    }
/*
// O - alternativa a lo anterior - Getter múltiple
if (fusion) {
    fusion->getTOFAll(txFrame.payload.tof_front_mm, 
                  txFrame.payload.tof_left_mm, 
                  txFrame.payload.tof_right_mm);
}
*/  
    // Datos de encoders (los tienes en las variables globales)
    txFrame.payload.encoder_left_cnt = pulsesLeft.load(std::memory_order_acquire);
    txFrame.payload.encoder_right_cnt = pulsesRight.load(std::memory_order_acquire);
    
    // Estado del sistema
    txFrame.payload.system_flags = 0;
    if (emergencyStop) txFrame.payload.system_flags |= 0x01;
    if (failsafe)      txFrame.payload.system_flags |= 0x02;
    txFrame.payload.battery_mv = 0;  // Si tienes sensor de batería, actualizar
    txFrame.payload.sensor_timestamp = millis();
    
    // CRC
    prepareResponseFrame(&txFrame);
}


void SPISlave::queueNextTransaction() {
    // Limpiar buffer de recepción
    memset(spi_rx_buffer, 0, sizeof(SPIResponseFrame_t));
    
    // Configurar transacción
    memset(&transaction, 0, sizeof(spi_slave_transaction_t));
    transaction.length = sizeof(SPIResponseFrame_t) * 8;  // En bits
    transaction.rx_buffer = spi_rx_buffer;
    transaction.tx_buffer = spi_tx_buffer;
    
    esp_err_t ret = spi_slave_queue_trans(spi_host, &transaction, portMAX_DELAY);
    if (ret != ESP_OK) {
        Serial.printf("❌ Error encolando transacción SPI: 0x%X\n", ret);
    }
}

bool SPISlave::isCommandReady() {
    if (!cmd_ready_sem) return false;
    return xSemaphoreTake(cmd_ready_sem, 0) == pdTRUE;
}

ControlPayload_t SPISlave::getLastCommand() {
    return last_command;
}

void SPISlave::setEmergencyStop(bool stop) {
    emergencyStop = stop;
    if (stop) {
        targetSpeed.store(0, std::memory_order_release);
    }
}

bool SPISlave::isEmergencyStop() {
    return emergencyStop;
}