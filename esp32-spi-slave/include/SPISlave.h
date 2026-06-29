// SPISlave.h
#ifndef SPI_SLAVE_H
#define SPI_SLAVE_H


#include <driver/spi_slave.h>
#include <freertos/semphr.h>
#include "SPIDataProtocol.h"
#include "SensorFusion.h"


class SPISlave {
private:
    // Hardware SPI
    static constexpr spi_host_device_t spi_host = SPI2_HOST;
    
    // Pines HSPI ESP32-S3
    // Configuración óptima para ESP32-S3
    static constexpr int SPI_SLAVE_CLK  = 39; // -> 39 Master 
    static constexpr int SPI_SLAVE_MISO = 40; // -> 40 Master
    static constexpr int SPI_SLAVE_MOSI = 41; // -> 41 Master   
    static constexpr int SPI_SLAVE_SS   = 42; // -> 42 Master


    
    // Buffers DMA (alineados a 4 bytes)
    static SPICommandFrame_t* spi_rx_buffer;
    static SPIResponseFrame_t* spi_tx_buffer;
    
    // Control de transacciones
    static spi_slave_transaction_t transaction;
    static SemaphoreHandle_t buffer_mutex;
    static SemaphoreHandle_t cmd_ready_sem;
    
    // Estado
    static bool initialized;
    static ControlPayload_t last_command;
    
    // Referencias externas (inyectadas en init)
    static SensorFusion* fusion;
    static OdometryProcessor* odom;
    
    // Métodos privados
    static void prepareSensorResponse(SPIResponseFrame_t& txFrame);
    static void queueNextTransaction();
    static void processReceivedCommand(const SPICommandFrame_t& rx_frame);
    
public:
    // Inicialización
    static bool init(SensorFusion* fusion_ptr, OdometryProcessor* odom_ptr);
    
    // Procesamiento principal (llamar en loop/tarea)
    static void processSPICommunication();
    // Actualizar datos de estado para enviar al maestro
    static void updateState(const RobotState& state, const ObstacleData& obstacles);

    static bool isCommandReady();
    static ControlPayload_t getLastCommand();
    
    // Control de emergencia
    static void setEmergencyStop(bool stop);
    static bool isEmergencyStop();
};

#endif

/*
#ifndef SPI_SLAVE_H
#define SPI_SLAVE_H

#include <Arduino.h>
#include <driver/spi_slave.h> // ⚠️ Driver nativo de ESP-IDF
#include "SPIDefinitions.h"
#include "MotorControl.h"
#include "speed_controller.h"
#include "sonar_integration.h"
#include "freertos/semphr.h"


    // Calcular CRC simple
    void computeCRC() {
        crc16 = 0;
        uint8_t* data = (uint8_t*)this;
        for (size_t i = 0; i < sizeof(SlaveToMasterData) - 2; i++) {
            crc16 ^= data[i] << 8;
            for (int j = 0; j < 8; j++) {
                if (crc16 & 0x8000) crc16 = (crc16 << 1) ^ 0x1021;
                else crc16 <<= 1;
            }
        }
    }

class SPISlave {
private:
    MotorControl* motor_controller;
    SpeedController* pSpeedCtrl;
    // Handle para el driver
    spi_host_device_t spi_host;
    spi_slave_transaction_t transaction; // Transacción persistente
    bool initialized = false;

    // Buffer de recepción (alineado para DMA)

    static SPIResponseFrame_t* spi_tx_buffer; 
    static SPIFrame_t* spi_rx_buffer;
        // Sincronización entre tasks
    //static SemaphoreHandle_t response_mutex;
    // static SemaphoreHandle_t cmd_mutex;
    static SemaphoreHandle_t cmd_ready_sem;
    static SemaphoreHandle_t buffer_mutex;

    
     // Variables de estado
    static ControlCommand_t last_command;
    ResponseType pending_response_type;
    uint8_t last_rx_msg_id;
    
    //static SpeedController* speedCtrl;
    static SPIResponseFrame_t last_response;
    // Pone los datos en el buffer DMA y le dice al hardware "Listos para recibir"
    void queueNextTransaction();
    //void processReceivedCommand(const SPIFrame_t& rxFrame);
    void prepareResponse(SPIResponseFrame_t &txFrame);
    uint8_t getNextMsgId();

    //spi_slave_transaction_t transactions[3]; // Una por cada slot de la cola
    //int transaction_index = 0;
    // Buffers para DMA (deben ser accesibles por el hardware)
    // Se definen en el .cpp para asegurar alineación, aquí usamos punteros
    //SPIFrame_t *rxBuffer;
    //SPIResponseFrame_t *txBuffer;
    

public:
    SPISlave(MotorControl &motor); // El constructor sigue recibiendo refs
    bool init();
    void processSPICommunication();
    void processCommand(const SPICommandFrame_t& cmd);
    void prepareResponse(const SPIResponseFrame_t& txFrame);
    // Métodos estáticos para acceso desde tasks
    static ControlPayload_t getLastCommand();
    static void commandProcessed();
    static SPICommandFrame_t getReceivedFrame();
    static bool isCommandReady();
    static void signalDataProcessed();

    
    //static void setMotorController(MotorControl* mc) { motor_controller = mc; }   
    //static void setSensorManager(SensorManager* sm) { sensor_manager = sm; }
    bool getLatestSensorData(SonarSensorData_t &outData);

    // Para debugging
    static SPIResponseFrame_t getLastResponse() { return last_response; }

    void checkHealth();
    void printStats();
    void validateBuffers();

};

// Task de SPI Slave
//void spiSlaveTask(void *pvParameters);

#endif
*/