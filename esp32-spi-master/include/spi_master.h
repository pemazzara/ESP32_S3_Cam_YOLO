// spi_master.h
#ifndef SPI_MASTER_H
#define SPI_MASTER_H

#include <Arduino.h>
#include <driver/spi_master.h>
#include <freertos/semphr.h>
#include "SPIDataProtocol.h"

/* Pines SPI (ajusta según tu hardware)
#define SPI_MASTER_CLK  21   
#define SPI_MASTER_MISO 47     
#define SPI_MASTER_MOSI 41   
#define SPI_MASTER_SS   42
*/
class SPIMaster {                                                                                                                                                                                                                                                                                                                                                                                                                        
private:
    // Configuración óptima para ESP32-S3
    static constexpr int SPI_MASTER_CLK  = 21; // -> 39 Master // 18;
    static constexpr int SPI_MASTER_MISO = 47; // -> 40 Master // 19;
    static constexpr int SPI_MASTER_MOSI = 41; // -> 41 Master // 23;   
    static constexpr int SPI_MASTER_SS   = 42; // -> 42 Master // 5;
    spi_device_handle_t spi;
    bool initialized;
    int errorCounter = 0;
    
    // ✅ Buffers alineados a 32 bytes para DMA
    SPICommandFrame_t* tx_buffer_dma;   // Asignado con heap_caps_aligned_alloc
    SPIResponseFrame_t* rx_dma;  // Asignado con heap_caps_aligned_alloc
    SPIResponseFrame_t* rx_safe;
    // Buffers de intercambio (también alineados)
    //SPIResponseFrame_t* rx_stable;
    //SPIResponseFrame_t* rx_active;

    SPIResponseFrame_t* rx_buffer_a;
    SPIResponseFrame_t* rx_buffer_b;

    // Mutex para thread-safety
    SemaphoreHandle_t spi_mutex;
    SemaphoreHandle_t swap_mutex;
    
    // Último comando enviado (para reenviar si es necesario)
    ControlPayload_t last_drive_payload;
    
    // Métodos privados
    void swapBuffers();
    bool sendSPIFrame(uint8_t* data, size_t len);
    bool receiveSPIFrame(uint8_t* data, size_t len);
    void handleCommunicationErrors(uint8_t magic);
    
public:
    SPIMaster();
    ~SPIMaster();
    
    bool begin();
    bool reconnect();
    bool testSPI();
    
    // Envío de comandos
    void sendDriveCommand(int speed, float angle_deg, bool tracking);
    bool sendResetEmergency();
    bool sendResetOdometry();
    bool sendStopCommand();
    void sendHeartbeat();
    
    // Recepción de datos
    void receiveSensorData();
    bool getLastResponse(SensorPayload_t* payload = nullptr);
    
    // Consultas de estado
    bool isInitialized() const { return initialized; }
    bool isSlaveInEmergency();
    bool isSlaveMoving();
    int getErrorCount() const { return errorCounter; }
};

#endif // SPI_MASTER_H
/*
#ifndef SPIMASTER_H
#define SPIMASTER_H

#include <Arduino.h>
#include <driver/spi_master.h>
#include <stdint.h>
#include "SPIDataProtocol.h"
#include "esp_task_wdt.h"

// Pines HSPI ESP32-S3
// Configuración óptima para ESP32-S3
#define SPI_MASTER_CLK  39   
#define SPI_MASTER_MISO 40     
#define SPI_MASTER_MOSI 41   
#define SPI_MASTER_SS   42

extern bool autonomousMode; // "Aviso" al compilador de que la variable vive en otro lugar

#define DISTANCIA_CRITICA_STOP 150 // 15cm o 150mm - Detener inmediatamente
class SPIMaster {
public:
    SPIMaster();
    
    bool begin();
    void sendDriveCommand(float speed, float angle_deg, bool tracking);
    void receiveSensorData();
    int errorCounter = 0;
    // Getters para estado del Slave
    SensorPayload_t getLastResponse(SensorPayload_t* payload);
    bool isSlaveInEmergency();
    bool isSlaveMoving();
    void getCleanSensorData();
    bool requestSensorData();
    // Utilidades
    void testCommunication();
    bool testSPI();
    
private:
    SemaphoreHandle_t xSpiMasterMutex; // Mutex para proteger acceso a targets y comunicación
    // Targets
    float base_angle = 90.0f;
    float target_pwm = 0.0f;
    uint16_t x_centroid = 0;
    uint16_t y_centroid = 0;
    ControlPayload_t last_drive_payload; // Para recordar qué enviamos
    bool initialized;
    spi_device_handle_t spi;
    spi_transaction_t t;
    // Buffers para comunicación
    SPICommandFrame_t master_tx_buffer;
    SPIResponseFrame_t master_rx_buffer;
    bool sendSPIFrame(uint8_t* data, size_t len);
    bool receiveSPIFrame(uint8_t* data, size_t len);
    void handleCommunicationErrors(uint8_t magic);
    bool reconnect();
    
    
};


#endif // SPIMASTER_H
*/