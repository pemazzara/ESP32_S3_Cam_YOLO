// SPIDataProtocol.h
#ifndef SPI_DATA_PROTOCOL_H
#define SPI_DATA_PROTOCOL_H

#include <stdint.h>
#include <Arduino.h>
#include "MappingTypes.h"

// ============================================
// CONSTANTES DEL PROTOCOLO SPI
// ============================================
#define SPI_MAGIC_MASTER    0xA5
#define SPI_MAGIC_SLAVE     0x5A
#define SPI_MAX_FRAME_SIZE  128  // Tamaño máximo de trama

// ============================================
// TIPOS DE COMANDOS (Master → Slave)
// ============================================
typedef enum : uint8_t {
    CMD_STOP             = 0x00,  // Parada de emergencia
    CMD_DRIVE            = 0x01,  // Comando de movimiento normal
    CMD_HEARTBEAT        = 0x02,  // Keep-alive
    CMD_READ_SENSORS     = 0x03,  // Solicitar datos de sensores
    CMD_RESET_EMERGENCY  = 0x04,  // Salir de parada de emergencia
    CMD_RECONNECT        = 0x05,  // Reestablecer conexión
    CMD_SET_EMERGENCY    = 0x06,  // Activar parada de emergencia
    CMD_RESET_ODOMETRY   = 0x07,  // Resetear posición (0,0,0)
    CMD_CALIBRATE        = 0x08   // Modo calibración
} ControlCommandType_t;

// ============================================
// PAYLOAD: Master → Slave (comandos de control)
// ============================================
typedef struct __attribute__((packed)) {
    uint8_t  type;          // ControlCommandType_t
    int16_t  speed;         // Velocidad lineal (-1023 a 1023)
    int16_t  angle;         // Ángulo en grados * 10 (ej: 900 = 90.0°)
    uint8_t  xCentroid;     // Centroide X mapeado a 0-255
    uint8_t  yCentroid;     // Centroide Y mapeado a 0-255
    uint8_t  flags;         // Bits de control:
                             //   bit0: tracking activo
                             //   bit1: modo evasión automática
                             //   bit2-7: reservados
} ControlPayload_t;

// ============================================
// PAYLOAD: Slave → Master (datos sensoriales)
// ============================================
typedef struct __attribute__((packed)) {
    // --- Odometría ---
    float    pos_x_m;           // Posición X en metros
    float    pos_y_m;           // Posición Y en metros
    float    theta_rad;         // Orientación en radianes
    float    linear_vel_m_s;    // Velocidad lineal m/s
    float    angular_vel_rad_s; // Velocidad angular rad/s
    uint32_t odom_timestamp;    // Timestamp de odometría
    
    // --- Obstáculos detectados ---
    uint8_t  obstacle_count;    // Número de obstáculos (máx 3)
    uint16_t obstacle_dist[3];  // Distancias en mm (0-4000)
    int16_t  obstacle_angle[3]; // Ángulos en grados (0=frente, 90=izq, -90=der)
    uint8_t  obstacle_sensor[3];// Sensor ID (0=front, 1=left, 2=right)
    
    // --- Datos crudos de sensores ---
    uint16_t tof_front_mm;      // TOF frontal (mm)
    uint16_t tof_left_mm;       // TOF izquierdo (mm)
    uint16_t tof_right_mm;      // TOF derecho (mm)
    uint32_t encoder_left_cnt;  // Contador absoluto encoder izq
    uint32_t encoder_right_cnt; // Contador absoluto encoder der
    
    // --- Estado del sistema ---
    uint8_t  system_flags;      // bit0: emergency_stop, bit1: failsafe, etc.
    uint16_t battery_mv;        // Tensión batería en mV (opcional)
    uint32_t sensor_timestamp;  // Timestamp de esta lectura
} SensorPayload_t;

// ============================================
// TRAMAS SPI COMPLETAS
// ============================================

// Trama Master → Slave
typedef struct __attribute__((packed)) {
    uint8_t          magic;      // 0xA5
    ControlPayload_t payload;    // 8 bytes
    uint16_t         crc16;      // CRC-16
} SPICommandFrame_t;

// Trama Slave → Master
typedef struct __attribute__((packed)) {
    uint8_t         magic;       // 0x5A
    SensorPayload_t payload;     // Datos sensoriales
    uint16_t        crc16;       // CRC-16
} SPIResponseFrame_t;

// ============================================
// FUNCIONES DE UTILIDAD PARA CRC
// ============================================

// CRC-16/CCITT-FALSE
inline uint16_t calculateCRC16(const uint8_t* data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

// Verificar integridad de trama de comando
inline bool isValidCommandFrame(const SPICommandFrame_t* frame) {
    if (frame->magic != SPI_MAGIC_MASTER) return false;
    uint16_t computed = calculateCRC16((const uint8_t*)&frame->payload, 
                                       sizeof(ControlPayload_t));
    return computed == frame->crc16;
}

// Verificar integridad de trama de respuesta
inline bool isValidResponseFrame(const SPIResponseFrame_t* frame) {
    if (frame->magic != SPI_MAGIC_SLAVE) return false;
    uint16_t computed = calculateCRC16((const uint8_t*)&frame->payload, 
                                       sizeof(SensorPayload_t));
    return computed == frame->crc16;
}

// Preparar trama de comando con CRC
inline void prepareCommandFrame(SPICommandFrame_t* frame) {
    frame->magic = SPI_MAGIC_MASTER;
    frame->crc16 = calculateCRC16((const uint8_t*)&frame->payload, 
                                  sizeof(ControlPayload_t));
}

// Preparar trama de respuesta con CRC
inline void prepareResponseFrame(SPIResponseFrame_t* frame) {
    frame->magic = SPI_MAGIC_SLAVE;
    frame->crc16 = calculateCRC16((const uint8_t*)&frame->payload, 
                                  sizeof(SensorPayload_t));
}

// ============================================
// FUNCIONES DE CONVERSIÓN (ayudan en la web)
// ============================================

// Convertir ángulo de grados (float) a formato int16 (*10)
inline int16_t angleToPayload(float degrees) {
    return (int16_t)(degrees * 10.0f);
}

// Convertir de formato int16 a grados float
inline float payloadToAngle(int16_t payload) {
    return payload / 10.0f;
}

// Empaquetar flags
inline uint8_t packFlags(bool tracking, bool autoEvasion) {
    uint8_t flags = 0;
    if (tracking)    flags |= 0x01;
    if (autoEvasion) flags |= 0x02;
    return flags;
}
static_assert(sizeof(SPIResponseFrame_t) == 64, "SPIResponseFrame_t debe ser 64 bytes");
// ============================================
// MACROS PARA DEBUG (opcional)
// ============================================
#ifdef DEBUG_SPI_PROTOCOL
    #define SPI_DEBUG_PRINT(fmt, ...) Serial.printf("[SPI] " fmt "\n", ##__VA_ARGS__)
#else
    #define SPI_DEBUG_PRINT(fmt, ...)
#endif

#endif // SPI_DATA_PROTOCOL_H