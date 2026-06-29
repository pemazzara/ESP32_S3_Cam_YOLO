// MappingTypes.h - Nuevo archivo para el esclavo
#pragma once
#include <stdint.h>

#define MAP_SIZE_X 50    // 50 celdas
#define MAP_SIZE_Y 50
#define CELL_SIZE_MM 100 // Cada celda = 10cm → mapa de 5m x 5m
#define MAX_OBSTACLES 10

// Punto en el mapa global
struct MapPoint {
    float x_m;      // metros
    float y_m;
    uint8_t confidence; // 0-255
};

// Mapa de ocupación compacto (1 byte por celda)
struct OccupancyGrid {
    uint8_t grid[MAP_SIZE_X][MAP_SIZE_Y]; // 0=libre, 255=ocupado, 127=desconocido
    float origin_x_m;  // Esquina inferior izquierda del mapa
    float origin_y_m;
    uint32_t timestamp_ms;
};

// Paquete de obstáculos detectados para enviar por SPI
struct ObstacleData {
    uint8_t count;  // Número de obstáculos
    struct {
        uint16_t distance_mm;   // Distancia
        int16_t angle_deg;      // Ángulo respecto al robot (0=frente, 90=izq, -90=der)
        uint8_t sensor_id;      // 0=frente, 1=izq, 2=der
    } obstacles[MAX_OBSTACLES];
};

// Datos de odometría y estado
struct RobotState {
    float x_m;
    float y_m;
    float theta_rad;
    float linear_vel_m_s;
    float angular_vel_rad_s;
    uint32_t timestamp_ms;
};