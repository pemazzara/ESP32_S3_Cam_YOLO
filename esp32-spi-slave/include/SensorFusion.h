// SensorFusion.h
#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

#include "MappingTypes.h"
#include "OdometryProcessor.h"
#include <atomic>
#include <math.h>
#include <freertos/semphr.h>

class SensorFusion {
private:
    OccupancyGrid localMap;
    OdometryProcessor* odom;
    
    // Distancias actuales de los 3 TOF
    std::atomic<uint16_t> tof_front{0};
    std::atomic<uint16_t> tof_left{0};
    std::atomic<uint16_t> tof_right{0};
    std::atomic<bool> new_tof_data{false};
    
    SemaphoreHandle_t mapMutex;
    
    // ✅ SOLUCIÓN: inline constexpr evita múltiples definiciones
static const float* getTOFAngles() {
        static const float angles[3] = {
            0.0f,           // Frente
            M_PI / 2.0f,    // Izquierda (90°)
            -M_PI / 2.0f    // Derecha (-90°)
        };
        return angles;
    }
    
    // Convertir coordenadas de mundo a índice del mapa
    void worldToGrid(float x_m, float y_m, int& gx, int& gy) {
        gx = (int)((x_m - localMap.origin_x_m) / (CELL_SIZE_MM / 1000.0f));
        gy = (int)((y_m - localMap.origin_y_m) / (CELL_SIZE_MM / 1000.0f));
    }
    
    // Algoritmo de Bresenham para marcar celdas libres
    void markFreeLine(int x0, int y0, int x1, int y1) {
        int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
        int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
        int err = dx + dy, e2;
        
        while (true) {
            if (x0 >= 0 && x0 < MAP_SIZE_X && y0 >= 0 && y0 < MAP_SIZE_Y) {
                if (localMap.grid[x0][y0] < 128) {
                    localMap.grid[x0][y0] = max(0, (int)localMap.grid[x0][y0] - 5);
                }
            }
            if (x0 == x1 && y0 == y1) break;
            e2 = 2 * err;
            if (e2 >= dy) { err += dy; x0 += sx; }
            if (e2 <= dx) { err += dx; y0 += sy; }
        }
    }
    
public:
    void init(OdometryProcessor* odom_ptr) {
        odom = odom_ptr;
        mapMutex = xSemaphoreCreateMutex();
        
        // Inicializar mapa como desconocido
        memset(localMap.grid, 127, sizeof(localMap.grid));
        localMap.origin_x_m = -1.25f;
        localMap.origin_y_m = -1.25f;
    }
    
    void updateTOF(uint8_t sensor_id, uint16_t distance_mm) {
        switch(sensor_id) {
            case 0: tof_front.store(distance_mm, std::memory_order_release); break;
            case 1: tof_left.store(distance_mm, std::memory_order_release); break;
            case 2: tof_right.store(distance_mm, std::memory_order_release); break;
        }
        new_tof_data.store(true, std::memory_order_release);
    }
    
    void processFusion() {
        if (!new_tof_data.load(std::memory_order_acquire)) return;
        new_tof_data.store(false);
        
        RobotState state = odom->getState();
        uint16_t distances[3] = {
            tof_front.load(std::memory_order_acquire),
            tof_left.load(std::memory_order_acquire),
            tof_right.load(std::memory_order_acquire)
        };
        
        xSemaphoreTake(mapMutex, portMAX_DELAY);
        
        int robotGX, robotGY;
        worldToGrid(state.x_m, state.y_m, robotGX, robotGY);
        
        for (int i = 0; i < 3; i++) {
            if (distances[i] == 0 || distances[i] > 4000) continue;
            
            float global_angle = state.theta_rad + getTOFAngles()[i];
            float obs_x = state.x_m + (distances[i] / 1000.0f) * cosf(global_angle);
            float obs_y = state.y_m + (distances[i] / 1000.0f) * sinf(global_angle);
            
            int obsGX, obsGY;
            worldToGrid(obs_x, obs_y, obsGX, obsGY);
            
            if (robotGX >= 0 && robotGX < MAP_SIZE_X && 
                robotGY >= 0 && robotGY < MAP_SIZE_Y &&
                obsGX >= 0 && obsGX < MAP_SIZE_X && 
                obsGY >= 0 && obsGY < MAP_SIZE_Y) {
                
                markFreeLine(robotGX, robotGY, obsGX, obsGY);
                localMap.grid[obsGX][obsGY] = min(255, (int)localMap.grid[obsGX][obsGY] + 50);
            }
        }
        
        localMap.timestamp_ms = millis();
        xSemaphoreGive(mapMutex);
    }
    
    ObstacleData getObstacles() {
        ObstacleData data;
        data.count = 0;
        
        RobotState state = odom->getState();
        uint16_t distances[3] = {
            tof_front.load(), tof_left.load(), tof_right.load()
        };
        
        for (int i = 0; i < 3 && data.count < MAX_OBSTACLES; i++) {
            if (distances[i] > 0 && distances[i] <= 4000) {
                data.obstacles[data.count].distance_mm = distances[i];
                data.obstacles[data.count].angle_deg = (int16_t)(getTOFAngles()[i] * 180.0f / M_PI);
                data.obstacles[data.count].sensor_id = i;
                data.count++;
            }
        }
        
        return data;
    }
    // ✅ Getters para valores TOF (lectura atómica segura)
    uint16_t getTOFFront() const { return tof_front.load(std::memory_order_acquire); }
    uint16_t getTOFLeft()  const { return tof_left.load(std::memory_order_acquire); }
    uint16_t getTOFRight() const { return tof_right.load(std::memory_order_acquire); }
    
    // También puedes añadir un getter para los 3 a la vez
    void getTOFAll(uint16_t& front, uint16_t& left, uint16_t& right) const {
        front = tof_front.load(std::memory_order_acquire);
        left  = tof_left.load(std::memory_order_acquire);
        right = tof_right.load(std::memory_order_acquire);
    }
    
    void getMap(OccupancyGrid& map_out) {
        xSemaphoreTake(mapMutex, portMAX_DELAY);
        memcpy(&map_out, &localMap, sizeof(OccupancyGrid));
        xSemaphoreGive(mapMutex);
    }
};

// ❌ ELIMINAR esta línea que causaba el error:
// constexpr float SensorFusion::TOF_ANGLES[3];

#endif // SENSOR_FUSION_H