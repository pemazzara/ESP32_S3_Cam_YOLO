// OdometryProcessor.h
#ifndef ODOMETRY_PROCESSOR_H
#define ODOMETRY_PROCESSOR_H

#include "MappingTypes.h"
#include <atomic>
#include <math.h>

class OdometryProcessor {
private:
    // Parámetros físicos del robot (ajusta según tu robot)
    static constexpr float WHEEL_RADIUS_M = 0.0325;  // 32.5mm radio rueda
    static constexpr float WHEEL_BASE_M = 0.15;      // 150mm entre ruedas
    static constexpr float PULSES_PER_REV = 20.0;    // Pulsos por revolución del encoder
    static constexpr float DIST_PER_PULSE_M = (2.0 * M_PI * WHEEL_RADIUS_M) / PULSES_PER_REV;
    
    // Estado actual
    float x_m = 0.0f;
    float y_m = 0.0f;
    float theta_rad = 0.0f;
    float linear_vel = 0.0f;
    float angular_vel = 0.0f;
    
    SemaphoreHandle_t mutex;
    
public:
    void init() {
        mutex = xSemaphoreCreateMutex();
    }
    
    // Llamar cada 20ms con los pulsos acumulados
    void update(uint32_t pulsesLeft, uint32_t pulsesRight) {
        float dL = pulsesLeft * DIST_PER_PULSE_M;
        float dR = pulsesRight * DIST_PER_PULSE_M;
        float dCenter = (dL + dR) / 2.0f;
        float dTheta = (dR - dL) / WHEEL_BASE_M;
        
        xSemaphoreTake(mutex, portMAX_DELAY);
        
        // Actualizar posición
        x_m += dCenter * cosf(theta_rad);
        y_m += dCenter * sinf(theta_rad);
        theta_rad += dTheta;
        
        // Normalizar theta a [-π, π]
        theta_rad = atan2f(sinf(theta_rad), cosf(theta_rad));
        
        // Velocidades (para 20ms)
        linear_vel = dCenter / 0.02f;
        angular_vel = dTheta / 0.02f;
        
        xSemaphoreGive(mutex);
    }
    
    RobotState getState() {
        RobotState state;
        xSemaphoreTake(mutex, portMAX_DELAY);
        state.x_m = x_m;
        state.y_m = y_m;
        state.theta_rad = theta_rad;
        state.linear_vel_m_s = linear_vel;
        state.angular_vel_rad_s = angular_vel;
        state.timestamp_ms = millis();
        xSemaphoreGive(mutex);
        return state;
    }
    
    void resetPosition(float x = 0, float y = 0, float theta = 0) {
        xSemaphoreTake(mutex, portMAX_DELAY);
        x_m = x;
        y_m = y;
        theta_rad = theta;
        xSemaphoreGive(mutex);
    }
};

#endif // ODOMETRY_PROCESSOR_H