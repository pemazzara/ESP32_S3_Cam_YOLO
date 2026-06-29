// SpeedController.h
/* 
* Esta implementación sigue un patrón de diseño profesional 
* (separación de responsabilidades entre sensado, control y actuadores). 
* Estamos usando un controlador de velocidad incremental, 
* lo cual es excelente para evitar saltos bruscos en los motores.
*/
#ifndef SPEED_CONTROLLER_H
#define SPEED_CONTROLLER_H

#include <Arduino.h>
#include <atomic>

#define ENCODER_PPR 20  // Ajusta según tu encoder

class SpeedController {
private:
    // Mutex para thread safety
    SemaphoreHandle_t xSpeedControllerMutex;
    
    // Targets
    float base_angle = 90.0f;
    float target_pwm = 0.0f;
    uint16_t x_centroid = 0;
    uint16_t y_centroid = 0;
    
    // Variables PID
    float integral_left = 0;
    float integral_right = 0;
    
    // Variables de debug
    int16_t last_pwm_left = 0;
    int16_t last_pwm_right = 0;
    float last_rpm_left = 0;
    float last_rpm_right = 0;
    float last_target_rpm_left = 0;
    float last_target_rpm_right = 0;
    
    void resetIntegral();
    
public:
    void begin();
    void setTarget(float angle, float pwm, uint16_t xCent, uint16_t yCent);
    void updateControl();
    void stop();
    void reset();
    void printDebug();
};

#endif

