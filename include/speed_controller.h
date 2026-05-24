#ifndef SPEED_CONTROLLER_H
#define SPEED_CONTROLLER_H

/* 
* Esta implementación sigue un patrón de diseño profesional 
* (separación de responsabilidades entre sensado, control y actuadores). 
* Estamos usando un controlador de velocidad incremental, 
* lo cual es excelente para evitar saltos bruscos en los motores.
*/

#include <Arduino.h>

#define CONTROL_PERIOD_MS 20
#define CONTROL_PERIOD_S (CONTROL_PERIOD_MS / 1000.0f)
#define MAX_VELOCITY_MM_S 50

class SpeedController {
private:
    // Nuevos miembros para seguimiento visual
    uint16_t targetXCentroid = 0;
    uint16_t targetYCentroid = 0;
    uint16_t referenceXCentroid = 160;  // Centro de la imagen (ajustar)
    uint16_t referenceYCentroid = 120;
    
    // PID para seguimiento angular
    float angleCorrection = 0;
    float angleError = 0;
    float angleIntegral = 0;
    float lastAngleError = 0;
    float Kp_angle = 0.5f;   // Ganancia proporcional para ángulo
    float Ki_angle = 0.05f;  // Ganancia integral
    float Kd_angle = 0.02f;  // Ganancia derivativa
    
    // Control de velocidad por distancia
    float speedGain = 0.3f;  // Ganancia para velocidad basada en distancia


    float target_avel = 0;      // Velocidad adimensional deseada
    float new_target = 0;       // Para detectar cambios bruscos
    float avel_current = 0;     // Velocidad actual medida
    float error_integral = 0;
    float last_error = 0;
   
    // Constantes del PID (ajustar experimentalmente)
    float Kp = 1.5f;    // Ganancia proporcional
    float Ki = 0.3f;    // Ganancia integral
    float Kd = 0.0f;    // Derivativo desactivado (no se usa sin encoders)
    float K_ff = 1.0f;  // Feedforward 1:1 (target_avel ya es PWM)
 
    float K = 0.3; // Ganancia feedforward (ajustar // Valor de calibración (ajustar según necesario)
    float tau = 0.5; // Constante de tiempo (ajustar // Valor
    int base_angle = 90;        // Ángulo deseado para ir recto (90 = adelante, <90 = izquierda, >90 = derecha)
    int current_pwm = 300;   // Valor inicial de PWM (ajustar según tu hardware)
    void loadCalibration(float K, float tau);
    
public:
    void begin();
    float getCurrentAvel();
    int16_t getCurrentPWM();
    float getLastError();
    // Llamada desde la tarea de control del Slave (ej. 50Hz)
    void updateControl();
    void setGains(float kp, float ki) { Kp = kp; Ki = ki; }
    // Llamada cuando llega un comando SPI del Master
    void setTarget(float angle, float pwm, uint16_t xCent = 0, uint16_t yCent = 0);
    void setReferencePoint(uint16_t x, uint16_t y);  // Configurar centro de referencia
    //void setTarget(int angle, float target_Sppeed_mm_s);
    void setCalibration(float K, float tau); 
    void setFeedforwardGain(float k) {
        K_ff = k;
    }
    
};
    //int pwmFromAvel(float target_avel);
    //uint8_t getStatus();

#endif