#ifndef ULTRASONIC_MEASURE_H
#define ULTRASONIC_MEASURE_H

#include <Arduino.h>
#include "sensor_control.h"
#include <driver/rmt.h>


// Pines Sonar (HC-SR04)
const uint8_t SONAR_TRIG_PIN = 20;
const uint8_t SONAR_ECHO_PIN = 21;
const uint8_t EMERGENCY_THRESHOLD = 3; // 3 lecturas seguidas
const uint8_t FILTER_SIZE = 5;     // Tamaño del filtro de media móvil
//#define RMT_TX_CHANNEL RMT_CHANNEL_0  // Canal para el Trigger
#define RMT_RX_CHANNEL RMT_CHANNEL_4  // Canal para el Echo
#define RMT_CLK_DIV 80                 // 80MHz / 80 = 1 tick por microse
class UltraSonicMeasure {
public:
    UltraSonicMeasure(uint8_t trigPin = SONAR_TRIG_PIN, 
                       uint8_t echoPin = SONAR_ECHO_PIN);
    
    void begin();
    void sonarUpdate();
    bool isEmergency();
    bool isSensorOK();
    void updateSonarData();
    void update();
    // Getters                // Diagnóstico
    bool getLastSonarData(SensorData_t &data);
    float getApproachSpeed(); // Velocidad de aproximación (cm/s o mm/s)
    bool initialized;

private:
    uint8_t SONAR_TRIG;
    uint8_t SONAR_ECHO;
    
    SemaphoreHandle_t data_mutex;

    uint16_t duration = 0;
    // Para cálculo de velocidad
    uint16_t lastSonarDistance = 0;
    uint32_t lastSonarTime = 0;
    uint32_t lastRawTime = 0;
    uint8_t filterIndex = 0;
    float approachSpeed = 0; // Velocidad de aproximación
    uint16_t distanceBuffer[FILTER_SIZE];
    uint8_t emergency_counter = 0;
};

#endif
