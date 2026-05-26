#ifndef SENSOR_CONTROL_H
#define SENSOR_CONTROL_H

#include <Arduino.h>
#include <VL53L0X.h>  // Usar la biblioteca VL53L0X de Pololu

#define NUM_SENSORES 3
// Pines XSHUT
#define FRONT_XSHUT_PIN  38 
#define LEFT_XSHUT_PIN   1
#define RIGHT_XSHUT_PIN  40 

#define SENSOR_FRONT 0
#define SENSOR_LEFT  1 
#define SENSOR_RIGHT 2

// Pines I2C y sensores VL53L0X
#define I2C_TOF_SDA 4
#define I2C_TOF_SCL 5

// Dirección por defecto (todos los sensores usan la misma)
#define DEFAULT_ADDRESS  0x29
#define I2C_PORT I2C_NUM_0
#define FRONT_ADDRESS    0x20
#define LEFT_ADDRESS     0x21  
#define RIGHT_ADDRESS    0x22

typedef struct {
    uint16_t sonarDistance;
    uint16_t a_vel;  // velocidad de acercamiento (adimen) positivo si se aleja, negativo si se acerca
    uint16_t tofLeft;
    uint16_t tofFront;  
    uint16_t tofRight;   
    uint32_t lastSonarUpdate;
    uint32_t lastTofUpdate; 
    uint32_t timestamp; // Última actualización de cualquier sensor   
    bool emergency;
    uint8_t sensorStatus;       // Bit 0: Sonar OK, Bit 1: TOFs OK
} SensorData_t;

class SensorControl {
public:
  void begin();
  void readAll();
  uint16_t readSensor(uint8_t sensorIndex); // Nueva: leer sensor individual
  void printDistances();
  void diagnoseSensors();
  
  uint16_t frontDistance;
  uint16_t leftDistance; 
  uint16_t rightDistance;

private:
  void initSensors();
  void enableSensor(uint8_t sensorIndex);
  void disableAllSensors();
   
  
  TwoWire* i2cBus;
};
#endif
