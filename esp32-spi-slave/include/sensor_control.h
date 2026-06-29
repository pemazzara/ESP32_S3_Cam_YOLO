#ifndef SENSOR_CONTROL_H
#define SENSOR_CONTROL_H

#include <Arduino.h>
#include <VL53L0X.h>  // Usar la biblioteca VL53L0X de Pololu

#define NUM_SENSORES 3
// Pines XSHUT
#define FRONT_XSHUT_PIN  4 
#define LEFT_XSHUT_PIN   5
#define RIGHT_XSHUT_PIN  6 

#define SENSOR_FRONT 0
#define SENSOR_LEFT  1 
#define SENSOR_RIGHT 2

// Pines I2C y sensores VL53L0X
#define I2C_TOF_SDA 1
#define I2C_TOF_SCL 2

// Dirección por defecto (todos los sensores usan la misma)
#define DEFAULT_ADDRESS  0x29
#define I2C_PORT I2C_NUM_0
#define FRONT_ADDRESS    0x20
#define LEFT_ADDRESS     0x21  
#define RIGHT_ADDRESS    0x22

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
