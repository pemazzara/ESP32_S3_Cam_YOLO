#include "sensor_control.h"

struct SensorInstancia {
    VL53L0X device;
    bool online = false; // <-- Importante
    int xshutPin;
    uint8_t address;
};
SensorInstancia sensores[3];

void SensorControl::begin() {
  Serial.println("Inicializando sensores VL53L0X con multiplexación XSHUT...");
  // Inicializar bus I2C para sensores VL53L0X
  i2cBus = &Wire;
  i2cBus->begin(I2C_TOF_SDA, I2C_TOF_SCL, 400000); // 400kHz para TOF

  // Configurar pines XSHUT
  pinMode(FRONT_XSHUT_PIN, OUTPUT);
  pinMode(LEFT_XSHUT_PIN, OUTPUT);
  pinMode(RIGHT_XSHUT_PIN, OUTPUT);
  
  // Apagar todos inicialmente
  disableAllSensors();
  Serial.println("Inicializando sensor FRONTAL...");
  digitalWrite(FRONT_XSHUT_PIN, HIGH); // Solo el frontal encendido
  delay(100); // Esperar a que estabilice
  sensores[0].device.setBus(i2cBus);
  if (!sensores[0].device.init()) {
    Serial.println("❌ Error al iniciar sensor FRONTAL con dirección 0x29");
    return;
  }
  sensores[0].device.setBus(i2cBus);
  if (!sensores[0].device.init()) {
    Serial.println("❌ Error al iniciar sensor FRONTAL con dirección 0x29");
    return;
  }
  sensores[0].device.setAddress(FRONT_ADDRESS); // Cambiar a 0x20
  sensores[0].device.startContinuous();
  sensores[0].online = true; // Marcar como online
  Serial.println("✅ Sensor FRONTAL listo en dirección 0x20");

  // --- Paso 3: Inicializar y asignar dirección al sensor IZQUIERDO ---
  Serial.println("Inicializando sensor IZQUIERDO...");
  digitalWrite(LEFT_XSHUT_PIN, HIGH); // Frontal e izquierdo encendidos
  delay(100);
  sensores[1].device.setBus(i2cBus);
  if (!sensores[1].device.init()) {
    Serial.println("❌ Error al iniciar sensor IZQUIERDO con dirección 0x29");
    return;
  }
  sensores[1].device.setAddress(LEFT_ADDRESS); // Cambiar a 0x21
  sensores[1].device.startContinuous();
  sensores[1].online = true; // Marcar como online
  Serial.println("✅ Sensor IZQUIERDO listo en dirección 0x21");

  // --- Paso 4: Inicializar y asignar dirección al sensor DERECHO ---
  Serial.println("Inicializando sensor DERECHO...");
  digitalWrite(RIGHT_XSHUT_PIN, HIGH); // Todos encendidos ahora
  delay(100);
  sensores[2].device.setBus(i2cBus);
  if (!sensores[2].device.init()) {
    Serial.println("❌ Error al iniciar sensor DERECHO con dirección 0x29");
    return;
  }
  sensores[2].device.setAddress(RIGHT_ADDRESS); // Cambiar a 0x32
  sensores[2].device.startContinuous();
  sensores[2].online = true; // Marcar como online
  Serial.println("✅ Sensor DERECHO listo en dirección 0x32");

  Serial.println("✅ Todos los sensores inicializados y con direcciones únicas.");
  Serial.println("Sensores listos para multiplexación XSHUT");
}

void SensorControl::disableAllSensors() {
  digitalWrite(FRONT_XSHUT_PIN, LOW);
  digitalWrite(LEFT_XSHUT_PIN, LOW);
  digitalWrite(RIGHT_XSHUT_PIN, LOW);
  delay(10);
}
/*
void SensorControl::enableSensor(uint8_t sensorIndex) {
  disableAllSensors();
  
  switch(sensorIndex) {
    case SENSOR_FRONT: // Frontal
      digitalWrite(FRONT_XSHUT_PIN, HIGH);
      break;
    case SENSOR_LEFT: // Izquierdo
      digitalWrite(LEFT_XSHUT_PIN, HIGH);
      break;
    case SENSOR_RIGHT: // Derecho
      digitalWrite(RIGHT_XSHUT_PIN, HIGH);
      break;
  }
  
  delay(100); // Esperar a que el sensor se active

  // Re-inicializar el sensor con dirección por defecto
  //sensor.setBus(i2cBus);
  
  if (!sensor.init()) {
    Serial.printf("❌ Error iniciando sensor %d\n", sensorIndex);
    return;
  }
  
  if(sensorIndex == SENSOR_FRONT) {
      sensor.setAddress(FRONT_ADDRESS);
  } else if(sensorIndex == SENSOR_LEFT) {
      sensor.setAddress(LEFT_ADDRESS);
  } else if(sensorIndex == SENSOR_RIGHT) {
      sensor.setAddress(RIGHT_ADDRESS);
  }
  sensor.setAddress(DEFAULT_ADDRESS); 
  sensor.setTimeout(500);
  sensor.startContinuous();
}*/
/*
uint16_t SensorControl::readSensor(uint8_t sensorIndex) {
  
  uint16_t distance = 0;
  switch(sensorIndex) {
    case SENSOR_FRONT:
      distance = sensorFront.readRangeContinuousMillimeters();
      break;
    case SENSOR_LEFT:
      distance = sensorLeft.readRangeContinuousMillimeters();
      break;
    case SENSOR_RIGHT:
      distance = sensorRight.readRangeContinuousMillimeters();
      break;
  }
  
  if (sensorFront.timeoutOccurred() && sensorIndex == SENSOR_FRONT) {
    distance = 1200;
  } else if (sensorLeft.timeoutOccurred() && sensorIndex == SENSOR_LEFT) {
    distance = 1200;
  } else if (sensorRight.timeoutOccurred() && sensorIndex == SENSOR_RIGHT) {
    distance = 1200;
  } else if (distance > 1200) {
    distance = 1200;
  }
  
  switch(sensorIndex) {
    case 0: frontDistance = distance; break;
    case 1: leftDistance = distance; break;
    case 2: rightDistance = distance; break;
  }
  return distance;
}
*/

void SensorControl::readAll() {
  uint16_t distancia[NUM_SENSORES];
  for (int i = 0; i < NUM_SENSORES; i++) {
    if (sensores[i].online) {
      distancia[i] = sensores[i].device.readRangeContinuousMillimeters();
    }else{
      distancia[i] = 1200; // Valor máximo si el sensor no está online
    }
  }

    frontDistance = distancia[0];
    leftDistance = distancia[1];
    rightDistance = distancia[2];

}
void SensorControl::printDistances() {
  Serial.printf("📍 Sensores - F: %dmm, L: %dmm, R: %dmm\n", 
                frontDistance, leftDistance, rightDistance);
}
/*
void SensorControl::diagnoseSensors() {
  Serial.println("\n=== DIAGNÓSTICO SENSORES VL53L0X ===");
  
  for (int i = 0; i < 3; i++) {
    Serial.printf("\nProbando sensor %d...\n", i);
    enableSensor(i);
    
    // Test de comunicación I2C
    i2cBus->beginTransmission(DEFAULT_ADDRESS);
    byte error = i2cBus->endTransmission();
    
    if (error == 0) {
      Serial.printf("✅ Sensor %d responde en I2C\n", i);
      
      // Test de lectura
      uint16_t testDist = sensor.readRangeSingleMillimeters();
      if (!sensor.timeoutOccurred() && testDist < 1200) {
        Serial.printf("✅ Sensor %d lectura OK: %dmm\n", i, testDist);
      } else {
        Serial.printf("❌ Sensor %d error en lectura\n", i);
      }
    } else {
      Serial.printf("❌ Sensor %d sin respuesta I2C (error: %d)\n", i, error);
    }
    
    delay(100);
  }
  Serial.println("====================================\n");
}

*/
