#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

#include <atomic>
#include <cstdint>

// Variables globales adicionales
extern std::atomic<uint32_t> lastCommandTime;

// Variables atómicas compartidas entre WebSocket y SPI
extern std::atomic<uint16_t> targetXCentroid;
extern std::atomic<uint16_t> targetYCentroid;
extern std::atomic<uint16_t> targetSpeed;
extern std::atomic<float> targetAngle;
extern std::atomic<uint32_t> lastCommandTime;

// Contadores de pulsos ISR
extern std::atomic<uint32_t> pulsesLeft;
extern std::atomic<uint32_t> pulsesRight;


#endif // ROBOT_CONTROL_H