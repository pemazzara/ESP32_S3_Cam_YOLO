#ifndef SLAVE_ROBOT_CONTROL_H
#define SLAVE_ROBOT_CONTROL_H

#include <atomic>
#include <cstdint>

extern std::atomic<float> targetAngle;
extern std::atomic<int> targetSpeed;
extern std::atomic<uint16_t> targetXCentroid;
extern std::atomic<uint16_t> targetYCentroid;
extern std::atomic<uint32_t> lastValidCommandTime;
// Contadores de pulsos ISR
extern std::atomic<uint32_t> pulsesLeft;
extern std::atomic<uint32_t> pulsesRight;

extern std::atomic<bool> emergencyStop;      // ← AGREGAR
extern std::atomic<bool> failsafe;           // ← AGREGAR (opcional)


#endif