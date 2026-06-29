#ifndef SLAVE_FSM_H
#define SLAVE_FSM_H

#include <Arduino.h>
#include "SPIDefinitions.h"   // Para las constantes de estado (SLAVE_IDLE, etc.)
#include "MotorControl.h"
#include "speed_controller.h"
#include "sonar_integration.h"

#define CALIB_PWM 400
#define DISTANCIA_CRITICA_STOP 150 // 15cm o 150mm - Detener inmediatamente
#define DISTANCIA_SEGURA_RESET 300 // 30cm o 300mm - Condición para resetear emergencia
// Estados del Slave
typedef enum : uint8_t {
    SLAVE_STATE_IDLE = 0,
    SLAVE_STATE_CALIBRATION,
    SLAVE_STATE_READY,
    SLAVE_STATE_EMERGENCY
} SlaveStates;

// Estado compartido (protegido por mutex)
typedef struct {
    SlaveStates state;
    ResponseType response_type;   // TYPE_SENSORS o TYPE_SYSTEM
    // Datos de sistema
    uint16_t K_fixed;
    uint16_t tau_fixed;
    uint8_t calibration_progress;
    uint8_t calibration_valid;
    // Datos de sensores (si la FSM los actualiza)
    int16_t rpm_left, rpm_right;
    uint16_t a_vel, distance;
    uint8_t motor_flags;
} SlaveState_t;

// Estructura para parámetros de calibración
struct CalibrationParams {
    float K;          // Ganancia estática
    float tau;        // Constante de tiempo
    bool valid;
};

class SlaveFSM {
public:
    // Singleton (opcional) o instancia única
    static SlaveFSM& getInstance();

    // Inicialización
    void begin();

    // Actualización periódica (llamar cada 20-50ms)
    void update(const ControlCommand_t* cmd);
    void saveCalibrationToEEPROM();
    // Setters
    void setEmergencyFlag();
    void setIdleFlag();
    void setMotorController(MotorControl* controller);
    void setSonar(UltraSonicMeasure* sonar);
    void setSpeedController(SpeedController* speedCtrl);
    
     // Comandos desde el Master (llamar cuando se recibe un comando SPI)
    // Getters
    SlaveStates getCurrentState() const;
    ResponseType getPendingResponseType() const; 
    uint8_t getProgressOrFlags() const;
    uint8_t getProgress() const;          // 0-100 durante calibración
    uint8_t getStatusFlags() const;       // Bits de estado actuales
    CalibrationParams getCalibrationParams() const;
    void loadCalibrationFromEEPROM();
    // Getters para valores específicos (usados por prepareResponse)
    uint16_t getDistance() const;
    uint16_t getApproachVelocity() const;
    bool isEmergency() const;
    bool isMoving() const;
    uint8_t statusFlags;

    // Comandos desde el Master (llamar cuando se recibe un comando SPI)
    void handleCommand(uint8_t commandType, int16_t speed = 0, int16_t angle = 0);

    // Eventos internos (pueden ser llamados por otras tareas)
    void onSonarEmergency();              // Distancia peligrosa detectada
    void onMotorTimeout();                // Timeout de seguridad

private:
    // Constructor privado (singleton)
    SlaveFSM() = default;
    ~SlaveFSM() = default;

    // Prohibir copia
    SlaveFSM(const SlaveFSM&) = delete;
    SlaveFSM& operator=(const SlaveFSM&) = delete;

    enum CalStep { MOVING, ANALYZING, DONE };
    CalStep calStep;
    unsigned long calStepStart;
    int calIndex;
    float calMeasurements[100];

    MotorControl* motorController;
    UltraSonicMeasure* sonar;
    SpeedController* speedController; // En lugar de objeto directo // Controlador de velocidad para la referencia de velocidad

    // Transición de estado (con callbacks opcionales)
    void transitionTo(SlaveStates newState);

    // Callbacks de entrada/salida de estados
    void onEnterIdle();
    void onExitIdle();
    void onEnterCalibration();
    void onExitCalibration();
    void onEnterReady();
    void onExitReady();
    void onEnterEmergency();
    void onExitEmergency();

    // Sub-máquina de calibración
    void updateCalibration();
    

    // Variables de estado
    SlaveStates currentState;
    SlaveStates previousState;
    uint32_t stateStartTime;

    // Calibración
    bool calibrationRunning;
    bool safetyTimeout = false;
    uint8_t calibrationProgress;
    uint32_t calibrationStepStart;
    float calibrationMeasurements[100];
    uint8_t calibrationIndex;
    CalibrationParams calibParams;

    // Datos de sensores (pueden venir de otra clase)
    uint16_t lastDistance;
    uint16_t  approachVelocity;
    bool emergencyFlag;
    bool errorFlag;
    bool movingFlag;
    // Mutex para acceso thread-safe
    SemaphoreHandle_t fsmMutex;

    // Helper para calcular K y τ al finalizar calibración
    void calculateCalibrationParams();
};

#endif // SLAVE_FSM_H