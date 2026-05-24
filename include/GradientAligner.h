#ifndef GRADIENT_ALIGNER_H
#define GRADIENT_ALIGNER_H

#include <Arduino.h>
#include <deque>  // Para el filtro de media móvil (opcional, podemos implementar circular buffer)

class GradientAligner {
private:
    // Parámetros configurables
    const int16_t ANGLE_STEP_MAX = 10;      // Paso máximo de giro (grados)
    const int16_t ANGLE_STEP_MIN = 2;       // Paso mínimo de giro
    const uint16_t HYSTERESIS_THRESHOLD = 20; // Umbral para cambio de dirección (mm)
    const float GRADIENT_GAIN = 0.5;         // Ganancia para paso adaptativo
    const uint8_t FILTER_SIZE = 5;           // Tamaño del filtro de media móvil

    // Variables de estado
    int16_t current_angle = 90;               // Ángulo actual (0-359)
    int16_t search_direction = 1;              // +1: derecha, -1: izquierda
    uint32_t last_update_time = 0;
    
    // Para filtro de media móvil
    std::deque<uint16_t> distance_history;
    uint16_t filtered_distance = 0;

    // Para gradiente
    uint16_t last_filtered_distance = 0;
    int16_t last_angle = 90;
    float estimated_gradient = 0;  // Δdistancia/Δángulo

    // Para sensores laterales (opcional)
    bool use_side_sensors = false;
    uint16_t left_distance = 0;
    uint16_t right_distance = 0;

public:
    GradientAligner() {
        // Inicializar historial con un valor por defecto
        for (int i = 0; i < FILTER_SIZE; i++) {
            distance_history.push_back(0);
        }
    }

    // Actualizar el filtro y devolver distancia suavizada
    uint16_t updateFilter(uint16_t raw_distance) {
        distance_history.push_back(raw_distance);
        if (distance_history.size() > FILTER_SIZE) {
            distance_history.pop_front();
        }
        uint32_t sum = 0;
        for (uint16_t d : distance_history) {
            sum += d;
        }
        filtered_distance = sum / distance_history.size();
        return filtered_distance;
    }

    // Establecer si usamos sensores laterales y sus valores
    void setSideSensors(uint16_t left, uint16_t right) {
        left_distance = left;
        right_distance = right;
        use_side_sensors = true;
    }

    // Desactivar uso de sensores laterales
    void disableSideSensors() {
        use_side_sensors = false;
    }

    // Función principal: calcular el nuevo ángulo basado en la distancia actual
    int16_t computeAngle(uint16_t raw_distance) {
        uint32_t now = millis();
        // Control de frecuencia mínima (por ejemplo, 100ms)
        if (now - last_update_time < 100) {
            return current_angle;
        }
        last_update_time = now;

        // 1. Filtrar la distancia
        uint16_t dist = updateFilter(raw_distance);

        // 2. Si es la primera medición, inicializar
        if (last_filtered_distance == 0) {
            last_filtered_distance = dist;
            last_angle = current_angle;
            return current_angle;
        }

        // 3. Calcular el gradiente (cambio de distancia por cambio de ángulo)
        int16_t delta_angle = current_angle - last_angle;
        // Normalizar delta_angle al rango [-180, 180] para evitar vueltas completas
        if (delta_angle > 180) delta_angle -= 360;
        else if (delta_angle < -180) delta_angle += 360;

        if (abs(delta_angle) > 0) {
            int16_t delta_dist = dist - last_filtered_distance;
            estimated_gradient = (float)delta_dist / delta_angle;
        } else {
            estimated_gradient = 0;
        }

        // 4. Decidir la nueva dirección usando sensores laterales si están disponibles
        if (use_side_sensors && left_distance > 0 && right_distance > 0) {
            // Si hay diferencia significativa entre laterales, ajustamos directamente
            if (abs((int16_t)(left_distance - right_distance)) > 50) {
                if (left_distance < right_distance) {
                    // Pared más cerca a la izquierda, debemos girar a la derecha
                    search_direction = 1;
                } else {
                    search_direction = -1;
                }
                // Reiniciamos el gradiente para dar prioridad a esta información
                estimated_gradient = 0;
            }
        }

        // 5. Determinar el paso adaptativo basado en la magnitud del gradiente
        float abs_gradient = abs(estimated_gradient);
        int16_t step_size = ANGLE_STEP_MAX;
        if (abs_gradient > 0) {
            // Cuanto mayor el gradiente, mayor el paso (hasta un límite)
            step_size = (int16_t)(abs_gradient * GRADIENT_GAIN);
            step_size = constrain(step_size, ANGLE_STEP_MIN, ANGLE_STEP_MAX);
        } else {
            step_size = ANGLE_STEP_MIN;  // Si no hay gradiente, paso mínimo
        }

        // 6. Aplicar histéresis: solo cambiar dirección si el cambio es significativo
        int16_t delta_dist = dist - last_filtered_distance;
        if (abs(delta_dist) > HYSTERESIS_THRESHOLD) {
            if (delta_dist > 0) {
                // Mejoramos (nos alejamos): mantener dirección
                // (no hacemos nada, la dirección sigue igual)
            } else {
                // Empeoramos (nos acercamos): invertir dirección
                search_direction = -search_direction;
            }
        } else {
            // Cambio pequeño: no cambiar dirección (histéresis)
        }

        // 7. Actualizar ángulo con paso adaptativo
        current_angle += search_direction * step_size;

        // Normalizar ángulo a [0, 359]
        current_angle = (current_angle + 360) % 360;

        // 8. Guardar valores para la próxima iteración
        last_filtered_distance = dist;
        last_angle = current_angle;

        return current_angle;
    }

    // Reiniciar el algoritmo (por ejemplo, cuando cambia el objetivo)
    void reset() {
        last_filtered_distance = 0;
        last_angle = current_angle;
        estimated_gradient = 0;
        search_direction = 1;
        distance_history.clear();
        for (int i = 0; i < FILTER_SIZE; i++) {
            distance_history.push_back(0);
        }
    }

    // Obtener el gradiente estimado (para depuración)
    float getGradient() const { return estimated_gradient; }
    int16_t getDirection() const { return search_direction; }
    uint16_t getFilteredDistance() const { return filtered_distance; }
};

#endif
/*
    class GradientAligner {
    private:
        int16_t last_distance = 0;
        int16_t last_angle = 90;  // Ángulo actual
        int16_t search_direction = 1;  // +1: derecha, -1: izquierda
        const int16_t ANGLE_STEP = 5;  // Paso de exploración (grados)
        const int16_t MIN_DISTANCE = 200;  // Distancia mínima deseada
        uint32_t last_measure_time = 0;
        
    public:
        int16_t computeAngle(uint16_t current_distance) {
            uint32_t now = millis();
            
            // Solo actualizar cada cierto tiempo (ej. 200ms)
            if (now - last_measure_time < 200) {
                return last_angle;  // Mantener ángulo actual
            }
            last_measure_time = now;
            
            // 1. Si es la primera medida, no tenemos referencia
            if (last_distance == 0) {
                last_distance = current_distance;
                return last_angle;
            }
            
            // 2. Calcular cambio en distancia
            int16_t delta_distance = current_distance - last_distance;

            // 3. Lógica de decisión Proporcional
            if (abs(delta_distance) < 5) {
                // Estamos muy cerca del óptimo, no mover o mover mínimo
            } else {
                // El paso de ángulo depende de qué tan rápido está cambiando la distancia
                // Esto es: x3(k+1) = x3(k) + alpha * gradiente
                int16_t adaptive_step = constrain(abs(delta_distance) / 10, 1, ANGLE_STEP);
    
                if (delta_distance > 0) {
                    last_angle += search_direction * adaptive_step;
                } else {
                    search_direction = -search_direction;
                    last_angle += search_direction * adaptive_step;
                }
        }
            // 3. Lógica de decisión
            if (abs(delta_distance) < 10) {
                // Cambio pequeño: mantener dirección actual
                // (podríamos aumentar paso para explorar más)
            }
            else if (delta_distance > 0) {
                // ¡Nos alejamos! Seguir en esta dirección
                // (el movimiento fue beneficioso)
                last_angle += search_direction * ANGLE_STEP;
            }
            else {
                // Nos acercamos: cambiar dirección
                search_direction = -search_direction;
                last_angle += search_direction * ANGLE_STEP;
            }
            
            // 4. Mantener ángulo en rango [0, 359]
            last_angle = (last_angle + 360) % 360;
            
            // 5. Guardar distancia para próxima iteración
            last_distance = current_distance;
            
            return last_angle;
        }
        
        // Método para cuando queremos orientarnos a distancia mínima (perpendicular)
        void setTargetPerpendicular() {
            search_direction = 1;  // Reiniciar dirección
            last_distance = 0;      // Forzar reinicio en próxima lectura
        }
    };*/