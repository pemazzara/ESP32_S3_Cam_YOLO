## 📄 **README.md**

```markdown
# 🤖 Robot Car Autónomo con YOLOv11n y ESP32-S3

Robot seguidor de objetos con visión artificial en tiempo real. Usa una **ESP32-S3** con cámara OV2640 para transmitir video por Wi-Fi a un navegador web, donde **YOLOv11n** (ejecutado con ONNX Runtime) detecta personas y envía comandos de navegación por WebSocket. El ESP32 ejecuta un control PID a 50 Hz sobre dos motores con driver L298N, asistido por sensores VL53L0X ToF para frenado de emergencia.

---

## 📐 Arquitectura

```
Cámara OV2640 ──► ESP32-S3 (Stream MJPEG + WebSocket)
                        │
                        Wi-Fi
                        │
Navegador (HTML5 + ONNX Runtime Web) ──► YOLOv11n ──► Comandos (WebSocket binario)
                        │
                        ▼
              ESP32-S3 (motorTask 50 Hz + PID)
                        │
              Driver L298N ──► Motores DC
```

---

## ✨ Características

- **Detección en tiempo real**: YOLOv11n corre en el navegador con WebGL/WASM.
- **Stream MJPEG**: La cámara OV2640 transmite a 320×240.
- **Control PID**: Bucle de control a 50 Hz en núcleo dedicado del ESP32.
- **Modo manual**: Joystick táctil con botones direccionales.
- **Failsafe**: Frenado automático si se pierde la señal (500 ms).
- **Telemetría**: Batería y distancias de sensores en HUD.
- **Cinemática diferencial**: Control preciso de dirección (0‑360°) y velocidad.

---

## 🛠️ Hardware

| Componente | Descripción |
|------------|-------------|
| **Placa** | ESP32-S3 DevKit-N16R8 CAM |
| **Cámara** | OV2640 integrada |
| **Driver** | L298N (2 canales) |
| **Sensores** | 3× VL53L0X ToF (frontal, izquierdo, derecho) |
| **Sonar** | HC-SR04 (RMT) |
| **Alimentación** | Batería 7.4 V LiPo (2S) |

### Pinout

| Componente | Pines ESP32-S3 |
|------------|----------------|
| Cámara OV2640 | Ver `src/main.cpp` |
| L298N ENA | 48 |
| L298N IN1 | 47 |
| L298N IN2 | 21 |
| L298N ENB | 2 |
| L298N IN3 | 41 |
| L298N IN4 | 42 |
| VL53L0X (I2C) | SDA=4, SCL=5 |
| HC-SR04 | TRIG=?, ECHO=? |
![alt text](image.png)
Ver:
https://www.oceanlabz.in/getting-started-with-esp32-s3-wroom-n16r8-cam-dev-board/

## 📦 Software

### ESP32 (PlatformIO)

```
src/
├── main.cpp              # Punto de entrada, WiFi, WebSocket, tareas FreeRTOS
├── camera.cpp/h          # Inicialización OV2640
├── motor_control.cpp/h   # Cinemática diferencial y L298N
├── speed_controller.cpp/h # PID a 50 Hz
├── sensors.cpp/h         # VL53L0X + HC-SR04
└── ble/                  # Comunicación WebSocket
```

### Frontend (HTML5 + ONNX Runtime Web)

- `data/index.html`: Interfaz completa con modo manual y autónomo.
- Modelo YOLOv11n servido desde GitHub Pages.

---

## 🚀 Instalación

### 1. Clonar el repositorio

### 2. Configurar Wi-Fi

Edita `src/main.cpp`:

```cpp
const char* ssid = "TU_WIFI";
const char* password = "TU_CONTRASEÑA";
```

### 3. Subir el firmware

```bash
pio run --target upload
```

### 4. Subir el frontend a SPIFFS

```bash
pio run --target uploadfs
```

### 5. Subir el modelo ONNX a GitHub Pages

1. Crea un repositorio público `robot-models`.
2. Sube `yolo11n.onnx` a la raíz.
3. Activa GitHub Pages en Settings → Pages.
4. La URL será: `https://TU_USUARIO.github.io/robot-models/yolo11n.onnx`

### 6. Acceder al robot

Abre en el navegador: `http://<IP_ESP32>/`

---

## 🎮 Uso

### Modo Manual
- Usa los botones direccionales para mover el robot.
- **↑** Avanzar | **↓** Retroceder | **← →** Girar.
- **PARAR**: Detiene los motores.

### Modo YOLO Autónomo
1. Selecciona **"YOLO Autónomo"**.
2. Apunta la cámara hacia una persona.
3. El robot la seguirá automáticamente:
   - Mantiene distancia de seguridad.
   - Gira para centrar a la persona.
   - Frena si la persona está demasiado cerca.

### Telemetría
- El HUD muestra batería y distancias de los sensores ToF.
- El indicador WS muestra el estado de la conexión WebSocket.

---

## ⚙️ Ajustes

### Umbrales de YOLO

En `index.html`:

```javascript
const CONF_THRESHOLD = 0.45;   // Confianza mínima
const CLASS_TARGET = 0;        // 0=persona (COCO)
const AREA_FRENO = 0.6;        // % de pantalla para frenar
```

### PID

En `speed_controller.cpp`:

```cpp
float Kp = 1.5f;    // Ganancia proporcional
float Ki = 0.3f;    // Ganancia integral
```

---

## 📁 Estructura del proyecto

```
esp32-robot-yolo/
├── src/
│   ├── main.cpp
│   ├── camera.cpp
│   ├── camera.h
│   ├── motor_control.cpp
│   ├── motor_control.h
│   ├── speed_controller.cpp
│   ├── speed_controller.h
│   ├── sensors.cpp
│   └── sensors.h
├── data/
│   └── index.html
├── platformio.ini
├── partitions_spiffs_big.csv
└── README.md
```

---

## 🔗 Dependencias

### PlatformIO

```ini
lib_deps =
    esp32-camera
    WebSockets
    VL53L0X
```

### Frontend (CDN)

- [ONNX Runtime Web](https://cdn.jsdelivr.net/npm/onnxruntime-web/dist/ort.min.js)

---

## 📝 Licencia

MIT © 2025 [Tu Nombre]

---

## 🙏 Agradecimientos

- [Ultralytics](https://github.com/ultralytics/ultralytics) por YOLOv11.
- [Espressif](https://github.com/espressif/esp32-camera) por la librería de cámara.
- [ONNX Runtime](https://onnxruntime.ai/) por la inferencia en navegador.
```

---

## 📌 **Instrucciones para usar el README**


