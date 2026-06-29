// ==================== PINES CÁMARA (Goouuu ESP32-S3-WROOM OV2640) ====================
#ifndef CAMERA_PINS_H
#define CAMERA_PINS_H
/*
// CAMERA_MODEL_ESP32S3_EYE
  #define PWDN_GPIO_NUM  -1
  #define RESET_GPIO_NUM -1
  #define XCLK_GPIO_NUM  4
  #define SIOD_GPIO_NUM  18
  #define SIOC_GPIO_NUM  23

  #define Y9_GPIO_NUM    36
  #define Y8_GPIO_NUM    37
  #define Y7_GPIO_NUM    38
  #define Y6_GPIO_NUM    39
  #define Y5_GPIO_NUM    35
  #define Y4_GPIO_NUM    14
  #define Y3_GPIO_NUM    13
  #define Y2_GPIO_NUM    34
  #define VSYNC_GPIO_NUM 5
  #define HREF_GPIO_NUM  27
  #define PCLK_GPIO_NUM  25

  #define LED_GPIO_NUM 22
  */
// Pines para ESP32-S3-WROOM con OV2640 (Goouuu)
#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  15
#define SIOD_GPIO_NUM  4
#define SIOC_GPIO_NUM  5

#define Y2_GPIO_NUM 11
#define Y3_GPIO_NUM 9
#define Y4_GPIO_NUM 8
#define Y5_GPIO_NUM 10
#define Y6_GPIO_NUM 12
#define Y7_GPIO_NUM 18
#define Y8_GPIO_NUM 17
#define Y9_GPIO_NUM 16

#define VSYNC_GPIO_NUM 6
#define HREF_GPIO_NUM  7
#define PCLK_GPIO_NUM  13

#endif
