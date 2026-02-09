#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "esp_err.h"

// 핀 설정
#define UART1_GPIO_TXD           (17)
#define UART1_GPIO_RXD           (18)
#define RX1_BUF_SIZE             (1024)

// MD750T 통신 프로토콜 상수
#define MD_BAUD_RATE             57600
#define MD_ID_DEFAULT            1      // 모터 드라이버 ID (기본 1)
#define MD_RMID                  183    // 수신자 ID (MD750T Dual Motor Driver)
#define MD_TMID                  172    // 송신자 ID (ESP32-S3)

void MdRobot_Init(void);
void MdRobot_Clean_Buffer(void);

// --- 1 Byte Data (PID 0 ~ 100) ---
esp_err_t MdRobot_Get_PID_1Byte(uint8_t pid, uint8_t *value);
esp_err_t MdRobot_Set_PID_1Byte(uint8_t pid, uint8_t value);

// --- 2 Byte Data (PID 101 ~ 190) ---
esp_err_t MdRobot_Get_PID_2Byte(uint8_t pid, uint16_t *value);
esp_err_t MdRobot_Set_PID_2Byte(uint8_t pid, uint16_t value);

#ifdef __cplusplus
}
#endif