#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "esp_err.h"

// 하드웨어 핀 설정
#define UART1_GPIO_TXD           (17)
#define UART1_GPIO_RXD           (18)

// 통신 설정
#define MD_BAUD_RATE             57600
#define MD_RMID                  183    // MD750T 수신 ID
#define MD_TMID                  172    // ESP32 송신 ID
#define MD_ID_DEFAULT            1      // 드라이버 국번

// 주요 PID 정의 (MCAR 및 모니터링용)
#define PID_REQ_PID_DATA         4      // 데이터 요청
#define PID_VEL_CMD              130    // 단일/공통 속도 명령 (2Byte)
#define PID_RESET_ALARM          151    // 알람 리셋 (Cmd)
#define PID_MAIN_DATA            193    // 메인 데이터 모니터링 (N-Byte)
#define PID_PNT_VEL_CMD          207    // 듀얼 채널 속도 명령 (N-Byte)

// PID 210 모니터링 데이터 구조체 
typedef struct {
    int16_t fb_rpm_l;   // Left RPM
    int16_t current_l;  // Left Current (0.1A)
    uint8_t status_l;   // Left Status
    int32_t pos_l;      // Left Position (Pulse)
    
    int16_t fb_rpm_r;   // Right RPM
    int16_t current_r;  // Right Current (0.1A)
    uint8_t status_r;   // Right Status
    int32_t pos_r;      // Right Position (Pulse)
} md_dual_monitor_t;

// 초기화
void MdRobot_Init(void);

// 1Byte / 2Byte 편의 함수
esp_err_t MdRobot_Get_PID_1Byte(uint8_t pid, uint8_t *value);
esp_err_t MdRobot_Set_PID_1Byte(uint8_t pid, uint8_t value);
esp_err_t MdRobot_Get_PID_2Byte(uint8_t pid, uint16_t *value);
esp_err_t MdRobot_Set_PID_2Byte(uint8_t pid, uint16_t value);

// [New] N-Byte 송수신 함수
/**
 * @brief N-Byte 데이터 읽기 요청 (예: PID 193 모니터링)
 */
esp_err_t MdRobot_Get_PID_NByte(uint8_t pid, uint8_t *data_buf, uint8_t expected_len);

/**
 * @brief N-Byte 데이터 쓰기 명령 (예: PID 207 속도제어)
 */
// rx_buf가 NULL이면 송신만 수행, NULL이 아니면 응답 대기 및 수신
esp_err_t MdRobot_Set_PID_NByte(uint8_t pid, uint8_t *tx_data, uint8_t tx_len, uint8_t *rx_buf, uint8_t max_rx_len);

// =========================================================
// [NEW] 고수준 제어 함수 (High-Level API)
// =========================================================

// MCAR 모드 설정 (PID 183)
esp_err_t MdRobot_Set_MCAR_Mode(void);

// 듀얼 모터 속도 지령 (PID 214: RPM + SlowDown)
// Write-Only 방식으로 전송 (응답 대기 X)
esp_err_t MdRobot_Drive_Dual_With_SD(int16_t rpm_l, int16_t rpm_r, uint16_t slow_down);

// 모니터링 데이터 읽기 (PID 210)
// 성공 시 out_data 구조체에 파싱된 값 저장
esp_err_t MdRobot_Read_Monitor_Data(md_dual_monitor_t *out_data);

#ifdef __cplusplus
}
#endif