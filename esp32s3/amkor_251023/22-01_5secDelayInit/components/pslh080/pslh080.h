#ifndef PSL_H080_H
#define PSL_H080_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "esp_log.h"
#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

//================================================================
// 1. 하드웨어 설정 및 물리적 상수
//================================================================

#define PHASE_A_GPIO         GPIO_NUM_41
#define PHASE_B_GPIO         GPIO_NUM_40
#define PHASE_Z_GPIO         GPIO_NUM_42
#define ENCODER_PCNT_UNIT    PCNT_UNIT_0

// 0점 조절(Zero Setting) 버튼으로 사용할 GPIO 핀
#define ZERO_SET_BUTTON_GPIO   GPIO_NUM_0

/**
 * @brief Z상 물리적 상수 (캘리브레이션 완료된 값)
 */
#define Z_PHASE_INTERVAL_MM         (96.0f)
#define Z_PHASE_INTERVAL_PULSES     (96000)
// #define INITIAL_PULSE_COUNT         (540000)  // 초기 펄스 카운트 값 (540.000mm 위치에 해당)
#define INITIAL_PULSE_COUNT         (750000)  // 초기 펄스 카운트 값 (750.000mm 위치로 변경)


//================================================================
// 2. 함수 프로토타입
//================================================================

void pslh080_init(void);
int64_t pslh080_get_pulse_count(void);
double pslh080_get_position_mm(void);
void pslh080_reset_counter(void);
bool pslh080_get_z_phase_event(int64_t *count_at_z, int64_t *error_pulses);
void pslh080_apply_position_correction(int64_t error_to_correct);
esp_err_t pslh080_save_position_to_nvs(void);

/**
 * @brief 0점 조절 요청이 있었는지 확인하고 요청 플래그를 초기화합니다.
 * @return true 버튼이 눌렸었음, false 눌리지 않았음
 */
bool pslh080_get_and_clear_zero_set_request(void);


#ifdef __cplusplus
}
#endif

#endif // PSL_H080_H
