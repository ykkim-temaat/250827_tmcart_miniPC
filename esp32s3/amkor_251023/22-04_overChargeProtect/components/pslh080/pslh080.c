#include "pslh080.h"
#include <math.h>
#include "nvs_flash.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

static const char *TAG = "PSLH080";
static const char* NVS_NAMESPACE = "encoder_pos";
static const char* NVS_KEY_PULSE_COUNT = "pulse_count";

// 공유 자원 보호를 위한 스핀락
static portMUX_TYPE pslh080_spinlock = portMUX_INITIALIZER_UNLOCKED;

// PCNT 및 Z상 이벤트 관련 변수
static pcnt_unit_handle_t pcnt_unit = NULL;
static volatile int64_t accumulated_pulse_count = 0;
static volatile bool z_phase_event_flag = false;
static volatile int64_t z_phase_position_error = 0;
static volatile int64_t z_phase_capture_count = 0;

// 0점 조절 및 디바운싱 관련 변수
static volatile bool g_zero_set_requested = false;

/**
 * @brief PCNT 카운터가 high/low limit에 도달했을 때 호출되는 콜백 (오버플로우 처리)
 */
static bool IRAM_ATTR pcnt_watch_point_callback(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx) {
    int64_t *p_acc_count = (int64_t *)user_ctx;
    if (edata->watch_point_value == 10000) {
        *p_acc_count += 10000;
    } else if (edata->watch_point_value == -10000) {
        *p_acc_count += -10000;
    }
    return false;
}

/**
 * @brief Z상 GPIO 핀에 신호가 감지되면 실행되는 인터럽트 핸들러
 */
static void IRAM_ATTR z_phase_isr_handler(void *arg) {
    portENTER_CRITICAL_ISR(&pslh080_spinlock);
    int current_pcnt_val;
    pcnt_unit_get_count(pcnt_unit, &current_pcnt_val);
    int64_t current_total_pulses = accumulated_pulse_count + current_pcnt_val;
    z_phase_capture_count = current_total_pulses;
    int64_t expected_pulses = round((double)current_total_pulses / Z_PHASE_INTERVAL_PULSES) * Z_PHASE_INTERVAL_PULSES;
    z_phase_position_error = current_total_pulses - expected_pulses;
    z_phase_event_flag = true;
    portEXIT_CRITICAL_ISR(&pslh080_spinlock);
}

void pslh080_init(void) {
    // NVS에서 마지막으로 저장된 위치를 불러옵니다.
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
    } else {
        int64_t saved_pulse_count = 0;
        err = nvs_get_i64(my_handle, NVS_KEY_PULSE_COUNT, &saved_pulse_count);
        switch (err) {
            case ESP_OK:
                accumulated_pulse_count = saved_pulse_count;
                ESP_LOGI(TAG, "Position restored from NVS: %lld pulses", saved_pulse_count);
                break;
            case ESP_ERR_NVS_NOT_FOUND:
                ESP_LOGI(TAG, "No saved position found in NVS. Starting at 0.");
                break;
            default :
                ESP_LOGE(TAG, "Error (%s) reading from NVS!", esp_err_to_name(err));
        }
        nvs_close(my_handle);
    }

    ESP_LOGI(TAG, "Initializing Encoder hardware...");

    // PCNT 유닛 설정
    pcnt_unit_config_t unit_config = { .high_limit = 10000, .low_limit = -10000 };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));

    // PCNT 채널 A, B 설정
    pcnt_chan_config_t chan_a_config = { .edge_gpio_num = PHASE_A_GPIO, .level_gpio_num = PHASE_B_GPIO };
    pcnt_channel_handle_t pcnt_chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a));
    pcnt_chan_config_t chan_b_config = { .edge_gpio_num = PHASE_B_GPIO, .level_gpio_num = PHASE_A_GPIO };
    pcnt_channel_handle_t pcnt_chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_b_config, &pcnt_chan_b));

    // 쿼드러처 디코딩 모드 설정
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    // Watch point (오버플로우 콜백) 설정
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, unit_config.high_limit));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, unit_config.low_limit));
    pcnt_event_callbacks_t cbs = { .on_reach = pcnt_watch_point_callback };
    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(pcnt_unit, &cbs, (void *)&accumulated_pulse_count));

    // PCNT 유닛 시작
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));
    
    // ISR 서비스 한 번만 설치 (Z상, 0점 버튼이 공유)
    // ESP_ERROR_CHECK(gpio_install_isr_service(0));   // app_main에서 이미 호출

    // Z상 GPIO 인터럽트 설정 및 핸들러 등록
    gpio_config_t z_phase_io_conf = { .pin_bit_mask = (1ULL << PHASE_Z_GPIO), .mode = GPIO_MODE_INPUT, .pull_up_en = GPIO_PULLUP_ENABLE, .intr_type = GPIO_INTR_POSEDGE };
    ESP_ERROR_CHECK(gpio_config(&z_phase_io_conf));
    ESP_ERROR_CHECK(gpio_isr_handler_add(PHASE_Z_GPIO, z_phase_isr_handler, NULL));

    ESP_LOGI(TAG, "PSLH080 Encoder Initialized Successfully.");
}

int64_t pslh080_get_pulse_count(void) {
    portENTER_CRITICAL(&pslh080_spinlock);
    int current_pcnt_val;
    pcnt_unit_get_count(pcnt_unit, &current_pcnt_val);
    int64_t count = accumulated_pulse_count + current_pcnt_val;
    portEXIT_CRITICAL(&pslh080_spinlock);
    return count;
}

double pslh080_get_position_mm(void) {
    return (double)pslh080_get_pulse_count() / (Z_PHASE_INTERVAL_PULSES / Z_PHASE_INTERVAL_MM);
}

void pslh080_reset_counter(void) {
    portENTER_CRITICAL(&pslh080_spinlock);
    accumulated_pulse_count = INITIAL_PULSE_COUNT;
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    portEXIT_CRITICAL(&pslh080_spinlock);
    
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err == ESP_OK) {
        nvs_erase_key(my_handle, NVS_KEY_PULSE_COUNT);
        nvs_commit(my_handle);
        nvs_close(my_handle);
    }
    // ESP_LOGI(TAG, "Encoder count has been reset to 0 and erased from NVS.");
    ESP_LOGI(TAG, "Encoder count has been reset to INITIAL_PULSE_COUNT (%lld) and erased from NVS.", (int64_t)INITIAL_PULSE_COUNT);
}

bool pslh080_get_z_phase_event(int64_t *count_at_z, int64_t *error_pulses) {
    bool event_occurred = false;
    portENTER_CRITICAL(&pslh080_spinlock);
    if (z_phase_event_flag) {
        *count_at_z = z_phase_capture_count;
        *error_pulses = z_phase_position_error;
        z_phase_event_flag = false;
        event_occurred = true;
    }
    portEXIT_CRITICAL(&pslh080_spinlock);
    return event_occurred;
}

void pslh080_apply_position_correction(int64_t error_to_correct) {
    portENTER_CRITICAL(&pslh080_spinlock);
    accumulated_pulse_count -= error_to_correct;
    portEXIT_CRITICAL(&pslh080_spinlock);
}

esp_err_t pslh080_save_position_to_nvs(void) {
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
        return err;
    }

    int64_t current_pulse_count = pslh080_get_pulse_count();
    err = nvs_set_i64(my_handle, NVS_KEY_PULSE_COUNT, current_pulse_count);
    if (err == ESP_OK) {
        err = nvs_commit(my_handle);
    }
    
    nvs_close(my_handle);
    return err;
}

bool pslh080_get_and_clear_zero_set_request(void) {
    bool request_status = false;
    portENTER_CRITICAL(&pslh080_spinlock);
    if (g_zero_set_requested) {
        request_status = true;
        g_zero_set_requested = false;
    }
    portEXIT_CRITICAL(&pslh080_spinlock);
    return request_status;
}

/**
 * @brief 외부 태스크에서 0점 조절을 요청하기 위한 함수
 */
void pslh080_request_zero_set(void) {
    portENTER_CRITICAL(&pslh080_spinlock);
    g_zero_set_requested = true;
    portEXIT_CRITICAL(&pslh080_spinlock);
}
