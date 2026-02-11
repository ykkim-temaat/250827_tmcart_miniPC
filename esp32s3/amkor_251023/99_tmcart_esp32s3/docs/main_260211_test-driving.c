// ===================================
// TMCart ESP32-S3 Firmware
// ===================================
// ## 펌웨어 기본사양
// - esp32-s3
// - esp-idf v5.1.2
// - micro-ROS Agent
// - ros2 humble
// - micro_ros_espidf_component
//   https://github.com/micro-ROS/micro_ros_espidf_component
// programed by ykkim.temaat@gmail.com
// ===================================

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "rom/ets_sys.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#include <geometry_msgs/msg/vector3.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <micro_ros_utilities/string_utilities.h>

#include "esp_mac.h"
#include "nvs_flash.h"

#include "i2c_bus_manager.h"
#include "linear_actuator.h"
#include "pcf8574.h"
#include "ads1115.h"
#include "mcp4728.h"
#include "pslh080.h"

#include "uart1_mdrobot.h"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc); vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n", __LINE__, (int)temp_rc);}}

#define ROS_NAMESPACE      CONFIG_MICRO_ROS_NAMESPACE
#define ROS_DOMAIN_ID      CONFIG_MICRO_ROS_DOMAIN_ID
#define ROS_AGENT_IP       CONFIG_MICRO_ROS_AGENT_IP
#define ROS_AGENT_PORT     CONFIG_MICRO_ROS_AGENT_PORT

static const char *TAG = "MAIN";
static const char *FIRMWARE_VERSION = "TMCart_ESP32S3_v1.0.4_260210; md750T RS485 test";

typedef enum {
    CPU_NUM_0 = 0,
    CPU_NUM_1
} my_core_id;

typedef enum {
    MOVE_STOP = 0,
    MOVE_UP = 1,
    MOVE_DN = 2,
    MOVE_FINE_UP = 3,
    MOVE_FINE_DN = 4
} pos_cmd_status;

// 큐에 저장할 명령어 데이터 구조체 정의
typedef struct {
    uint32_t cmd;
    float val1;
    float val2;
} RosCommand_t;

// ===========================================================
// Global variables for ROS communication & Queue
// ===========================================================
QueueHandle_t g_ros_cmd_queue; // 명령어 큐 핸들 선언

uint32_t g_ros_status = 0x00; // Status to ROS
float g_ros_status1 = 0.0;
float g_ros_status2 = 0.0;

float g_loadcell_avg0 = 0.0;
float g_loadcell_avg1 = 0.0;

float g_z_pos_mm = 0.0;
float g_z_pos_target = 0.0;
pos_cmd_status g_z_pos_cmd_status = MOVE_STOP;

float g_x_pos_mm = 0.0;
float g_x_pos_target = 0.0;
pos_cmd_status g_x_pos_cmd_status = MOVE_STOP;

float g_motor_voltage = 0.0;
float g_battery_voltage = 0.0;

bool g_z_axis_moving = false;       // Z축 동작 상태
bool g_x_axis_moving = false;       // X축 동작 상태
bool g_is_driving = false;          // 주행 상태
bool g_stopper_down_state = false;  // 스토퍼 상태 (true: Down, false: Up)
bool g_emlock_on_state = false;     // EM-Lock 상태 (true: Locked, false: Unlocked)
bool g_laser_state = false;         // 가이드 레이저 상태 (true: On, false: Off)
bool g_estop_state = false;         // 비상정지 상태 (true: Triggered, false: Normal)
bool g_charging_state = false;      // 충전 릴레이 상태 (true: Charging, false: Not Charging)
bool g_load1_detected = false;      // Load1 (REAR) 상태 (true: Detected, false: Not Detected)
bool g_load2_detected = false;      // Load2 (FRONT) 상태 (true: Detected, false: Not Detected)
bool g_rear_bumper_detected = false;      // REAR_BUMPER 상태 (true: Detected, false: Not Detected)
bool g_docking_complete = false;      // Docking 상태 (true: Complete, false: Incomplete)
bool g_clutch_on_state = false;       // Clutch 상태 (true: On, false: Off)

// ===========================================================
// micro-ROS Publisher related
// ===========================================================
rcl_publisher_t z_pos_publisher;
std_msgs__msg__Float32 z_pos_msg;

rcl_publisher_t x_pos_publisher;
std_msgs__msg__Float32 x_pos_msg;

rcl_publisher_t battery_publisher;
std_msgs__msg__Float32 battery_voltage_msg;

rcl_publisher_t tmcart_status_publisher;
std_msgs__msg__Int32 tmcart_status_msg;

/** Status Bitmask ===========================================
 * 비트 (Bit) | 연산 (1 << n) | 상태 (Status) | true일 때의 의미
 * Bit 0	| (1 << 0) | z_axis_moving | Z축 이동 중
 * Bit 1	| (1 << 1) | x_axis_moving | X축 이동 중
 * Bit 2	| (1 << 2) | is_driving | 주행 중
 * Bit 3	| (1 << 3) | stopper_down_state | 스토퍼 내려감
 * Bit 4	| (1 << 4) | emlock_on_state | EM-Lock 켜짐
 * Bit 5	| (1 << 5) | laser_state | 레이저 켜짐
 * Bit 6	| (1 << 6) | estop_state | 비상정지 발동
 * Bit 7	| (1 << 7) | charging_state | 충전 중
 * Bit 8	| (1 << 8) | load1_detected | 적재물1(안쪽) 감지
 * Bit 9	| (1 << 9) | load2_detected | 적재물2(바깥쪽) 감지
 * Bit 10	| (1 << 10) | rear_bumper_detected | 후면 범퍼 감지
 * Bit 11	| (1 << 11) | docking_complete | 도킹 완료
 * Bit 12	| (1 << 12) | clutch_on_state | CLUTCH 풀림 (사람에 의한 제어)
 * Bit 13-31 	-	예비 (Reserved)	-
**/

// ===========================================================
// micro-ROS Timer related
// ===========================================================
rcl_timer_t z_pos_timer;
rcl_timer_t x_pos_timer;
rcl_timer_t battery_timer;

rcl_timer_t status_timer; // 8개 상태를 발행할 통합 타이머

// ===========================================================
// micro-ROS Subscriber related
// ===========================================================
rcl_subscription_t subscriber;
geometry_msgs__msg__Vector3 cmd_msg;


// Timer callback function for Z-axis position publisher
void z_pos_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        z_pos_msg.data = g_z_pos_mm;
        RCSOFTCHECK(rcl_publish(&z_pos_publisher, &z_pos_msg, NULL));
    }
}

// Timer callback function for x-axis position publisher
void x_pos_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        x_pos_msg.data = g_x_pos_mm;
        RCSOFTCHECK(rcl_publish(&x_pos_publisher, &x_pos_msg, NULL));
    }
}

// Timer callback function for battery voltage publisher
void battery_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        if (g_battery_voltage > 0) {
            battery_voltage_msg.data = g_battery_voltage;
            RCSOFTCHECK(rcl_publish(&battery_publisher, &battery_voltage_msg, NULL));
        }
    }
}

// Timer callback function for system status publishers
void status_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        // 1. 인코딩할 32비트 정수 변수를 0으로 초기화
        int32_t encoded_status = 0;

        // 2. 각 bool 상태가 true일 경우, 해당하는 비트를 1로 설정 (Bitwise OR 연산)
        if (g_z_axis_moving)      { encoded_status |= (1 << 0); }
        if (g_x_axis_moving)      { encoded_status |= (1 << 1); }
        if (g_is_driving)         { encoded_status |= (1 << 2); }
        if (g_stopper_down_state) { encoded_status |= (1 << 3); }
        if (g_emlock_on_state)    { encoded_status |= (1 << 4); }
        if (g_laser_state)        { encoded_status |= (1 << 5); }
        if (g_estop_state)        { encoded_status |= (1 << 6); }
        if (g_charging_state)     { encoded_status |= (1 << 7); }
        if (g_load1_detected)     { encoded_status |= (1 << 8); }
        if (g_load2_detected)     { encoded_status |= (1 << 9); }
        if (g_rear_bumper_detected)     { encoded_status |= (1 << 10); }
        if (g_docking_complete)     { encoded_status |= (1 << 11); }
        if (g_clutch_on_state)      { encoded_status |= (1 << 12); }

        // 3. 인코딩된 값을 메시지에 담아 한 번만 발행
        tmcart_status_msg.data = encoded_status;
        RCSOFTCHECK(rcl_publish(&tmcart_status_publisher, &tmcart_status_msg, NULL));
    }
}

// Subscriber callback: 수신한 명령어를 전역 변수 대신 큐에 저장
void subscription_callback(const void *msgin)
{
    const geometry_msgs__msg__Vector3 *received_cmd = (const geometry_msgs__msg__Vector3 *)msgin;

    // 큐에 보낼 데이터 구조체 생성 및 값 할당
    RosCommand_t cmd_to_queue;
    cmd_to_queue.cmd = (uint32_t)received_cmd->x;
    cmd_to_queue.val1 = (float)received_cmd->y;
    cmd_to_queue.val2 = (float)received_cmd->z;

    // 생성한 데이터를 큐로 전송
    if (xQueueSend(g_ros_cmd_queue, &cmd_to_queue, pdMS_TO_TICKS(10)) != pdPASS) {
        ESP_LOGW(TAG, "Failed to post command to the queue. Queue might be full.");
    }
}

// micro ros processes tasks
void micro_ros_task(void *arg) {
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));
    RCCHECK(rcl_init_options_set_domain_id(&init_options, ROS_DOMAIN_ID));
    rmw_init_options_t *rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
    RCCHECK(rmw_uros_options_set_udp_address(ROS_AGENT_IP, ROS_AGENT_PORT, rmw_options));

    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    uint32_t client_key = 0;
    client_key |= (uint32_t)mac[2] << 24;
    client_key |= (uint32_t)mac[3] << 16;
    client_key |= (uint32_t)mac[4] << 8;
    client_key |= (uint32_t)mac[5];
    client_key = client_key + 21;
    RCCHECK(rmw_uros_options_set_client_key(client_key, rmw_options));

    uint8_t conn_agent_cnt = 0;
    while (rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator) != RCL_RET_OK) {         
        conn_agent_cnt++;
        ESP_LOGI(TAG, "Connecting try #%d to agent... %s:%s", conn_agent_cnt, ROS_AGENT_IP, ROS_AGENT_PORT); 
        if (conn_agent_cnt > 1) {
            ESP_LOGE(TAG, "Failed to connect to agent. Please check the agent status.");
            esp_restart();
        }
        vTaskDelay(pdMS_TO_TICKS(500)); 
    }
    ESP_LOGI(TAG, "Connected to agent.");
    
    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "tmcart_esp32_node", ROS_NAMESPACE, &support));

    // Initialize Publishers
    RCCHECK(rclc_publisher_init_default(&z_pos_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "z_axis_position"));
    RCCHECK(rclc_publisher_init_default(&x_pos_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "x_axis_position"));
    RCCHECK(rclc_publisher_init_default(&battery_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "battery_voltage"));
    
    // 신규 통합 상태 Publisher 초기화
    RCCHECK(rclc_publisher_init_default(&tmcart_status_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "tmcart_status_code"));

    RCCHECK(rclc_subscription_init_default(&subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3), "tmcart_actuator_cmd"));

    const unsigned int z_pos_timer_timeout = 100;
    const unsigned int x_pos_timer_timeout = 100;
    const unsigned int battery_timer_timeout = 10000; 
    const unsigned int status_timer_timeout = 1000; // 1000ms 주기로 상태 발행

    RCCHECK(rclc_timer_init_default(&z_pos_timer, &support, RCL_MS_TO_NS(z_pos_timer_timeout), z_pos_timer_callback));
    RCCHECK(rclc_timer_init_default(&x_pos_timer, &support, RCL_MS_TO_NS(x_pos_timer_timeout), x_pos_timer_callback));
    RCCHECK(rclc_timer_init_default(&battery_timer, &support, RCL_MS_TO_NS(battery_timer_timeout), battery_timer_callback));
    RCCHECK(rclc_timer_init_default(&status_timer, &support, RCL_MS_TO_NS(status_timer_timeout), status_timer_callback)); 

    rclc_executor_t executor;    
    int handle_num = 5; // 타이머 4개 + 구독 1개
    RCCHECK(rclc_executor_init(&executor, &support.context, handle_num, &allocator));
    unsigned int rcl_wait_timeout = 1000;
    RCCHECK(rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(rcl_wait_timeout)));

    RCCHECK(rclc_executor_add_timer(&executor, &z_pos_timer));
    RCCHECK(rclc_executor_add_timer(&executor, &x_pos_timer));
    RCCHECK(rclc_executor_add_timer(&executor, &battery_timer));
    RCCHECK(rclc_executor_add_timer(&executor, &status_timer)); 
    
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &cmd_msg, &subscription_callback, ON_NEW_DATA));

    while (1)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        usleep(1000);
    }

    RCCHECK(rcl_subscription_fini(&subscriber, &node));

    RCCHECK(rcl_publisher_fini(&tmcart_status_publisher, &node));
    RCCHECK(rcl_publisher_fini(&battery_publisher, &node));
    RCCHECK(rcl_publisher_fini(&x_pos_publisher, &node));
    RCCHECK(rcl_publisher_fini(&z_pos_publisher, &node));
    RCCHECK(rcl_node_fini(&node));
    vTaskDelete(NULL);
}

// MD750T Motor Driver Control Task
void md750t_ctrl_task(void *arg) {
    uint8_t input_20 = 0;
    uint8_t input_21 = 0;
    uint8_t input_23 = 0;
    uint8_t input_24 = 0;
    uint8_t input_26 = 0;
    uint8_t input_27 = 0;

    int32_t curr_ros_cmd = 0;

    float read_voltage_ch0 = 2.5f;
    float read_voltage_ch1 = 2.5f;
    float read_voltage_ch2 = 24.0f;
    float read_voltage_ch3 = 24.0f;

    float set_voltage_ch0 = 2.5f;
    float set_voltage_ch1 = 2.5f;

    uint32_t slow_drive_left_cnt = 0;
    uint32_t slow_drive_right_cnt = 0;
    uint32_t guide_laser_cnt = 0;

    // float twist_left_vel = 0.0f;
    // float twist_right_vel = 0.0f;
    float simple_cmd_vel_left = 0.0f;
    float simple_cmd_vel_right = 0.0f;

    bool twist_mode = false;
    bool prohibit_twist = false;
    bool simple_cmd_vel = false;
    bool charging_dock_mode = false;
    bool clutch_on_mode = false;
    bool z_pos_correction_mode = false;
    bool over_charge_state_mode = false;    // 29.2V 이상 충전금지 모드

    float z_pos_error = 0.0f;
    
    // last_actuator_time은 큐 방식으로 인해 더 이상 필요 없음 ---
    // unsigned long last_actuator_time = 0;
    unsigned long last_loadcell_time = 0;
    unsigned long last_z_pos_error_correction_time = 0;      
    // unsigned long last_pushbutton_time = 0; 
    unsigned long last_ext_din_time = 0;
    // unsigned long last_guide_laser_time = 0;
    unsigned long last_log_time = 0;

    // int simple_cmd_vel_count = 0;

    unsigned long md750t_start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    RosCommand_t received_cmd; // 큐에서 꺼낸 명령어를 저장할 지역 변수

    L298N_PWM_Init();
    L298N_PWM_Set_Speed_M1(200);
    L298N_PWM_Set_Speed_M2(200);

    static float neutral_voltage_ch0 = 2.5f; // 초기화 전 기본값
    static float neutral_voltage_ch1 = 2.5f;

    // ===========================================================
    // [Safety Improved] Load Cell Initial Calibration Logic
    // ===========================================================
    
    // 캘리브레이션 관련 상수 설정
    const float CALIB_VALID_MIN = 2.30f;    // 유효성 검사 최소값
    const float CALIB_VALID_MAX = 2.70f;    // 유효성 검사 최대값
    const float CALIB_STABILITY_TH = 0.05f; // 안정화 판정 임계치 (Max - Min 차이)
    const int CALIB_SAMPLES = 30;           // 샘플링 횟수 증가 (20 -> 30)
    const int MAX_CALIB_RETRIES = 5;        // 최대 재시도 횟수

    float final_avg_ch0 = 2.5f;
    float final_avg_ch1 = 2.5f;
    bool calibration_success = false;
    int retry_cnt = 0;

    // 캘리브레이션 실패 영구 저장 플래그
    bool is_calibration_failed = false;

    // 초기 DAC 설정 (안전을 위해 2.5V 중립 강제 출력)
    MCP4728_SetVoltage_InternalVref(MCP4728_CHANNEL_A, 2.5f, false);
    vTaskDelay(pdMS_TO_TICKS(10));
    MCP4728_SetVoltage_InternalVref(MCP4728_CHANNEL_B, 2.5f, false);

    ESP_LOGI(TAG, ">>> Starting Load Cell Calibration...");

    while (retry_cnt < MAX_CALIB_RETRIES) {
        float sum_ch0 = 0.0f;
        float sum_ch1 = 0.0f;
        
        // 안정도 판정을 위한 Min/Max 변수 초기화
        float min_val_ch0 = 10.0f, max_val_ch0 = -10.0f;
        float min_val_ch1 = 10.0f, max_val_ch1 = -10.0f;

        // 샘플링 루프
        for (int i = 0; i < CALIB_SAMPLES; i++) {
            float temp_ch0, temp_ch1;
            ADS1115_ReadVoltage(ADS1115_MUX_AIN0_GND, &temp_ch0);
            vTaskDelay(pdMS_TO_TICKS(5)); // ADC 변환 시간 고려
            ADS1115_ReadVoltage(ADS1115_MUX_AIN1_GND, &temp_ch1);
            
            sum_ch0 += temp_ch0;
            sum_ch1 += temp_ch1;

            // Update Min/Max for CH0
            if (temp_ch0 < min_val_ch0) min_val_ch0 = temp_ch0;
            if (temp_ch0 > max_val_ch0) max_val_ch0 = temp_ch0;

            // Update Min/Max for CH1
            if (temp_ch1 < min_val_ch1) min_val_ch1 = temp_ch1;
            if (temp_ch1 > max_val_ch1) max_val_ch1 = temp_ch1;

            vTaskDelay(pdMS_TO_TICKS(50)); // 샘플링 간격 (약 1.5초 소요)
        }

        float avg_ch0 = sum_ch0 / CALIB_SAMPLES;
        float avg_ch1 = sum_ch1 / CALIB_SAMPLES;
        
        // 1. 안정화 판정 (Stability Check)
        float stability_diff_ch0 = max_val_ch0 - min_val_ch0;
        float stability_diff_ch1 = max_val_ch1 - min_val_ch1;
        bool is_stable = (stability_diff_ch0 < CALIB_STABILITY_TH) && (stability_diff_ch1 < CALIB_STABILITY_TH);

        // 2. 유효성 검사 (Validity Check)
        bool is_valid_range = (avg_ch0 > CALIB_VALID_MIN && avg_ch0 < CALIB_VALID_MAX) &&
                              (avg_ch1 > CALIB_VALID_MIN && avg_ch1 < CALIB_VALID_MAX);

        if (is_stable && is_valid_range) {
            final_avg_ch0 = avg_ch0;
            final_avg_ch1 = avg_ch1;
            calibration_success = true;
            ESP_LOGI(TAG, "Calibration Success! Neutral CH0: %.3f V, CH1: %.3f V", final_avg_ch0, final_avg_ch1);
            break; // 성공 시 루프 탈출
        } else {
            retry_cnt++;
            ESP_LOGE(TAG, "Calibration Fail! Stable: %d (Diff: %.3f/%.3f), Valid: %d (Avg: %.3f/%.3f)", 
                     is_stable, stability_diff_ch0, stability_diff_ch1, 
                     is_valid_range, avg_ch0, avg_ch1);
            vTaskDelay(pdMS_TO_TICKS(500)); // 재시도 전 대기
        }
    }

    if (!calibration_success) {
        // [Critical Error] 최종 실패 시 안전 조치
        ESP_LOGE(TAG, "Calibration FAILED after retries. Entering SAFE MODE.");
        
        // 1. 중립값 강제 설정 (Safety Default)
        neutral_voltage_ch0 = 2.50f;
        neutral_voltage_ch1 = 2.50f;
        
        // 2. 주행 금지 플래그 활성화 (매우 중요)
        prohibit_twist = true; 

        // 3. 비상정지 DOUT 하드웨어 즉시 활성화 (HW 차단)
        // Emergency Switch DOUT_07 (PCF8574 0x20, P6 0x40) -> HIGH(Active)
        pcf8574_read_byte(PCF8574_ADDR_0x20, &input_20); 
        input_20 = input_20 | 0x40;   // Set P6 to HIGH
        pcf8574_write_byte(PCF8574_ADDR_0x20, input_20);
        
        // 4. 실패 상태 플래그 설정 (메인 루프에서 해제 방지용)
        is_calibration_failed = true;
        g_estop_state = true;

    } else {
        // 성공 시 정상 값 적용
        neutral_voltage_ch0 = final_avg_ch0;
        neutral_voltage_ch1 = final_avg_ch1;
        
        // 안전을 위해 1회 더 중립 출력
        MCP4728_SetVoltage_InternalVref(MCP4728_CHANNEL_A, neutral_voltage_ch0, false);
        MCP4728_SetVoltage_InternalVref(MCP4728_CHANNEL_B, neutral_voltage_ch1, false);
    }

    while(1) {        
        unsigned long current_read_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

        // 큐에서 새로운 명령어가 있는지 확인 (비동기, 0ms 대기)
        // 기존 100ms 시간 제한 블록을 큐 확인 블록으로 대체
        if (xQueueReceive(g_ros_cmd_queue, &received_cmd, 0) == pdTRUE) {
            ESP_LOGD(TAG, "New CMD from Q: 0x%02lX, val1: %.2f, val2: %.2f", received_cmd.cmd, received_cmd.val1, received_cmd.val2);
            
            // 모든 g_ros_cmd, g_ros_val1, g_ros_val2를 received_cmd 구조체 멤버로 변경
            // Z_Axis Linear Actuator Command (Manual Control)
            if (received_cmd.cmd == 0x10 && curr_ros_cmd != 0x10) {
                curr_ros_cmd = 0x10;
                L298N_PWM_Set_Speed_M1(0);
                pcf8574_read_byte(PCF8574_ADDR_0x27, &input_27);
                input_27 = (input_27 & ~0x01) & ~0x04;
                pcf8574_write_byte(PCF8574_ADDR_0x27, input_27);
                ESP_LOGD(TAG, "Z_STOP (Move STOP) ... ");
                z_pos_error = 0.0f;
                g_z_pos_cmd_status = MOVE_STOP;
                g_z_axis_moving = false; // 상태 업데이트
            } else if (received_cmd.cmd == 0x11 && curr_ros_cmd != 0x11) {
                curr_ros_cmd = 0x11;
                pcf8574_read_byte(PCF8574_ADDR_0x27, &input_27);
                input_27 = (input_27 | 0x04) & ~0x01;
                pcf8574_write_byte(PCF8574_ADDR_0x27, input_27);
                ESP_LOGD(TAG, "Z_UP (Move Backward) ... ");
                L298N_PWM_Set_Speed_M1(received_cmd.val1);
                g_z_pos_target = received_cmd.val2;
                z_pos_error = g_z_pos_target - g_z_pos_mm;
                g_z_pos_cmd_status = MOVE_UP;
                g_z_axis_moving = true; // 상태 업데이트
            } else if (received_cmd.cmd == 0x12 && curr_ros_cmd != 0x12) {
                curr_ros_cmd = 0x12;
                pcf8574_read_byte(PCF8574_ADDR_0x27, &input_27);
                input_27 = (input_27 | 0x01) & ~0x04;
                pcf8574_write_byte(PCF8574_ADDR_0x27, input_27);
                ESP_LOGD(TAG, "Z_DN (Move Forward) ... ");
                L298N_PWM_Set_Speed_M1(received_cmd.val1);
                g_z_pos_target = received_cmd.val2;
                z_pos_error = g_z_pos_target - g_z_pos_mm;
                g_z_pos_cmd_status = MOVE_DN;
                g_z_axis_moving = true; // 상태 업데이트
            } else if (received_cmd.cmd == 0x13 && curr_ros_cmd != 0x13) {
                curr_ros_cmd = 0x13;
                pcf8574_read_byte(PCF8574_ADDR_0x27, &input_27);
                input_27 = (input_27 | 0x04) & ~0x01;
                pcf8574_write_byte(PCF8574_ADDR_0x27, input_27);
                ESP_LOGD(TAG, "Z_FINE_UP (Move Backward) ... ");
                    if (g_z_pos_mm < 600.0f) {
                        L298N_PWM_Set_Speed_M1(240); 
                    } else if (g_z_pos_mm >= 600.0f && g_z_pos_mm < 650.0f) {
                        L298N_PWM_Set_Speed_M1(210); 
                    } else if (g_z_pos_mm >= 650.0f && g_z_pos_mm < 700.0f) {
                        L298N_PWM_Set_Speed_M1(180); 
                    } else if (g_z_pos_mm >= 700.0f) {
                        L298N_PWM_Set_Speed_M1(150); 
                    }
                g_z_pos_target = 0.0f;
                z_pos_error = 0.0f;
                g_z_pos_cmd_status = MOVE_FINE_UP;
                g_z_axis_moving = true; // 상태 업데이트
            } else if (received_cmd.cmd == 0x14 && curr_ros_cmd != 0x14) {
                curr_ros_cmd = 0x14;
                pcf8574_read_byte(PCF8574_ADDR_0x27, &input_27);
                input_27 = (input_27 | 0x01) & ~0x04;
                pcf8574_write_byte(PCF8574_ADDR_0x27, input_27);
                ESP_LOGD(TAG, "Z_FINE_DN (Move Forward) ... ");
                L298N_PWM_Set_Speed_M1(50.0f);
                g_z_pos_target = 0.0f;
                z_pos_error = 0.0f;
                g_z_pos_cmd_status = MOVE_FINE_DN;
                g_z_axis_moving = true; // 상태 업데이트
            }

            // Stopper Command
            if (received_cmd.cmd == 0x21 && curr_ros_cmd != 0x21) {
                curr_ros_cmd = 0x21;
                pcf8574_read_byte(PCF8574_ADDR_0x20, &input_20);
                input_20 = input_20 & ~0x08;
                pcf8574_write_byte(PCF8574_ADDR_0x20, input_20);
                ESP_LOGD(TAG, "Stopper Down ... ");
                g_stopper_down_state = true; // 상태 업데이트
            } else if (received_cmd.cmd == 0x20 && curr_ros_cmd != 0x20){
                curr_ros_cmd = 0x20;
                pcf8574_read_byte(PCF8574_ADDR_0x20, &input_20);
                input_20 = input_20 | 0x08;
                pcf8574_write_byte(PCF8574_ADDR_0x20, input_20);
                ESP_LOGD(TAG, "Stopper Up ... ");
                g_stopper_down_state = false; // 상태 업데이트
            }

            // X_Axis Linear Actuator Command
            if (received_cmd.cmd == 0x30 && curr_ros_cmd != 0x30) {
                curr_ros_cmd = 0x30;
                L298N_PWM_Set_Speed_M2(0);
                pcf8574_read_byte(PCF8574_ADDR_0x27, &input_27);
                input_27 = (input_27 & ~0x02) & ~0x08;
                pcf8574_write_byte(PCF8574_ADDR_0x27, input_27);
                ESP_LOGD(TAG, "X_STOP (Move Stop) ... ");
                g_x_pos_cmd_status = MOVE_STOP;
                g_x_axis_moving = false; // 상태 업데이트
            } else if (received_cmd.cmd == 0x31 && curr_ros_cmd != 0x31) {
                curr_ros_cmd = 0x31;
                pcf8574_read_byte(PCF8574_ADDR_0x27, &input_27);
                input_27 = (input_27 | 0x02) & ~0x08;
                pcf8574_write_byte(PCF8574_ADDR_0x27, input_27);
                ESP_LOGD(TAG, "X_Move Forward ... ");
                // L298N_PWM_Set_Speed_M2(received_cmd.val1);
                L298N_PWM_Set_Speed_M2(400);
                g_x_pos_cmd_status = MOVE_UP;
                g_x_axis_moving = true; // 상태 업데이트
            } else if (received_cmd.cmd == 0x32 && curr_ros_cmd != 0x32) {
                curr_ros_cmd = 0x32;
                pcf8574_read_byte(PCF8574_ADDR_0x27, &input_27);
                input_27 = (input_27 | 0x08) & ~0x02;
                pcf8574_write_byte(PCF8574_ADDR_0x27, input_27);
                ESP_LOGD(TAG, "X_Move Backward ... ");
                // L298N_PWM_Set_Speed_M2(received_cmd.val1);
                L298N_PWM_Set_Speed_M2(400);
                g_x_pos_cmd_status = MOVE_DN;
                g_x_axis_moving = true; // 상태 업데이트
            }
            
            // LEFT Push-button Command
            if (received_cmd.cmd == 0x40 && curr_ros_cmd != 0x40) {
                curr_ros_cmd = 0x40;
                slow_drive_left_cnt++;
                guide_laser_cnt++;
                ESP_LOGD(TAG, "LEFT Push-button ON ... ");
            } else if (received_cmd.cmd == 0x42 && curr_ros_cmd != 0x42) {
                curr_ros_cmd = 0x42;
                slow_drive_left_cnt = 0;
                guide_laser_cnt = 0;
                ESP_LOGD(TAG, "LEFT Push-button OFF ... ");
            }

            // RIGHT Push-button Command
            if (received_cmd.cmd == 0x41 && curr_ros_cmd != 0x41) {
                curr_ros_cmd = 0x41;
                slow_drive_right_cnt++;
                guide_laser_cnt++;
                ESP_LOGD(TAG, "RIGHT Push-button ON ... ");
            } else if (received_cmd.cmd == 0x43 && curr_ros_cmd != 0x43) {
                curr_ros_cmd = 0x43;
                slow_drive_right_cnt = 0;
                guide_laser_cnt = 0;
                ESP_LOGD(TAG, "RIGHT Push-button OFF ... ");
            } 
            
            // EM-LOCK Command
            if (received_cmd.cmd == 0x50 && curr_ros_cmd != 0x50) {
                curr_ros_cmd = 0x50;
                pcf8574_read_byte(PCF8574_ADDR_0x26, &input_26);
                input_26 = input_26 & ~0x10;
                pcf8574_write_byte(PCF8574_ADDR_0x26, input_26);
                ESP_LOGD(TAG, "EM-Lock ON ... ");
                g_emlock_on_state = true; // 상태 업데이트
            } else if (received_cmd.cmd == 0x51 && curr_ros_cmd != 0x51) {
                curr_ros_cmd = 0x51;
                pcf8574_read_byte(PCF8574_ADDR_0x26, &input_26);
                input_26 = input_26 | 0x10;
                pcf8574_write_byte(PCF8574_ADDR_0x26, input_26);
                ESP_LOGD(TAG, "EM-Lock OFF ... ");
                g_emlock_on_state = false; // 상태 업데이트
            }
            
            // TWIST Command (RFID 위치인식 후 저속주행)
            if (received_cmd.cmd == 0x60 && curr_ros_cmd != 0x60) { 
                curr_ros_cmd = 0x60;
                // twist_left_vel = received_cmd.val1;
                // twist_right_vel = received_cmd.val2;
                twist_mode = true;
                ESP_LOGD(TAG, "Twist Mode ON ... ");
            } else if (received_cmd.cmd == 0x61 && curr_ros_cmd != 0x61) { 
                curr_ros_cmd = 0x61;
                // twist_left_vel = 2.5f;
                // twist_right_vel = 2.5f;
                twist_mode = false;
                ESP_LOGD(TAG, "Twist Mode OFF ... ");
            }
            
            // Prohibit TWIST (주행금지 명령)
            if (received_cmd.cmd == 0x62 && curr_ros_cmd != 0x62) { 
                curr_ros_cmd = 0x62;
                prohibit_twist = true;
                ESP_LOGD(TAG, "Prohibit Twist ON ... ");
            } else if (received_cmd.cmd == 0x63 && curr_ros_cmd != 0x63) { 
                curr_ros_cmd = 0x63;
                prohibit_twist = false;
                ESP_LOGD(TAG, "Prohibit Twist OFF ... ");
            }

            // simple cmd_vel 
            if (received_cmd.cmd == 0x64 && curr_ros_cmd != 0x64) { 
                curr_ros_cmd = 0x64;
                simple_cmd_vel = true;
                simple_cmd_vel_left = received_cmd.val1;
                simple_cmd_vel_right = received_cmd.val2;
                ESP_LOGD(TAG, "Simple cmd_vel ON ... ");
            } 
            
            // Charging Dock Command 
            if (received_cmd.cmd == 0x70 && curr_ros_cmd != 0x70) { 
                curr_ros_cmd = 0x70;
                charging_dock_mode = true;
                ESP_LOGD(TAG, "Charging Dock Mode ON ... ");
            } else if (received_cmd.cmd == 0x71 && curr_ros_cmd != 0x71) { 
                curr_ros_cmd = 0x71;
                charging_dock_mode = false;
                ESP_LOGD(TAG, "Charging Dock Mode OFF ... ");
            }
            
            // CLUTCH ON/OFF Command 
            if (received_cmd.cmd == 0x80 && curr_ros_cmd != 0x80) { 
                curr_ros_cmd = 0x80;
                clutch_on_mode = false;
                ESP_LOGD(TAG, "Clutch OFF (Powered by Motor)... ");
            } else if (received_cmd.cmd == 0x81 && curr_ros_cmd != 0x81) { 
                curr_ros_cmd = 0x81;
                clutch_on_mode = true;
                ESP_LOGD(TAG, "Clutch ON (Powered by Human)... ");
            }
        }   

        // 100ms loop, Read Load Cell Voltages & Update MCP4728 (주행모드)
        if (current_read_time - last_loadcell_time > 100) { 
            last_loadcell_time = current_read_time;

            // 1. Read Load Cell Voltages
            ADS1115_ReadVoltage(ADS1115_MUX_AIN0_GND, &read_voltage_ch0);
            vTaskDelay(pdMS_TO_TICKS(10));
            ADS1115_ReadVoltage(ADS1115_MUX_AIN1_GND, &read_voltage_ch1);
            
            // ESP_LOGD(TAG, "read_voltage_ch0: %.2f V, ch1: %.2f V", read_voltage_ch0, read_voltage_ch1);
            // ESP_LOGD(TAG, "neutral_voltage_ch0: %.2f V, ch1: %.2f V", neutral_voltage_ch0, neutral_voltage_ch1);

            // 2. 제어 변수 설정
            float diff_step = 0.1f;       // 편차 스텝 (diff_step)
            float ramp_up_step = 0.01f;   // 가속 스텝
            float ramp_dn_step = 0.01f;   // 감속 스텝
            // float ramp_dn_step = 0.02f;   // 감속 스텝 (가속보다 빠르게 멈춤)

            // 3. CH0 제어 로직 (역방향 제어: 입력 High -> 출력 Low)
            float diff_ch0 = read_voltage_ch0 - neutral_voltage_ch0;

            if (diff_ch0 > diff_step) {
                // 반응하지 않는 구간 건너뛰기 (2.2V ~ 2.5V, 2.5V ~ 2.8V)
                if (set_voltage_ch0 > 2.2f && set_voltage_ch0 < 2.5f) {
                    set_voltage_ch0 = 2.2f;
                } else if (set_voltage_ch0 > 2.5f && set_voltage_ch0 < 2.8f) {
                    set_voltage_ch0 = 2.8f;
                }
                // 입력이 기준보다 0.1V 이상 높음 -> 출력 감소 (후진 or 전진)
                set_voltage_ch0 += ramp_up_step;
            } else if (diff_ch0 < -diff_step) {
                // 반응하지 않는 구간 건너뛰기 (2.2V ~ 2.5V, 2.5V ~ 2.8V)
                if (set_voltage_ch0 > 2.2f && set_voltage_ch0 < 2.5f) {
                    set_voltage_ch0 = 2.2f;
                } else if (set_voltage_ch0 > 2.5f && set_voltage_ch0 < 2.8f) {
                    set_voltage_ch0 = 2.8f;
                }
                // 입력이 기준보다 0.1V 이상 낮음 -> 출력 증가
                set_voltage_ch0 -= ramp_up_step;
            } else {
                if (set_voltage_ch0 > 2.5f) {
                    set_voltage_ch0 -= ramp_dn_step;
                    if (set_voltage_ch0 < 2.8f) set_voltage_ch0 = 2.5f;
                } else if (set_voltage_ch0 < 2.5f) {
                    set_voltage_ch0 += ramp_dn_step;
                    if (set_voltage_ch0 > 2.2f) set_voltage_ch0 = 2.5f;
                }
            }

            // 4. CH1 제어 로직 (동일 로직)
            float diff_ch1 = read_voltage_ch1 - neutral_voltage_ch1;

            if (diff_ch1 > diff_step) {
                // 반응하지 않는 구간 건너뛰기 (2.2V ~ 2.5V, 2.5V ~ 2.8V)
                if (set_voltage_ch1 > 2.2f && set_voltage_ch1 < 2.5f) {
                    set_voltage_ch1 = 2.2f;
                } else if (set_voltage_ch1 > 2.5f && set_voltage_ch1 < 2.8f) {
                    set_voltage_ch1 = 2.8f;
                }
                set_voltage_ch1 += ramp_up_step;
            } else if (diff_ch1 < -diff_step) {
                // 반응하지 않는 구간 건너뛰기 (2.2V ~ 2.5V, 2.5V ~ 2.8V)
                if (set_voltage_ch1 > 2.2f && set_voltage_ch1 < 2.5f) {
                    set_voltage_ch1 = 2.2f;
                } else if (set_voltage_ch1 > 2.5f && set_voltage_ch1 < 2.8f) {
                    set_voltage_ch1 = 2.8f;
                }
                set_voltage_ch1 -= ramp_up_step;
            } else {
                if (set_voltage_ch1 > 2.5f) {
                    set_voltage_ch1 -= ramp_dn_step;
                    if (set_voltage_ch1 < 2.8f) set_voltage_ch1 = 2.5f;
                } else if (set_voltage_ch1 < 2.5f) {
                    set_voltage_ch1 += ramp_dn_step;
                    if (set_voltage_ch1 > 2.2f) set_voltage_ch1 = 2.5f;
                }
            }

            // ESP_LOGD(TAG, "set_voltage_ch0: %.2f V, ch1: %.2f V", set_voltage_ch0, set_voltage_ch1);

            // 전진속도 제한
            if (set_voltage_ch0 < 2.19f || set_voltage_ch1 < 2.19f) {
                float set_voltage_tmp = (set_voltage_ch0 + set_voltage_ch1) / 2.0f; 
                set_voltage_ch0 = set_voltage_tmp;
                set_voltage_ch1 = set_voltage_tmp;
                ESP_LOGI(TAG, "set_voltage_ch0 & ch1 limited to %.2f V", set_voltage_tmp);
            }

            // 전진속도 제한
            if (set_voltage_ch0 < 2.15) {
                set_voltage_ch0 = 2.15f;
                ESP_LOGI(TAG, "set_voltage_ch0 limited to 2.15f V");
            }
            if (set_voltage_ch1 < 2.15) {
                set_voltage_ch1 = 2.15f;
                ESP_LOGI(TAG, "set_voltage_ch1 limited to 2.15f V");
            }  

            // 후진속도 제한
            if (set_voltage_ch0 > 2.81) {
                set_voltage_ch0 = 2.81f;
                ESP_LOGI(TAG, "set_voltage_ch0 limited to 2.81f V");
            }
            if (set_voltage_ch1 > 2.81) {
                set_voltage_ch1 = 2.81f;
                ESP_LOGI(TAG, "set_voltage_ch1 limited to 2.81f V");
            }            

            // ESP_LOGD(TAG, "set_voltage_ch0: %.2f V, ch1: %.2f V\n", set_voltage_ch0, set_voltage_ch1);

            // Slow drive Mode 1 (Push-button)
            if (slow_drive_left_cnt > 0 && slow_drive_right_cnt > 0) { 
                if (set_voltage_ch0 < 2.2f || set_voltage_ch1 < 2.2f) {
                    set_voltage_ch0 = 2.19f;
                    set_voltage_ch1 = 2.19f;
                    ESP_LOGI(TAG, "Slow Drive Mode: FWD limited to 2.19f V");
                } else if (set_voltage_ch0 > 2.8f || set_voltage_ch1 > 2.8f) {
                    set_voltage_ch0 = 2.81f;
                    set_voltage_ch1 = 2.81f;
                    ESP_LOGI(TAG, "Slow Drive Mode: BACK limited to 2.81f V");
                }
            }

            // Slow drive Mode 2 (RFID) 
            if (twist_mode == true) { 
                if (set_voltage_ch0 < 2.2f) {
                    set_voltage_ch0 = 2.19f;
                    ESP_LOGI(TAG, "Twist Mode: LEFT FWD limited to 2.19f V");
                } else if (set_voltage_ch0 > 2.8f) {
                    set_voltage_ch0 = 2.81f;
                    ESP_LOGI(TAG, "Twist Mode: LEFT BACK limited to 2.81f V");
                }
                if (set_voltage_ch1 < 2.2f) {
                    set_voltage_ch1 = 2.19f;
                    ESP_LOGI(TAG, "Twist Mode: RIGHT FWD limited to 2.19f V");
                } else if (set_voltage_ch1 > 2.8f) {
                    set_voltage_ch1 = 2.81f;
                    ESP_LOGI(TAG, "Twist Mode: RIGHT BACK limited to 2.81f V");
                }
            }

            // // prohibit one-hand operation
            // if (read_voltage_ch0 > 2.4 && read_voltage_ch0 < 2.6) { 
            //     set_voltage_ch1 = 2.5; 
            // }

            // if (read_voltage_ch1 > 2.4 && read_voltage_ch1 < 2.6) { 
            //     set_voltage_ch0 = 2.5; 
            // }

            // simple cmd_vel (0x64)
            if (simple_cmd_vel == true) { 
                simple_cmd_vel = false; 
                curr_ros_cmd = 0;
                
                set_voltage_ch0 = simple_cmd_vel_left;
                set_voltage_ch1 = simple_cmd_vel_right;
                // simple_cmd_vel_count++;

                // ESP_LOGD(TAG, "simple_cmd_vel_left: %.2f V, simple_cmd_vel_right: %.2f V", simple_cmd_vel_left, simple_cmd_vel_right);
                // ESP_LOGD(TAG, "Simple cmd_vel ACTIVE, count: %d", simple_cmd_vel_count);

                // if (simple_cmd_vel_count > 5) {
                //     simple_cmd_vel_count = 1;
                //     set_voltage_ch0 = 2.5f;
                //     set_voltage_ch1 = 2.5f;
                //     simple_cmd_vel = false; 
                //     curr_ros_cmd = 0;
                //     ESP_LOGD(TAG, "Simple cmd_vel TIMEOUT -> STOP");
                // }
            }
            
            // prohibit twist (GUI에 의한 주행금지 명령)
            if (prohibit_twist == true) { 
                set_voltage_ch0 = 2.5; 
                set_voltage_ch1 = 2.5; 
            }

            // 전원 투입 후 5초 이내에는 무조건 정지명령
            if (current_read_time - md750t_start_time < 5000) { 
                // ESP_LOGD(TAG, "current_read_time: %lu, md750t_start_time: %lu, diff: %lu", current_read_time, md750t_start_time, current_read_time - md750t_start_time);
                set_voltage_ch0 = 2.5; 
                set_voltage_ch1 = 2.5; 
            }

            // 5. Update MCP4728 (마지막 실행)
            MCP4728_SetVoltage_InternalVref(MCP4728_CHANNEL_A, set_voltage_ch0, false);
            vTaskDelay(pdMS_TO_TICKS(10));
            MCP4728_SetVoltage_InternalVref(MCP4728_CHANNEL_B, set_voltage_ch1, false);
            // ESP_LOGD(TAG, "set_voltage_ch0: %.2f V, ch1: %.2f V\n", set_voltage_ch0, set_voltage_ch1);

            // 6. 주행 상태 업데이트
            // 출력값이 2.5V(중립)와 거의 같으면 정지 상태로 간주
            if (fabs(set_voltage_ch0 - 2.5f) < 0.01f && fabs(set_voltage_ch1 - 2.5f) < 0.01f) {
                g_is_driving = false;
            } else {
                g_is_driving = true;
            }
        }

        // 100ms loop, Z_Axis Linear Actuator Task (Automatic Position Control)
        if (current_read_time - last_z_pos_error_correction_time > 100) {
            last_z_pos_error_correction_time = current_read_time;
            if ((g_z_pos_cmd_status == MOVE_UP || g_z_pos_cmd_status == MOVE_DN) && (g_z_pos_mm < 528.0f || g_z_pos_mm > 920.0f)) {
                RosCommand_t stop_cmd = {0x10, 0, 0};
                if (xQueueSendToFront(g_ros_cmd_queue, &stop_cmd, 0) == pdPASS) {
                    ESP_LOGW(TAG, "Z-Axis LIMIT REACHED! Inserting STOP command to front of queue.");
                }
            }

            z_pos_error = g_z_pos_target - g_z_pos_mm;

            if (g_z_pos_cmd_status == MOVE_UP && g_z_pos_target != 0.0f) {
                ESP_LOGD(TAG, "Z_UP Target: %.2f mm, Current: %.2f mm, Error: %.2f mm", g_z_pos_target, g_z_pos_mm, z_pos_error);
                if (z_pos_error >= 100.0f) { 
                    L298N_PWM_Set_Speed_M1(400); 
                } else if (z_pos_error < 100.0f && z_pos_error >= 40.0f) { 
                    L298N_PWM_Set_Speed_M1(400); 
                } else if (z_pos_error < 40.0f && z_pos_error >= 20.0f) { 
                    L298N_PWM_Set_Speed_M1(300); 
                } else if (z_pos_error < 20.0f && z_pos_error >= 10.0f) { 
                    L298N_PWM_Set_Speed_M1(200); 
                } else if (z_pos_error < 10.0f && z_pos_error >= 5.0f) { 
                    if (g_z_pos_mm < 600.0f) {
                        L298N_PWM_Set_Speed_M1(200); 
                    } else if (g_z_pos_mm >= 600.0f && g_z_pos_mm < 700.0f) {
                        L298N_PWM_Set_Speed_M1(180); 
                    } else if (g_z_pos_mm >= 700.0f) {
                        L298N_PWM_Set_Speed_M1(150); 
                    }
                } else if (z_pos_error <= 0.0f) {
                    L298N_PWM_Set_Speed_M1(0);
                    RosCommand_t stop_cmd = {0x10, 0, 0};
                    xQueueSendToFront(g_ros_cmd_queue, &stop_cmd, 0);
                    ESP_LOGD(TAG, "MOVE_UP Auto STOP ... ");
                }
            }
            if (g_z_pos_cmd_status == MOVE_DN && g_z_pos_target != 0.0f) {
                ESP_LOGD(TAG, "Z_DN Target: %.2f mm, Current: %.2f mm, Error: %.2f mm", g_z_pos_target, g_z_pos_mm, z_pos_error);
                if (z_pos_error <= -100.0f) { 
                    if (g_z_pos_mm < 600.0f) {
                        L298N_PWM_Set_Speed_M1(50); 
                    } else if (g_z_pos_mm >= 600.0f && g_z_pos_mm < 700.0f) {
                        L298N_PWM_Set_Speed_M1(100); 
                    } else if (g_z_pos_mm >= 700.0f) {
                        L298N_PWM_Set_Speed_M1(400); 
                    }
                } else if (z_pos_error > -100.0f && z_pos_error <= -40.0f) { 
                    if (g_z_pos_mm < 600.0f) {
                        L298N_PWM_Set_Speed_M1(50); 
                    } else if (g_z_pos_mm >= 600.0f && g_z_pos_mm < 700.0f) {
                        L298N_PWM_Set_Speed_M1(100); 
                    } else if (g_z_pos_mm >= 700.0f) {
                        L298N_PWM_Set_Speed_M1(300); 
                    }
                } else if (z_pos_error > -40.0f && z_pos_error <= -30.0f) { 
                    if (g_z_pos_mm < 600.0f) {
                        L298N_PWM_Set_Speed_M1(50); 
                    } else if (g_z_pos_mm >= 600.0f && g_z_pos_mm < 700.0f) {
                        L298N_PWM_Set_Speed_M1(100); 
                    } else if (g_z_pos_mm >= 700.0f) {
                        L298N_PWM_Set_Speed_M1(200); 
                    }
                } else if (z_pos_error > -30.0f && z_pos_error <= -20.0f) { 
                    L298N_PWM_Set_Speed_M1(50); 
                } else if (z_pos_error > -20.0f && z_pos_error <= -10.0f) { 
                    L298N_PWM_Set_Speed_M1(50); 
                } else if (z_pos_error > -10.0f && z_pos_error <= -5.0f) { 
                    L298N_PWM_Set_Speed_M1(50); 
                // } else if (z_pos_error >= 0.0f) {
                } else if (z_pos_error >= -1.5f) {
                    L298N_PWM_Set_Speed_M1(0);
                    RosCommand_t stop_cmd = {0x10, 0, 0};
                    xQueueSendToFront(g_ros_cmd_queue, &stop_cmd, 0);
                    ESP_LOGD(TAG, "MOVE_DN Auto STOP ... ");
                }
            }
        }
        
        // 1000ms loop, Read External Digital Inputs & Update Outputs
        if (current_read_time - last_ext_din_time > 1000) {
            last_ext_din_time = current_read_time;
            pcf8574_read_byte(PCF8574_ADDR_0x23, &input_23);
            pcf8574_read_byte(PCF8574_ADDR_0x24, &input_24);

            // Emergency Switch DOUT_07, 0x20, P6 0x40
            if (input_23 & 0x40 || g_rear_bumper_detected || is_calibration_failed) { 
                pcf8574_read_byte(PCF8574_ADDR_0x20, &input_20); 
                input_20 = input_20 | 0x40;   // Set P6 to HIGH (Active LOW)
                pcf8574_write_byte(PCF8574_ADDR_0x20, input_20);                 
                if (!g_estop_state) {
                     ESP_LOGE(TAG, "Emergency Stop Active! (Source: SW/HW/Calib)");
                }
                g_estop_state = true; // 상태 업데이트
            } else {  // EMG SW (Normal-Close)
                pcf8574_read_byte(PCF8574_ADDR_0x20, &input_20); 
                input_20 = input_20 & ~0x40;   // Set P6 to LOW (Active LOW)
                pcf8574_write_byte(PCF8574_ADDR_0x20, input_20);                  
                if (g_estop_state) {
                     ESP_LOGE(TAG, "Emergency Stop Inactive! Resumed.");
                }
                g_estop_state = false; // 상태 업데이트
            }

            // Detect Load1(rear, 안쪽)
            if (input_23 & 0x02) { 
                // ESP_LOGD(TAG, "Load1 (REAR) NOT DETECTED!");
                g_load1_detected = false; // 상태 업데이트
            } else {  
                // ESP_LOGD(TAG, "Load1 (REAR) DETECTED!");
                g_load1_detected = true; // 상태 업데이트
            }

            // Detect Load2(front, 바깥쪽)
            if (input_23 & 0x01) { 
                // ESP_LOGD(TAG, "Load2 (FRONT) NOT DETECTED!");
                g_load2_detected = false; // 상태 업데이트
            } else {  
                // ESP_LOGD(TAG, "Load2 (FRONT) DETECTED!");
                g_load2_detected = true; // 상태 업데이트
            }

            // Docking Complete Signal
            if (input_23 & 0x04 || input_23 & 0x08) { 
                // ESP_LOGD(TAG, "Docking NOT COMPLETE!");
                g_docking_complete = false; // 상태 업데이트
            } else {                
                // ESP_LOGD(TAG, "Docking COMPLETE!");
                g_docking_complete = true; // 상태 업데이트

            }

            // Detect REAR_BUMPER 
            if (input_24 & 0x40) { 
                // ESP_LOGD(TAG, "REAR_BUMPER NOT DETECTED!");
                g_rear_bumper_detected = false; // 상태 업데이트
            } else {                
                // ESP_LOGD(TAG, "REAR_BUMPER DETECTED!");
                g_rear_bumper_detected = true; // 상태 업데이트
            }

            // Detect Z-Home Sensor 
            if ((input_24 & 0x80) == 0 && z_pos_correction_mode == false) { 
                // ESP_LOGD(TAG, "Z-Home Sensor DETECTED!");
                pslh080_request_zero_set(); // 0점 조절(위치 초기화) 요청
                z_pos_correction_mode = true;
            } else {                
                // ESP_LOGD(TAG, "Z-Home Sensor NOT DETECTED!");
                z_pos_correction_mode = false;
            }

            vTaskDelay(pdMS_TO_TICKS(20));

            // Charging DET Signal DOUT_09, 0x21, P0 0x01
            // 조건: 수동충전(input_23의 8번째 비트가 0)이거나 충전 독 모드가 켜진 경우
            // 그리고 29.2V 이상의 과충전 상태가 아닌 경우
            if ((((input_23 & 0x80) == 0) || (charging_dock_mode == true)) && (over_charge_state_mode == false)) {
                // P0 핀을 LOW로 설정하여 회로를 활성화(CLOSE)합니다. (충전시작)
                pcf8574_read_byte(PCF8574_ADDR_0x21, &input_21);
                input_21 = input_21 & ~0x01; // Set P0 to LOW (Active LOW)
                pcf8574_write_byte(PCF8574_ADDR_0x21, input_21);
                // ESP_LOGD(TAG, "PCF8574(0x21) Output: 0x%02X", input_21);
                // ESP_LOGD(TAG, "Charge DET or Dock Mode -> CLOSE for PDIST80V2");
                g_charging_state = true; // 상태 업데이트
            } else {
                // 그 외의 경우 (수동충전 명령도 아니고 충전 독 명령 모드도 아닌 경우, 또는 과충전 상태인 경우)
                // P0 핀을 HIGH로 설정하여 회로를 비활성화(OPEN)합니다. (충전정지)
                pcf8574_read_byte(PCF8574_ADDR_0x21, &input_21);
                input_21 = input_21 | 0x01;  // Set P0 to HIGH (Active LOW)
                pcf8574_write_byte(PCF8574_ADDR_0x21, input_21);
                // ESP_LOGD(TAG, "PCF8574(0x21) Output: 0x%02X", input_21);
                // ESP_LOGD(TAG, "NO Charge DET -> OPEN for PDIST80V2");
                g_charging_state = false; // 상태 업데이트
            }

            // Guide Laser Task
            if (guide_laser_cnt > 0) {     // Guide Laser ON
                pcf8574_read_byte(PCF8574_ADDR_0x26, &input_26); 
                input_26 = input_26 & ~0x20;   // Set P5 to LOW (Active LOW)
                pcf8574_write_byte(PCF8574_ADDR_0x26, input_26);
                g_laser_state = true; // 상태 업데이트
            } else {   // Guide Laser OFF
                pcf8574_read_byte(PCF8574_ADDR_0x26, &input_26); 
                input_26 = input_26 | 0x20;   // Set P5 to HIGH (Active LOW)
                pcf8574_write_byte(PCF8574_ADDR_0x26, input_26); 
                g_laser_state = false; // 상태 업데이트
            }

            if (clutch_on_mode == true) {     // Clutch ON (Human Powered)
                pcf8574_read_byte(PCF8574_ADDR_0x21, &input_21); 
                input_21 = input_21 & ~0x04;   // Set P2 to LOW (Active LOW)
                pcf8574_write_byte(PCF8574_ADDR_0x21, input_21);
                g_clutch_on_state = true; // 상태 업데이트
            } else {   // Clutch OFF (Motor Powered)
                pcf8574_read_byte(PCF8574_ADDR_0x21, &input_21); 
                input_21 = input_21 | 0x04;   // Set P2 to HIGH (Active LOW)
                pcf8574_write_byte(PCF8574_ADDR_0x21, input_21); 
                g_clutch_on_state = false; // 상태 업데이트
            }
            
            // MD750T Enable Signal DOUT_10, 0x21, P1 0x02
            if (true) { // MD750T Enable (Always)
                pcf8574_read_byte(PCF8574_ADDR_0x21, &input_21); 
                input_21 = input_21 & ~0x02;   // Set P1 to LOW (Active LOW)
                pcf8574_write_byte(PCF8574_ADDR_0x21, input_21); 
                // ESP_LOGD(TAG, "PCF8574(0x21) Output: 0x%02X", input_21);
                // ESP_LOGD(TAG, "MD750T Enable Signal for MD750T");
            }
        }

        // 10,000ms(10sec), Battery monitoring & Periodic Logging
        if (current_read_time - last_log_time > 10000) {
            last_log_time = current_read_time;
            
            // Read Motor & Battery Voltages
            ADS1115_ReadVoltage(ADS1115_MUX_AIN2_GND, &read_voltage_ch2);
            vTaskDelay(pdMS_TO_TICKS(10));
            ADS1115_ReadVoltage(ADS1115_MUX_AIN3_GND, &read_voltage_ch3);
            g_motor_voltage = read_voltage_ch2 * 10.0f;
            g_battery_voltage = read_voltage_ch3 * 10.0f;

            ESP_LOGD(TAG, "Load Cell Voltages - CH0: %.3f V, CH1: %.3f V", read_voltage_ch0, read_voltage_ch1);
            ESP_LOGD(TAG, "Motor Voltages(CH2): %.3f V, Battery Voltage(CH3): %.3f V", g_motor_voltage, g_battery_voltage);

            if (g_battery_voltage > 29.2f) {    // 29.2V 이상 과충전상태 진입 -> 충전중지
                ESP_LOGW(TAG, "Battery voltage is over Charging status: %.2f V", g_battery_voltage);
                over_charge_state_mode = true;
                charging_dock_mode = false;
            } else if (g_battery_voltage < 27.2f) { // 27.2V 이하로 복귀시 과충전상태 해제 -> 충전재개 가능
                // ESP_LOGI(TAG, "Battery voltage is back to Normal status: %.2f V", g_battery_voltage);
                over_charge_state_mode = false;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// Z-Axis Linear Scale Task
void linear_scale_task(void *arg)
{
    ESP_LOGI(TAG, "linear_scale_task started. System is in normal operation.");
    uint32_t last_save_time = 0;
    uint32_t last_log_time = 0;
    int64_t last_saved_pulse_count = pslh080_get_pulse_count();
    int64_t last_logged_pulse_count = last_saved_pulse_count;
    int64_t count_at_z_event, position_error;

    while (1) {
        if (pslh080_get_and_clear_zero_set_request()) {
            ESP_LOGW(TAG, "Zero Set button pressed! Resetting current position to 0.");
            pslh080_reset_counter();
            last_logged_pulse_count = 0;
            last_saved_pulse_count = 0;
            last_log_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        }

        uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        int64_t current_pulses = pslh080_get_pulse_count();

        if (current_time - last_save_time > 5000) {
            if (current_pulses != last_saved_pulse_count) {
                if (pslh080_save_position_to_nvs() == ESP_OK) {
                    last_saved_pulse_count = current_pulses;
                    ESP_LOGI(TAG, "Position successfully saved to NVS.");
                } else {
                    ESP_LOGE(TAG, "Failed to save position to NVS.");
                }
            }
            last_save_time = current_time;
        }

        if (pslh080_get_z_phase_event(&count_at_z_event, &position_error)) {
            ESP_LOGW(TAG, "Z-Phase Event! Count: %lld, Error: %lld pulses", count_at_z_event, position_error);
            if (llabs(position_error) <= 50) {
                pslh080_apply_position_correction(position_error);
                ESP_LOGI(TAG, "Position corrected by %lld pulses.", -position_error);
            } else {
                ESP_LOGE(TAG, "Position error is too large for automatic correction.");
            }
        }
        
        if (llabs(current_pulses - last_logged_pulse_count) > 5 ||
            current_time - last_log_time > 5000)
        {
            g_z_pos_mm = pslh080_get_position_mm();

            // // For test purpose
            // static float test_float = 0.0f;
            // g_z_pos_mm = test_float++;    
            // if (g_z_pos_mm > 650.0f) {
            //     g_z_pos_mm = 650.0f;
            // }

            last_logged_pulse_count = current_pulses;
            last_log_time = current_time;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// X-Axis Hall Sensor Task
void hall_sensor_task(void *arg) {
    ESP_LOGI(TAG, "hall_sensor_task started. System is in normal operation.");
    unsigned long last_x_hall_read_time = 0;
    float x_hall_factor = 0.106926; // mm per count
    Encoder_Init();
    float last_x_pos_mm = 0.0f;

    while (1) {
        unsigned long current_read_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (current_read_time - last_x_hall_read_time > 1000) {
            last_x_hall_read_time = current_read_time;

            int x_hall_cnt = Encoder_Get_Count_M2();
            g_x_pos_mm = (float)x_hall_cnt * x_hall_factor;
            
            // // For test purpose
            // static float test_float = 20.0f;
            // g_x_pos_mm = test_float--; 
            // if (g_x_pos_mm < 0.0f) {
            //     g_x_pos_mm = 0.0f;
            // }

            if (g_x_pos_cmd_status == MOVE_DN) { // Move Backward
                if (last_x_pos_mm - g_x_pos_mm == 0.0f) {
                    g_x_pos_mm = 0.0f;
                    Encoder_Clear_Count_M2();
                    RosCommand_t stop_cmd = {0x30, 0, 0};
                    xQueueSendToFront(g_ros_cmd_queue, &stop_cmd, 0);
                    ESP_LOGI(TAG, "X-Axis stalled while moving backward. Sending X-Stop.");
                }
            } else if (g_x_pos_cmd_status == MOVE_UP) { // Move Forward
                if (last_x_pos_mm - g_x_pos_mm == 0.0f) {
                    RosCommand_t stop_cmd = {0x30, 0, 0};
                    xQueueSendToFront(g_ros_cmd_queue, &stop_cmd, 0);
                    ESP_LOGI(TAG, "X-Axis stalled while moving forward. Sending X-Stop.");
                }
            }
            
            last_x_pos_mm = g_x_pos_mm;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// ===============================================================
// [NEW] MCAR 주행 및 모니터링 테스트 태스크
// ===============================================================
#if 0
void test_mdrobot_drive_task(void *arg) {
    uint8_t val8;
    uint16_t val16;
    esp_err_t ret;

    // PID 210 (PID_PNT_MAIN_DATA) 수신용 데이터 구조체
    // 총 18 Byte (Motor1: 9Byte, Motor2: 9Byte)
    typedef struct {
        // --- Motor 1 (Left) ---
        int16_t fb_rpm_l;   // 현재 속도 (0.1 RPM 단위)
        int16_t current_l;  // 현재 전류 (0.1 A 단위)
        uint8_t status_l;   // 상태 플래그 (Alarm, Run/Stop 등)
        int32_t pos_l;      // 현재 위치 (Encoder Pulse Count, 4 Bytes)

        // --- Motor 2 (Right) ---
        int16_t fb_rpm_r;   // 현재 속도 (0.1 RPM 단위)
        int16_t current_r;  // 현재 전류 (0.1 A 단위)
        uint8_t status_r;   // 상태 플래그
        int32_t pos_r;      // 현재 위치 (Encoder Pulse Count, 4 Bytes)
    } md_dual_monitor_t;

    md_dual_monitor_t mon; // 구조체 변수 선언 ('mon')
    uint8_t rx_buf[64];

    // UART1 초기화 대기
    ESP_LOGI("TEST", "Waiting for MDRobot UART initialization...");
    vTaskDelay(pdMS_TO_TICKS(5000));

    // ESP_LOGI(TAG, "Elec Brake ON...");
    // MdRobot_Set_PID_1Byte(10, 17); 
    // vTaskDelay(pdMS_TO_TICKS(1000));

    // 통신제어권 확보 (PID 78 -> 1)
    ESP_LOGI(TAG, "PID_UI_COM(78), data: 1");
    MdRobot_Set_PID_1Byte(78, 1); 
    vTaskDelay(pdMS_TO_TICKS(100));

    // 알람 리셋 (PID 12 -> 1)
    ESP_LOGI("TEST", "Resetting Alarm (PID 12)...");
    MdRobot_Set_PID_2Byte(12, 1);
    vTaskDelay(pdMS_TO_TICKS(500));

    // -------------------------------------------------------------
    // [속도 지령 추가] PID 207 (PID_PNT_VEL_CMD)
    // -------------------------------------------------------------    
    int16_t target_l = 1000;
    int16_t target_r = 1000;
    uint16_t slow_down = 500;
    uint8_t tx_buf[9];  // 송신용 (PID 214 데이터)
    ESP_LOGI("TEST", ">>> RPM Left: %d / Right: %d RPM / SD: %d (PID 214)", target_l, target_r, slow_down);

    // [PID 214 패킷 구성] - 9 Bytes
    tx_buf[0] = 1; // Motor 1 Enable
    tx_buf[1] = (uint8_t)(target_l & 0xFF);
    tx_buf[2] = (uint8_t)((target_l >> 8) & 0xFF);
    tx_buf[3] = 1; // Motor 2 Enable
    tx_buf[4] = (uint8_t)(target_r & 0xFF);
    tx_buf[5] = (uint8_t)((target_r >> 8) & 0xFF);
    tx_buf[6] = (uint8_t)(slow_down & 0xFF);
    tx_buf[7] = (uint8_t)((slow_down >> 8) & 0xFF);
    tx_buf[8] = 2; // Return Data Request PID_PNT_MAIN_DATA(2) -> PID 210 (Main Data) 요청

    // [함수 호출] 송신(9bytes)
    MdRobot_Set_PID_NByte(214, tx_buf, 9, NULL, 0);
    
    vTaskDelay(pdMS_TO_TICKS(500));

    // [nByte PID] PID 210 (PID_PNT_MAIN_DATA)        
    ret = MdRobot_Get_PID_NByte(210, rx_buf, 32);
    if (ret == ESP_OK) {        
        // [모터 1: Offset 0~8]
        mon.fb_rpm_l  = (int16_t)(rx_buf[0] | (rx_buf[1] << 8));
        mon.current_l = (int16_t)(rx_buf[2] | (rx_buf[3] << 8));
        mon.status_l  = rx_buf[4];
        // 위치: 4 Bytes (Little Endian)
        mon.pos_l     = (int32_t)(rx_buf[5] | (rx_buf[6] << 8) | (rx_buf[7] << 16) | (rx_buf[8] << 24));

        // [모터 2: Offset 9~17]
        mon.fb_rpm_r  = (int16_t)(rx_buf[9] | (rx_buf[10] << 8));
        mon.current_r = (int16_t)(rx_buf[11] | (rx_buf[12] << 8));
        mon.status_r  = rx_buf[13];
        // 위치: 4 Bytes (Little Endian)
        mon.pos_r     = (int32_t)(rx_buf[14] | (rx_buf[15] << 8) | (rx_buf[16] << 16) | (rx_buf[17] << 24));

        // 로그 출력
        ESP_LOGI("MON", "RPM: %d/%d, CUR: %.1f/%.1f A, POS: %ld/%ld", 
                    mon.fb_rpm_l, mon.fb_rpm_r, 
                    mon.current_l/10.0f, mon.current_r/10.0f,
                    mon.pos_l, mon.pos_r);

    } else {
        ESP_LOGW("MON", "PID 210 Read Failed");
    }




    vTaskDelay(pdMS_TO_TICKS(1000));

    // MD750T-V7.1-MCAR-E 엔코더 모드로 설정
    ESP_LOGI(TAG, "PID_UI_COM(78), data: 0");
    MdRobot_Set_PID_1Byte(78, 0); 
    vTaskDelay(pdMS_TO_TICKS(50));   

    ESP_LOGI(TAG, "PID_FUNC_CMD_TYPE(183), data: 15 (MCAR Mode)");
    MdRobot_Set_PID_2Byte(183, 15);
    vTaskDelay(pdMS_TO_TICKS(50));

    ESP_LOGI(TAG, "PID_ENC_PPR(156), data: 16384");
    MdRobot_Set_PID_2Byte(156, 16384); 
    vTaskDelay(pdMS_TO_TICKS(50));

    ESP_LOGI(TAG, "PID_USE_EPOSI(46), data: 1");
    MdRobot_Set_PID_1Byte(46, 1); 
    vTaskDelay(pdMS_TO_TICKS(50));

    while (1) {
        ESP_LOGI("RS485", "--- Checking PID Values ---");

        // [1Byte PID] PID 1 (Firmware Version)
        ret = MdRobot_Get_PID_1Byte(1, &val8);
        if (ret == ESP_OK) ESP_LOGI("RS485", "PID 1 (Firmware Version): %d", val8);
        else ESP_LOGE("RS485", "PID 1 Read Failed");
        vTaskDelay(pdMS_TO_TICKS(50));

        // [1Byte PID] PID 78 (UI Command)
        ret = MdRobot_Get_PID_1Byte(78, &val8);
        if (ret == ESP_OK) ESP_LOGI("RS485", "PID 78 (UI Command): %d", val8);
        else ESP_LOGE("RS485", "PID 78 Read Failed");
        vTaskDelay(pdMS_TO_TICKS(50));

        // [1Byte PID] PID 149 (PID_RETURN_TYPE)
        ret = MdRobot_Get_PID_1Byte(149, &val8);
        if (ret == ESP_OK) ESP_LOGI("RS485", "PID 149 (PID_RETURN_TYPE): %d (0x%02X)", val8, val8);
        else ESP_LOGE("RS485", "PID 149 Read Failed");
        vTaskDelay(pdMS_TO_TICKS(50));

        // [2Byte PID] PID 183 (Func Cmd Type)
        ret = MdRobot_Get_PID_2Byte(183, &val16);
        if (ret == ESP_OK) ESP_LOGI("RS485", "PID 183 (Func Cmd Type): %d (0x%04X)", val16, val16);
        else ESP_LOGE("RS485", "PID 183 Read Failed");
        vTaskDelay(pdMS_TO_TICKS(50));

        // [2Byte PID] PID 185 (PID_COM_WATCH_DELAY)
        ret = MdRobot_Get_PID_2Byte(185, &val16);
        if (ret == ESP_OK) ESP_LOGI("RS485", "PID 185 (PID_COM_WATCH_DELAY): %d (0x%04X)", val16, val16);
        else ESP_LOGE("RS485", "PID 185 Read Failed");
        vTaskDelay(pdMS_TO_TICKS(50));

        // [nByte PID] PID 205 (PID_TYPE)
        ret = MdRobot_Get_PID_NByte(205, (uint8_t*)rx_buf, 20);
        if (ret == ESP_OK) ESP_LOGI("RS485", "PID 205 (PID_TYPE): %s", rx_buf);
        else ESP_LOGE("RS485", "PID 205 Read Failed");
        vTaskDelay(pdMS_TO_TICKS(50));  
        
        // [nByte PID] PID 210 (PID_PNT_MAIN_DATA)        
        ret = MdRobot_Get_PID_NByte(210, rx_buf, 32);
        if (ret == ESP_OK) {
            
            // --- [18 Byte 구조 파싱] ---
            
            // [모터 1: Offset 0~8]
            mon.fb_rpm_l  = (int16_t)(rx_buf[0] | (rx_buf[1] << 8));
            mon.current_l = (int16_t)(rx_buf[2] | (rx_buf[3] << 8));
            mon.status_l  = rx_buf[4];
            // 위치: 4 Bytes (Little Endian)
            mon.pos_l     = (int32_t)(rx_buf[5] | (rx_buf[6] << 8) | (rx_buf[7] << 16) | (rx_buf[8] << 24));

            // [모터 2: Offset 9~17]
            mon.fb_rpm_r  = (int16_t)(rx_buf[9] | (rx_buf[10] << 8));
            mon.current_r = (int16_t)(rx_buf[11] | (rx_buf[12] << 8));
            mon.status_r  = rx_buf[13];
            // 위치: 4 Bytes (Little Endian)
            mon.pos_r     = (int32_t)(rx_buf[14] | (rx_buf[15] << 8) | (rx_buf[16] << 16) | (rx_buf[17] << 24));

            // 로그 출력
            ESP_LOGI("MON", "RPM: %d/%d, CUR: %.1f/%.1f A, POS: %ld/%ld", 
                     mon.fb_rpm_l, mon.fb_rpm_r, 
                     mon.current_l/10.0f, mon.current_r/10.0f,
                     mon.pos_l, mon.pos_r);

        } else {
            ESP_LOGW("MON", "PID 210 Read Failed");
        }

        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
#endif

void test_mdrobot_drive_task(void *arg) {
    esp_err_t ret_rx;
    uint8_t val8;
    uint16_t val16;
    uint8_t rx_buf[64];
    md_dual_monitor_t mon;

    // 1. UART 초기화 대기
    ESP_LOGI("RS485", "Waiting for UART1_MDROBOT - initialization...");
    vTaskDelay(pdMS_TO_TICKS(5000));

    // 통신제어권 확보 (PID 78 -> 1)
    ESP_LOGI("RS485", "PID_UI_COM(78), data: 1");
    MdRobot_Set_PID_1Byte(78, 1); 
    vTaskDelay(pdMS_TO_TICKS(100));

    // // 알람 리셋 (PID 12 -> 1)
    // ESP_LOGI("RS485", "Resetting Alarm (PID 12)...");
    // MdRobot_Set_PID_2Byte(12, 1);
    // vTaskDelay(pdMS_TO_TICKS(500));
    
    // Slow Start 설정 (PID 153)
    ESP_LOGI("RS485", "Setting Slow Start (PID 153)...");
    MdRobot_Set_PID_2Byte(153, 500);
    vTaskDelay(pdMS_TO_TICKS(500));

    if (MdRobot_Drive_Dual_With_SD(1000, 1000, 500) == ESP_OK) {
        ESP_LOGI("RS485", "MDRobot Dual Drive Command Sent: Left 1000 RPM, Right 1000 RPM, Slow Down 500");
    } else {
        ESP_LOGE("RS485", "MDRobot Dual Drive Command Failed");
    }

    vTaskDelay(pdMS_TO_TICKS(1500));

    // [모니터 데이터 읽기]
    ret_rx = MdRobot_Read_Monitor_Data(&mon);

    if (ret_rx == ESP_OK) {
        ESP_LOGI("RS485", "RPM: %d/%d | POS: %ld/%ld | CUR: %.1f/%.1f", 
                    mon.fb_rpm_l, mon.fb_rpm_r, 
                    mon.pos_l, mon.pos_r,
                    mon.current_l/10.0f, mon.current_r/10.0f);
    } else {
        ESP_LOGW("RS485", "Monitor Failed");
    }

    vTaskDelay(pdMS_TO_TICKS(500));

    if (MdRobot_Drive_Dual_With_SD(0, 0, 500) == ESP_OK) {
        ESP_LOGI("RS485", "MDRobot Dual Drive Command Sent: Left 0 RPM, Right 0 RPM, Slow Down 500");
    } else {
        ESP_LOGE("RS485", "MDRobot Dual Drive Command Failed");
    }

    vTaskDelay(pdMS_TO_TICKS(500));

    // [모니터 데이터 읽기]
    ret_rx = MdRobot_Read_Monitor_Data(&mon);

    if (ret_rx == ESP_OK) {
        ESP_LOGI("RS485", "RPM: %d/%d | POS: %ld/%ld | CUR: %.1f/%.1f", 
                    mon.fb_rpm_l, mon.fb_rpm_r, 
                    mon.pos_l, mon.pos_r,
                    mon.current_l/10.0f, mon.current_r/10.0f);
    } else {
        ESP_LOGW("RS485", "Monitor Failed");
    }

    vTaskDelay(pdMS_TO_TICKS(1000));

    // 2. 초기 설정 (MD750T-V7.1-MCAR-E 기준)
    ESP_LOGI("RS485", "Configuring MD750T to MCAR mode...");
    MdRobot_Set_MCAR_Mode();
    // [nByte PID] PID 205 (PID_TYPE)
    ret_rx = MdRobot_Get_PID_NByte(205, (uint8_t*)rx_buf, 20);
    if (ret_rx == ESP_OK) ESP_LOGI("RS485", "PID 205 (PID_TYPE): %s", rx_buf);
    else ESP_LOGE("RS485", "PID 205 Read Failed");
    vTaskDelay(pdMS_TO_TICKS(50));  

    while (1) {
        ESP_LOGI("RS485", "--- Checking PID Values ---");

        // [1Byte PID] PID 1 (Firmware Version)
        ret_rx = MdRobot_Get_PID_1Byte(1, &val8);
        if (ret_rx == ESP_OK) ESP_LOGI("RS485", "PID 1 (Firmware Version): %d", val8);
        else ESP_LOGE("RS485", "PID 1 Read Failed");
        vTaskDelay(pdMS_TO_TICKS(50));

        // [1Byte PID] PID 78 (UI Command)
        ret_rx = MdRobot_Get_PID_1Byte(78, &val8);
        if (ret_rx == ESP_OK) ESP_LOGI("RS485", "PID 78 (UI Command): %d", val8);
        else ESP_LOGE("RS485", "PID 78 Read Failed");
        vTaskDelay(pdMS_TO_TICKS(50));

        // [1Byte PID] PID 149 (PID_RETURN_TYPE)
        ret_rx = MdRobot_Get_PID_1Byte(149, &val8);
        if (ret_rx == ESP_OK) ESP_LOGI("RS485", "PID 149 (PID_RETURN_TYPE): %d (0x%02X)", val8, val8);
        else ESP_LOGE("RS485", "PID 149 Read Failed");
        vTaskDelay(pdMS_TO_TICKS(50));

        // [2Byte PID] PID 183 (Func Cmd Type)
        ret_rx = MdRobot_Get_PID_2Byte(183, &val16);
        if (ret_rx == ESP_OK) ESP_LOGI("RS485", "PID 183 (Func Cmd Type): %d (0x%04X)", val16, val16);
        else ESP_LOGE("RS485", "PID 183 Read Failed");
        vTaskDelay(pdMS_TO_TICKS(50));

        // [2Byte PID] PID 185 (PID_COM_WATCH_DELAY)
        ret_rx = MdRobot_Get_PID_2Byte(185, &val16);
        if (ret_rx == ESP_OK) ESP_LOGI("RS485", "PID 185 (PID_COM_WATCH_DELAY): %d (0x%04X)", val16, val16);
        else ESP_LOGE("RS485", "PID 185 Read Failed");
        vTaskDelay(pdMS_TO_TICKS(50));

        // [nByte PID] PID 205 (PID_TYPE)
        ret_rx = MdRobot_Get_PID_NByte(205, (uint8_t*)rx_buf, 20);
        if (ret_rx == ESP_OK) ESP_LOGI("RS485", "PID 205 (PID_TYPE): %s", rx_buf);
        else ESP_LOGE("RS485", "PID 205 Read Failed");
        vTaskDelay(pdMS_TO_TICKS(50));

        // [모니터 데이터 읽기]
        esp_err_t ret_rx = MdRobot_Read_Monitor_Data(&mon);

        if (ret_rx == ESP_OK) {
            ESP_LOGI("RS485", "RPM: %d/%d | POS: %ld/%ld | CUR: %.1f/%.1f", 
                     mon.fb_rpm_l, mon.fb_rpm_r, 
                     mon.pos_l, mon.pos_r,
                     mon.current_l/10.0f, mon.current_r/10.0f);
        } else {
            ESP_LOGW("RS485", "Monitor Failed");
        }

        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set(TAG, ESP_LOG_DEBUG);

    // 큐 생성 (큐 길이: 10, 아이템 크기: RosCommand_t 구조체 크기)
    g_ros_cmd_queue = xQueueCreate(10, sizeof(RosCommand_t));
    if (g_ros_cmd_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create command queue. Halting.");
        while(1); // 시스템 정지
    }

    ESP_ERROR_CHECK(uros_network_interface_initialize());

    i2c_bus_manager_init();
    ADS1115_Init();
    MCP4728_Init();

    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    PCF8574_Init();
    pslh080_init();

    MdRobot_Init();

    xTaskCreate(micro_ros_task, "micro_ros_task", CONFIG_MICRO_ROS_APP_STACK, NULL, CONFIG_MICRO_ROS_APP_TASK_PRIO, NULL);
    xTaskCreatePinnedToCore(md750t_ctrl_task, "md750t_ctrl_task", 1024 * 4, NULL, 4, NULL, CPU_NUM_1);
    xTaskCreatePinnedToCore(linear_scale_task, "linear_scale_task", 4096, NULL, 3, NULL, CPU_NUM_1);
    xTaskCreatePinnedToCore(hall_sensor_task, "hall_sensor_task", 4096, NULL, 2, NULL, CPU_NUM_1);

    xTaskCreatePinnedToCore(test_mdrobot_drive_task, "test_mdrobot_drive_task", 4096, NULL, 1, NULL, CPU_NUM_1);

    ESP_LOGI(TAG, "TMCart Control Firmware Version: %s \n\n", FIRMWARE_VERSION);
}