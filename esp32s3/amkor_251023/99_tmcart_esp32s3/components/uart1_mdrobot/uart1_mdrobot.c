#include "uart1_mdrobot.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/uart.h"

static const char *TAG = "MD_UART";
#define BUF_SIZE (256)

// 체크섬 계산 (sum의 1의 보수 + 1)
static uint8_t MdRobot_CalcChecksum(uint8_t *pkt, uint16_t len) {
    uint8_t sum = 0;
    for (int i = 0; i < len; i++) {
        sum += pkt[i];
    }
    return (~sum) + 1;
}

void MdRobot_Init(void) {
    const uart_config_t uart_config = {
        .baud_rate = MD_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, UART1_GPIO_TXD, UART1_GPIO_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_set_mode(UART_NUM_1, UART_MODE_RS485_HALF_DUPLEX));
    ESP_LOGI(TAG, "MdRobot UART Initialized (Baud: %d)", MD_BAUD_RATE);
}

// 공통 요청 및 파싱 함수 (기존 로직 유지)
static esp_err_t MdRobot_Request_Common(uint8_t target_pid, uint8_t *rx_buf, uint8_t max_buf_len) {
    // 1. 버퍼 비우기 (플러시)
    uart_flush_input(UART_NUM_1);
    memset(rx_buf, 0, max_buf_len); 

    // 2. 요청 패킷 전송 (항상 7바이트: PID 4번 요청)
    uint8_t req[7];
    req[0] = MD_RMID; req[1] = MD_TMID; req[2] = MD_ID_DEFAULT;
    req[3] = PID_REQ_PID_DATA; // 4: Data Request
    req[4] = 1;                // Len: 1
    req[5] = target_pid;       // Data: Target PID
    req[6] = MdRobot_CalcChecksum(req, 6);

    uart_write_bytes(UART_NUM_1, (const char*)req, 7);

    // 3. 응답 수신
    // 넉넉한 임시 버퍼에 한 번에 읽어옵니다. (타임아웃 100ms)
    uint8_t temp_buf[BUF_SIZE];
    int rx_len = uart_read_bytes(UART_NUM_1, temp_buf, BUF_SIZE, pdMS_TO_TICKS(100));

    // 최소 헤더 길이(5) + 체크섬(1) = 6바이트 미만이면 실패
    if (rx_len < 6) { 
        return ESP_ERR_TIMEOUT; 
    }

    // 4. 헤더 찾기 (TMID: 172, RMID: 183)
    int idx = -1;
    for (int i = 0; i <= rx_len - 6; i++) {
        if (temp_buf[i] == MD_TMID && temp_buf[i+1] == MD_RMID) {
            idx = i;
            break;
        }
    }

    if (idx == -1) {
        // ESP_LOGW(TAG, "Header not found");
        return ESP_FAIL;
    }

    // 5. 패킷 필드 파싱
    // [0]TMID, [1]RMID, [2]ID, [3]PID, [4]DataNum, [5...]Data, [Last]Checksum
    uint8_t recv_pid = temp_buf[idx + 3];
    uint8_t data_num = temp_buf[idx + 4]; // [중요] 수신된 데이터 길이 (예: 18)

    // PID 일치 확인
    if (recv_pid != target_pid) {
        // ESP_LOGW(TAG, "PID Mismatch. Req: %d, Recv: %d", target_pid, recv_pid);
        return ESP_FAIL;
    }

    // 6. 전체 패킷 길이 검증
    // 전체 길이 = 헤더(5) + 데이터(data_num) + 체크섬(1)
    int packet_total_len = 5 + data_num + 1;

    // 수신된 바이트가 예상 패킷 길이보다 짧으면 데이터가 잘린 것임
    if (rx_len < (idx + packet_total_len)) {
        ESP_LOGW(TAG, "Incomplete Packet. DataNum: %d, TotalRecv: %d", data_num, rx_len);
        return ESP_FAIL;
    }

    // 7. 체크섬 검증
    // 체크섬은 패킷의 마지막 바이트에 위치합니다.
    // 계산 범위: 헤더부터 데이터 끝까지 (체크섬 제외)
    uint8_t recv_checksum = temp_buf[idx + packet_total_len - 1];
    uint8_t calc_checksum = MdRobot_CalcChecksum(&temp_buf[idx], packet_total_len - 1);

    if (recv_checksum != calc_checksum) {
        ESP_LOGW(TAG, "Checksum Error. Recv: 0x%02X, Calc: 0x%02X", recv_checksum, calc_checksum);
        return ESP_FAIL;
    }

    // 8. 데이터 복사 (Buffer Overflow 방지)
    // 실제 데이터 길이(data_num)와 유저 버퍼 크기(max_buf_len) 중 작은 값만큼 복사
    uint8_t copy_len = (data_num < max_buf_len) ? data_num : max_buf_len;
    
    memcpy(rx_buf, &temp_buf[idx + 5], copy_len);

    return ESP_OK;
}

// --- 1Byte / 2Byte Wrappers ---
esp_err_t MdRobot_Get_PID_1Byte(uint8_t pid, uint8_t *value) {
    return MdRobot_Request_Common(pid, value, 1);
}

esp_err_t MdRobot_Set_PID_1Byte(uint8_t pid, uint8_t value) {
    uint8_t pkt[7];
    pkt[0] = MD_RMID; pkt[1] = MD_TMID; pkt[2] = MD_ID_DEFAULT;
    pkt[3] = pid; pkt[4] = 1; pkt[5] = value;
    pkt[6] = MdRobot_CalcChecksum(pkt, 6);
    uart_write_bytes(UART_NUM_1, (const char*)pkt, 7);
    vTaskDelay(pdMS_TO_TICKS(5)); // 쓰기 안정화 대기
    return ESP_OK;
}

esp_err_t MdRobot_Get_PID_2Byte(uint8_t pid, uint16_t *value) {
    uint8_t buf[2];
    esp_err_t ret = MdRobot_Request_Common(pid, buf, 2);
    if (ret == ESP_OK) *value = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
    return ret;
}

esp_err_t MdRobot_Set_PID_2Byte(uint8_t pid, uint16_t value) {
    uint8_t pkt[8];
    pkt[0] = MD_RMID; pkt[1] = MD_TMID; pkt[2] = MD_ID_DEFAULT;
    pkt[3] = pid; pkt[4] = 2;
    pkt[5] = (uint8_t)(value & 0xFF);
    pkt[6] = (uint8_t)((value >> 8) & 0xFF);
    pkt[7] = MdRobot_CalcChecksum(pkt, 7);
    uart_write_bytes(UART_NUM_1, (const char*)pkt, 8);
    vTaskDelay(pdMS_TO_TICKS(5));
    return ESP_OK;
}

// --- [New] N-Byte Functions ---

// N-Byte 읽기 (모니터링용)
esp_err_t MdRobot_Get_PID_NByte(uint8_t pid, uint8_t *data_buf, uint8_t expected_len) {
    return MdRobot_Request_Common(pid, data_buf, expected_len);
}

// // N-Byte 쓰기 및 응답 수신 (PID 검증 없음, Pass-Through)
// esp_err_t MdRobot_Set_PID_NByte(uint8_t pid, uint8_t *tx_data, uint8_t tx_len, uint8_t *rx_buf, uint8_t max_rx_len) {
//     if (tx_len > 200) return ESP_ERR_INVALID_ARG;

//     // 1. 송신 버퍼 비우기
//     uart_flush_input(UART_NUM_1);

//     // 2. 패킷 생성 (헤더5 + 데이터 + 체크섬1)
//     uint8_t pkt[256]; 
//     pkt[0] = MD_RMID;
//     pkt[1] = MD_TMID;
//     pkt[2] = MD_ID_DEFAULT;
//     pkt[3] = pid;
//     pkt[4] = tx_len;
    
//     memcpy(&pkt[5], tx_data, tx_len);
//     pkt[5 + tx_len] = MdRobot_CalcChecksum(pkt, 5 + tx_len);

//     // 3. 전송
//     uart_write_bytes(UART_NUM_1, (const char*)pkt, 6 + tx_len);

//     // 4. 수신 버퍼가 없다면 여기서 종료 (Write Only)
//     if (rx_buf == NULL || max_rx_len == 0) {
//         vTaskDelay(pdMS_TO_TICKS(5)); 
//         return ESP_OK;
//     }

//     // ============================================================
//     // 5. 응답 수신 로직 (PID 검증 없이 Pass-Through)
//     // ============================================================
    
//     uint8_t temp_buf[BUF_SIZE];
//     // 응답 대기
//     int rx_len = uart_read_bytes(UART_NUM_1, temp_buf, BUF_SIZE, pdMS_TO_TICKS(100));

//     if (rx_len < 6) return ESP_ERR_TIMEOUT;

//     // 헤더 찾기
//     int idx = -1;
//     for (int i = 0; i <= rx_len - 6; i++) {
//         if (temp_buf[i] == MD_TMID && temp_buf[i+1] == MD_RMID) {
//             idx = i;
//             break;
//         }
//     }
//     if (idx == -1) return ESP_FAIL;

//     // [변경] PID 검증 로직 제거됨 
//     // 수신된 PID가 요청 PID와 달라도(예: 214 요청 -> 210 응답) 데이터가 유효하면 통과시킵니다.
//     // 사용자가 필요하다면 rx_buf 등에서 확인할 수 있습니다. (현재 rx_buf에는 데이터만 담김)
    
//     // 6. 데이터 파싱
//     uint8_t recv_data_len = temp_buf[idx + 4];
//     int packet_total_len = 5 + recv_data_len + 1;

//     // 길이 검증
//     if (rx_len < (idx + packet_total_len)) {
//         ESP_LOGW(TAG, "Incomplete Response. Len: %d, Recv: %d", recv_data_len, rx_len);
//         return ESP_FAIL;
//     }

//     // 체크섬 검증
//     uint8_t calc_sum = MdRobot_CalcChecksum(&temp_buf[idx], packet_total_len - 1);
//     if (temp_buf[idx + packet_total_len - 1] != calc_sum) {
//         ESP_LOGW(TAG, "Checksum Error in Response");
//         return ESP_FAIL;
//     }

//     // 데이터 복사 (PID와 상관없이 데이터 내용물만 복사)
//     uint8_t copy_len = (recv_data_len < max_rx_len) ? recv_data_len : max_rx_len;
//     memcpy(rx_buf, &temp_buf[idx + 5], copy_len);

//     return ESP_OK;
// }

// [디버깅용 수정] MdRobot_Set_PID_NByte
esp_err_t MdRobot_Set_PID_NByte(uint8_t pid, uint8_t *tx_data, uint8_t tx_len, uint8_t *rx_buf, uint8_t max_rx_len) {
    if (tx_len > 200) return ESP_ERR_INVALID_ARG;

    uart_flush_input(UART_NUM_1);

    // 패킷 생성 및 전송 (동일)
    uint8_t pkt[256]; 
    pkt[0] = MD_RMID; pkt[1] = MD_TMID; pkt[2] = MD_ID_DEFAULT; pkt[3] = pid; pkt[4] = tx_len;
    memcpy(&pkt[5], tx_data, tx_len);
    pkt[5 + tx_len] = MdRobot_CalcChecksum(pkt, 5 + tx_len);
    uart_write_bytes(UART_NUM_1, (const char*)pkt, 6 + tx_len);

    if (rx_buf == NULL || max_rx_len == 0) {
        vTaskDelay(pdMS_TO_TICKS(5)); 
        return ESP_OK;
    }

    // [디버깅 1] 타임아웃 확인 (타임아웃 200ms로 증가)
    uint8_t temp_buf[BUF_SIZE];
    int rx_len = uart_read_bytes(UART_NUM_1, temp_buf, BUF_SIZE, pdMS_TO_TICKS(200)); 

    if (rx_len < 6) {
        ESP_LOGE(TAG, "Timeout or Too Short: rx_len=%d", rx_len);
        return ESP_ERR_TIMEOUT;
    }

    // [디버깅 2] 수신 데이터 덤프 (매우 중요)
    ESP_LOGI(TAG, "RX Dump (%d bytes):", rx_len);
    ESP_LOG_BUFFER_HEX(TAG, temp_buf, rx_len);

    int idx = -1;
    for (int i = 0; i <= rx_len - 6; i++) {
        if (temp_buf[i] == MD_TMID && temp_buf[i+1] == MD_RMID) {
            idx = i;
            break;
        }
    }
    if (idx == -1) {
        ESP_LOGE(TAG, "Header Not Found");
        return ESP_FAIL;
    }

    // 파싱
    uint8_t recv_data_len = temp_buf[idx + 4];
    int packet_total_len = 5 + recv_data_len + 1;

    // [디버깅 3] 길이 검증
    if (rx_len < (idx + packet_total_len)) {
        ESP_LOGE(TAG, "Length Mismatch: Expected=%d, Actual=%d, DataNum=%d", 
                 packet_total_len, rx_len - idx, recv_data_len);
        return ESP_FAIL;
    }

    // 체크섬 검증
    uint8_t calc_sum = MdRobot_CalcChecksum(&temp_buf[idx], packet_total_len - 1);
    if (temp_buf[idx + packet_total_len - 1] != calc_sum) {
        ESP_LOGE(TAG, "Checksum Fail: Recv=0x%02X, Calc=0x%02X", 
                 temp_buf[idx + packet_total_len - 1], calc_sum);
        return ESP_FAIL;
    }

    // 데이터 복사
    uint8_t copy_len = (recv_data_len < max_rx_len) ? recv_data_len : max_rx_len;
    memcpy(rx_buf, &temp_buf[idx + 5], copy_len);

    return ESP_OK;
}

// =========================================================
// [NEW] 고수준 제어 함수 구현
// =========================================================

// MCAR 모드 설정 (PID 183)
esp_err_t MdRobot_Set_MCAR_Mode(void) {
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

    return ESP_OK;
}

// 듀얼 모터 속도 지령 (PID 214) - 8 Bytes, Write Only
esp_err_t MdRobot_Drive_Dual_With_SD(int16_t rpm_l, int16_t rpm_r, uint16_t slow_down) {
    uint8_t tx_buf[9];

    // 패킷 구성 (Little Endian)
    tx_buf[0] = 1; // Left Enable
    tx_buf[1] = (uint8_t)(rpm_l & 0xFF);
    tx_buf[2] = (uint8_t)((rpm_l >> 8) & 0xFF);
    tx_buf[3] = 1; // Right Enable
    tx_buf[4] = (uint8_t)(rpm_r & 0xFF);
    tx_buf[5] = (uint8_t)((rpm_r >> 8) & 0xFF);
    tx_buf[6] = (uint8_t)(slow_down & 0xFF);
    tx_buf[7] = (uint8_t)((slow_down >> 8) & 0xFF);
    tx_buf[8] = 2; // Return Data Request PID_PNT_MAIN_DATA(2)
    

    // 수신 버퍼 NULL -> 송신만 수행
    return MdRobot_Set_PID_NByte(214, tx_buf, 9, NULL, 0);
}

// 모니터링 데이터 읽기 (PID 210)
esp_err_t MdRobot_Read_Monitor_Data(md_dual_monitor_t *out_data) {
    uint8_t rx_buf[64];
    
    // PID 210 요청 (예상 응답 길이: 18 Bytes 이상 -> 32버퍼 사용)
    esp_err_t ret = MdRobot_Get_PID_NByte(210, rx_buf, 32);
    
    if (ret == ESP_OK) {
        // 데이터 파싱 (18 Bytes 구조)
        // [Left Motor]
        out_data->fb_rpm_l  = (int16_t)(rx_buf[0] | (rx_buf[1] << 8));
        out_data->current_l = (int16_t)(rx_buf[2] | (rx_buf[3] << 8));
        out_data->status_l  = rx_buf[4];
        out_data->pos_l     = (int32_t)(rx_buf[5] | (rx_buf[6] << 8) | (rx_buf[7] << 16) | (rx_buf[8] << 24));

        // [Right Motor]
        out_data->fb_rpm_r  = (int16_t)(rx_buf[9] | (rx_buf[10] << 8));
        out_data->current_r = (int16_t)(rx_buf[11] | (rx_buf[12] << 8));
        out_data->status_r  = rx_buf[13];
        out_data->pos_r     = (int32_t)(rx_buf[14] | (rx_buf[15] << 8) | (rx_buf[16] << 16) | (rx_buf[17] << 24));
    }
    
    return ret;
}