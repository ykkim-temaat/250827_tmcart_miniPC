#include "uart1_mdrobot.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/uart.h"

static const char *TAG = "MD_UART";
#define BUF_SIZE (256)

// [중요] 데이터 요청용 PID 정의
#define PID_REQ_PID_DATA         4  

// 체크섬 계산 함수 (Sum의 1의 보수 + 1)
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

    // UART 드라이버 설치
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart_config));
    
    // 핀 설정
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, UART1_GPIO_TXD, UART1_GPIO_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    
    // RS485 모드 설정
    ESP_ERROR_CHECK(uart_set_mode(UART_NUM_1, UART_MODE_RS485_HALF_DUPLEX));
    
    ESP_LOGI(TAG, "MdRobot UART Initialized (Baud: %d)", MD_BAUD_RATE);
}

// 공통 요청 및 파싱 함수
// pid: 읽고자 하는 타겟 PID 번호 (예: 17, 130)
static esp_err_t MdRobot_Request_Common(uint8_t target_pid, uint8_t *rx_buf, uint8_t expected_data_len) {
    // 1. 버퍼 비우기
    uart_flush_input(UART_NUM_1);

    // 2. 요청 패킷 생성 (수정된 부분)
    // 구조: [RMID] [TMID] [ID] [PID=4] [Len=1] [Data=TargetPID] [Chk]
    uint8_t req[7];
    req[0] = MD_RMID;          // 183
    req[1] = MD_TMID;          // 172
    req[2] = MD_ID_DEFAULT;    // 1
    
    req[3] = PID_REQ_PID_DATA; // [중요] 4번 명령 (데이터 요청)
    req[4] = 1;                // 데이터 개수 1개 (PID 번호를 보내야 하므로)
    req[5] = target_pid;       // [중요] 실제 읽고 싶은 PID 번호를 데이터로 실어 보냄
    
    req[6] = MdRobot_CalcChecksum(req, 6); // 체크섬 계산

    // 3. 전송 (총 7바이트)
    uart_write_bytes(UART_NUM_1, (const char*)req, 7);

    // ---------------------------------------------------------
    // 4. 응답 대기
    // 드라이버는 요청받은 PID(target_pid)를 담아 응답합니다.
    // 예상 응답: [TMID] [RMID] [ID] [TargetPID] [Len] [Data...] [Chk]
    // ---------------------------------------------------------
    
    int expected_packet_len = 6 + expected_data_len;
    uint8_t temp_buf[BUF_SIZE];
    
    // 타임아웃 100ms
    int rx_len = uart_read_bytes(UART_NUM_1, temp_buf, BUF_SIZE, pdMS_TO_TICKS(100));

    if (rx_len < expected_packet_len) {
        // 일부라도 들어왔다면 로그 출력 (디버깅용)
        if(rx_len > 0) {
             ESP_LOGW(TAG, "Partial/Invalid Data: %d bytes (Expected: %d)", rx_len, expected_packet_len);
             ESP_LOG_BUFFER_HEX(TAG, temp_buf, rx_len);
        }
        return ESP_ERR_TIMEOUT;
    }

    // 5. 파싱 (헤더 찾기: 172, 183)
    int idx = -1;
    for (int i = 0; i <= rx_len - expected_packet_len; i++) {
        if (temp_buf[i] == MD_TMID && temp_buf[i+1] == MD_RMID) {
            idx = i;
            break;
        }
    }

    if (idx == -1) {
        ESP_LOGW(TAG, "Header (172, 183) not found");
        ESP_LOG_BUFFER_HEX(TAG, temp_buf, rx_len);
        return ESP_FAIL;
    }

    // 응답 패킷의 PID 확인
    // 요청은 4번으로 보냈지만, 응답은 실제 데이터 PID(target_pid)로 옵니다.
    uint8_t recv_pid = temp_buf[idx + 3];
    
    if (recv_pid != target_pid) {
        // 혹시 드라이버가 에러 응답이나 다른 패킷을 보냈는지 확인
        ESP_LOGW(TAG, "PID Mismatch. Req: %d, Recv: %d", target_pid, recv_pid);
        ESP_LOG_BUFFER_HEX(TAG, &temp_buf[idx], expected_packet_len);
        return ESP_FAIL;
    }

    // 데이터 복사
    memcpy(rx_buf, &temp_buf[idx + 5], expected_data_len);

    return ESP_OK;
}

// 1 Byte 읽기 인터페이스
esp_err_t MdRobot_Get_PID_1Byte(uint8_t pid, uint8_t *value) {
    uint8_t buf[1];
    esp_err_t ret = MdRobot_Request_Common(pid, buf, 1);
    if (ret == ESP_OK) {
        *value = buf[0];
    }
    return ret;
}

// 2 Byte 읽기 인터페이스
esp_err_t MdRobot_Get_PID_2Byte(uint8_t pid, uint16_t *value) {
    uint8_t buf[2];
    esp_err_t ret = MdRobot_Request_Common(pid, buf, 2);
    if (ret == ESP_OK) {
        // Little Endian: Low Byte First
        *value = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
    }
    return ret;
}