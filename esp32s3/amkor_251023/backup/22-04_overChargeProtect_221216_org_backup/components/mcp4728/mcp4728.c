#include "mcp4728.h"
// *** 수정: i2c_bus_manager.h 포함 ***
#include "i2c_bus_manager.h"
#include "esp_log.h"

static const char *TAG = "MCP4728";

void MCP4728_Init(void)
{
    ESP_LOGI(TAG, "MCP4728 component initialized.");
}

// *** 수정: i2c_bus_manager 함수 사용 ***
esp_err_t MCP4728_SetVoltage(mcp4728_channel_t channel, float voltage, bool store_eeprom)
{
    if (voltage < 0.0f || voltage > MCP4728_VREF) {
        ESP_LOGE(TAG, "Voltage out of range (0.0 - %.2fV)", MCP4728_VREF);
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t dac_value = (uint16_t)((voltage / MCP4728_VREF) * 4095.0f);
    if (dac_value > 4095) dac_value = 4095;

    uint8_t buffer[3];

    if (store_eeprom) {
        // Single Write with EEPROM
        buffer[0] = 0b01011000 | (channel << 1);
    } else {
        // Fast Write (no EEPROM)
        buffer[0] = 0b01000000 | (channel << 1);
    }

    buffer[1] = (uint8_t)(dac_value >> 8); 
    buffer[2] = (uint8_t)(dac_value & 0xFF);

    // MCP4728 write does not use a register address.
    return i2c_bus_manager_write_no_reg(MCP4728_I2C_ADDR, buffer, 3);
}

// 내부 Vref 및 2x Gain을 사용하여 VDD와 무관하게 전압 설정
esp_err_t MCP4728_SetVoltage_InternalVref(mcp4728_channel_t channel, float voltage, bool store_eeprom)
{
    // 새로운 최대 출력 전압은 4.096V 입니다.
    const float VREF_FULL_SCALE = 4.096f; 

    // 1. 전압 범위 확인 수정
    // if (voltage < 0.0f || voltage > VREF_FULL_SCALE) {
    //     // VDD가 4.096V 보다 낮으면 출력이 VDD에서 제한될 수 있음을 경고
    //     if (voltage < MCP4728_VDD_SUPPLY) { // MCP4728_VDD_SUPPLY는 실제 VDD 값 (예: 5.0f)
    //          ESP_LOGW(TAG, "Requested voltage (%.3fV) is valid, but output will be clipped by VDD (%.2fV)", voltage, MCP4728_VDD_SUPPLY);
    //     } else {
    //          ESP_LOGE(TAG, "Voltage out of range (0.0 - %.3fV)", VREF_FULL_SCALE);
    //          return ESP_ERR_INVALID_ARG;
    //     }
    // }

    // 2. 전압 계산식 수정
    uint16_t dac_value = (uint16_t)((voltage / VREF_FULL_SCALE) * 4095.0f);
    if (dac_value > 4095) dac_value = 4095;

    uint8_t buffer[3];

    // 커맨드 바이트 설정 (기존과 동일)
    if (store_eeprom) {
        buffer[0] = 0b01011000 | (channel << 1);
    } else {
        buffer[0] = 0b01000000 | (channel << 1);
    }

    // ✨ 3. 데이터 바이트 수정 (Vref, Gain 설정 포함)
    // VREF=1(Internal), PD=00(Normal), G=1(x2) => 0b1001....
    buffer[1] = 0b10010000 | (uint8_t)(dac_value >> 8); 
    buffer[2] = (uint8_t)(dac_value & 0xFF);

    return i2c_bus_manager_write_no_reg(MCP4728_I2C_ADDR, buffer, 3);
}