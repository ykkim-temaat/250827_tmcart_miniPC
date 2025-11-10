#include "ads1115.h"
// *** 수정: i2c_bus_manager.h 포함 ***
#include "i2c_bus_manager.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rom/ets_sys.h" // esp_rom_delay_us를 위해 포함

static const char *TAG = "ADS1115";

// Register Pointers
#define ADS1115_REG_POINTER_CONVERT (0x00)
#define ADS1115_REG_POINTER_CONFIG  (0x01)

void ADS1115_Init(void)
{
    ESP_LOGI(TAG, "ADS1115 component initialized.");
}

// *** 수정: i2c_bus_manager 함수 사용 ***
esp_err_t ADS1115_ReadVoltage(ads1115_mux_t channel, float *voltage)
{
    uint8_t config[2];
    config[0] = 0b10000000 | (channel << 4) | 0b00000011;   // fullscale 4.096V
    config[1] = 0b10000011;

    esp_err_t err = i2c_bus_manager_write(ADS1115_I2C_ADDR, ADS1115_REG_POINTER_CONFIG, config, 2);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write config to ADS1115, err: %d", err);
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(10));
    
    // // vTaskDelay 대신 esp_rom_delay_us 사용. 10ms = 10000us
    // // 이 함수는 태스크를 잠재우지 않고 CPU를 사용하면서 대기합니다.
    // esp_rom_delay_us(10000); //

    uint8_t read_data[2];
    err = i2c_bus_manager_read(ADS1115_I2C_ADDR, ADS1115_REG_POINTER_CONVERT, read_data, 2);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read conversion result, err: %d", err);
        return err;
    }

    int16_t raw_adc = (read_data[0] << 8) | read_data[1];
    *voltage = (float)raw_adc * 4.096f / 32767.0f;

    return ESP_OK;
}
