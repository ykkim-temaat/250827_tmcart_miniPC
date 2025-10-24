#include "pcf8574.h"
#include "i2c_bus_manager.h"
#include "esp_log.h"

static const char *TAG = "PCF8574";

void PCF8574_Init(void)
{

    pcf8574_write_byte(PCF8574_ADDR_0x20, 0xFF); // Set all pins to high
    pcf8574_write_byte(PCF8574_ADDR_0x21, 0xFF);
    pcf8574_write_byte(PCF8574_ADDR_0x22, 0xFF);
    pcf8574_write_byte(PCF8574_ADDR_0x23, 0xFF);
    pcf8574_write_byte(PCF8574_ADDR_0x24, 0xFF);
    pcf8574_write_byte(PCF8574_ADDR_0x25, 0xFF);
    pcf8574_write_byte(PCF8574_ADDR_0x26, 0xFF);
    pcf8574_write_byte(PCF8574_ADDR_0x27, 0xFF);

    ESP_LOGI(TAG, "PCF8574 devices initialized.");
}

esp_err_t pcf8574_write_byte(uint8_t device_address, uint8_t data)
{
    return i2c_bus_manager_write_no_reg(device_address, &data, 1);
}

esp_err_t pcf8574_read_byte(uint8_t device_address, uint8_t *data)
{
    return i2c_bus_manager_read_no_reg(device_address, data, 1);
}
