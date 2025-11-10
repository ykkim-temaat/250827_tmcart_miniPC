#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"
#include <stdbool.h>

#define MCP4728_I2C_ADDR 0x60
#define MCP4728_VREF 5.0f // Assuming VDD is 5.0V and is used as Vref
#define MCP4728_VDD_SUPPLY 5.0f // Actual VDD value (e.g., 5.0f)

typedef enum {
    MCP4728_CHANNEL_A = 0, // D0
    MCP4728_CHANNEL_B = 1, // D1
    MCP4728_CHANNEL_C = 2, // D2
    MCP4728_CHANNEL_D = 3, // D3
} mcp4728_channel_t;

/**
 * @brief Initializes the MCP4728 component.
 */
void MCP4728_Init(void);

/**
 * @brief Sets the output voltage for a specific DAC channel.
 *
 * @param channel The DAC channel to set (A, B, C, or D).
 * @param voltage The desired output voltage (0.0 to VREF).
 * @param store_eeprom If true, the value is stored in the device's EEPROM.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t MCP4728_SetVoltage(mcp4728_channel_t channel, float voltage, bool store_eeprom);
esp_err_t MCP4728_SetVoltage_InternalVref(mcp4728_channel_t channel, float voltage, bool store_eeprom);


#ifdef __cplusplus
}
#endif
