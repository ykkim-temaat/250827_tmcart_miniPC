#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"

#define ADS1115_I2C_ADDR 0x48

// Multiplexer configuration for single-ended inputs
typedef enum {
    ADS1115_MUX_AIN0_GND = 0b100,
    ADS1115_MUX_AIN1_GND = 0b101,
    ADS1115_MUX_AIN2_GND = 0b110,
    ADS1115_MUX_AIN3_GND = 0b111,
} ads1115_mux_t;

/**
 * @brief Initializes the ADS1115 component.
 */
void ADS1115_Init(void);

/**
 * @brief Reads a voltage from a specified single-ended channel.
 *
 * @param channel The input channel to read from.
 * @param voltage Pointer to a float to store the resulting voltage.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ADS1115_ReadVoltage(ads1115_mux_t channel, float *voltage);

#ifdef __cplusplus
}
#endif
