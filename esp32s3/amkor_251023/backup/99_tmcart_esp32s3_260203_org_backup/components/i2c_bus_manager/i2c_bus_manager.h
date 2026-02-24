#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "esp_err.h"
#include "driver/i2c.h" // For I2C_NUM_0

// I2C master parameters
#define I2C_BUS_MANAGER_SCL_GPIO      5
#define I2C_BUS_MANAGER_SDA_GPIO      4
#define I2C_BUS_MANAGER_PORT_NUM      I2C_NUM_0
#define I2C_BUS_MANAGER_FREQ_HZ       400000
#define I2C_BUS_MANAGER_TIMEOUT_MS    100

/**
 * @brief Initializes the I2C bus and the mutex for thread-safe access.
 * This function must be called once before any other I2C operations.
 */
void i2c_bus_manager_init(void);

/**
 * @brief Deinitializes the I2C bus and deletes the mutex.
 */
void i2c_bus_manager_deinit(void);

/**
 * @brief Thread-safe function to read multiple bytes from an I2C device with a register address.
 *
 * @param addr I2C device address.
 * @param reg Register address to read from.
 * @param data Pointer to the buffer to store read data.
 * @param len Number of bytes to read.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_bus_manager_read(uint8_t addr, uint8_t reg, uint8_t *data, size_t len);

/**
 * @brief Thread-safe function to write multiple bytes to an I2C device with a register address.
 *
 * @param addr I2C device address.
 * @param reg Register address to write to.
 * @param data Pointer to the data to write.
 * @param len Number of bytes to write.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_bus_manager_write(uint8_t addr, uint8_t reg, const uint8_t *data, size_t len);

/**
 * @brief Thread-safe function for devices that do not use register addressing (e.g., PCF8574).
 * Reads a sequence of bytes directly from the device.
 *
 * @param addr I2C device address.
 * 
 * @param data Pointer to the buffer to store read data.
 * @param len Number of bytes to read.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_bus_manager_read_no_reg(uint8_t addr, uint8_t *data, size_t len);

/**
 * @brief Thread-safe function for devices that do not use register addressing (e.g., PCF8574).
 * Writes a sequence of bytes directly to the device.
 *
 * @param addr I2C device address.
 * @param data Pointer to the data to write.
 * @param len Number of bytes to write.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_bus_manager_write_no_reg(uint8_t addr, const uint8_t *data, size_t len);


#ifdef __cplusplus
}
#endif
