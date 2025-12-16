#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"
#include "driver/gpio.h"

// PCF8574 I2C Addresses
#define PCF8574_ADDR_0x20 0x20
#define PCF8574_ADDR_0x21 0x21
#define PCF8574_ADDR_0x22 0x22
#define PCF8574_ADDR_0x23 0x23
#define PCF8574_ADDR_0x24 0x24
#define PCF8574_ADDR_0x25 0x25
#define PCF8574_ADDR_0x26 0x26
#define PCF8574_ADDR_0x27 0x27

// PCF8574 Interrupt Pins
#define PCF8574_INT_PIN_0x22 (GPIO_NUM_6)
#define PCF8574_INT_PIN_0x23 (GPIO_NUM_7)
#define PCF8574_INT_PIN_0x24 (GPIO_NUM_8)

/**
 * @brief Initialize all PCF8574 devices and configure interrupt pins.
 */
void PCF8574_Init(void);

/**
 * @brief Write a byte to a specific PCF8574 device's I/O pins.
 */
esp_err_t pcf8574_write_byte(uint8_t device_address, uint8_t data);

/**
 * @brief Read a byte from a specific PCF8574 device's I/O pins.
 */
esp_err_t pcf8574_read_byte(uint8_t device_address, uint8_t *data);


#ifdef __cplusplus
}
#endif
