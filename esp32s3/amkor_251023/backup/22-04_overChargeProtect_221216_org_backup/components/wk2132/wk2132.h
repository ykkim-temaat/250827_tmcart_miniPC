#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#ifndef WK2132_H
#define WK2132_H

#include "driver/i2c.h"
#include "esp_err.h"

// =============================================================================
// Hardware Configuration (User configurable)
// =============================================================================

// I2C Bus Settings
#define WK2132_I2C_PORT       I2C_NUM_1
#define WK2132_I2C_SCL_PIN    GPIO_NUM_2 // I2C Clock Pin
#define WK2132_I2C_SDA_PIN    GPIO_NUM_1 // I2C Data Pin
#define WK2132_I2C_FREQ_HZ    400000      // I2C Clock Speed (400kHz)

// External crystal oscillator frequency connected to WK2132
#define WK2132_FOSC           14745600L

// --- Address settings for WK2132 Chip 1 (handles ext_uart1 & ext_uart2) ---
#define WK2132_1_IA1          1 // MP3 pin level
#define WK2132_1_IA0          1 // MP1 pin level

// --- Address settings for WK2132 Chip 2 (handles ext_uart3 & ext_uart4) ---
#define WK2132_2_IA1          1 // MP3 pin level
#define WK2132_2_IA0          0 // MP1 pin level

// --- Default UART Settings ---
#define DEFAULT_BAUDRATE      115200
#define DEFAULT_DATA_FORMAT   WK2132_FORMAT_8N1

// =============================================================================
// End of Configuration
// =============================================================================

// Data Format Definitions
#define WK2132_FORMAT_8N1     0x00 // 8 data bits, no parity, 1 stop bit
#define WK2132_FORMAT_8N2     0x01 // 8 data bits, no parity, 2 stop bits
#define WK2132_FORMAT_8Z1     0x08 // 8 data bits, 0 parity, 1 stop bit
#define WK2132_FORMAT_8Z2     0x09 // 8 data bits, 0 parity, 2 stop bits
#define WK2132_FORMAT_8O1     0x0A // 8 data bits, odd parity, 1 stop bit
#define WK2132_FORMAT_8O2     0x0B // 8 data bits, odd parity, 2 stop bits
#define WK2132_FORMAT_8E1     0x0C // 8 data bits, even parity, 1 stop bit
#define WK2132_FORMAT_8E2     0x0D // 8 data bits, even parity, 2 stop bits
#define WK2132_FORMAT_8F1     0x0E // 8 data bits, 1 parity, 1 stop bit
#define WK2132_FORMAT_8F2     0x0F // 8 data bits, 1 parity, 2 stop bits

// Enum to identify the four external UARTs
typedef enum {
    EXT_UART_1 = 0, // Chip 1, Sub UART 1
    EXT_UART_2,     // Chip 1, Sub UART 2
    EXT_UART_3,     // Chip 2, Sub UART 1
    EXT_UART_4      // Chip 2, Sub UART 2
} ext_uart_num_t;

/**
 * @brief Initializes the I2C bus and the two WK2132 chips.
 */
esp_err_t wk2132_init(void);

/**
 * @brief Gets the number of bytes available for reading from the UART's receive FIFO.
 */
uint16_t wk2132_available(ext_uart_num_t uart_num);

/**
 * @brief Reads bytes from the UART's receive FIFO.
 */
size_t wk2132_read(ext_uart_num_t uart_num, uint8_t *data, size_t max_len);

/**
 * @brief Writes bytes to the UART's transmit FIFO.
 */
size_t wk2132_write(ext_uart_num_t uart_num, const uint8_t *data, size_t len);

/**
 * @brief Sets the baud rate for a specific external UART.
 */
esp_err_t wk2132_set_baudrate(ext_uart_num_t uart_num, uint32_t baudrate);

/**
 * @brief Sets the data format for a specific external UART.
 */
esp_err_t wk2132_set_format(ext_uart_num_t uart_num, uint8_t format);

/**
 * @brief Reads data from the UART until a specified delimiter is found or a timeout occurs.
 *
 * This function uses an internal buffer to reassemble complete messages from the data stream.
 *
 * @param uart_num The external UART to read from (EXT_UART_1 to EXT_UART_4).
 * @param delimiter The character that marks the end of a message (e.g., '\n').
 * @param buffer Buffer to store the read message.
 * @param max_len The maximum size of the buffer.
 * @param timeout_ms Timeout duration in milliseconds. A value of 0 will check once and return immediately (non-blocking). A common default might be 1000ms.
 * @return The length of the read message (including the delimiter), or 0 if no message was found or timeout occurred.
 */
size_t wk2132_read_until(ext_uart_num_t uart_num, char delimiter, char *buffer, size_t max_len, uint32_t timeout_ms);


#endif // WK2132_H

#ifdef __cplusplus
}
#endif
