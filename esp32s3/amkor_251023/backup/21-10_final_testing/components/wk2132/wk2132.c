#include <string.h>
#include <math.h>
#include "wk2132.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

static const char *TAG = "WK2132";

// I2C transaction timeout
#define I2C_MASTER_TIMEOUT_MS 1000

// Internal constants
#define SUBUART_CHANNEL_1   0x00 // As per datasheet C1=0, C0=0
#define SUBUART_CHANNEL_2   0x01 // As per datasheet C1=0, C0=1

// --- Internal buffer for message reassembly ---
#define REASSEMBLY_BUFFER_SIZE 256
static char reassembly_buffers[4][REASSEMBLY_BUFFER_SIZE];
static size_t reassembly_len[4] = {0};
// ------------------------------------

// Register Definitions
#define REG_WK2132_GENA     0x00
#define REG_WK2132_GRST     0x01
#define REG_WK2132_GIER     0x10
#define REG_WK2132_SPAGE    0x03
#define REG_WK2132_SCR      0x04
#define REG_WK2132_LCR      0x05
#define REG_WK2132_FCR      0x06
#define REG_WK2132_SIER     0x07
#define REG_WK2132_TFCNT    0x09
#define REG_WK2132_RFCNT    0x0A
#define REG_WK2132_FSR      0x0B
#define REG_WK2132_BAUD1    0x04
#define REG_WK2132_BAUD0    0x05
#define REG_WK2132_PRES     0x06

// Struct to map external UARTs to chip addresses and sub-channels
typedef struct {
    uint8_t reg_i2c_addr; // Register access address (type=0)
    uint8_t fifo_i2c_addr; // FIFO access address (type=1)
    uint8_t sub_uart_ch;
} wk2132_uart_map_t;

static wk2132_uart_map_t uart_map[4];

// =============================================================================
// Internal helper functions
// =============================================================================

// Write to register
static esp_err_t _write_reg(ext_uart_num_t uart_num, uint8_t reg, const uint8_t *data, size_t len) {
    uint8_t i2c_addr = uart_map[uart_num].reg_i2c_addr;
    uint8_t write_buf[len + 1];
    write_buf[0] = reg;
    memcpy(&write_buf[1], data, len);
    return i2c_master_write_to_device(WK2132_I2C_PORT, i2c_addr, write_buf, sizeof(write_buf), pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
}

// Read from register
static esp_err_t _read_reg(ext_uart_num_t uart_num, uint8_t reg, uint8_t *data, size_t len) {
    uint8_t i2c_addr = uart_map[uart_num].reg_i2c_addr;
    return i2c_master_write_read_device(WK2132_I2C_PORT, i2c_addr, &reg, 1, data, len, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
}

// Write to FIFO
static esp_err_t _write_fifo(ext_uart_num_t uart_num, const uint8_t *data, size_t len) {
    uint8_t i2c_addr = uart_map[uart_num].fifo_i2c_addr;
    return i2c_master_write_to_device(WK2132_I2C_PORT, i2c_addr, data, len, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
}

// Read from FIFO
static esp_err_t _read_fifo(ext_uart_num_t uart_num, uint8_t *data, size_t len) {
    uint8_t i2c_addr = uart_map[uart_num].fifo_i2c_addr;
    return i2c_master_read_from_device(WK2132_I2C_PORT, i2c_addr, data, len, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
}


// Write a single byte to a register
static esp_err_t _write_reg_byte(ext_uart_num_t uart_num, uint8_t reg, uint8_t value) {
    return _write_reg(uart_num, reg, &value, 1);
}

// Read a single byte from a register
static esp_err_t _read_reg_byte(ext_uart_num_t uart_num, uint8_t reg, uint8_t *value) {
    return _read_reg(uart_num, reg, value, 1);
}

// Switch page
static esp_err_t _page_switch(ext_uart_num_t uart_num, uint8_t page) {
    return _write_reg_byte(uart_num, REG_WK2132_SPAGE, (page == 1) ? 0x01 : 0x00);
}

// Set baud rate
static esp_err_t _set_baudrate(ext_uart_num_t uart_num, uint32_t baudrate) {
    float divisor = (float)WK2132_FOSC / (16.0f * baudrate);
    if (divisor < 1) divisor = 1;
    uint16_t n = (uint16_t)divisor - 1;
    uint8_t m = (uint8_t)roundf((divisor - (n + 1)) * 16.0f);

    uint8_t baud1 = (uint8_t)(n >> 8);
    uint8_t baud0 = (uint8_t)(n & 0xFF);
    uint8_t pres = m;

    esp_err_t ret = _page_switch(uart_num, 1);
    if (ret != ESP_OK) return ret;

    if ((ret = _write_reg_byte(uart_num, REG_WK2132_BAUD1, baud1)) != ESP_OK) goto exit;
    if ((ret = _write_reg_byte(uart_num, REG_WK2132_BAUD0, baud0)) != ESP_OK) goto exit;
    if ((ret = _write_reg_byte(uart_num, REG_WK2132_PRES, pres)) != ESP_OK) goto exit;

exit:
    _page_switch(uart_num, 0); // Always return to page 0
    return ret;
}

// Initialize a single UART hardware
static esp_err_t _single_uart_hw_init(ext_uart_num_t uart_num) {
    // Determine the base UART for accessing the chip's global registers.
    ext_uart_num_t chip_base_uart = (uart_num == EXT_UART_2) ? EXT_UART_1 :
                                    (uart_num == EXT_UART_4) ? EXT_UART_3 :
                                    uart_num;
    esp_err_t err;
    uint8_t val;

    // Enable clocks for both sub-UARTs of the chip (executed once per chip)
    if (uart_map[uart_num].sub_uart_ch == SUBUART_CHANNEL_1) {
        err = _read_reg_byte(chip_base_uart, REG_WK2132_GENA, &val);
        if (err != ESP_OK) return ESP_FAIL;
        val |= 0x03; 
        err = _write_reg_byte(chip_base_uart, REG_WK2132_GENA, val);
        if (err != ESP_OK) return ESP_FAIL;
    }

    // Software reset sub-UART (using global address)
    uint8_t reset_val = (uart_map[uart_num].sub_uart_ch == SUBUART_CHANNEL_1) ? 0x01 : 0x02;
    err = _write_reg_byte(chip_base_uart, REG_WK2132_GRST, reset_val);
    if (err != ESP_OK) return ESP_FAIL;
    vTaskDelay(pdMS_TO_TICKS(10));

    // Enable global interrupt (executed once per chip, using global address)
    if (uart_map[uart_num].sub_uart_ch == SUBUART_CHANNEL_1) {
        val = 0x03;
        err = _write_reg_byte(chip_base_uart, REG_WK2132_GIER, val);
        if (err != ESP_OK) return ESP_FAIL;
    }

    // --- From here on, use the individual UART address for its specific registers ---
    // Set to page 0
    if (_page_switch(uart_num, 0) != ESP_OK) return ESP_FAIL;
    // Set sub-UART interrupt (SIER)
    if (_write_reg_byte(uart_num, REG_WK2132_SIER, 0x8F) != ESP_OK) return ESP_FAIL;
    // Set FIFO (FCR) - Reset and enable FIFO
    if (_write_reg_byte(uart_num, REG_WK2132_FCR, 0x0F) != ESP_OK) return ESP_FAIL;
    vTaskDelay(pdMS_TO_TICKS(10));
    // Enable RX/TX (SCR)
    if (_write_reg_byte(uart_num, REG_WK2132_SCR, 0x03) != ESP_OK) return ESP_FAIL;
    // Set default baud rate and data format
    if (_set_baudrate(uart_num, DEFAULT_BAUDRATE) != ESP_OK) return ESP_FAIL;
    if (wk2132_set_format(uart_num, DEFAULT_DATA_FORMAT) != ESP_OK) return ESP_FAIL;

    ESP_LOGI(TAG, "EXT_UART_%d initialized successfully.", uart_num + 1);
    return ESP_OK;
}

// =============================================================================
// Public API Functions
// =============================================================================

esp_err_t wk2132_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = WK2132_I2C_SDA_PIN,
        .scl_io_num = WK2132_I2C_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = WK2132_I2C_FREQ_HZ,
    };
    esp_err_t ret = i2c_param_config(WK2132_I2C_PORT, &conf);
    if (ret != ESP_OK) return ret;
    ret = i2c_driver_install(WK2132_I2C_PORT, conf.mode, 0, 0, 0);
    if (ret != ESP_OK) return ret;

    vTaskDelay(pdMS_TO_TICKS(100)); 

    // Datasheet 8-bit address format: 0b0<A1><A0>10<C1><C0><type>
    // The 7-bit address is the upper 7 bits, excluding the type bit
    const uint8_t base_addr_fixed_part = (1 << 4); // 0b00010000

    uint8_t base_addr1 = (WK2132_1_IA1 << 6) | (WK2132_1_IA0 << 5) | base_addr_fixed_part;
    uint8_t base_addr2 = (WK2132_2_IA1 << 6) | (WK2132_2_IA0 << 5) | base_addr_fixed_part;

    uint8_t reg_addr1_ch1 = base_addr1 | (SUBUART_CHANNEL_1 << 1);
    uint8_t reg_addr1_ch2 = base_addr1 | (SUBUART_CHANNEL_2 << 1);
    uint8_t reg_addr2_ch1 = base_addr2 | (SUBUART_CHANNEL_1 << 1);
    uint8_t reg_addr2_ch2 = base_addr2 | (SUBUART_CHANNEL_2 << 1);

    uart_map[EXT_UART_1] = (wk2132_uart_map_t){.reg_i2c_addr = reg_addr1_ch1, .fifo_i2c_addr = reg_addr1_ch1 | 1, .sub_uart_ch = SUBUART_CHANNEL_1};
    uart_map[EXT_UART_2] = (wk2132_uart_map_t){.reg_i2c_addr = reg_addr1_ch2, .fifo_i2c_addr = reg_addr1_ch2 | 1, .sub_uart_ch = SUBUART_CHANNEL_2};
    uart_map[EXT_UART_3] = (wk2132_uart_map_t){.reg_i2c_addr = reg_addr2_ch1, .fifo_i2c_addr = reg_addr2_ch1 | 1, .sub_uart_ch = SUBUART_CHANNEL_1};
    uart_map[EXT_UART_4] = (wk2132_uart_map_t){.reg_i2c_addr = reg_addr2_ch2, .fifo_i2c_addr = reg_addr2_ch2 | 1, .sub_uart_ch = SUBUART_CHANNEL_2};

    for (int i = 0; i < 4; i++) {
        if (_single_uart_hw_init((ext_uart_num_t)i) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize EXT_UART_%d", i + 1);
            return ESP_FAIL;
        }
    }

    ESP_LOGI(TAG, "WK2132 component initialized.");
    return ESP_OK;
}

uint16_t wk2132_available(ext_uart_num_t uart_num) {
    uint8_t count = 0;
    _read_reg_byte(uart_num, REG_WK2132_RFCNT, &count);
    return count;
}

size_t wk2132_read(ext_uart_num_t uart_num, uint8_t *data, size_t max_len) {
    size_t available = wk2132_available(uart_num);
    if (available == 0) return 0;
    size_t read_len = (available > max_len) ? max_len : available;
    if (_read_fifo(uart_num, data, read_len) != ESP_OK) {
        return 0;
    }
    return read_len;
}

size_t wk2132_write(ext_uart_num_t uart_num, const uint8_t *data, size_t len) {
    size_t written = 0;
    size_t to_write;
    const size_t max_fifo_chunk = 32;
    while(written < len) {
        uint8_t tfcnt = 0;
        _read_reg_byte(uart_num, REG_WK2132_TFCNT, &tfcnt);
        size_t free_space = 256 - tfcnt;
        to_write = len - written;
        if (to_write > free_space) to_write = free_space;
        if (to_write > max_fifo_chunk) to_write = max_fifo_chunk;
        if (to_write > 0) {
            if (_write_fifo(uart_num, data + written, to_write) != ESP_OK) return written;
            written += to_write;
        } else {
            vTaskDelay(pdMS_TO_TICKS(1)); // Short delay if FIFO is full
        }
    }
    return written;
}

esp_err_t wk2132_set_baudrate(ext_uart_num_t uart_num, uint32_t baudrate) {
    return _set_baudrate(uart_num, baudrate);
}

esp_err_t wk2132_set_format(ext_uart_num_t uart_num, uint8_t format) {
    return _write_reg_byte(uart_num, REG_WK2132_LCR, format);
}

size_t wk2132_read_until(ext_uart_num_t uart_num, char delimiter, char *buffer, size_t max_len, uint32_t timeout_ms) {
    int64_t start_time = esp_timer_get_time();
    
    while ((esp_timer_get_time() - start_time) / 1000 < timeout_ms) {
        // 1. First, search for the delimiter in the internal buffer.
        char* delimiter_ptr = memchr(reassembly_buffers[uart_num], delimiter, reassembly_len[uart_num]);
        if (delimiter_ptr != NULL) {
            size_t msg_len = (delimiter_ptr - reassembly_buffers[uart_num]) + 1;
            if (msg_len > max_len) { // Do not process if the user buffer is too small
                // To prevent data loss, discard the oversized message from the buffer
                reassembly_len[uart_num] -= msg_len;
                memmove(reassembly_buffers[uart_num], reassembly_buffers[uart_num] + msg_len, reassembly_len[uart_num]);
                return 0; 
            }
            // Copy the message to the user buffer
            memcpy(buffer, reassembly_buffers[uart_num], msg_len);
            
            // Remove the processed message from the internal buffer
            reassembly_len[uart_num] -= msg_len;
            memmove(reassembly_buffers[uart_num], reassembly_buffers[uart_num] + msg_len, reassembly_len[uart_num]);
            
            return msg_len;
        }

        // 2. Check if there is new data in the FIFO and move it to the internal buffer.
        if (wk2132_available(uart_num) > 0) {
            size_t free_space = REASSEMBLY_BUFFER_SIZE - reassembly_len[uart_num];
            if (free_space > 0) {
                size_t read_len = wk2132_read(uart_num, (uint8_t*)reassembly_buffers[uart_num] + reassembly_len[uart_num], free_space);
                reassembly_len[uart_num] += read_len;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // Short delay to reduce CPU load
    }
    return 0; // Timeout
}
