#include "i2c_bus_manager.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include <string.h> // For memcpy

static const char *TAG = "I2C_BUS_MANAGER";

// Mutex to protect the I2C bus
static SemaphoreHandle_t i2c_mutex = NULL;

void i2c_bus_manager_init(void)
{
    // Create the mutex before initializing the bus
    if (i2c_mutex == NULL) {
        i2c_mutex = xSemaphoreCreateMutex();
        if (i2c_mutex == NULL) {
            ESP_LOGE(TAG, "Failed to create I2C mutex!");
            return;
        }
    }

    ESP_LOGI(TAG, "Initializing I2C master: SCL->GPIO%d, SDA->GPIO%d", I2C_BUS_MANAGER_SCL_GPIO, I2C_BUS_MANAGER_SDA_GPIO);
    
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_BUS_MANAGER_SDA_GPIO,
        .scl_io_num = I2C_BUS_MANAGER_SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_BUS_MANAGER_FREQ_HZ,
    };

    esp_err_t err = i2c_param_config(I2C_BUS_MANAGER_PORT_NUM, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C param config failed: %s", esp_err_to_name(err));
        return;
    }

    // Check if driver is already installed before installing
    err = i2c_driver_install(I2C_BUS_MANAGER_PORT_NUM, conf.mode, 0, 0, 0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) { // ESP_ERR_INVALID_STATE means it's already installed
        ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "I2C Bus Manager initialized successfully.");
}

void i2c_bus_manager_deinit(void)
{
    if (i2c_mutex != NULL) {
        vSemaphoreDelete(i2c_mutex);
        i2c_mutex = NULL;
    }
    i2c_driver_delete(I2C_BUS_MANAGER_PORT_NUM);
    ESP_LOGI(TAG, "I2C Bus Manager deinitialized.");
}

esp_err_t i2c_bus_manager_read(uint8_t addr, uint8_t reg, uint8_t *data, size_t len)
{
    if (i2c_mutex == NULL) return ESP_FAIL;
    esp_err_t ret = ESP_FAIL;

    if (xSemaphoreTake(i2c_mutex, portMAX_DELAY) == pdTRUE) {
        ret = i2c_master_write_read_device(I2C_BUS_MANAGER_PORT_NUM, addr, &reg, 1, data, len, pdMS_TO_TICKS(I2C_BUS_MANAGER_TIMEOUT_MS));
        // ESP_LOGW(TAG, "I2C Bus Manager read: %d", ret);
        xSemaphoreGive(i2c_mutex);
    }
    return ret;
}

esp_err_t i2c_bus_manager_write(uint8_t addr, uint8_t reg, const uint8_t *data, size_t len)
{
    if (i2c_mutex == NULL) return ESP_FAIL;
    esp_err_t ret = ESP_FAIL;

    if (xSemaphoreTake(i2c_mutex, portMAX_DELAY) == pdTRUE) {
        uint8_t *write_buf = malloc(len + 1);
        if (write_buf == NULL) {
            ESP_LOGE(TAG, "Malloc failed for write buffer");
            xSemaphoreGive(i2c_mutex);
            return ESP_ERR_NO_MEM;
        }

        write_buf[0] = reg;
        memcpy(write_buf + 1, data, len);

        ret = i2c_master_write_to_device(I2C_BUS_MANAGER_PORT_NUM, addr, write_buf, len + 1, pdMS_TO_TICKS(I2C_BUS_MANAGER_TIMEOUT_MS));
        
        free(write_buf);
        xSemaphoreGive(i2c_mutex);
    }
    return ret;
}

esp_err_t i2c_bus_manager_read_no_reg(uint8_t addr, uint8_t *data, size_t len)
{
    if (i2c_mutex == NULL) return ESP_FAIL;
    esp_err_t ret = ESP_FAIL;

    if (xSemaphoreTake(i2c_mutex, portMAX_DELAY) == pdTRUE) {
        ret = i2c_master_read_from_device(I2C_BUS_MANAGER_PORT_NUM, addr, data, len, pdMS_TO_TICKS(I2C_BUS_MANAGER_TIMEOUT_MS));
        xSemaphoreGive(i2c_mutex);
    }
    return ret;
}

esp_err_t i2c_bus_manager_write_no_reg(uint8_t addr, const uint8_t *data, size_t len)
{
    if (i2c_mutex == NULL) return ESP_FAIL;
    esp_err_t ret = ESP_FAIL;

    if (xSemaphoreTake(i2c_mutex, portMAX_DELAY) == pdTRUE) {
        ret = i2c_master_write_to_device(I2C_BUS_MANAGER_PORT_NUM, addr, data, len, pdMS_TO_TICKS(I2C_BUS_MANAGER_TIMEOUT_MS));
        xSemaphoreGive(i2c_mutex);
    }
    return ret;
}
