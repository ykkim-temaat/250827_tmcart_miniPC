#include "linear_actuator.h"

#include "stdio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/pulse_cnt.h"
#include "esp_log.h"

#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"
#include "bdc_motor.h"

static const char *TAG = "Encoder";

bdc_motor_handle_t motor_pwm_m1 = NULL;
bdc_motor_handle_t motor_pwm_m2 = NULL;

pcnt_unit_handle_t encoder_unit_m1;
pcnt_unit_handle_t encoder_unit_m2;

// Initialize motor 1 encoder, bind GPIO and configure PCNT counter.
static void Encoder_M1_Init(void)   // Z축
{
    pcnt_unit_config_t unit_config = {
        .high_limit = ENCODER_PCNT_HIGH_LIMIT,
        .low_limit = ENCODER_PCNT_LOW_LIMIT,
        .flags.accum_count = false, // true, // enable counter accumulation
    };
    pcnt_unit_handle_t pcnt_unit = NULL;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));
    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = ENCODER_GPIO_H1A,
        .level_gpio_num = ENCODER_GPIO_H1B,
    };
    pcnt_channel_handle_t pcnt_chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a));
    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = ENCODER_GPIO_H1B,
        .level_gpio_num = ENCODER_GPIO_H1A,
    };
    pcnt_channel_handle_t pcnt_chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_b_config, &pcnt_chan_b));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, ENCODER_PCNT_HIGH_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, ENCODER_PCNT_LOW_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));
    encoder_unit_m1 = pcnt_unit;
}

// Initialize motor 2 encoder, bind GPIO and configure PCNT counter.
static void Encoder_M2_Init(void)   // X축
{
    pcnt_unit_config_t unit_config = {
        .high_limit = ENCODER_PCNT_HIGH_LIMIT,
        .low_limit = ENCODER_PCNT_LOW_LIMIT,
        .flags.accum_count = false, // true, // enable counter accumulation
    };
    pcnt_unit_handle_t pcnt_unit = NULL;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));
    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = ENCODER_GPIO_H2A,
        .level_gpio_num = ENCODER_GPIO_H2B,
    };
    pcnt_channel_handle_t pcnt_chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a));
    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = ENCODER_GPIO_H2B,
        .level_gpio_num = ENCODER_GPIO_H2A,
    };
    pcnt_channel_handle_t pcnt_chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_b_config, &pcnt_chan_b));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, ENCODER_PCNT_HIGH_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, ENCODER_PCNT_LOW_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));
    encoder_unit_m2 = pcnt_unit;
}

// Get the cumulative number of encoder pulses for motor 1
int Encoder_Get_Count_M1(void)
{
    static int count_m1 = 0;
    pcnt_unit_get_count(encoder_unit_m1, &count_m1);
    return count_m1;
}

// Get the cumulative number of encoder pulses for motor 2
int Encoder_Get_Count_M2(void)
{
    static int count_m2 = 0;
    pcnt_unit_get_count(encoder_unit_m2, &count_m2);
    return count_m2;
}

int Encoder_Clear_Count_M1(void)
{
    return pcnt_unit_clear_count(encoder_unit_m1);
}

int Encoder_Clear_Count_M2(void)
{
    return pcnt_unit_clear_count(encoder_unit_m2);
}

// Initialize the motor encoder
void Encoder_Init(void)
{
    ESP_LOGI(TAG, "Init pcnt driver to decode");
    Encoder_M1_Init();
    Encoder_M2_Init();
}

void L298N_PWM_M1_Init(void)
{
    bdc_motor_config_t motor_config = {
        .pwm_freq_hz = PWM_MOTOR_FREQ_HZ,
        .pwma_gpio_num = L298N_PWM_M1,
        .pwmb_gpio_num = GPIO_NUM_3,    // Dummy pin, not used
    };
    bdc_motor_mcpwm_config_t mcpwm_config = {
        .group_id = PWM_MOTOR_TIMER_GROUP_ID_M1,
        .resolution_hz = PWM_MOTOR_TIMER_RESOLUTION_HZ,
    };
    bdc_motor_handle_t motor = NULL;
    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&motor_config, &mcpwm_config, &motor));
    ESP_ERROR_CHECK(bdc_motor_enable(motor));
    motor_pwm_m1 = motor;
}

void L298N_PWM_M2_Init(void)
{
    bdc_motor_config_t motor_config = {
        .pwm_freq_hz = PWM_MOTOR_FREQ_HZ,
        .pwma_gpio_num = L298N_PWM_M2,
        .pwmb_gpio_num = GPIO_NUM_3,    // Dummy pin, not used
    };
    bdc_motor_mcpwm_config_t mcpwm_config = {
        .group_id = PWM_MOTOR_TIMER_GROUP_ID_M2,
        .resolution_hz = PWM_MOTOR_TIMER_RESOLUTION_HZ,
    };
    bdc_motor_handle_t motor = NULL;
    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&motor_config, &mcpwm_config, &motor));
    ESP_ERROR_CHECK(bdc_motor_enable(motor));
    motor_pwm_m2 = motor;
}

void L298N_PWM_Init(void)
{
    L298N_PWM_M1_Init();
    L298N_PWM_M2_Init();
}

void L298N_PWM_Set_Speed_M1(uint32_t speed)
{
    if (speed > 0) // forward
    {
        bdc_motor_forward(motor_pwm_m1);
        bdc_motor_set_speed(motor_pwm_m1, speed);
    }
}

void L298N_PWM_Set_Speed_M2(uint32_t speed)
{
    if (speed > 0) // forward
    {
        bdc_motor_forward(motor_pwm_m2);
        bdc_motor_set_speed(motor_pwm_m2, speed);
    }
}

