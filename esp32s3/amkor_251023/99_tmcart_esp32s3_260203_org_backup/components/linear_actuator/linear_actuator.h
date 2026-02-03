#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "stdbool.h"
#include "stdint.h"

// H1A: 9, H1B: 10 으로 하면 전진 시 엔코더 값이 (-)로 나와서 두 개를 바꿈. 이제 전진 시 (+) 값.
#define ENCODER_GPIO_H1A            GPIO_NUM_10  // Z축
#define ENCODER_GPIO_H1B            GPIO_NUM_9

#define ENCODER_GPIO_H2A            GPIO_NUM_16  // X축
#define ENCODER_GPIO_H2B            GPIO_NUM_15

#define ENCODER_PCNT_HIGH_LIMIT     5000
#define ENCODER_PCNT_LOW_LIMIT     -5000

#define L298N_PWM_M1                   GPIO_NUM_38   // Z축
#define L298N_PWM_M2                   GPIO_NUM_39   // X축

// PWM motor clock frequency, 10MHz, 1 tick = 0.1us
#define PWM_MOTOR_TIMER_RESOLUTION_HZ    10000000
// PWM motor control frequency, 25KHz
#define PWM_MOTOR_FREQ_HZ                25000
// PWM Theoretical maximum (400). 오실로스코프로 확인 완료
#define PWM_MOTOR_DUTY_TICK_MAX          (PWM_MOTOR_TIMER_RESOLUTION_HZ / PWM_MOTOR_FREQ_HZ)

// Motor timer group ID
#define PWM_MOTOR_TIMER_GROUP_ID_M1      (0)
#define PWM_MOTOR_TIMER_GROUP_ID_M2      (0)

void Encoder_Init(void);
int Encoder_Get_Count_M1(void);
int Encoder_Get_Count_M2(void);
int Encoder_Clear_Count_M1(void);
int Encoder_Clear_Count_M2(void);

// Motor stop mode
typedef enum _stop_mode{
    STOP_COAST = 0,
    STOP_BRAKE = 1
} stop_mode_t;

void L298N_PWM_Init(void);
void L298N_PWM_Set_Speed_M1(uint32_t speed);
void L298N_PWM_Set_Speed_M2(uint32_t speed);

#ifdef __cplusplus
}
#endif
