#ifndef LED_H
#define LED_H

#include "main.h"

// ===== CONFIG =====
#define LED_COUNT 8       // number of LEDs
#define BIT_1_HIGH 70     // Duty cycle for "1" (ARR=99)
#define BIT_0_HIGH 35     // Duty cycle for "0"
#define TIM_PERIOD 99     // ARR

// ==================

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim);
void WS2812_SetLED(uint16_t index, uint8_t r, uint8_t g, uint8_t b);
void WS2812_Send(void);

#endif
