/*
 * SWICH.c
 *
 *  Created on: Jun 12, 2025
 *      Author: suzuk
 */

#include "main.h"
#include "MOTOR.h"
#include "led_control.h"
#include "tim.h"
#include "SPEAKER.h"
extern TIM_HandleTypeDef htim2;
uint16_t swich_press_count = 0;

static uint32_t switch_press_start_time = 0;
static uint8_t switch_was_pressed = 0;

#define DEBOUNCE_DELAY_MS 50    // チャタリング対策のディレイ（ms）

#define LONG_PRESS_DURATION_MS 3000 // 3秒長押し



