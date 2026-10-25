/*
 * led_control.h
 *
 *  Created on: Jun 21, 2025
 *      Author: suzuk
 */

#ifndef INC_LED_CONTROL_H_
#define INC_LED_CONTROL_H_

#include "stm32f4xx_hal_conf.h"  // 必要に応じて変更
#include "stm32f4xx_it.h"  // 必要に応じて変更
#include "stdint.h"

void LED_On(uint8_t n);
void LED_Off(uint8_t n);
void LED_Left_On();

void LED_Left_Off();
void LED_Right_On();
void LED_Right_Off() ;
void LED_Toggle(uint8_t n);
void LED_All_On(void);
void LED_All_Off(void);
void LED_Chase_OneByOne(void);

void Toggle_LED5_and_LED4();

void some_function(void);


#endif /* INC_LED_CONTROL_H_ */
