/*
 * PL_timer.h
 *
 *  Created on: Nov 25, 2024
 *      Author: lingm
 */

#ifndef INC_PL_TIMER_H_
#define INC_PL_TIMER_H_


//#include "stm32f4xx_hal_conf.h"
#include "stm32f4xx_it.h"

#include "stdint.h"

extern volatile uint32_t g_timCount;
void pl_timer_init(void);
void pl_timer_count(void);
void wait_ms(uint32_t);

#endif /* INC_PL_TIMER_H_ */
