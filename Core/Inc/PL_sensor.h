/*
 * PL_sensor.h
 *
 *  Created on: Nov 25, 2024
 *      Author: lingm
 */

#ifndef INC_PL_SENSOR_H_
#define INC_PL_SENSOR_H_


#include "stdint.h"

extern uint16_t g_ADCBuffer[5];


extern uint16_t g_sensor_on[4];
extern uint16_t g_sensor_off[4];
extern uint16_t g_sensor[4][5];

extern uint16_t g_sensor_avg[4]; // センサー値の平均

extern int g_sensor_diff[4];
extern uint16_t pre_g_sensor[4];

extern float g_V_batt;

void pl_callback_getSensor(void);

void pl_interupt_getSensor(void);
#endif /* INC_PL_SENSOR_H_ */
