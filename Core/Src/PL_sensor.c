#include "stdio.h"
#include "usart.h"
#include "PL_sensor.h"
#include "adc.h"



int g_sensor_diff[4];

float g_V_batt;
// ADCの分解能に合わせて型をuint16_tに変更
uint16_t g_ADCBuffer[5];
char AD_step;

uint16_t g_sensor_on[4];
uint16_t g_sensor_off[4];
uint16_t g_sensor[4][5];
uint16_t pre_g_sensor[4];
int g_sensor_diff[4];

float g_V_batt;

/*******************************************************************/
/* 電圧の取得 (pl_getbatt)                                         */
/*******************************************************************/
float pl_getbatt(void) {
    float batt;
    uint16_t battAD;

    HAL_ADC_Start(&hadc1);
    // DMAを使用しない単一変換の場合は、ポーリングで変換完了を待つ
    HAL_ADC_PollForConversion(&hadc1, 50);
    battAD = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    // 12ビット分解能 (最大値4095) に合わせて修正
    batt = 2 * 3.3 * (float)battAD / 4095.0 * (100.0 + 47.0) / 47.0;

    return batt;
}

/*******************************************************************/
/* callback用関数 (pl_callback_getSensor)                          */
/*******************************************************************/
void pl_callback_getSensor(void) {
    uint16_t V_battAD;
    int i;

    // DMAを使用しているため、ポーリングや遅延は不要
    HAL_ADC_Stop_DMA(&hadc1);

    switch (AD_step) {
        case 0:
            // ADC変換が完了したのでLEDの状態を変更し、次の変換を開始する準備をする
            HAL_GPIO_WritePin(SENSOR_LED_1_GPIO_Port, SENSOR_LED_1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(SENSOR_LED_2_GPIO_Port, SENSOR_LED_2_Pin, GPIO_PIN_SET);
            for(i=0;i<=100;i++){

            }
            break;
        case 1:
            g_sensor_on[3] = g_ADCBuffer[4];
            g_sensor_on[2] = g_ADCBuffer[3];
            HAL_GPIO_WritePin(SENSOR_LED_1_GPIO_Port, SENSOR_LED_1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(SENSOR_LED_2_GPIO_Port, SENSOR_LED_2_Pin, GPIO_PIN_RESET);
            for(i=0;i<=2000;i++){

            }
            break;
        case 2:
            g_sensor_on[0] = g_ADCBuffer[1];
            g_sensor_on[1] = g_ADCBuffer[2];
            HAL_GPIO_WritePin(SENSOR_LED_1_GPIO_Port, SENSOR_LED_1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(SENSOR_LED_2_GPIO_Port, SENSOR_LED_2_Pin, GPIO_PIN_RESET);
            for(i=0;i<=100;i++){

            }
            break;
        case 3:
            g_sensor_off[0] = g_ADCBuffer[1];
            g_sensor_off[1] = g_ADCBuffer[2];
            g_sensor_off[2] = g_ADCBuffer[3];
            g_sensor_off[3] = g_ADCBuffer[4];
            HAL_GPIO_WritePin(SENSOR_LED_1_GPIO_Port, SENSOR_LED_1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(SENSOR_LED_2_GPIO_Port, SENSOR_LED_2_Pin, GPIO_PIN_RESET);
            break;
    }

    V_battAD = g_ADCBuffer[0];

    g_V_batt = 3.29 * (float)V_battAD / 1024.0 * (10+ 47) / 10;
    AD_step++;

    if (AD_step != 4) {
        // 次のADC変換を開始
        HAL_ADC_Start_DMA(&hadc1, (uint32_t*)g_ADCBuffer, sizeof(g_ADCBuffer) / sizeof(uint16_t));
    } else {
        AD_step = 0;


        for (int i = 4; i > 0; i--) {
            for (int j = 0; j < 4; j++) {
                g_sensor[j][i] = g_sensor[j][i-1];
            }
        }

        for (int i = 0; i < 4; i++) {
            pre_g_sensor[i] = g_sensor[i][1];
            int16_t new_value = g_sensor_on[i] - g_sensor_off[i];

            if (new_value > 0 && new_value < 4095) {
                g_sensor[i][0] = new_value;
            } else {
                g_sensor[i][0] = g_sensor[i][1];
            }
        }

        for (i = 0; i < 4; i++) {
            g_sensor_diff[i] =	pre_g_sensor[i] - g_sensor[i][0];
        }
    }
}

/*******************************************************************/
/* 割り込み用動作関数 (センサー取得)                               */
/*******************************************************************/
void pl_interupt_getSensor(void) {
    // 修正後: DMAの長さをsizeof(uint32_t)で計算
    HAL_ADC_Start_DMA(&hadc1, g_ADCBuffer, sizeof(g_ADCBuffer) / sizeof(uint32_t));
}
