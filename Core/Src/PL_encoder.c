/*
 * PL_encoder.c
 *
 *  Created on: Jan 6, 2023
 *      Author: sf199
 */

#include "PL_encoder.h"
#include "spi.h"
#include "stdio.h"
#include "Define.h"
#include <math.h>
#include "lsm6dsr.h"
#include "define.h"
#include "log.h"

float encoder_R, encoder_L; //エンコーダ検出角度

float Tire_Speed_L_LOG[10000], Tire_Speed_R_LOG[10000], GYRO_LOG[1000];

float G_Tire_Speed_R, G_Tire_Speed_L; //タイヤスピード

float Tire_Speed_R[40], Tire_Speed_L[40];

float encoder_R_old, encoder_L_old; //前回のエンコーダ角度

float encoder_R_delta, encoder_L_delta;

float encoder_R_sum, encoder_L_sum;



float Measure_Gv_L;
float Measure_Gv_R;
/*float Measure_Gv_R_sum,Measure_Gv_L_sum;*/


int Program_number, Program_mode;

uint16_t encoder_read_byte_R(uint16_t address, uint16_t data) {

	uint8_t addBuffer[2];
//	uint16_t data;
	uint8_t dataBuffer[2];
	uint16_t parity;

	HAL_GPIO_WritePin(CSN_EN2_GPIO_Port, CSN_EN2_Pin, GPIO_PIN_RESET); //cs = 0;

	address = address | 0x4000; //先頭から2つ目のbitを1に
	parity = 0;
	for (int i = 0; i < 15; i++)
		parity += (address >> i) & 1;
	address = address | ((parity % 2) << 15);
	addBuffer[0] = address >> 8;
	addBuffer[1] = address & 0x00FF;

	HAL_SPI_Transmit(&hspi3, (uint8_t*) addBuffer, 2, 100);

	HAL_GPIO_WritePin( CSN_EN2_GPIO_Port, CSN_EN2_Pin, GPIO_PIN_SET); //cs = 1;

	for (int i = 0; i < 150; i++) {
	}

	HAL_GPIO_WritePin( CSN_EN2_GPIO_Port, CSN_EN2_Pin, GPIO_PIN_RESET); //cs = 0;

//	data=0x0000;
	dataBuffer[0] = data >> 8;
	dataBuffer[1] = data & 0x00FF;
	HAL_SPI_Receive(&hspi3, (uint8_t*) dataBuffer, 2, 100);
	data = ((uint16_t) (dataBuffer[0]) << 8) | (uint16_t) (dataBuffer[1]);
	HAL_GPIO_WritePin( CSN_EN2_GPIO_Port, CSN_EN2_Pin, GPIO_PIN_SET); //cs = 1;

	return data;

}

void encoder_write_byte_R(uint16_t address, uint16_t data) {

	uint8_t addBuffer[2];
	uint8_t dataBuffer[2];
	uint16_t parity;

	HAL_GPIO_WritePin( CSN_EN2_GPIO_Port, CSN_EN2_Pin, GPIO_PIN_RESET); //cs = 0;

	address = address & 0xBFFF; //先頭から2つ目のbitを1に
	parity = 0;
	for (int i = 0; i < 15; i++)
		parity += (address >> i) & 1;
	address = address | ((parity % 2) << 15);
	addBuffer[0] = address >> 8;
	addBuffer[1] = address & 0x00FF;

	HAL_SPI_Transmit(&hspi3, (uint8_t*) addBuffer, 2, 100);

	HAL_GPIO_WritePin( CSN_EN2_GPIO_Port, CSN_EN2_Pin, GPIO_PIN_SET); //cs = 1;

	for (int i = 0; i < 100; i++) {
	}

	HAL_GPIO_WritePin(CSN_EN2_GPIO_Port, CSN_EN2_Pin, GPIO_PIN_RESET); //cs = 0;

	data = data & 0xBFFF; //先頭から2つ目のbitを0に
	parity = 0;
	for (int i = 0; i < 15; i++)
		parity += (data >> i) & 1;
	data = data | ((parity % 2) << 15);
	dataBuffer[0] = data >> 8;
	dataBuffer[1] = data & 0x00FF;
	HAL_SPI_Transmit(&hspi3, (uint8_t*) dataBuffer, 2, 100);

	HAL_GPIO_WritePin( CSN_EN2_GPIO_Port, CSN_EN2_Pin, GPIO_PIN_SET); //cs = 1;
}

uint16_t encoder_read_byte_L(uint16_t address, uint16_t data) {

	uint8_t addBuffer[2];
//	uint16_t data;
	uint8_t dataBuffer[2];
	uint16_t parity;

	HAL_GPIO_WritePin(CSN_EN1_GPIO_Port, CSN_EN1_Pin, GPIO_PIN_RESET); //cs = 0;

	address = address | 0x4000; //先頭から2つ目のbitを1に
	parity = 0;
	for (int i = 0; i < 15; i++)
		parity += (address >> i) & 1;
	address = address | ((parity % 2) << 15);
	addBuffer[0] = address >> 8;
	addBuffer[1] = address & 0x00FF;

	HAL_SPI_Transmit(&hspi3, (uint8_t*) addBuffer, 2, 100);

	HAL_GPIO_WritePin( CSN_EN1_GPIO_Port, CSN_EN1_Pin, GPIO_PIN_SET); //cs = 1;

	for (int i = 0; i < 150; i++) {
	}

	HAL_GPIO_WritePin( CSN_EN1_GPIO_Port, CSN_EN1_Pin, GPIO_PIN_RESET); //cs = 0;

//	data=0x0000;
	dataBuffer[0] = data >> 8;
	dataBuffer[1] = data & 0x00FF;
	HAL_SPI_Receive(&hspi3, (uint8_t*) dataBuffer, 2, 100);
	data = ((uint16_t) (dataBuffer[0]) << 8) | (uint16_t) (dataBuffer[1]);
	HAL_GPIO_WritePin( CSN_EN1_GPIO_Port, CSN_EN1_Pin, GPIO_PIN_SET); //cs = 1;
	HAL_StatusTypeDef res;
	res = HAL_SPI_Transmit(&hspi3, addBuffer, 2, 100);
	//printf("TX result: %d\n", res);
	return data;

}

void encoder_write_byte_L(uint16_t address, uint16_t data) {

	uint8_t addBuffer[2];
	uint8_t dataBuffer[2];
	uint16_t parity;

	HAL_GPIO_WritePin( CSN_EN1_GPIO_Port, CSN_EN1_Pin, GPIO_PIN_RESET); //cs = 0;

	address = address & 0xBFFF; //先頭から2つ目のbitを1に
	parity = 0;
	for (int i = 0; i < 15; i++)
		parity += (address >> i) & 1;
	address = address | ((parity % 2) << 15);
	addBuffer[0] = address >> 8;
	addBuffer[1] = address & 0x00FF;

	HAL_SPI_Transmit(&hspi3, (uint8_t*) addBuffer, 2, 100);

	HAL_GPIO_WritePin( CSN_EN1_GPIO_Port, CSN_EN1_Pin, GPIO_PIN_SET); //cs = 1;

	for (int i = 0; i < 100; i++) {
	}

	HAL_GPIO_WritePin(CSN_EN1_GPIO_Port, CSN_EN1_Pin, GPIO_PIN_RESET); //cs = 0;

	data = data & 0xBFFF; //先頭から2つ目のbitを0に
	parity = 0;
	for (int i = 0; i < 15; i++)
		parity += (data >> i) & 1;
	data = data | ((parity % 2) << 15);
	dataBuffer[0] = data >> 8;
	dataBuffer[1] = data & 0x00FF;
	HAL_SPI_Transmit(&hspi3, (uint8_t*) dataBuffer, 2, 100);

	HAL_GPIO_WritePin( CSN_EN1_GPIO_Port, CSN_EN1_Pin, GPIO_PIN_SET); //cs = 1;

}

void AS5047_DataUpdate(void) {

	//encoder_read_byte_L(0x3FFF,0xC000);
	//HAL_Delay(5);
	encoder_L = (float) (encoder_read_byte_R(0x3FFF, 0x0000) & 0x3FFF) * 360
			/ 8192;
	//HAL_Delay(500);

	//encoder_read_byte_R(0x3FFF,0xC000);
	//HAL_Delay(5);
	encoder_R = (float) (encoder_read_byte_L(0x3FFF, 0x0000) & 0x3FFF) * 360
			/ 8192;
	//HAL_Delay(5);

}

/*void Encorder_Speed_Calculate() {		//一回転360°

	float sumL = 0, sumR = 0;

	encoder_R_old = encoder_R;
	encoder_L_old = encoder_L;

	AS5047_DataUpdate();

	encoder_R_delta = encoder_R - encoder_R_old;
	if (encoder_R_delta < -300) {
		encoder_R_delta += 360;
	} else if (encoder_R_delta > 300) {
		encoder_R_delta -= 360;
	}

	encoder_L_delta = encoder_L - encoder_L_old;
	if (encoder_L_delta < -300) {
		encoder_L_delta += 360;
	} else if (encoder_L_delta > 300) {
		encoder_L_delta -= 360;
	}

	Tire_Speed_R[0] = -1*encoder_R_delta / (360 *reduction_ratio) *Tire_dia * PI * 1000;
	Tire_Speed_L[0] = encoder_L_delta / (360 *reduction_ratio) * Tire_dia * PI * 1000;

	for (int i = 19; i > 0; i--) {
		Tire_Speed_R[i] = Tire_Speed_R[i - 1];
		Tire_Speed_L[i] = Tire_Speed_L[i - 1];
	}

	for (int i = 0; i < 20; i++) {
		sumR += Tire_Speed_R[i];
		sumL += Tire_Speed_L[i];
	}

	float pre_Measure_Gv_R  = Measure_Gv_R;
	float pre_Measure_Gv_L = Measure_Gv_L;

	Measure_Gv_R = sumR / 20;
	Measure_Gv_L = sumL / 20;



	Measure_Gv_R = 0.9*Measure_Gv_R+0.1*pre_Measure_Gv_R;// 1mm/ms
	Measure_Gv_L = 0.9*Measure_Gv_L+0.1*pre_Measure_Gv_L;


	printf("EncorderL__%.3f   EncorderR__%.3f\n\r",encoder_L, encoder_R);
	printf("sumL__%.3f   sumR__%.3f\n\r",sumL, sumR);
	printf("Tire_Speed_L__%.3f.Tire_Speed_R%.3f\n\r",G_Tire_Speed_L,G_Tire_Speed_R);


}*/
void Encorder_Speed_Calculate() {
    float sumL = 0, sumR = 0;

    // 前回の値を保存
    encoder_R_old = encoder_R;
    encoder_L_old = encoder_L;

    // 最新の角度を取得
    AS5047_DataUpdate();

    // 右車輪の差分計算
    encoder_R_delta = encoder_R - encoder_R_old;
    // 【修正ポイント】しきい値を -180 / +180 に変更
    if (encoder_R_delta < -180.0f) {
        encoder_R_delta += 360.0f;
    } else if (encoder_R_delta > 180.0f) {
        encoder_R_delta -= 360.0f;
    }

    // 左車輪の差分計算
    encoder_L_delta = encoder_L - encoder_L_old;
    // 【修正ポイント】しきい値を -180 / +180 に変更
    if (encoder_L_delta < -180.0f) {
        encoder_L_delta += 360.0f;
    } else if (encoder_L_delta > 180.0f) {
        encoder_L_delta -= 360.0f;
    }

    // 配列のシフト
    for (int i = 19; i > 0; i--) {
        Tire_Speed_R[i] = Tire_Speed_R[i - 1];
        Tire_Speed_L[i] = Tire_Speed_L[i - 1];
    }

    // 速度計算 (mm/s)
    Tire_Speed_R[0] = -1.0f * encoder_R_delta / (360.0f * reduction_ratio) * Tire_dia * PI * 1000.0f;
    Tire_Speed_L[0] = encoder_L_delta / (360.0f * reduction_ratio) * Tire_dia * PI * 1000.0f;

    // 単純な直近値の取得（移動平均を使わない場合）
    Measure_Gv_R = Tire_Speed_R[0];
    Measure_Gv_L = Tire_Speed_L[0];

    /*
    // ※もし5サンプル程度の移動平均をかける場合は以下のように記述します
    int filter_window = 5;
    for (int i = 0; i < filter_window; i++) {
        sumR += Tire_Speed_R[i];
        sumL += Tire_Speed_L[i];
    }
    Measure_Gv_R = sumR / (float)filter_window;
    Measure_Gv_L = sumL / (float)filter_window;
    */
}

void Encorder_count_mode() {
	encoder_R_sum += encoder_R_delta;
	encoder_L_sum += encoder_L_delta;

	if (encoder_R_sum > 500) {
		Program_number++;
		encoder_R_sum = 0;
	} else if (encoder_R_sum < -500) {
		Program_number--;
		encoder_R_sum = 0;
	}

	if (Program_number > 7) {
		Program_number = 0;
	} else if (Program_number < 0) {
		Program_number = 7;
	}

	if (encoder_L_sum > 500) {
		Program_mode--;
		encoder_L_sum = 0;
	} else if (encoder_L_sum < -500) {
		Program_mode++;
		encoder_L_sum = 0;
	}

	if (Program_mode > 15) {
		Program_mode = 0;
	} else if (Program_mode < 0) {
		Program_mode = 15;
	}

}

void Encorder_count_reset() {
	encoder_R_sum = 0;
	encoder_L_sum = 0;
	Program_number = 0;
	Program_mode = 0;
}

int Encorder_number_out() {
	return Program_number;
}

int Encorder_mode_out() {
	return Program_mode;
}

