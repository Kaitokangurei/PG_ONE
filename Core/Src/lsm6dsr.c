/*
 * imu.c
 *
 *  Created on: Jun 3, 2023
 *      Author: sato1
 */

#include "lsm6dsr.h"
#include "math.h"
#include "main.h"
#include "spi.h"
#include "stdio.h"
#include <stdint.h>
#include "led_control.h"
#include "MOTOR.h"
/*
 @brief spi : read 1 byte
 @param uint8_t Register
 @return read 1byte data
 */

float gyro_z_old = 0;
float gyro_z_offset = 0.2;

float accel_x_data[10];


float G_Gyro_X, G_Gyro_Y, G_Gyro_Z;
float G_Accel_X, G_Accel_Y, G_Accel_Z;
float Gyro_X_D, Gyro_Y_D, Gyro_Z_D;  // pre（LPF）
float accel_x_reference = 0;
float accel_y_reference = 0;
float accel_z_reference = 0;
float gyro_x_reference = 0;
float gyro_y_reference = 0;
float gyro_z_reference = 0;


uint8_t read_byte(uint8_t reg) {
	uint8_t ret;
	uint8_t dammy = 0x00;
	uint8_t bureg = (reg | 0x80);

	HAL_GPIO_WritePin(LSM6DSR_CS_GPIO_Port,LSM6DSR_CS_Pin, RESET);
	HAL_SPI_Transmit(&hspi1, &bureg, 1, 100);
	HAL_SPI_TransmitReceive(&hspi1, &dammy, &ret, 1, 100);
	HAL_GPIO_WritePin(LSM6DSR_CS_GPIO_Port, LSM6DSR_CS_Pin, SET);

	return ret;
}

/*
 @brief spi : write 1 byte
 @param uint8_t Register
 @param uint8_t Write Data
 */
void write_byte(uint8_t reg, uint8_t val) {
	uint8_t bureg = reg & 0x7F;

	HAL_GPIO_WritePin(LSM6DSR_CS_GPIO_Port, LSM6DSR_CS_Pin, RESET);
	HAL_SPI_Transmit(&hspi1, &bureg, 1, 100);
	HAL_SPI_Transmit(&hspi1, &val, 1, 100);
	HAL_GPIO_WritePin(LSM6DSR_CS_GPIO_Port, LSM6DSR_CS_Pin, SET);
}

/*
 * @breif initialize mpu 6500
 */

void lsm6dsr_init(void) {
	uint8_t who_am_i;

		HAL_Delay(1000); // wait start up
		who_am_i = read_byte(WHO_AM_I); // 1. read who am i
		printf("\r\n0x%x\r\n", who_am_i); // 2. check who am i value

		if(who_am_i==0){
			LED_All_On();
			while(1){
				printf("\r\n0x%x\r\n", who_am_i); // 2. check who am i value
			}

		}
		// 2. error check


		HAL_Delay(50); // wait

	//	write_byte( PWR_MGMT_1, 0x00); // 3. set pwr_might
	//
	//	HAL_Delay(50);
	//
	//	write_byte(MPU6500_RA_CONFIG, 0x00); // 4. set config
	//
	//	HAL_Delay(50);

		write_byte(CTRL2_G, GYRO_ODR_SET | GYRO_4000_DPS); // 5. set gyro config

		HAL_Delay(50);

		write_byte(CTRL1_XL, ACCEL_ODR_SET | ACCEL_8G); // 6. set accel config

		HAL_Delay(50);

}

float read_gyro_x_axis(void) {
    int16_t data = (int16_t)(((uint16_t)read_byte(OUTX_H_G) << 8) | (uint16_t)read_byte(OUTX_L_G));
    return (float)data * 140.0f / 1000.0f;  // スケールは ±4000dps の場合
}

float read_gyro_y_axis(void) {
    int16_t data = (int16_t)(((uint16_t)read_byte(OUTY_H_G) << 8) | (uint16_t)read_byte(OUTY_L_G));
    return (float)data * 140.0f / 1000.0f;
}

float read_gyro_z_axis() {
	float gyro_z;
	int16_t data = (int16_t) ((uint16_t) read_byte(OUTZ_H_G) << 8)| (uint16_t) read_byte(OUTZ_L_G);
	gyro_z = -1*(float) ((float) data * (1.0f) * 140.0f / 1000.0f);

	return gyro_z;
}



float read_accel_x_axis() {
	float accel_x = 0;

	int16_t data = (int16_t) ((uint16_t) (read_byte(OUTX_H_A) << 8)
			| (uint16_t) read_byte(OUTX_L_A));
	accel_x = (float) (data * 0.244 / 1000.0f * 9.8);

	return accel_x;
}

float read_accel_y_axis() {
	float accel_y = 0;

	int16_t data = (int16_t) ((uint16_t) (read_byte(OUTY_H_A) << 8)
			| (uint16_t) read_byte(OUTY_L_A));
	accel_y = (float) (data * 0.244 / 1000.0f * 9.8);
	return accel_y;
}

float read_accel_z_axis() {
	float accel_z = 0;

	int16_t data = (int16_t) ((uint16_t) (read_byte(OUTZ_H_A) << 8)
			| (uint16_t) read_byte(OUTZ_L_A));
	accel_z = (float) (data * 0.244 / 1000.0f * 9.8);
	return accel_z;
}

float read_NoiseCut_gyro_z() {

	float gyro_z = read_gyro_z_axis();

	float z = 1.0043*(gyro_z * 0.9 + gyro_z_old * 0.1) ;//- gyro_z_offset

	gyro_z_old = gyro_z;

	if(fabs(z)<1){
		z=0;
	}

	return z;
}

float read_average_acc_x() {
	float sum = 0;
	accel_x_data[0] = read_accel_x_axis();

	for (int i = 9; i > 0; i--) {
		accel_x_data[i] = accel_x_data[i - 1];
	}

	for (int i = 0; i < 10; i++) {
		sum += accel_x_data[i];
	}

	return sum/10;

}

void lsm6dsr_reference(){
    //float accel_x_sum = 0;
    float accel_y_sum = 0;
    //float accel_z_sum = 0;
   // float gyro_x_sum = 0;
    //float gyro_y_sum = 0;
    float gyro_z_sum = 0;
    HAL_Delay(2000); // 1ms 待機
    for(int i = 0; i < reset_reference_count; i++){
        HAL_Delay(1); // 1ms 待機
        //accel_x_sum += read_accel_x_axis();
        accel_y_sum += read_accel_y_axis();
        ///accel_z_sum += read_accel_z_axis();
        //gyro_x_sum  += read_gyro_x_axis(); // ADD THIS
        //gyro_y_sum  += read_gyro_y_axis(); // ADD THIS
        gyro_z_sum  += read_gyro_z_axis();
    }

    //accel_x_reference = accel_x_sum / reset_reference_count;
    accel_y_reference = accel_y_sum / reset_reference_count;
   // accel_z_reference = accel_z_sum / reset_reference_count;
   // gyro_x_reference  = gyro_x_sum  / reset_reference_count;
   // gyro_y_reference  = gyro_y_sum  / reset_reference_count;
    gyro_z_reference  = gyro_z_sum  / reset_reference_count;
}

static float prev_G_Gyro_Z = 0.0f; // 前回の G_Gyro_Z の値
static uint16_t no_change_count = 0; // G_Gyro_Z が変化しなかった連続回数
#define NO_CHANGE_THRESHOLD 0.1f // G_Gyro_Z が変化なしとみなす閾値 (例: 0.01 deg/s)
#define NO_CHANGE_MAX_COUNT 100   // 変化なしを判定する連続回数

void lsm6dsr_get(void) {
    // （m/s^2）
    //G_Accel_X = read_accel_x_axis() - accel_x_reference;
    G_Accel_Y = read_accel_y_axis();// - accel_y_reference;
    //G_Accel_Z = read_accel_z_axis() - accel_z_reference;

    // pre_Gyro
    //Gyro_X_D = G_Gyro_X;
   // Gyro_Y_D = G_Gyro_Y;
    Gyro_Z_D = G_Gyro_Z;

    if(Gyro_Z_D<1&&Gyro_Z_D>-1){
    	Gyro_Z_D=0;
    }


    G_Gyro_X = ((read_gyro_x_axis() - gyro_x_reference) * 0.9f + Gyro_X_D * 0.1f);
    G_Gyro_Y = ((read_gyro_y_axis() - gyro_y_reference) * 0.9f + Gyro_Y_D * 0.1f);
    G_Gyro_Z = ((read_gyro_z_axis() - gyro_z_reference) * 0.9f + Gyro_Z_D * 0.1f);
    if(G_Gyro_Z <1&&G_Gyro_Z >-1){
    	G_Gyro_Z =0;
    }
}
void lsm6dsr_print(){
/*    printf("Gyroscope: X=%.2f deg/s, Y=%.2f deg/s, Z=%.2f deg/s\n\r",
           G_Gyro_X, G_Gyro_Y, G_Gyro_Z);
    printf("Accelerometer: X=%.2f m/s^2, Y=%.2f m/s^2, Z=%.2f m/s^2\n\r",
           G_Accel_X, G_Accel_Y, G_Accel_Z);*/
    printf("Y=%.2f m/s^2,Z=%.2f deg/s, %.2f\n\r",G_Accel_Y,G_Gyro_Z, Measure_degree);
}

