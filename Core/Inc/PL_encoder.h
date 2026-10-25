/*
 * PL_encoder.h
 *
 *  Created on: Jan 6, 2023
 *      Author: sf199
 */

#ifndef INC_PL_ENCODER_H_
#define INC_PL_ENCODER_H_



extern float G_Tire_Speed_R,G_Tire_Speed_L;

void AS5047_DataUpdate();
void Encorder_Speed_Calculate();


void Encorder_count_mode();
void Encorder_count_reset();

int Encorder_number_out();
int Encorder_mode_out() ;

extern float encoder_L;
extern float encoder_R;
extern float encoder_count_L;
extern float encoder_count_R;
extern float Measure_omega_L;
extern float Measure_omega_R;
extern float Measure_Gv_L;
extern float Measure_Gv_R;
extern float Measure_Gv_L_sum;
extern float Measure_Gv_R_sum;


#endif /* INC_PL_ENCODER_H_ */
