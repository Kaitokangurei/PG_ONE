/*
 * MOTOR.h
 *
 *  Created on: Nov 25, 2024
 *      Author: lingm
 */

#ifndef SRC_MOTOR_H_
#define SRC_MOTOR_H_



void Motor_PWM_Generate() ;
void Motor_Setup() ;
void Motor_PWM_Stop();

void FUN_control();
void FUN_PWM_Setup();
void FUN_PWM_Generate();
void FUN_PWM_Stop();

void Motor_Control_Main(void);
void Motor_Task();

void Feedforward_control();


void Feedforward_calculate_duty_ratio(float, float , float , float  );
//void motor_variable_accel( *);
//void sta_change_para();


void motor_daikei(float , float , float , float , float );
void motor_daikei_sta(float , float , float , float , float );
void motor_daikei_kabekire(float , float , float , float , float );
void motor_daikei_back(float , float , float , float , float );
void omega_daikei_L(float , float , float , float , float );
void omega_daikei_R(float , float , float , float , float );
void senkai_omega_daikei_L(float , float , float , float , float );
void senkai_omega_daikei_R(float , float , float , float , float );
void calculate_duty_ratio(float , float , float , float );
void failsafe();
int check_lifted_condition(float , float );
int check_angle_error_condition(float, float );
int check_velocity_error_condition(float , float, float);
extern float Gv;
extern float Gt;
extern float Gx;
extern float Ga;
extern float Gomega;
extern float Gomega_acc;
extern float Gdegree;
extern float Measure_degree;

extern int motor_stop_flag;
extern int failsafe_flag;

extern int FUN_MOTOR_flag;
extern int senkai_flag;



extern float duty_ratio_L, duty_ratio_R;
extern float ff_duty_ratio_L, ff_duty_ratio_R;

extern float Motor_PWM_L,Motor_PWM_R;
#endif /* SRC_MOTOR_H_ */
