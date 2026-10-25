
/*
 * wall.c
 *
 *  Created on: Nov 27, 2024
 *      Author: lingm
 */

#include "spi.h"
#include "main.h"
#include "adc.h"
//#include "usart.h"
#include "PL_sensor.h"
#include "wall.h"
#include "stdio.h"
#include <stdlib.h>
#include "math.h"
#include "stdio.h"
#include "MOTOR.h"
#include "PID.h"
#include "led_control.h"
#include "SPEAKER.h"
#include "define.h"
#include "log.h"

int g_wall_control_status = 0;
int pre_wall_control_status =0;
int wall_counter[4] = {0, 0, 0, 0}; // 各センサの連続検知回数を記録
int wall_flag[4] = {0, 0, 0, 0}; // 各センサの壁検知フラグ
float PID_wall;
float pre_PID_wall;

//kabekire

int MAX_SENSOR_L = 300;
int MAX_SENSOR_R = 300;
int Wall_kabegire_Threshold[4] = {100,60, 60, 100};
int Wall_kabegire_End_Threshold[4] = {220,60, 60, 270};
int wall_maekabe_Threshold[4] = {230, 20, 20, 140};
float Wall_kabegire_smoothed_diff_Threshold[4] = {10,2,2,10};

int Wall_kabegire_diagoal_Threshold[4] = {400, 25, 25, 400};
int Wall_kabegire_diagoal_DIFF[4]= {30, 0, 0, 30};
int Wall_kabegire_diagoal_End_Threshold[4] = {50, 80,120, 50};

//kabeseigyo
//int PID_CENTER_L=850;
//int PID_CENTER_R=520;
//int PID_CENTER_L=900;
//int PID_CENTER_R=470;
int PID_CENTER_L=228;
int PID_CENTER_R=170;
int PID_CENTER_L_F =70;
int PID_CENTER_R_F =70;
int Wall_Threshold[4] = {70, 120,180 , 70}; // 各センサの連続検知回数を記録]
int THRESHOLD_DIFF[4]= {600,2, 2, 1600};
int THRESHOLD_DIFF_tansaku[4]= {10, 3, 3, 10};


int FRONT_CENTER_L=580;//kabeate
int FRONT_CENTER_R=534;

int wall_kabegire_counter[4] = {0, 0, 0, 0}; // 各センサの連続検知回数を記録
int wall_kabegire_status[4] = {0, 0, 0, 0}; // 各センサの壁検知フラグ
int wall_kabegire_R_flag;//右の柱の読み取り開始
int wall_kabegire_L_flag;//左の柱の読み取り開始
int wall_kabegire_flag;//壁のなくなり判定

int diagonal_THRESHOLD_L=270;//(2)
int diagonal_THRESHOLD_R=270;//(1)

int wall_conf_flag;
int wall_get_flag;
int wall_distance_flag;
int wall_maekyori_flag;
int wall_start_flag;

int dtkabegire_flag;

float g_sensors_smoothed_diff[4];

void get_wall() {
	if(wall_conf_flag==1){
		for (int i = 0; i < 4; i++) {
			if (wall_counter[i] >= 1) {
				if ((g_sensor[i][0] >= Wall_Threshold[i]) && (abs(g_sensor_diff[i]) <= THRESHOLD_DIFF_tansaku[i])) {
					g_wall_control_status |= (1 << i); // iビット目を1にする
				}else{
					g_wall_control_status &= ~(1 << i); // iビット目を0にする
				}
			} else {
				g_wall_control_status &= ~(1 << i); // iビット目を0にする

			}


			if (g_sensor[i][0] >= Wall_Threshold[i]){
				wall_counter[i]=1;
			}else{
				wall_counter[i]=0;
			}
		}
	}
	else if(wall_conf_flag==2||wall_conf_flag==3||wall_conf_flag==4||wall_conf_flag==5){
		for (int i = 0; i < 4; i++) {
			if (wall_counter[i] >= 1) {
				if ((g_sensor[i][0] >= Wall_Threshold[i]) && (abs(g_sensor_diff[i]) <= THRESHOLD_DIFF[i])) {
					g_wall_control_status |= (1 << i); // iビット目を1にする
				}else{
					g_wall_control_status &= ~(1 << i); // iビット目を0にする
				}
			} else {
				g_wall_control_status &= ~(1 << i); // iビット目を0にする

			}


			if (g_sensor[i][0] >= Wall_Threshold[i]){
				wall_counter[i]=1;
			}else{
				wall_counter[i]=0;
			}
		}
	}
}
/*
void get_wall() {
if(wall_conf_flag==1){
    for (int i = 0; i < 4; i++) {
    	if ((g_sensor[i][0] >= Wall_Threshold[i]) && (g_sensor_diff[i] <= THRESHOLD_DIFF_tansaku[i])) {
            wall_flag[i] = 1;
            wall_counter[i]++;



        } else {

            wall_flag[i] = 0;
            wall_counter[i] = 0;

        }

        if (wall_counter[i] >= 1) {
            g_wall_control_status |= (1 << i); // iビット目を1にする
        } else {
            g_wall_control_status &= ~(1 << i); // iビット目を0にする

        }
    }
}
else if(wall_conf_flag==2||wall_conf_flag==3){

    for (int i = 0; i < 4; i++) {
    	if ((g_sensor[i][0] >= Wall_Threshold[i]) && (g_sensor_diff[i] <= THRESHOLD_DIFF[i])) {
            wall_flag[i] = 1;
            wall_counter[i]++;



        } else {

            wall_flag[i] = 0;
            wall_counter[i] = 0;

        }

        if (wall_counter[i] >= 1) {
            g_wall_control_status |= (1 << i); // iビット目を1にする
        } else {
            g_wall_control_status &= ~(1 << i); // iビット目を0にする

        }
    }
}
}*/


void wall_control() {

/*	if(pre_wall_control_status == g_wall_control_status ){
      	pre_PID_wall = 0;
		PID_wall=0;
	}*/
/*	float margin_L = PID_CENTER_L * 0.05f;
	float margin_R = PID_CENTER_R * 0.05f;

	// Calculate the absolute error
	float error_L = fabsf((float)g_sensor[2][0] - PID_CENTER_L);
	float error_R = fabsf((float)g_sensor[1][0] - PID_CENTER_R);

	// If both sensors are within 5% of their centers
	if (error_L <= margin_L && error_R <= margin_R) {
	    pre_PID_wall = 0;
	}
	pre_wall_control_status = g_wall_control_status ;*/

	pre_PID_wall =PID_wall;
	if(failsafe_flag!=1){// &&Gv!=0){
		switch (wall_conf_flag) { // tansaku
		    case 1:
		        if ((g_wall_control_status & (1 << 1)) && (g_wall_control_status & (1 << 2))) {
		            PID_wall = SENSOR_GAIN * (-(float)(g_sensor[2][0] - PID_CENTER_L) + (float)(g_sensor[1][0] - PID_CENTER_R));
		            LED_Right_On(); LED_Left_On();
		        } else if ((g_wall_control_status & (1 << 1)) && !(g_wall_control_status & (1 << 2))) {
		            // 右壁のみ：右に近づいたら左に補正（プラス方向）
		            PID_wall = SENSOR_GAIN * (2.0f * (float)(g_sensor[1][0] - PID_CENTER_R));
		            LED_Right_On(); LED_Left_Off();
		        } else if (!(g_wall_control_status & (1 << 1)) && (g_wall_control_status & (1 << 2))) {
		            // 左壁のみ：左に近づいたら右に補正（マイナス方向）
		            PID_wall = -SENSOR_GAIN * (2.0f * (float)(g_sensor[2][0] - PID_CENTER_L));
		            LED_Right_Off(); LED_Left_On();
		        } else {
		            PID_wall = 0.0f;
		            LED_Right_Off(); LED_Left_Off();
		        }

		        if(g_sensor[0][0]+g_sensor[3][0]>=200){
		        	PID_wall =0;
		        }
		        // ローパスフィルタ処理と前回値の保存（★バグ修正）
		        PID_wall = PID_wall * 0.70f + pre_PID_wall * 0.30f;
		        pre_PID_wall = PID_wall;
		        break;

		    case 2: // naname
/*		        if ((g_wall_control_status & (1 << 0)) && (g_wall_control_status & (1 << 3))) {
		            PID_wall = SENSOR_GAIN2 * (-(float)(g_sensor[0][0] - PID_CENTER_L_F) + (float)(g_sensor[3][0] - PID_CENTER_R_F));
		            LED_Right_On(); LED_Left_On();
		        } else */if ((g_wall_control_status & (1 << 0)) && !(g_wall_control_status & (1 << 3))) {
		            // 左壁のみ（★符号のバグを修正：左に近づいたら右に逃げるためマイナスにする）
		            PID_wall = -SENSOR_GAIN2 * (2.0f * (float)(g_sensor[0][0] - PID_CENTER_L_F));
		            LED_Right_On(); LED_Left_Off();
		            LED_Right_Off(); LED_Left_On();
		        } else if (!(g_wall_control_status & (1 << 0)) && (g_wall_control_status & (1 << 3))) {
		            // 右壁のみ（★符号のバグを修正：右に近づいたら左に逃げるためプラスにする）
		            PID_wall = SENSOR_GAIN2 * (2.0f * (float)(g_sensor[3][0] - PID_CENTER_R_F));
		            LED_Right_On(); LED_Left_Off();
		        } else {
		            PID_wall = 0.0f;
		            LED_Right_Off(); LED_Left_Off();
		        }

		        // 斜めもノイズ対策でフィルタを入れるのがおすすめ
		        PID_wall = PID_wall * 0.80f + pre_PID_wall * 0.20f;
		        pre_PID_wall = PID_wall;
		        PID_wall = (PID_wall / 1500.0f) * Gv;
		        // リミッタ処理
		        if (PID_wall > 200.0f)  PID_wall = 200.0f;
		        if (PID_wall < -200.0f) PID_wall = -200.0f;
		        break;

		    case 3: // saitan
		        if ((g_wall_control_status & (1 << 1)) && (g_wall_control_status & (1 << 2))) {
		            PID_wall = SENSOR_GAIN3 * (-(float)(g_sensor[2][0] - PID_CENTER_L) + (float)(g_sensor[1][0] - PID_CENTER_R));
		            LED_Right_On(); LED_Left_On();
		        } else if ((g_wall_control_status & (1 << 1)) && !(g_wall_control_status & (1 << 2))) {
		            // 右壁のみ
		            PID_wall = SENSOR_GAIN3 * (2.0f * (float)(g_sensor[1][0] - PID_CENTER_R));
		            LED_Right_On(); LED_Left_Off();
		        } else if (!(g_wall_control_status & (1 << 1)) && (g_wall_control_status & (1 << 2))) {
		            // 左壁のみ
		            PID_wall = -SENSOR_GAIN3 * (2.0f * (float)(g_sensor[2][0] - PID_CENTER_L));
		            LED_Right_Off(); LED_Left_On();
		        } else {
		            PID_wall = 0.0f;
		            LED_Right_Off(); LED_Left_Off();
		        }

		        // ローパスフィルタ処理と前回値の保存（★バグ修正）
		        PID_wall = PID_wall * 0.90f + pre_PID_wall * 0.10f;
		        pre_PID_wall = PID_wall;

		        // 速度制御依存（分母の調整は実機に合わせて行ってください）
		        PID_wall = (PID_wall / 1000.0f) * Gv;

		        // リミッタ処理
		        if (PID_wall > 500.0f)  PID_wall = 500.0f;
		        if (PID_wall < -500.0f) PID_wall = -500.0f;
		        break;

		    case 4: // naname//L
		    	if ((g_wall_control_status & (1 << 0)) && !(g_wall_control_status & (1 << 3))) {
		            // 左壁のみ（★符号のバグを修正：左に近づいたら右に逃げるためマイナスにする）
		            PID_wall = -SENSOR_GAIN2 * (2.0f * (float)(g_sensor[0][0] - PID_CENTER_L_F));
		            LED_Right_On(); LED_Left_Off();
		        }else {
		            PID_wall = 0.0f;
		            LED_Right_Off(); LED_Left_Off();
		        }

		        // 斜めもノイズ対策でフィルタを入れるのがおすすめ
		        PID_wall = PID_wall * 0.80f + pre_PID_wall * 0.20f;
		        pre_PID_wall = PID_wall;
		        PID_wall = (PID_wall / 1500.0f) * Gv;
		        // リミッタ処理
		        if (PID_wall > 20.0f)  PID_wall = 200.0f;
		        if (PID_wall < -20.0f) PID_wall = -200.0f;
		        break;
		    case 5: // naname R
		    	if (!(g_wall_control_status & (1 << 0)) && (g_wall_control_status & (1 << 3))) {
		            // 右壁のみ（★符号のバグを修正：右に近づいたら左に逃げるためプラスにする）
		            PID_wall = SENSOR_GAIN2 * (2.0f * (float)(g_sensor[3][0] - PID_CENTER_R_F));
		            LED_Right_Off(); LED_Left_On();
		        } else {
		            PID_wall = 0.0f;
		            LED_Right_Off(); LED_Left_Off();
		        }

		        // 斜めもノイズ対策でフィルタを入れるのがおすすめ
		        PID_wall = PID_wall * 0.80f + pre_PID_wall * 0.20f;
		        pre_PID_wall = PID_wall;
		        PID_wall = (PID_wall / 1500.0f) * Gv;
		        // リミッタ処理
		        if (PID_wall > 20.0f)  PID_wall = 200.0f;
		        if (PID_wall < -20.0f) PID_wall = -200.0f;
		        break;

		    default:
		        PID_wall = 0.0f;
		        pre_PID_wall = 0.0f;
		        if (Gv != 0) {
		            LED_Right_Off();
		            LED_Left_Off();
		        }
		        break;
		}

	}
}
// =============================================================================
// 壁切れ制御メインタスク（周期処理から呼出）
// =============================================================================
void wall_kabegire_control(void) {
    // 直進時の壁切れ検知 (flag == 1)
    if (wall_kabegire_R_flag == 1)  wall_kabegire_R();
    if (wall_kabegire_L_flag == 1)  wall_kabegire_L();

    // ありなし (flag == -1)
    if (wall_kabegire_R_flag == -1) wall_kabegire_R_SL();
    if (wall_kabegire_L_flag == -1) wall_kabegire_L_SL();

    // 斜め走行用壁切れ (flag == 2)
    if (wall_kabegire_R_flag == 2)  wall_kabegire_R_DL();
    if (wall_kabegire_L_flag == 2)  wall_kabegire_L_DL();
}

// =============================================================================
// 各種壁切れ判定ロジック
// =============================================================================
int dt_flag=0;
void wall_kabegire_R(void) {
	if(dtkabegire_flag==1){
	    if (g_sensors_smoothed_diff[1] <= -Wall_kabegire_smoothed_diff_Threshold[1]){
	    	dt_flag=1;
	    }
	    if (dt_flag==1
			&&(g_sensor[1][0]<=Wall_kabegire_End_Threshold[1])) {
	        wall_kabegire_flag = 1;
	        LED_Right_On();
	        dt_flag=0;
	    }
	}else{
	    if (g_sensor[1][0]<=Wall_kabegire_End_Threshold[1]) {
	        wall_kabegire_flag = 1;
	        LED_Right_On();
	    }
	}


}

void wall_kabegire_L(void) {
	if(dtkabegire_flag==1){
	    if (g_sensors_smoothed_diff[2] <= -Wall_kabegire_smoothed_diff_Threshold[2]){
	    	dt_flag=1;
	    }
	    if (dt_flag==1
			&&(g_sensor[2][0]<=Wall_kabegire_End_Threshold[2])) {
	        wall_kabegire_flag = 1;

			LED_Left_On();
	        dt_flag=0;
	    }
	}
	else{
		if (g_sensor[2][0]<=Wall_kabegire_End_Threshold[2]) {
			wall_kabegire_flag = 1;

			LED_Left_On();
		}
	}
}

void wall_kabegire_R_DL(void) {
/*    if (wall_kabegire_status[1] == 0) {
        if (g_sensor[1][0] <= Wall_kabegire_diagoal_End_Threshold[1]) {
            wall_kabegire_flag = 1;
        }
    }*/
    if (g_sensor[1][0] <= Wall_kabegire_diagoal_End_Threshold[1]) {
        wall_kabegire_flag = 1;
    }
/*
    if (g_sensors_smoothed_diff[1] <= -Wall_kabegire_smoothed_diff_Threshold[1]) {
        wall_kabegire_flag = 1;
        LED_Right_On();
    }*/
}

void wall_kabegire_L_DL(void) {
    // 【修正】status[2] を見るように変更
/*    if (wall_kabegire_status[2] == 0) {
        if (g_sensor[2][0] <= Wall_kabegire_diagoal_End_Threshold[2]) {
            wall_kabegire_flag = 1;
        }
    }*/
    if (g_sensor[2][0] <= Wall_kabegire_diagoal_End_Threshold[2]) {
        wall_kabegire_flag = 1;
    }
/*
    if (g_sensors_smoothed_diff[2] <= -Wall_kabegire_smoothed_diff_Threshold[2]) {
        wall_kabegire_flag = 1;
        LED_Left_On();
    }*/
}

void wall_kabegire_R_SL(void) {
    if (g_sensor[1][0] <= Wall_kabegire_End_Threshold[1]) {
        wall_kabegire_flag = 1;
    }
}

void wall_kabegire_L_SL(void) {
    if (g_sensor[2][0] <= Wall_kabegire_End_Threshold[2]) {
        wall_kabegire_flag = 1;
    }
}

// =============================================================================
// 壁切れ検知待ち関数（ブロッキング関数）
// =============================================================================
void wall_kabegire_L_R_detection(int kabegire_gyro_flag, int kabegire_wall_flag) {
    Gx = 0;
    PID_turn_flag = kabegire_gyro_flag;
    wall_conf_flag = kabegire_wall_flag;
    wall_kabegire_L_START();
    wall_kabegire_R_START();
    while (1) {
        if (wall_kabegire_flag == 1) {
            LED_All_On();
            wall_kabegire_ALL_RESET();
            break;
        } else if (Gx >= 360) {
            wall_kabegire_ALL_RESET();
            failsafe_flag = 1;
            break;
        }
    }
}

void wall_kabegire_L_detection(int kabegire_gyro_flag, int kabegire_wall_flag) {
    Gx = 0;
    PID_turn_flag = kabegire_gyro_flag;
    wall_conf_flag = kabegire_wall_flag;
    wall_kabegire_L_START();
    while (1) {
        if (wall_kabegire_flag == 1) {
            LED_All_On();
            wall_kabegire_ALL_RESET();
            break;
        } else if (Gx >= 360) {
            wall_kabegire_ALL_RESET();
            failsafe_flag = 1;
            break;
        }
    }
}

void wall_kabegire_R_detection(int kabegire_gyro_flag, int kabegire_wall_flag) {
    Gx = 0;
    PID_turn_flag = kabegire_gyro_flag;
    wall_conf_flag = kabegire_wall_flag;
    wall_kabegire_R_START();
    while (1) {
        if (wall_kabegire_flag == 1) {
            LED_Right_On();
            wall_kabegire_ALL_RESET();
            break;
        } else if (Gx >= 360) {
            wall_kabegire_ALL_RESET();
            failsafe_flag = 1;
            break;
        }
    }
}

void wall_kabegire_L_detection_DL(int kabegire_gyro_flag, int kabegire_wall_flag) {
    Gx = 0;
    PID_turn_flag = kabegire_gyro_flag;
    wall_conf_flag = kabegire_wall_flag;
    wall_kabegire_L_START_DL();
    while (1) {
        if (wall_kabegire_flag == 1) {
            LED_All_On();
            wall_kabegire_ALL_RESET();
            break;
        } else if (Gx >= 150) {
            wall_kabegire_ALL_RESET();
            break;
        }
    }
}

void wall_kabegire_R_detection_DL(int kabegire_gyro_flag, int kabegire_wall_flag) {
    Gx = 0;
    PID_turn_flag = kabegire_gyro_flag;
    wall_conf_flag = kabegire_wall_flag;
    wall_kabegire_R_START_DL();
    while (1) {
        if (wall_kabegire_flag == 1) {
            LED_All_On();
            wall_kabegire_ALL_RESET();
            break;
        } else if (Gx >= 150) {
            wall_kabegire_ALL_RESET();
            break;
        }
    }
}

void wall_kabegire_L_detection_SL(int kabegire_gyro_flag, int kabegire_wall_flag) {
    Gx = 0;
    PID_turn_flag = kabegire_gyro_flag;
    wall_conf_flag = kabegire_wall_flag;
    wall_kabegire_L_START_SL();
    while (1) {
        if (wall_kabegire_flag == 1) {
            wall_kabegire_ALL_RESET();
            break;
        } else if (Gx >= 180) {
            wall_kabegire_ALL_RESET();
            failsafe_flag = 1;
            break;
        }
    }
}

void wall_kabegire_R_detection_SL(int kabegire_gyro_flag, int kabegire_wall_flag) {
    Gx = 0;
    PID_turn_flag = kabegire_gyro_flag;
    wall_conf_flag = kabegire_wall_flag;
    wall_kabegire_R_START_SL();
    while (1) {
        if (wall_kabegire_flag == 1) {
            wall_kabegire_ALL_RESET();
            break;
        } else if (Gx >= 180) {
            wall_kabegire_ALL_RESET();
            failsafe_flag = 1;
            break;
        }
    }
}

// =============================================================================
// 開始・リセット関数
// =============================================================================
void wall_kabegire_R_START(void) {
    wall_kabegire_status[1] = 0;
    wall_kabegire_counter[1] = 0;
    wall_kabegire_R_flag = 1;
    wall_start_flag = 0;
    wall_sensor_smoothed_diff_RESET();
    wall_kabegire_flag = 0;
}

void wall_kabegire_R_RESET(void) {
    wall_kabegire_flag = 0;
    wall_kabegire_status[1] = 0;
    wall_kabegire_counter[1] = 0;
    wall_kabegire_R_flag = 0;
    wall_sensor_smoothed_diff_RESET();
}

void wall_kabegire_L_START(void) {
    wall_kabegire_status[2] = 0;
    wall_kabegire_counter[2] = 0;
    wall_kabegire_L_flag = 1;
    wall_start_flag = 0;
    wall_sensor_smoothed_diff_RESET();
    wall_kabegire_flag = 0;
}

void wall_kabegire_L_RESET(void) {
    wall_kabegire_flag = 0;
    wall_kabegire_status[2] = 0;
    wall_kabegire_counter[2] = 0;
    wall_kabegire_L_flag = 0;
    wall_sensor_smoothed_diff_RESET();
}

void wall_kabegire_R_START_SL(void) {
    wall_kabegire_flag = 0;
    wall_kabegire_status[1] = 0;
    wall_kabegire_counter[1] = 0;
    wall_kabegire_R_flag = -1;
    wall_start_flag = 0;
}

void wall_kabegire_L_START_SL(void) {
    wall_kabegire_flag = 0;
    wall_kabegire_status[2] = 0;
    wall_kabegire_counter[2] = 0;
    wall_kabegire_L_flag = -1;
    wall_start_flag = 0;
}

void wall_kabegire_R_START_DL(void) {
    wall_kabegire_flag = 0;
    wall_kabegire_status[1] = 0;
    wall_kabegire_counter[1] = 0;
    wall_kabegire_R_flag = 2;
    wall_start_flag = 0;
}

void wall_kabegire_L_START_DL(void) {
    wall_kabegire_flag = 0;
    wall_kabegire_status[2] = 0;
    wall_kabegire_counter[2] = 0;
    wall_kabegire_L_flag = 2;
    wall_start_flag = 0;
}

void wall_kabegire_ALL_START(void) {
    wall_kabegire_L_START();
    wall_kabegire_R_START();
}

void wall_kabegire_ALL_RESET(void) {
    wall_kabegire_L_RESET();
    wall_kabegire_R_RESET();
}

void wall_maekyori_control(){
	if(wall_distance_flag==1){
		wall_MAX_sensor_diff();
	}else if(wall_kabegire_status[1]==2||wall_kabegire_status[2]==2){
		wall_distance_flag=0;
	}
}
int max_diff_R = 0;
int max_diff_L = 0;
int max_diff = 0;
int maekyori_L_R_flag=0;

void wall_MAX_sensor_diff(){
    if (g_sensor[1][0] > max_diff_R) {
    	if(g_sensor[1][0]>MAX_SENSOR_R){
    		max_diff_R = MAX_SENSOR_R;
    	}
    	else{
    		max_diff_R = g_sensor[1][0];
    	}
    }

    if (g_sensor[2][0] > max_diff_L) {
    	if(g_sensor[1][0]>MAX_SENSOR_R){
    		max_diff_L = MAX_SENSOR_L;
    	}else{
    		max_diff_L = g_sensor[2][0];
    	}
    }

    // 修正前:
    if (g_sensor[2][0] > max_diff_L) {
        if(g_sensor[1][0] > MAX_SENSOR_R){ // ★ミス: 右センサーを見ている
            max_diff_L = MAX_SENSOR_L;
        }else{
            max_diff_L = g_sensor[2][0];
        }
    }

    // 修正後:
    if (g_sensor[2][0] > max_diff_L) {
        if(g_sensor[2][0] > MAX_SENSOR_L){
            max_diff_L = MAX_SENSOR_L;
        }else{
            max_diff_L = g_sensor[2][0];
        }
    }

    if(max_diff < 0){
    	max_diff =0;
    	maekyori_L_R_flag=0;
    }
    //log_data(g_sensor[1][0], g_sensor[2][0],max_diff_R,max_diff_L,max_diff);
}

void wall_center_distance_START(){
	max_diff_R = 0;
	max_diff_L = 0;
	wall_distance_flag=1;
}
void wall_center_distance_RESET(){
	max_diff_R = 0;
	max_diff_L = 0;
	wall_distance_flag=0;
}
float calculate_maekyori(int makyori_dir){//R 1 L2 曲がりたい方向の指定
	float wall_distance_X=0;
	float MAEKYORI_GAIN = (float)(CENTER_WALL_DISTANCE)/(MAX_SENSOR_R * MAX_SENSOR_R) * (max_diff * max_diff);

	if(maekyori_L_R_flag==1){//右寄り　//右壁からの距離
		if(makyori_dir==1){
			wall_distance_X = CENTER_WALL_DISTANCE - MAEKYORI_GAIN;
		}else if(makyori_dir==2){
			wall_distance_X = CENTER_WALL_DISTANCE + MAEKYORI_GAIN;
		}else{
			wall_distance_X=0;
		}
	}else if(maekyori_L_R_flag==2){//L寄り　//左壁からの距離
		if(makyori_dir==1){
			wall_distance_X = CENTER_WALL_DISTANCE+ MAEKYORI_GAIN;
		}else if(makyori_dir==2){
			wall_distance_X = CENTER_WALL_DISTANCE- MAEKYORI_GAIN;
		}else{
			wall_distance_X=0;
		}
	}else {
		wall_distance_X=0;
	}

	return wall_distance_X;//曲がりたい壁からの距離
}

float calculate_diagoal_maekyori(int makyori_dir){//R 1 L2 曲がりたい方向の指定
	float wall_distance_X=0;
	float MAEKYORI_GAIN = (float)(CENTER_DIAGOAL_WALL_DISTANCE - 15) / (MAX_SENSOR_R * MAX_SENSOR_R) * (max_diff * max_diff);

	if(maekyori_L_R_flag==1){//右寄り　//右壁からの距離
		if(makyori_dir==1){
			wall_distance_X = CENTER_DIAGOAL_WALL_DISTANCE- MAEKYORI_GAIN;
		}else if(makyori_dir==2){
			wall_distance_X = CENTER_DIAGOAL_WALL_DISTANCE+ MAEKYORI_GAIN;
		}else{
			wall_distance_X=0;
		}
	}else if(maekyori_L_R_flag==2){//L寄り　//左壁からの距離
		if(makyori_dir==1){
			wall_distance_X = CENTER_DIAGOAL_WALL_DISTANCE + MAEKYORI_GAIN;
		}else if(makyori_dir==2){
			wall_distance_X = CENTER_DIAGOAL_WALL_DISTANCE - MAEKYORI_GAIN;
		}else{
			wall_distance_X=0;
		}
	}else {
		wall_distance_X=0;
	}

	return wall_distance_X;//曲がりたい壁からの距離
}
void wall_distance_move(void) {
	uint16_t sensor_l = g_sensor[0][0];
	uint16_t sensor_r = g_sensor[3][0];
	uint16_t g_sensor_front_sum = sensor_l + sensor_r;

	// --- 安全装置（壁がない場合は制御しない） ---
	if (g_sensor_front_sum <= 250) {
		Gv = 0.0f;
		Gomega = 0.0f;
		return;
	}

	// --- 【追加】近距離反転の検知と強制後退フェイルセーフ ---
	// 前方センサの合計値の減少量（前回 - 今回）を計算
	int diff_sum = (pre_g_sensor[0] + pre_g_sensor[3]) - g_sensor_front_sum;

	// 「壁にある程度近い(合計700以上)」かつ「前進しようとしている(Gv > 0)」かつ
	// 「センサ値が減少している(diff_sum > 10)」場合、ピークを越えたと判定する
	if (g_sensor_front_sum > 700 && Gv > 0.0f && diff_sum > 10) {
		// 反転領域に突入したため、目標値との誤差計算を無視して強制的に後退させる
		Gv = -150.0f; // 安全に下がれる速度を指定
		Gomega = 0.0f;
		return;       // ここで関数を抜ける
	}

    // --- 1. 直進速度 (Gv) の計算 ---
    // 近いとき (sensor_front_sum > target_sum) -> distance_error は「負」
    float target_sum = (float)FRONT_CENTER_L + (float)FRONT_CENTER_R;
    float distance_error = target_sum - g_sensor_front_sum;

    float Kp_v = 1.0f;
    Gv = Kp_v * distance_error;

    // リミッター
    if (Gv > 300.0f) {
        Gv = 300.0f;
    } else if (Gv < -300.0f) {
        Gv = -300.0f;
    }

    // 不感帯
    if (fabsf(distance_error) < 15.0f) {
        Gv = 0.0f;
    }

    // --- 2. 旋回速度 (Gomega) の計算 ---
    float left_ratio  = (sensor_l - (float)FRONT_CENTER_L) / (float)FRONT_CENTER_L;
    float right_ratio = (sensor_r - (float)FRONT_CENTER_R) / (float)FRONT_CENTER_R;

    float Kp_omega = -350.0f;
    Gomega = Kp_omega * (left_ratio - right_ratio);

    if (Gomega > 90.0f) {
        Gomega = 90.0f;
    } else if (Gomega < -90.0f) {
        Gomega = -90.0f;
    }

/*    if(Gv<=-100){
    	 Gomega = 0.0f;
    }*/

    if (fabsf(left_ratio - right_ratio) < 0.03f) {
        Gomega = 0.0f;
    }
}

void print_wall_control_status(){
	for(int i=4;i>0;i--){
		if( (g_wall_control_status & (1 << i)) != 0){
			printf("%d bit:1, ",i);

		}else{
			printf("%d bit:0, ",i);
		}
	}
	printf("\n\r");
}

void print_avg_sensor_100(){
	printf("%d,%d,%d,%d\n\r",g_sensor[0][0],g_sensor[1][0],g_sensor[2][0],g_sensor[3][0]);
}

void print_g_sensor(){
    // g_sensorの値をカンマ区切りで表示し、その後にg_sensor_onとg_sensor_offの
    // インデックス0, 1, 2, 3の値をそれぞれ表示します。
    printf("%d,%d,%d,%d\n\r" // g_sensor[0][0]～g_sensor[3][0]
           "on:%d,%d,%d,%d\n\r"  // g_sensor_on[0]～g_sensor_on[3]
           "off:%d,%d,%d,%d\n\r" // g_sensor_off[0]～g_sensor_off[3]
          , g_sensor[0][0], g_sensor[1][0], g_sensor[2][0], g_sensor[3][0],
           g_sensor_on[0], g_sensor_on[1], g_sensor_on[2], g_sensor_on[3],
           g_sensor_off[0], g_sensor_off[1], g_sensor_off[2], g_sensor_off[3]
          );
	printf("    %d,%d,%d,%d\r\n"   // センサー生値 (g_sensor[n][0])
	       "diff%d,%d,%d,%d\r\n",  // センサー差分値 (g_sensor_diff[n])
	       g_sensor[0][0], g_sensor[1][0], g_sensor[2][0], g_sensor[3][0],
	       g_sensor_diff[0], g_sensor_diff[1], g_sensor_diff[2], g_sensor_diff[3] // 最後のカンマを削除
	);
}
void Wall_ResetReference( void )
{
	int CENTER_L =850;
	int CENTER_R = 470;
//	int DIAGOAL_CENTER_L = 440;
//	int DIAGOAL_CENTER_R = 400;
	int16_t i;
	//int16_t	accel_x_reference_box = accel_x_reference;
	//int16_t	gyro_z_reference_box = gyro_z_reference;

	for(i = 0; i < 100; i++) {
		CENTER_R+=g_sensor[1][0];
		CENTER_L+=g_sensor[2][0];
	}
	CENTER_R /= 100;
	CENTER_L /= 100;

	Wall_Threshold[1] =CENTER_R/2;
	Wall_Threshold[2] =CENTER_L/2;
	PID_CENTER_L = CENTER_L;
	PID_CENTER_R = CENTER_R;
	// --- 取得した値の出力 ---
		printf("--- Wall_ResetReference Values ---\n\r");
		printf("CENTER_R (Avg.): %.3dn\r", CENTER_R);
		printf("CENTER_L (Avg.): %.3d\n\r", CENTER_L);
		printf("Wall_Threshold[1]: %.3d\n\r", Wall_Threshold[1]);
		printf("Wall_Threshold[2]: %.3d\n\r", Wall_Threshold[2]);
		printf("PID_CENTER_R: %.3d\n\r", PID_CENTER_R);
		printf("PID_CENTER_L: %.3d\n\r", PID_CENTER_L);
		printf("------------------------------------\n\r");
	    // ----------------------------
}

void wall_kabegire_status_R(){//When the section to proceed at the start is 1, the first turn occurs.
	wall_kabegire_status[1] = 1;
}

void get_sensor_smoothed_diff() {
	if(	wall_kabegire_R_flag ==1){
			g_sensors_smoothed_diff[1] = ( 2.0f * g_sensor[1][0]
			              + 1.0f * g_sensor[1][1]
			              // 0.0f * g_sensor[1][2] は省略
			              - 1.0f * g_sensor[1][3]
			              - 2.0f * g_sensor[1][4]) ;
	}

	if(	wall_kabegire_L_flag ==1){
			g_sensors_smoothed_diff[2] = ( 2.0f * g_sensor[2][0]
			              + 1.0f * g_sensor[2][1]
			              // 0.0f * g_sensor[2][2] は省略
			              - 1.0f * g_sensor[2][3]
			              - 2.0f * g_sensor[2][4]);
	}


}
void wall_sensor_smoothed_diff_RESET(){
	for(int i=0;i<4;i++){
		g_sensors_smoothed_diff[i]=0;
	}

}
