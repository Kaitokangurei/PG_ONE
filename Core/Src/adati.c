/*
 * adati.c
 *
 *  Created on: Nov 27, 2024
 *      Author: lingm
 */
#include "stdio.h"
#include <stdlib.h>

#include "usart.h"
#include "adati.h"
#include "move.h"
#include "MOTOR.h"
#include "wall.h"
#include "PL_sensor.h"
#include "PL_timer.h"
#include "PID.h"
#include "log.h"
#include "SPEAKER.h"
#include "define.h"

#include "saitan.h"


#define adati_cost 1

// 壁情報
uint32_t wall_X[Map_Size + 2] = {0}; // 水平方向の壁データ
uint32_t wall_Y[Map_Size + 2] = {0}; // 垂直方向の壁データ
uint32_t wall_log_X[Map_Size +2] = {0}; // 水平方向の壁データ
uint32_t wall_log_Y[Map_Size + 2] = {0}; // 垂直方向の壁データ

// 畳み込みコスト
uint16_t adati_num[Map_Size+2][Map_Size+2] = {0};
//int math_X, math_Y;

// キュー関連
uint16_t QUEUE_X[2000];
uint16_t QUEUE_Y[2000];
uint16_t QUEUE_head = 0;
uint16_t QUEUE_tail = 0;

uint16_t Player_X = 0;
uint16_t Player_Y = 0;
uint16_t Player_D = 90;

uint16_t GOAL_X = 0;
uint16_t GOAL_Y = 3;
uint16_t START_X=0;
uint16_t START_Y=0;

#define HISTORY_MAX 5

typedef struct {
    uint8_t x;      // 座標X
    uint8_t y;      // 座標Y
    uint8_t mask;   // 操作したビット位置 (1 << n)
    uint8_t is_X;   // 1ならwall_X, 0ならwall_Yへの操作
} WallUpdate;

typedef struct {
    WallUpdate updates[3]; // 1回のwall_getで最大3枚の壁を記録
    uint8_t count;         // 実際に記録した壁の数
} WallStep;

WallStep undo_buffer[HISTORY_MAX];
int undo_ptr = 0;



uint16_t math_GOAL_X = 0;
uint16_t math_GOAL_Y = 3;

int adati_pass[258];
uint16_t adati_pass_count;

int slalom_flag=0;

uint8_t mode_adati = 1;
uint8_t count =0;

int zenmen_math_GOAL_X[256 * 4]; // 256*4以上
int zenmen_math_GOAL_Y[256 * 4];
int zenmen_math_GOAL_count;
int pass[256];
int pass_count = 0;

float gain=0.7;

void change_goal(){
	GOAL_X =7;
	GOAL_Y = 8;
	pritn_GOAL();
}

float tansaku_Gv =600;
int adati_maekabe_Wall_Threshold[4]  ={120,15,15,120};
// = {500,300, 80, 120,0}; // 各センサの連続検知回数を記録[5] = {300,300, 80, 100,0}; // 各センサの連続検知回数を記録
//int adati_Wall_Threshold[5] = {100,400, 200, 100,0}; // 各センサの連続検知回数を記録
int adati_Wall_Threshold[5] = {100, 80, 80,100,00}; // 各センサの連続検知回数を記
void pritn_GOAL(){
	printf("GOAL_X:%d,Y:%d",GOAL_X,GOAL_Y);
}


void wall_get(void) {
    // 各壁の有無（0:なし, 1:あり）
    uint8_t front_wall = 0;
    uint8_t left_wall  = 0;
    uint8_t right_wall = 0;

    // 1. センサー値から前・左・右の壁の有無を判定
    if ((g_sensor[0][0] >= adati_Wall_Threshold[0] )||( g_sensor[3][0] >= adati_Wall_Threshold[3])) {
        front_wall = 1;
        maekabe_flag = 1;
    } else {
        maekabe_flag = 0;
    }

    if (g_sensor[1][0] >= adati_Wall_Threshold[1]) {
        left_wall = 1;
    }

    if (g_sensor[2][0] >= adati_Wall_Threshold[2]) {
        right_wall = 1;
    }

    // 2. 方向に応じた迷路マップ（wall_X, wall_Y）の更新
    if (Player_D == 90) {
        if (front_wall) wall_X[Player_X] |= (1 << (Player_Y + 1));
        if (left_wall)  wall_Y[Player_Y] |= (1 << (Player_X + 1));
        if (right_wall) wall_Y[Player_Y] |= (1 << Player_X);

        wall_log_X[Player_X] |= (1 << (Player_Y + 1));
        wall_log_Y[Player_Y] |= (1 << (Player_X + 1));
        wall_log_Y[Player_Y] |= (1 << Player_X);

    } else if (Player_D == 270) {
        if (front_wall) wall_X[Player_X] |= (1 << Player_Y);
        if (left_wall)  wall_Y[Player_Y] |= (1 << Player_X);
        if (right_wall) wall_Y[Player_Y] |= (1 << (Player_X + 1));

        wall_log_X[Player_X] |= (1 << Player_Y);
        wall_log_Y[Player_Y] |= (1 << Player_X);
        wall_log_Y[Player_Y] |= (1 << (Player_X + 1));

    } else if (Player_D == 0) {
        if (front_wall) wall_Y[Player_Y] |= (1 << (Player_X + 1));
        if (left_wall)  wall_X[Player_X] |= (1 << Player_Y);
        if (right_wall) wall_X[Player_X] |= (1 << (Player_Y + 1));

        wall_log_Y[Player_Y] |= (1 << (Player_X + 1));
        wall_log_X[Player_X] |= (1 << Player_Y);
        wall_log_X[Player_X] |= (1 << (Player_Y + 1));

    } else if (Player_D == 180) {
        if (front_wall) wall_Y[Player_Y] |= (1 << Player_X);
        if (left_wall)  wall_X[Player_X] |= (1 << (Player_Y + 1));
        if (right_wall) wall_X[Player_X] |= (1 << Player_Y);

        wall_log_Y[Player_Y] |= (1 << Player_X);
        wall_log_X[Player_X] |= (1 << (Player_Y + 1));
        wall_log_X[Player_X] |= (1 << Player_Y);
    }

    // 3. 壁の組み合わせパターンに応じて音を再生
    if (front_wall && left_wall && right_wall) {
        SPEAKER_PlaySE(SE_WALL_DEADEND);  // 袋小路（3面壁）
    } else if (front_wall) {
        SPEAKER_PlaySE(SE_WALL_FRONT);    // 前壁あり（左右の有無問わず）
    } else if (left_wall || right_wall) {
        SPEAKER_PlaySE(SE_WALL_SIDE);     // 横壁のみ
    } else {
        SPEAKER_PlaySE(SE_WALL_NONE);     // 壁なし（全方位オープン）
    }
}

void add_to_history(int x, int y, uint8_t mask, uint8_t is_X) {
    WallStep *step = &undo_buffer[undo_ptr];
    if (step->count < 3) {
        step->updates[step->count].x = (uint8_t)x;
        step->updates[step->count].y = (uint8_t)y;
        step->updates[step->count].mask = mask;
        step->updates[step->count].is_X = is_X;
        step->count++;
    }
}

void delete_last_5_logs() {
    for (int i = 0; i < HISTORY_MAX; i++) {
        WallStep *step = &undo_buffer[i];
        for (int j = 0; j < step->count; j++) {
            WallUpdate *u = &step->updates[j];
            if (u->is_X) {
                wall_X[u->x] &= ~(u->mask);     // 壁情報を消去
                wall_log_X[u->x] &= ~(u->mask); // 既知フラグも消去
            } else {
                wall_Y[u->y] &= ~(u->mask);
                wall_log_Y[u->y] &= ~(u->mask);
            }
        }
        step->count = 0; // 削除済みとしてリセット
    }
}

void adati_create_pass(){

	for(int i=0;i<=16;i++){
		for(int j=0;j<=16;j++){
			if(!(wall_log_Y[j] & (1 << i))){
				wall_Y[j] |=(1<<i);
			}
			if(!(wall_log_X[j] & (1 << i))){
				wall_X[j] |=(1<<i);
			}

		}
	}

	for(int i=0;i<=Map_Size;i++){
		wall_X[i] |=(1<<0);
		wall_X[i] |=(1<<16);
		wall_Y[i] |=(1<<0);
		wall_Y[i] |=(1<<16);
	}

	wall_Y[0] |=(1<<1);
	 math_GOAL_X = GOAL_X;
	 math_GOAL_Y = GOAL_Y;
	Calculate_adati_math();
	print_adatimap();

	int pass_X=0,pass_Y=1,pass_D=90;
	adati_pass[0]=2;
	adati_pass_count=1;
	scanf_adati_pass(2, 0);
	int MAX = 0;
	int pass_num ;
	for(int i =0;i<256;i++){
		MAX = 0;
		pass_num =0;

		MAX = adati_num[pass_X][pass_Y];
		if(pass_D==90){
			if(adati_num[pass_X][pass_Y+1]<MAX
					&&(!(wall_X[pass_X] & (1 << (pass_Y+1))))){
				MAX = adati_num[pass_X][pass_Y+1];
				pass_num = 2;
			}
			if(adati_num[pass_X+1][pass_Y]<MAX
					&&(!(wall_Y[pass_Y] & (1 << (pass_X+1))))){
				MAX =adati_num[pass_X+1][pass_Y];
				pass_num = 45;
			}
			if(adati_num[pass_X-1][pass_Y]<MAX
					&&(!(wall_Y[pass_Y] & (1 << (pass_X))))){
				MAX = adati_num[pass_X-1][pass_Y];
				pass_num = -45;
			}
			if(adati_num[pass_X][pass_Y-1]<MAX
					&&(!(wall_X[pass_X] & (1 << pass_Y)))){
				MAX = adati_num[pass_X][pass_Y-1];
				pass_num = -2;
			}
		}
		else if(pass_D==270){
			if(adati_num[pass_X][pass_Y-1]<MAX
					&&(!(wall_X[pass_X] & (1 << pass_Y)))){
				MAX = adati_num[pass_X][pass_Y-1];
				pass_num  = 2;
			}
			if(adati_num[pass_X-1][pass_Y]<MAX
					&&(!(wall_Y[pass_Y] & (1 << pass_X)))){
				MAX =adati_num[pass_X-1][pass_Y];
				pass_num  = 45;
			}
			if(adati_num[pass_X+1][pass_Y]<MAX
					&&(!(wall_Y[pass_Y] & (1 << (pass_X+1))))){
				MAX = adati_num[pass_X+1][pass_Y];
				pass_num = -45;
			}
			if(adati_num[pass_X][pass_Y+1]<MAX
					&&(!(wall_X[pass_X] & (1 << (pass_Y+1))))){
				MAX = adati_num[pass_X][pass_Y+1];
				pass_num = -2;
			}
		}

		else if(pass_D==0){
			if(adati_num[pass_X+1][pass_Y]<MAX
					&&(!(wall_Y[pass_Y] & (1 << (pass_X+1))))){
				MAX = adati_num[pass_X+1][pass_Y];
				pass_num  = 2;
			}
			if(adati_num[pass_X][pass_Y-1]<MAX
					&&(!(wall_X[pass_X] & (1 << pass_Y)))){
				MAX = adati_num[pass_X][pass_Y-1];
				pass_num  = 45;
			}
			if(adati_num[pass_X][pass_Y+1]<MAX
					&&(!(wall_X[pass_X] & (1 << (pass_Y+1))))){
				MAX = adati_num[pass_X][pass_Y+1];
				pass_num  = -45;
			}
			if(adati_num[pass_X-1][pass_Y]<MAX
					&&(!(wall_Y[pass_Y] & (1 << pass_X)))){
				MAX = adati_num[pass_X-1][pass_Y];
				pass_num  = -2;
			}
		}
		else if(pass_D==180){
			if(adati_num[pass_X-1][pass_Y]<MAX
					&&
					(!(wall_Y[pass_Y] & (1 << pass_X)))){
				MAX = adati_num[pass_X-1][pass_Y];
				pass_num = 2;
			}
			if(adati_num[pass_X][pass_Y+1]<MAX
					&&(!(wall_X[pass_X] & (1 << (pass_Y+1))))){
				MAX = adati_num[pass_X][pass_Y+1];
				pass_num  = 45;
			}
			if(adati_num[pass_X][pass_Y-1]<MAX
					&&(!(wall_X[pass_X] & (1 << pass_Y)))){
				MAX = adati_num[pass_X][pass_Y-1];
				pass_num  = -45;
			}
			if(adati_num[pass_X+1][pass_Y]<MAX
					&&(!(wall_Y[pass_Y] & (1 << (pass_X+1))))){
				MAX = adati_num[pass_X+1][pass_Y];
				pass_num = -2;
			}
		}

		adati_pass[adati_pass_count]= pass_num;
		scanf_adati_pass(pass_num, adati_pass_count);
		adati_pass_count++;


		if(	pass_num ==2){
				if(pass_D==90){
					pass_Y++;
				}
				else if(pass_D==270){
					pass_Y--;
				}
				else if(pass_D==0){
					pass_X++;
				}
				else if(pass_D==180){
					pass_X--;
				}

		}
		else if(pass_num ==45){
				if(pass_D==90){
					pass_X++;
					pass_D=0;
				}
				else if(pass_D==270){
					pass_X--;
					pass_D=180;
				}
				else if(pass_D==0){
					//Player_X++;
					pass_Y--;
					pass_D=270;
				}
				else if(pass_D==180){
					//Player_X--;
					pass_Y++;
					pass_D=90;
				}
		}
		else if(pass_num==-45){
				if(pass_D==90){
					pass_X--;
					pass_D=180;
				}
				else if(pass_D==270){
					pass_X++;
					pass_D=0;
				}
				else if(pass_D==0){
					//Player_X--;
					pass_Y++;
					pass_D=90;
				}
				else if(pass_D==180){
					//Player_X++;
					pass_Y--;
					pass_D=270;
				}
		}
		else if(pass_num ==-2){
				if(pass_D==90){
					pass_Y--;
					pass_D=270;
				}
				else if(pass_D==270){
					pass_Y++;
					pass_D=90;
				}
				else if(pass_D==0){
					pass_X--;
					pass_D=180;
				}
				else if(pass_D==180){
					pass_X++;
					pass_D=0;
				}
		}

		if(pass_X== GOAL_X &&pass_Y==GOAL_Y)
		{
			//move_stop_300(180);
			printf("GOAL");
			break;
		}
	}
	adati_pass[adati_pass_count]= 1;
	scanf_adati_pass(1, adati_pass_count+1);
}


void adati_move( uint8_t mode ){
	for(int i=0;i<=Map_Size;i++){
		wall_X[i] |=(1<<0);
		wall_X[i] |=(1<<16);
		wall_Y[i] |=(1<<0);
		//wall_Y[i] |=(1<<2);
		wall_Y[i] |=(1<<16);
	}
	wall_Y[0] |=(1<<1);
	 math_GOAL_X = GOAL_X;
	 math_GOAL_Y = GOAL_Y;
	 Player_X = START_X;
	 Player_Y = START_Y;
	 Player_D = 90;

	 if(mode ==1 ){
		 tansaku_Gv =tansaku_Gv_600;
		 motor_daikei(Gv,tansaku_Gv,tansaku_Gv,4000,110);
	 }else if(mode ==2){
		 tansaku_Gv =tansaku_Gv_600;
		 motor_daikei(Gv,tansaku_Gv,tansaku_Gv,4000,110);
	 }else if(mode ==3){
		 tansaku_Gv =tansaku_Gv_1000;
		 motor_daikei(Gv,tansaku_Gv,tansaku_Gv,10000,100-10);
	 }else{
		 tansaku_Gv =tansaku_Gv_600;
		 motor_daikei(Gv,tansaku_Gv,tansaku_Gv,4000,120);
	 }


	 slalom_flag=0;
	 Player_Y++;

	 wall_get_flag=0;
	 Gx=0;
	 for(int move_count= 0;move_count<=1000;move_count++){
		    PID_turn_flag = 1;
		    PID_flag = 1;

		 Gx=0;
		 wall_conf_flag=1;


		if(Player_X==math_GOAL_X&&Player_Y==math_GOAL_Y){
			 if(mode ==2 )break;
				 //break;
			 math_GOAL_X = START_X;
			 math_GOAL_Y = START_Y;
		}
		if(Player_X==START_X&&Player_Y==START_Y){
			 math_GOAL_X = START_X;
			 math_GOAL_Y = START_X;
			// kabeate_GOAL_180(180);
			 break;
		}
		wall_get();

		Calculate_adati_math();

		 if(adati_num[Player_X][Player_Y]>=300){
			 failsafe_flag=1;
			 break;
		}
		if(failsafe_flag == 1)break;
		if(motor_stop_flag == 1)break;

		Decide_move();
		if(Gx>=20)break;
		while(Gx<=20);
		move_one();
	}

	 if(mode ==1||mode==3 ){
         FUN_PID_flag   = 0;
          FUN_MOTOR_flag = 0;
		 kabeate_GOAL_180(tansaku_Gv);
			for(int i=0;i<=16;i++){
				for(int j=0;j<=16;j++){
					if(!(wall_log_Y[j] & (1 << i))){
						wall_Y[j] |=(1<<i);
					}
					if(!(wall_log_X[j] & (1 << i))){
						wall_X[j] |=(1<<i);
					}

				}
			}}
	 else if(mode ==2 )motor_daikei(Gv,Gv,0,6000,90);
}


void adati_move_GOTOSTART( uint8_t mode ){
	 //senkai_omega_daikei_R(0,400,0,2000,360);
	 motor_daikei_back(0,400,0,1000,180);
	 wall_conf_flag=1;
	 if(mode ==1 ){
		 tansaku_Gv =tansaku_Gv_600;
		 motor_daikei(Gv,tansaku_Gv,tansaku_Gv,4000,100);
	 }else if(mode ==2){
		 tansaku_Gv =tansaku_Gv_600;
		 motor_daikei(Gv,tansaku_Gv,tansaku_Gv,4000,100);
	 }else if(mode ==3){
		 tansaku_Gv =tansaku_Gv_1000;
		 motor_daikei(Gv,tansaku_Gv,tansaku_Gv,10000,100-10);
	 }else{
		 tansaku_Gv =tansaku_Gv_600;
		 motor_daikei(Gv,tansaku_Gv,tansaku_Gv,4000,120);
	 }
		if(g_sensor[1][0]<=adati_Wall_Threshold[1]){
			kabeate_L(tansaku_Gv);
		}else if(g_sensor[2][0]<=adati_Wall_Threshold[2]){
			kabeate_R(tansaku_Gv);
		}else{
			kabeate_R(tansaku_Gv);
		}

	for(int i=0;i<=Map_Size;i++){
		wall_X[i] |=(1<<0);
		wall_X[i] |=(1<<16);
		wall_Y[i] |=(1<<0);
		//wall_Y[i] |=(1<<2);
		wall_Y[i] |=(1<<16);
	}
	wall_Y[0] |=(1<<1);
	 math_GOAL_X = 0;
	 math_GOAL_Y = 0;

/*
	 if(mode ==1 ){
		 tansaku_Gv =tansaku_Gv_600;
		 motor_daikei(Gv,tansaku_Gv,tansaku_Gv,4000,100);
	 }else if(mode ==2){
		 tansaku_Gv =tansaku_Gv_600;
		 motor_daikei(Gv,tansaku_Gv,tansaku_Gv,4000,100);
	 }else if(mode ==3){
		 tansaku_Gv =tansaku_Gv_1000;
		 motor_daikei(Gv,tansaku_Gv,tansaku_Gv,10000,100-10);
	 }else{
		 tansaku_Gv =tansaku_Gv_600;
		 motor_daikei(Gv,tansaku_Gv,tansaku_Gv,4000,120);
	 }
*/



	 //goal後の動きを作る。

	 slalom_flag=0;
	//motor_daikei(600,600,600,2000,180-Gx);
		if(Player_D==90){
			Player_Y--;
			Player_D =270;
		}
		else if(Player_D==270){
			//Player_Y++;
			Player_D =90;
		}
		else if(Player_D==0){
			Player_X--;
			Player_D =180;
		}
		else if(Player_D==180){
			//Player_X++;
			Player_D =0;
		}


	 wall_get_flag=0;
	 Gx=0;
	 for(int move_count= 0;move_count<=1000;move_count++){
		    PID_turn_flag = 1;
		    PID_flag = 1;

		 Gx=0;
		 wall_conf_flag=1;


		if(Player_X==math_GOAL_X&&Player_Y==math_GOAL_Y){
			 if(mode ==2 )break;
				 //break;
			 math_GOAL_X = START_X;
			 math_GOAL_Y = START_Y;
		}
		if(Player_X==START_X&&Player_Y==START_Y){
			 math_GOAL_X = START_X;
			 math_GOAL_Y = START_X;
			// kabeate_GOAL_180(180);
			 break;
		}
		wall_get();

		Calculate_adati_math();

		 if(adati_num[Player_X][Player_Y]>=300){
			 failsafe_flag=1;
			 break;
		}
		if(failsafe_flag == 1)break;
		if(motor_stop_flag == 1)break;

		Decide_move();
		if(Gx>=20)break;
		while(Gx<=20);
		move_one();
	}
     FUN_PID_flag   = 0;
      FUN_MOTOR_flag = 0;
	 if(mode ==1||mode==3 ){
		 kabeate_GOAL_180(tansaku_Gv);
			for(int i=0;i<=16;i++){
				for(int j=0;j<=16;j++){
					if(!(wall_log_Y[j] & (1 << i))){
						wall_Y[j] |=(1<<i);
					}
					if(!(wall_log_X[j] & (1 << i))){
						wall_X[j] |=(1<<i);
					}

				}
			}}
	 else if(mode ==2 )motor_daikei(Gv,Gv,0,6000,90);
}




int MAX = 0;
int move_num =2;
void Decide_move(){
	move_num = 2;
	MAX = adati_num[Player_X][Player_Y];
	if(Player_D==90){
		if(adati_num[Player_X][Player_Y+1]<MAX
				&&(!(wall_X[Player_X] & (1 << (Player_Y+1))))){
			MAX = adati_num[Player_X][Player_Y+1];

			move_num = 2;
		}
		if(adati_num[Player_X+1][Player_Y]<MAX
				&&(!(wall_Y[Player_Y] & (1 << (Player_X+1))))){
			MAX =adati_num[Player_X+1][Player_Y];
			move_num = 45;
		}
		if(adati_num[Player_X-1][Player_Y]<MAX
				&&(!(wall_Y[Player_Y] & (1 << (Player_X))))){
			MAX = adati_num[Player_X-1][Player_Y];
			move_num = -45;
		}
		if(adati_num[Player_X][Player_Y-1]<MAX
				&&(wall_X[Player_X] & (1 << (Player_Y+1)))){
			MAX = adati_num[Player_X][Player_Y-1];
			move_num = -2;
		}
	}
	else if(Player_D==270){
		if(adati_num[Player_X][Player_Y-1]<MAX
				&&(!(wall_X[Player_X] & (1 << Player_Y)))){
			MAX = adati_num[Player_X][Player_Y-1];
			move_num = 2;

		}

		if(adati_num[Player_X-1][Player_Y]<MAX
				&&(!(wall_Y[Player_Y] & (1 << Player_X)))){
			MAX =adati_num[Player_X-1][Player_Y];
			move_num = 45;
		}
		if(adati_num[Player_X+1][Player_Y]<MAX
				&&(!(wall_Y[Player_Y] & (1 << (Player_X+1))))){
			MAX = adati_num[Player_X+1][Player_Y];
			move_num = -45;
		}
		if(adati_num[Player_X][Player_Y+1]<MAX
				&&(wall_X[Player_X] & (1 << (Player_Y)))){
			MAX= adati_num[Player_X][Player_Y+1];
			move_num = -2;
		}
	}

	else if(Player_D==0){
		if(adati_num[Player_X+1][Player_Y]<MAX
				&&(!(wall_Y[Player_Y] & (1 << (Player_X+1))))){
			MAX = adati_num[Player_X+1][Player_Y];
			move_num = 2;
		}
		if(adati_num[Player_X][Player_Y-1]<MAX
				&&(!(wall_X[Player_X] & (1 << Player_Y)))){
			MAX = adati_num[Player_X][Player_Y-1];
			move_num = 45;
		}
		if(adati_num[Player_X][Player_Y+1]<MAX
				&&(!(wall_X[Player_X] & (1 << (Player_Y+1))))){
			MAX = adati_num[Player_X][Player_Y+1];
			move_num = -45;
		}
		if(adati_num[Player_X-1][Player_Y]<MAX
				&&(wall_Y[Player_Y] & (1 << (Player_X+1)))){
			MAX = adati_num[Player_X-1][Player_Y];
			move_num = -2;
		}
	}
	else if(Player_D==180){
		if(adati_num[Player_X-1][Player_Y]<MAX
				&&
				(!(wall_Y[Player_Y] & (1 << Player_X)))){
			MAX = adati_num[Player_X-1][Player_Y];
			move_num = 2;
		}
		if(adati_num[Player_X][Player_Y+1]<MAX
				&&(!(wall_X[Player_X] & (1 << (Player_Y+1))))){
			MAX = adati_num[Player_X][Player_Y+1];
			move_num = 45;
		}
		if(adati_num[Player_X][Player_Y-1]<MAX
				&&(!(wall_X[Player_X] & (1 << Player_Y)))){
			MAX = adati_num[Player_X][Player_Y-1];
			move_num = -45;
		}
		if(adati_num[Player_X+1][Player_Y]<MAX
				&&(wall_Y[Player_Y] & (1 << (Player_X)))){
			MAX = adati_num[Player_X+1][Player_Y];
			move_num = -2;
		}
	}
}

void move_one(){
	if(move_num ==2){//straight
		 wall_conf_flag=1;
		slalom_flag=0;
		//wall_kabegire_ALL_START();
		while(1){
			if(Gx>=180){
				//wall_kabegire_ALL_RESET();
				break;
			}
			if((g_sensor[0][0]+g_sensor[3][0])>= (adati_maekabe_Wall_Threshold[0]+adati_maekabe_Wall_Threshold[3])&&Gx<=10){
				move_num = 2;

				//wall_get();
				maekabe_flag=1;
				if(g_sensor[1][0]<=adati_Wall_Threshold[1]){
					move_num = 45;
				}else if(g_sensor[2][0]<=adati_Wall_Threshold[2]){
					move_num = -45;
				}else{
					move_num=-2;
				}
				break;
			}
		}

		if(move_num == 2){
			if(Player_D==90){
				Player_Y++;
			}
			else if(Player_D==270){
				Player_Y--;
			}
			else if(Player_D==0){
				Player_X++;
			}
			else if(Player_D==180){
				Player_X--;
			}
		}
	}
	if(move_num ==45){
		slalom_R(tansaku_Gv);

		if(Player_D==90){
			Player_X++;
			Player_D=0;
		}
		else if(Player_D==270){
			Player_X--;
			Player_D=180;
		}
		else if(Player_D==0){
			Player_Y--;
			Player_D=270;
		}
		else if(Player_D==180){
			Player_Y++;
			Player_D=90;
		}
		slalom_flag=1;
	}
	if(move_num ==-45){
		slalom_L(tansaku_Gv);
		if(Player_D==90){
			Player_X--;
			Player_D=180;
		}
		else if(Player_D==270){
			Player_X++;
			Player_D=0;
		}
		else if(Player_D==0){
			Player_Y++;
			Player_D=90;
		}
		else if(Player_D==180){
			Player_Y--;
			Player_D=270;
		}
	}
	if(move_num ==-2){
		if(g_sensor[1][0]>= adati_Wall_Threshold[1]){
			//sensor_kabeate_R(tansaku_Gv);
			kabeate_R(tansaku_Gv);
			if(Player_D==90){
				Player_Y--;
				Player_D=270;
			}
			else if(Player_D==270){
				Player_Y++;
				Player_D=90;
			}
			else if(Player_D==0){
				Player_X--;
				Player_D=180;
			}
			else if(Player_D==180){
				Player_X++;
				Player_D=0;
			}
		}else if(g_sensor[2][0]>= adati_Wall_Threshold[2]){
			//sensor_kabeate_L(tansaku_Gv);
			kabeate_L(tansaku_Gv);
			if(Player_D==90){
				Player_Y--;
				Player_D=270;
			}
			else if(Player_D==270){
				Player_Y++;
				Player_D=90;
			}
			else if(Player_D==0){
				Player_X--;
				Player_D=180;
			}
			else if(Player_D==180){
				Player_X++;
				Player_D=0;
			}
		}else{
			kabeate_180(tansaku_Gv);
			if(Player_D==90){
				Player_Y--;
				Player_D=270;
			}
			else if(Player_D==270){
				Player_Y++;
				Player_D=90;
			}
			else if(Player_D==0){
				Player_X--;
				Player_D=180;
			}
			else if(Player_D==180){
				Player_X++;
				Player_D=0;
			}
		}
	}
}


void INPUT_ADATI_QUEUE(uint16_t x, uint16_t y) {
    QUEUE_X[QUEUE_tail] = x;
    QUEUE_Y[QUEUE_tail] = y;
    QUEUE_tail++;
    if(QUEUE_tail>2000)QUEUE_tail=0;
}

void OUTPUT_ADATI_QUEUE(uint16_t x, uint16_t y) {
    //math_X = QUEUE_X[QUEUE_head];
    //math_Y = QUEUE_Y[QUEUE_head];
    //QUEUE_head = (QUEUE_head + 1) % 2000; // リングバッファ
    //printf("o%d,%d,%d\n\r",QUEUE_X[QUEUE_head],QUEUE_Y[QUEUE_head],QUEUE_tail);
}

void Calculate_adati_math() {
    for (int i = 0; i < Map_Size; i++) {
        for (int j = 0; j < Map_Size; j++) {
            adati_num[i][j] = 999;
        }
    }

    QUEUE_head = 0;
    QUEUE_tail = 0;
    int math_X = math_GOAL_X, math_Y = math_GOAL_Y;
    adati_num[math_X][math_Y] = 0;
    INPUT_ADATI_QUEUE(math_X, math_Y);

    for(int i=0;i<=512;i++){
        OUTPUT_ADATI_QUEUE(math_X, math_Y);
        math_X=QUEUE_X[QUEUE_head];
        math_Y=QUEUE_Y[QUEUE_head];
        QUEUE_head++;
        if(QUEUE_head>2000)QUEUE_head=0;

        if (math_Y < Map_Size - 1 && !(wall_X[math_X] & (1 << (math_Y + 1))) &&
            adati_num[math_X][math_Y + 1] > adati_num[math_X][math_Y] + adati_cost) {
            adati_num[math_X][math_Y + 1] = adati_num[math_X][math_Y] + adati_cost;
            INPUT_ADATI_QUEUE(math_X, math_Y+1);
        }
        if (math_Y > 0 && !(wall_X[math_X] & (1 << math_Y)) &&
            adati_num[math_X][math_Y - 1] > adati_num[math_X][math_Y] + adati_cost) {
            adati_num[math_X][math_Y - 1] = adati_num[math_X][math_Y] + adati_cost;
            INPUT_ADATI_QUEUE(math_X, math_Y - 1);
        }
        if (math_X < Map_Size - 1 && !(wall_Y[math_Y] & (1 << (math_X + 1))) &&
            adati_num[math_X + 1][math_Y] > adati_num[math_X][math_Y] + adati_cost) {
            adati_num[math_X + 1][math_Y] = adati_num[math_X][math_Y] + adati_cost;
            INPUT_ADATI_QUEUE(math_X + 1 ,math_Y);
        }
        if (math_X > 0 && !(wall_Y[math_Y] & (1 << math_X)) &&
            adati_num[math_X - 1][math_Y] > adati_num[math_X][math_Y] + adati_cost) {
            adati_num[math_X - 1][math_Y] = adati_num[math_X][math_Y] + adati_cost;
            INPUT_ADATI_QUEUE(math_X - 1,math_Y);
        }

        if(QUEUE_tail==QUEUE_head){
        	//Gv=0;
        	//for(int i=0;i<=5;i++)LED_Chase_OneByOne();
        }
    }
}


void print_adatimap() {

    for (int i = 0; i < Map_Size; i++) {
        if (i >= 10) {
            printf(" %d ", i);
        } else {
            printf("  %d ", i);
        }
    }
    printf("\n\r");

    for (int j = 0; j < Map_Size; j++) {
        printf("+");
        if (wall_X[j] & (1 << (Map_Size ))) {
            printf("---");
        } else {
            printf("   ");
        }
    }
    printf("+\n\r");

    for (int i = Map_Size - 1; i >= 0; i--) {
        for (int j = 0; j < Map_Size; j++) {
            if (wall_Y[i] & (1 << j)) {
                printf("|");
            } else {
                printf(" ");
            }

            if(Player_X==j&&Player_Y==i){
            	printf(" @ ");
            }else if(adati_num[j][i]>=100){
            	printf("%d",adati_num[j][i]);
            }else if(adati_num[j][i]>=10){
            	printf("%d ",adati_num[j][i]);
            }
            else{
                printf(" %d ",adati_num[j][i]);
            }
        }

        if (wall_Y[i] & (1 << Map_Size)) {
            printf("|");
        } else {
            printf(" ");
        }

        printf("%2d\n\r", i);

        for (int j = 0; j < Map_Size; j++) {
            printf("+");
            if (wall_X[j] & (1 << i)) {
                printf("---");
            } else {
                printf("   ");
            }
        }
        printf("+\n\r");
    }
}


void create_pass()
{
	pass_count = 0;
	for (int i = 0; i <= 256; i++)
	{
		int MAX = 0;
		int move_num = 2;
		MAX = adati_num[Player_X][Player_Y];
		if (Player_D == 90)
		{
			if (adati_num[Player_X][Player_Y + 1] < MAX && (!(wall_X[Player_X] & (1 << (Player_Y + 1)))))
			{
				MAX = adati_num[Player_X][Player_Y + 1];
				move_num = 2;
			}
			if (adati_num[Player_X + 1][Player_Y] < MAX && (!(wall_Y[Player_Y] & (1 << (Player_X + 1)))))
			{
				MAX = adati_num[Player_X + 1][Player_Y];
				move_num = 45;
			}
			if (adati_num[Player_X - 1][Player_Y] < MAX && (!(wall_Y[Player_Y] & (1 << (Player_X)))))
			{
				MAX = adati_num[Player_X - 1][Player_Y];
				move_num = -45;
			}
			if (adati_num[Player_X][Player_Y - 1] < MAX && (wall_X[Player_X] & (1 << (Player_Y + 1))))
			{
				MAX = adati_num[Player_X][Player_Y - 1];
				move_num = -200;
			}
		}
		else if (Player_D == 270)
		{
			if (adati_num[Player_X][Player_Y - 1] < MAX && (!(wall_X[Player_X] & (1 << Player_Y))))
			{
				MAX = adati_num[Player_X][Player_Y - 1];
				move_num = 2;
			}
			if (adati_num[Player_X - 1][Player_Y] < MAX && (!(wall_Y[Player_Y] & (1 << Player_X))))
			{
				MAX = adati_num[Player_X - 1][Player_Y];
				move_num = 45;
			}
			if (adati_num[Player_X + 1][Player_Y] < MAX && (!(wall_Y[Player_Y] & (1 << (Player_X + 1)))))
			{
				MAX = adati_num[Player_X + 1][Player_Y];
				move_num = -45;
			}
			if (adati_num[Player_X][Player_Y + 1] < MAX && (wall_X[Player_X] & (1 << (Player_Y))))
			{
				MAX = adati_num[Player_X][Player_Y + 1];
				move_num = -200;
			}
		}

		else if (Player_D == 0)
		{
			if (adati_num[Player_X + 1][Player_Y] < MAX && (!(wall_Y[Player_Y] & (1 << (Player_X + 1)))))
			{
				MAX = adati_num[Player_X + 1][Player_Y];
				move_num = 2;
			}
			if (adati_num[Player_X][Player_Y - 1] < MAX && (!(wall_X[Player_X] & (1 << Player_Y))))
			{
				MAX = adati_num[Player_X][Player_Y - 1];
				move_num = 45;
			}
			if (adati_num[Player_X][Player_Y + 1] < MAX && (!(wall_X[Player_X] & (1 << (Player_Y + 1)))))
			{
				MAX = adati_num[Player_X][Player_Y + 1];
				move_num = -45;
			}
			if (adati_num[Player_X - 1][Player_Y] < MAX && (wall_Y[Player_Y] & (1 << (Player_X + 1))))
			{
				MAX = adati_num[Player_X - 1][Player_Y];
				move_num = -200;
			}
		}
		else if (Player_D == 180)
		{
			if (adati_num[Player_X - 1][Player_Y] < MAX &&
				(!(wall_Y[Player_Y] & (1 << Player_X))))
			{
				MAX = adati_num[Player_X - 1][Player_Y];
				move_num = 2;
			}
			if (adati_num[Player_X][Player_Y + 1] < MAX && (!(wall_X[Player_X] & (1 << (Player_Y + 1)))))
			{
				MAX = adati_num[Player_X][Player_Y + 1];
				move_num = 45;
			}
			if (adati_num[Player_X][Player_Y - 1] < MAX && (!(wall_X[Player_X] & (1 << Player_Y))))
			{
				MAX = adati_num[Player_X][Player_Y - 1];
				move_num = -45;
			}
			if (adati_num[Player_X + 1][Player_Y] < MAX && (wall_Y[Player_Y] & (1 << (Player_X))))
			{
				MAX = adati_num[Player_X + 1][Player_Y];
				move_num = -200;
			}
		}

		pass[pass_count] = move_num;

		if (move_num == 2)
		{
			if (Player_D == 90)
			{
				Player_Y++;
			}
			else if (Player_D == 270)
			{
				Player_Y--;
			}
			else if (Player_D == 0)
			{
				Player_X++;
			}
			else if (Player_D == 180)
			{
				Player_X--;
			}
		}
		else if (move_num == 45)
		{
			if (Player_D == 90)
			{
				Player_X++;
				Player_D = 0;
			}
			else if (Player_D == 270)
			{
				Player_X--;
				Player_D = 180;
			}
			else if (Player_D == 0)
			{
				Player_Y--;
				Player_D = 270;
			}
			else if (Player_D == 180)
			{
				Player_Y++;
				Player_D = 90;
			}
		}
		else if (move_num == -45)
		{
			if (Player_D == 90)
			{
				Player_X--;
				Player_D = 180;
			}
			else if (Player_D == 270)
			{
				Player_X++;
				Player_D = 0;
			}
			else if (Player_D == 0)
			{
				Player_Y++;
				Player_D = 90;
			}
			else if (Player_D == 180)
			{
				Player_Y--;
				Player_D = 270;
			}
		}
		else if (move_num == -200)
		{
			//pass_count++;
			if (g_sensor_avg[1] >= adati_Wall_Threshold[1])
			{
				// kabeate_R();
				if (Player_D == 90)
				{
					Player_Y--;
					Player_D = 270;
				}
				else if (Player_D == 270)
				{
					Player_Y++;
					Player_D = 90;
				}
				else if (Player_D == 0)
				{
					Player_X--;
					Player_D = 180;
				}
				else if (Player_D == 180)
				{
					Player_X++;
					Player_D = 0;
				}
			}
			else if (g_sensor_avg[2] >= adati_Wall_Threshold[2])
			{
				// kabeate_L();
				if (Player_D == 90)
				{
					Player_Y--;
					Player_D = 270;
				}
				else if (Player_D == 270)
				{
					Player_Y++;
					Player_D = 90;
				}
				else if (Player_D == 0)
				{
					Player_X--;
					Player_D = 180;
				}
				else if (Player_D == 180)
				{
					Player_X++;
					Player_D = 0;
				}
			}
			else
			{
				// kabeate_180();
				if (Player_D == 90)
				{
					Player_Y--;
					Player_D = 270;
				}
				else if (Player_D == 270)
				{
					Player_Y++;
					Player_D = 90;
				}
				else if (Player_D == 0)
				{
					Player_X--;
					Player_D = 180;
				}
				else if (Player_D == 180)
				{
					Player_X++;
					Player_D = 0;
				}
			}
		}

		if (adati_num[Player_X][Player_Y] == 0)
		{
			break;
		}

		pass_count++;
	}


	for (int i = 0; i <= pass_count; i++)
	{
		if (pass[i] == 2)
		{
			for (int j = i + 1; i <= pass_count; j++)
			{
				if (pass[j] == 2)
				{
					pass[i] = pass[i] + 2;
					pass[j] = 0;
				}
				else
				{
					break;
				}
			}
		}
	}
	for (int i = 0; i <= pass_count; i++)
	{
		if (pass[i] == 0)
		{
			for (int j = i; j < pass_count; j++)
			{
				pass[j] = pass[j + 1];
			}
			pass_count--;
			i--;
		}
		pass[pass_count + 1] = 0;
	}
	for (int i = 0; i <= pass_count; i++)
	{
		if (i >= 10 && i % 10 == 0)
		{
			printf("\n");
		}
		printf("%3d", pass[i]);
	}
	printf("\nS_pass_count:%d\n\r", pass_count);

	printf("\n\r");

}
void zenmen_log_map()
{
	zenmen_math_GOAL_count = 0;

	for (int i = 0; i < Map_Size; i++)
	{
		for (int j = 0; j < Map_Size; j++)
		{

			if (!(wall_log_X[i] & (1 << (j))))
			{ // x,y

				zenmen_math_GOAL_X[zenmen_math_GOAL_count] = i;
				zenmen_math_GOAL_Y[zenmen_math_GOAL_count] = j;
				zenmen_math_GOAL_count++;

				if (j != 0)
				{
					zenmen_math_GOAL_X[zenmen_math_GOAL_count] = i;
					zenmen_math_GOAL_Y[zenmen_math_GOAL_count] = j - 1;
					zenmen_math_GOAL_count++;
				}
			}
			if (!(wall_log_Y[j] & (1 << (i))))
			{ // y,x

				if (i != 0)
				{
					zenmen_math_GOAL_X[zenmen_math_GOAL_count] = i - 1;
					zenmen_math_GOAL_Y[zenmen_math_GOAL_count] = j;
					zenmen_math_GOAL_count++;
				}

				zenmen_math_GOAL_X[zenmen_math_GOAL_count] = i;
				zenmen_math_GOAL_Y[zenmen_math_GOAL_count] = j;
				zenmen_math_GOAL_count++;
			}
		}
	}
}
