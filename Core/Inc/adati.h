/*
 * adati.h
 *
 *  Created on: Nov 27, 2024
 *      Author: lingm
 */

#ifndef INC_ADATI_H_
#define INC_ADATI_H_

#include "define.h"

#define tansaku_Gv_600 600
#define tansaku_Gv_1000 1000

// 壁情報
extern uint32_t wall_X[Map_Size + 2] ; // 水平方向の壁データ
extern uint32_t wall_Y[Map_Size + 2] ; // 垂直方向の壁データ
extern uint32_t wall_log_X[Map_Size + 2] ; // 水平方向の壁データ
extern uint32_t wall_log_Y[Map_Size + 2] ; // 垂直方向の壁データ

extern uint16_t Player_X ;
extern uint16_t Player_Y;
extern uint16_t Player_D;

extern uint16_t GOAL_X;
extern uint16_t GOAL_Y ;

extern int maekabe_flag;

extern uint16_t math_GOAL_X;
extern uint16_t math_GOAL_Y;

void INPUT_ADATI_QUEUE(uint16_t, uint16_t ) ;

void OUTPUT_ADATI_QUEUE(uint16_t , uint16_t ) ;

// 畳み込み計算関数
void Calculate_adati_math();
void print_adatimap();

void INPUT_ADATI_QUEUE(uint16_t , uint16_t ) ;
void OUTPUT_ADATI_QUEUE(uint16_t , uint16_t) ;


void wall_get();

void adati_create_pass();
void adati_move(uint8_t);
void adati_move_2(uint8_t);
void adati_move_GOTOSTART( uint8_t );
void move_one();
void Decide_move();

void pritn_GOAL();
void control_Threshold(int );

// キュー操作関数
void INPUT_ADATI_QUEUE(uint16_t x, uint16_t y);


void OUTPUT_ADATI_QUEUE(uint16_t x, uint16_t y) ;
// 畳み込み計算関数
void Calculate_adati_math() ;

void print_adatimap() ;
void zenmen_log_map();
void create_pass();
void Calculate_adati_math_pass();

void adati_move_pass();

void add_to_history(int , int , uint8_t , uint8_t);
void delete_last_5_logs();

void change_goal();
#endif /* INC_ADATI_H_ */
