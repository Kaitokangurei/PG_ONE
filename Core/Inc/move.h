/*
 * move.h
 *
 *  Created on: Nov 27, 2024
 *      Author: lingm
 */

#ifndef INC_MOVE_H_
#define INC_MOVE_H_

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "motion_param.h"

// -------------------------------------------------------------
// 探索スラローム制御
// -------------------------------------------------------------
void slalom(TurnDir_t dir, const SlalomParam_t *param);
void slalom_R(float target_Gv);
void slalom_L(float target_Gv);

// -------------------------------------------------------------
// 直進・停止・加減速系
// -------------------------------------------------------------
void start(float target_Gv, float end_Gv, float X);
void start_300(float X);
void straight(float target_Gv, float end_Gv, float target_acc, float X);
void straight_300(float X);
void straight_nokabegire(float target_Gv, float end_Gv, float target_acc, float X);
void straight_start(float target_Gv, float end_Gv, float X);
void straight_finish(float target_Gv, float X);
void move_stop(float X);
void move_stop_300(float X);
void diagonal(float target_Gv, float end_Gv, float target_acc, float X);
void STF_Gx(float target_Gcv, float turn_Gx);

// -------------------------------------------------------------
// 壁当て・アライメント・安定待ち
// -------------------------------------------------------------
void kabeate_R(float target_Gv);
void kabeate_L(float target_Gv);
void kabeate_180(float target_Gv);
void kabeate_GOAL_180(float target_Gv);

void wait_for_stable(void);

// 大回り
void OOMWARI_R_90(float next_v);
void OOMWARI_L_90(float next_v);
void OOMWARI_R_180(float next_v);
void OOMWARI_L_180(float next_v);

// 小回り Start
void KOMWARI_R_135_S(float next_v);
void KOMWARI_L_135_S(float next_v);
void KOMWARI_R_45_S(float next_v);
void KOMWARI_L_45_S(float next_v);

// 小回り Finish
void KOMWARI_R_135_F(float next_v);
void KOMWARI_L_135_F(float next_v);
void KOMWARI_R_45_F(float next_v);
void KOMWARI_L_45_F(float next_v);

// V90
void V_90_R(float next_v);
void V_90_L(float next_v);


float normalize_to_cos(uint16_t sensor_val, uint16_t threshold);
float normalize_to_cos2(uint16_t sensor_val, uint16_t threshold);
float normalize_to_cos_adjusted(uint16_t sensor_val, uint16_t threshold);
float normalize_to_cos_adjusted2(uint16_t sensor_val, uint16_t threshold);
void max_comparison(uint16_t val);

// -------------------------------------------------------------
// 外部公開変数 (extern)
// -------------------------------------------------------------
extern int adjust_speed_mode_acc;
extern int adjust_speed_mode_omega;
extern uint8_t kabegire_flag;
extern float maekyori_X;
extern int slalom_flag;
extern int maekabe_flag;

#endif /* INC_MOVE_H_ */
