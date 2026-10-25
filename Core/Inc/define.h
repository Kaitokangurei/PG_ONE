/*
 * define.h
 *
 *  Created on: Nov 25, 2024
 *      Author: lingm
 */

#ifndef INC_DEFINE_H_
#define INC_DEFINE_H_


//MOTOR
#define reduction_ratio 73.0/41.0f
#define Tire_dia 25.0f/1000.0f//24.2f
#define TreadWidth 75.0f/1000.0f
#define m 85.0f / 1000.0f // kg
#define Tire_num 2.0f //合ってるわからない
#define V_bat 12.0f // V
#define R 1.0f // Ω
#define kE 1.47f / 1000.0f // 0.00571 V/(rad/s)
#define kT 1.47f / 1000.0f // 0.00571 N·m/A (kEと合わせるのがセオリー)



#define FUN_Duty_Rate 30

#define Map_Size 16

#define ANGLE_THRESHOLD 30.0f   // 角度偏差のしきい値（例: 30度）
#define SPEED_THRESHOLD 500.0f   // 速度偏差のしきい値（例: 500 mm/s）
#define ERROR_DURATION  80      // エラー判定までの連続時間（80ms）

#define adati_straight_cost 2
#define adati_slalom_L_cost -45
#define adati_slalom_R_cost 45
#define adati_back_cost 2

#define SENSOR_DEG 20
#define CENTER_WALL_DISTANCE 90
#define CENTER_DIAGOAL_WALL_DISTANCE 127.27

#define PI 3.141592

#endif /* INC_DEFINE_H_ */
