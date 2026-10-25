/*
 * wall.h
 *
 *  Created on: Nov 27, 2024
 *      Author: lingm
 */

#ifndef INC_WALL_H_
#define INC_WALL_H_


// 壁制御の閾値設定
/*
#define THRESHOLD_L 60
#define THRESHOLD_R 60
*/

// 壁切れ判定の差分閾値
#define THRESHOLD_DIFF_L 10
#define THRESHOLD_DIFF_R 10

// センサゲイン
#define SENSOR_GAIN 2.8f
#define SENSOR_GAIN2 3.4f
//#define SENSOR_GAIN3 0.03f
#define SENSOR_GAIN3 2.3f
// センター調整用のオフセット
extern int PID_CENTER_L;
extern int  PID_CENTER_R;



extern int g_wall_control_status;

extern int wall_get_flag;

void get_wall();
void wall_control() ;
extern float PID_wall;
extern int wall_conf_flag;

extern int wall_kabegire_R_flag;
extern int wall_kabegire_L_flag;
extern int wall_kabegire_flag;
extern int wall_distance_flag;
extern int 	wall_start_flag;
extern int dtkabegire_flag;
extern float g_sensors_smoothed_diff[4];

void Wall_ResetReference( void );
void wall_kabegire_control();
void wall_kabegire_R_detection(int ,int);
void wall_kabegire_L_detection(int ,int);
void wall_kabegire_L_R_detection(int ,int);

void wall_kabegire_R_detection_SL(int ,int);
void wall_kabegire_L_detection_SL(int ,int);
void wall_kabegire_R_detection_DL(int ,int);
void wall_kabegire_L_detection_DL(int ,int);
void wall_kabegire_R();
void wall_kabegire_L();
void wall_kabegire_R_SL();
void wall_kabegire_L_SL();
void wall_kabegire_R_DL();
void wall_kabegire_L_DL();
void wall_kabegire_R_START();
void wall_kabegire_R_RESET();
void wall_kabegire_L_START();
void wall_kabegire_L_RESET();
void wall_kabegire_R_START_SL();
void wall_kabegire_L_START_SL();
void wall_kabegire_R_START_DL();
void wall_kabegire_L_START_DL();
void wall_kabegire_ALL_START();
void wall_kabegire_ALL_RESET();

void wall_center_distance_START();
void wall_center_distance_RESET();
void wall_maekyori_control();
void wall_MAX_sensor_diff();
void wall_center_diff();
float calculate_maekyori(int );
float calculate_diagoal_maekyori(int );


void wall_distance_move();

void print_wall_control_status();
void print_avg_sensor_100();
void print_g_sensor();
void print_wall_control_status();
void print_THRESHOLD_DIFF();
void Wall_ResetReference( );
void wall_kabegire_status_R();
void get_sensor_smoothed_diff();
void wall_sensor_smoothed_diff_RESET();

#endif /* INC_WALL_H_ */
