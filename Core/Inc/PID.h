/*
 * PID.h
 *
 *  Created on: Nov 25, 2024
 *      Author: lingm
 */

#ifndef INC_PID_H_
#define INC_PID_H_

/*extern float logL[2000];
extern float logR[2000];
extern int log_flag;*/

extern int PID_gyro_reset_flag;

// ゲイン定数（適宜調整してください）
#define Kp_enc 0.38f   // わずかに下げて安定性向上
#define Ki_enc 0.08f  // ハンチング抑制のため大幅に低減
#define Kd_enc 0.008f  // ブレーキ（制動）を強める

#define Ktp_enc 1.4f // 比例ゲイン
#define Kti_enc 0.3f // 積分ゲイン
#define Ktd_enc 0.002f // 微分ゲイン
/*
#define  Kp_gyro  0.2f  // 比例ゲイン
#define  Ki_gyro  0.4f  // 積分ゲイン
#define  Kd_gyro  0.6f  // 微分ゲイン
*/

#define  Kp_gyro  4.0f  // 比例ゲイン
#define  Ki_gyro  0.001f  // 積分ゲイン
#define  Kd_gyro  0.001f  // 微分ゲイン

#define  Ksp_gyro  1.30f  // 比例ゲイン600
#define  Ksi_gyro  0.015f  // 積分ゲイン
#define  Ksd_gyro  0.0f  // 微分ゲイン

#define  Ktp_gyro 	1.8  // 比例ゲイン
#define  Kti_gyro  0.015f  // 積分ゲイン
#define  Ktd_gyro  0.001f  // 微分ゲイン
/*

#define Ki_enc 0.01f // かなり小さくして様子見
#define Kd_enc 0.1f // 少しブレーキを強める

#define Ktp_enc 5.0f // 比例ゲイン
#define Kti_enc 0.05f // 積分ゲイン
#define Ktd_enc 0.05f // 微分ゲイン

#define  Kp_gyro  0.2f  // 比例ゲイン
#define  Ki_gyro  0.4f  // 積分ゲイン
#define  Kd_gyro  0.6f  // 微分ゲイン


#define  Kp_gyro  4.2f  // 比例ゲイン
#define  Ki_gyro  0.15f  // 積分ゲイン
#define  Kd_gyro  0.01f  // 微分ゲイン

#define  Ksp_gyro  12.0f  // 比例ゲイン600
#define  Ksi_gyro  1.5f  // 積分ゲイン
#define  Ksd_gyro  0.01f  // 微分ゲイン

#define  Ktp_gyro 	7.5f  // 比例ゲイン
#define  Kti_gyro  0.25f  // 積分ゲイン
#define  Ktd_gyro  0.1f  // 微分ゲイン
*/
//
//直線修正前(2025_8_05)
//#define  Kp_FUN_enc  10.5f  // 比例ゲイン
//#define  Ki_FUN_enc  0.3f  // 積分ゲイン
//#define  Kd_FUN_enc  0.01f  // 微分ゲイン
#define  Kp_FUN_enc 2.9f  // 比例ゲイン
#define  Ki_FUN_enc  0.45f  // 積分ゲイン
#define  Kd_FUN_enc  0.001f  // 微分ゲイン

#define Ktp_FUN_enc 2.2f // 比例ゲイン
#define Kti_FUN_enc 0.5f // 積分ゲイン
#define Ktd_FUN_enc 0.001f // 微分ゲイン

#define  Kp_FUN_gyro  5.8f   // Halved to reduce initial oscillation
#define  Ki_FUN_gyro  0.2f  // Drastically reduced to prevent the huge undershoot
#define  Kd_FUN_gyro  0.001f // Reduced to dampen the "jitter"

#define  Ksp_FUN_gyro  2.8f  // 比例ゲイン
#define  Ksi_FUN_gyro  0.10f  // 積分ゲイン
#define  Ksd_FUN_gyro  0.001f  // 微分ゲイン

#define  Ktp_FUN_gyro  0.8f  // 比例ゲイン
#define  Kti_FUN_gyro  0.020f  // 積分ゲイン
#define  Ktd_FUN_gyro  0.001f  // 微分ゲイン


#define  Kp_FUN_tansaku_enc  2.0f  // 比例ゲイン
#define  Ki_FUN_tansaku_enc  0.15f  // 積分ゲイン
#define  Kd_FUN_tansaku_enc  0.006f  // 微分ゲイン

#define  Ktp_FUN_tansaku_enc  2.2f  // 比例ゲイン
#define  Kti_FUN_tansaku_enc  0.5f  // 積分ゲイン
#define  Ktd_FUN_tansaku_enc  0.001f  // 微分ゲイン

#define  Kp_FUN_tansaku_gyro  4.2f   // Halved to reduce initial oscillation
#define  Ki_FUN_tansaku_gyro  0.15f  // Drastically reduced to prevent the huge undershoot
#define  Kd_FUN_tansaku_gyro  0.001f  // Reduced to dampen the "jitter"

#define  Ksp_FUN_tansaku_gyro  1.8f  // 比例ゲイン
#define  Ksi_FUN_tansaku_gyro  0.023f  // 積分ゲイン
#define  Ksd_FUN_tansaku_gyro  0.001f  // 微分ゲイン

#define  Ktp_FUN_tansaku_gyro  0.8f  // 比例ゲイン
#define  Kti_FUN_tansaku_gyro  0.001f  // 積分ゲイン
#define  Ktd_FUN_tansaku_gyro  0.001f  // 微分ゲイン
/*

// ゲイン定数（適宜調整してください）
#define Kp_enc 18.0f
#define Ki_enc 0.1f
#define Kd_enc 4.50f

#define Ktp_enc 5.0f // 比例ゲイン
#define Kti_enc 0.05f // 積分ゲイン
#define Ktd_enc 0.05f // 微分ゲイン

#define  Kp_gyro  0.2f  // 比例ゲイン
#define  Ki_gyro  0.4f  // 積分ゲイン
#define  Kd_gyro  0.6f  // 微分ゲイン


#define  Kp_gyro  4.2f  // 比例ゲイン
#define  Ki_gyro  0.15f  // 積分ゲイン
#define  Kd_gyro  0.01f  // 微分ゲイン

#define  Ksp_gyro  12.0f  // 比例ゲイン600
#define  Ksi_gyro  0.8f  // 積分ゲイン
#define  Ksd_gyro  0.01f  // 微分ゲイン

#define  Ktp_gyro 	7.5f  // 比例ゲイン
#define  Kti_gyro  0.25f  // 積分ゲイン
#define  Ktd_gyro  0.1f  // 微分ゲイン


#define Ki_enc 0.01f // かなり小さくして様子見
#define Kd_enc 0.1f // 少しブレーキを強める

#define Ktp_enc 5.0f // 比例ゲイン
#define Kti_enc 0.05f // 積分ゲイン
#define Ktd_enc 0.05f // 微分ゲイン

#define  Kp_gyro  0.2f  // 比例ゲイン
#define  Ki_gyro  0.4f  // 積分ゲイン
#define  Kd_gyro  0.6f  // 微分ゲイン


#define  Kp_gyro  4.2f  // 比例ゲイン
#define  Ki_gyro  0.15f  // 積分ゲイン
#define  Kd_gyro  0.01f  // 微分ゲイン

#define  Ksp_gyro  12.0f  // 比例ゲイン600
#define  Ksi_gyro  1.5f  // 積分ゲイン
#define  Ksd_gyro  0.01f  // 微分ゲイン

#define  Ktp_gyro 	7.5f  // 比例ゲイン
#define  Kti_gyro  0.25f  // 積分ゲイン
#define  Ktd_gyro  0.1f  // 微分ゲイン

//
//直線修正前(2025_8_05)
//#define  Kp_FUN_enc  10.5f  // 比例ゲイン
//#define  Ki_FUN_enc  0.3f  // 積分ゲイン
//#define  Kd_FUN_enc  0.01f  // 微分ゲイン
#define  Kp_FUN_enc  35.5f  // 比例ゲイン
#define  Ki_FUN_enc  0.25f  // 積分ゲイン
#define  Kd_FUN_enc  35.5f  // 微分ゲイン

#define  Kp_FUN_gyro  6.5f   // Halved to reduce initial oscillation
#define  Ki_FUN_gyro  0.2f  // Drastically reduced to prevent the huge undershoot
#define  Kd_FUN_gyro  0.0f  // Reduced to dampen the "jitter"

#define  Ksp_FUN_gyro  10.2f  // 比例ゲイン
#define  Ksi_FUN_gyro  0.1f  // 積分ゲイン
#define  Ksd_FUN_gyro  0.3f  // 微分ゲイン

#define  Ktp_FUN_gyro  8.4f  // 比例ゲイン
#define  Kti_FUN_gyro  0.4f  // 積分ゲイン
#define  Ktd_FUN_gyro  0.1f  // 微分ゲイン


#define  Kp_FUN_tansaku_enc  10.5f  // 比例ゲイン
#define  Ki_FUN_tansaku_enc  0.1f  // 積分ゲイン
#define  Kd_FUN_tansaku_enc  0.5f  // 微分ゲイン

#define  Kp_FUN_tansaku_gyro  0.1f   // Halved to reduce initial oscillation
#define  Ki_FUN_tansaku_gyro  0.15f  // Drastically reduced to prevent the huge undershoot
#define  Kd_FUN_tansaku_gyro  0.08f  // Reduced to dampen the "jitter"

#define  Ksp_FUN_tansaku_gyro  16.2f  // 比例ゲイン
#define  Ksi_FUN_tansaku_gyro  0.2f  // 積分ゲイン
#define  Ksd_FUN_tansaku_gyro  0.3f  // 微分ゲイン

#define  Ktp_FUN_tansaku_gyro  8.4f  // 比例ゲイン
#define  Kti_FUN_tansaku_gyro  0.4f  // 積分ゲイン
#define  Ktd_FUN_tansaku_gyro  0.1f  // 微分ゲイン
*/


extern float PID_stra;
extern float PID_turn;
extern int PID_turn_flag;
extern int FUN_PID_flag;

typedef struct {
    float error;        // 現在の誤差
    float delta_error;  // 前回との差分（微分項）
    float old_error;    // 前回の誤差（微分項計算用）
    float sigma_error;  // 誤差の累積（積分項）
} PID_Control;





// グローバル変数の宣言
extern PID_Control Enc;
extern PID_Control Gyro;

void PID_Enc(float);
void PID_Turn_Enc(float);
void PID_Gyro(float);
void PID_SLALOM_Gyro(float);
void PID_TURN_Gyro(float);

void PID_FUN_tansaku_Enc(float);
void PID_Turn_FUN_tansaku_Enc(float);
void PID_FUN_tansaku_Gyro(float);
void PID_FUN_SLALOM_tansaku_Gyro(float);
void PID_FUN_TURN_tansaku_Gyro(float);

void PID_FUN_Enc(float);
void PID_Turn_FUN_Enc(float);
void PID_FUN_Gyro(float);
void PID_FUN_SLALOM_Gyro(float);
void PID_FUN_TURN_Gyro(float);

extern int v_end_flag;
extern int omega_end_flag;
extern int PID_flag;
extern int PID_Gyro_flag;

#endif /* INC_PID_H_ */
