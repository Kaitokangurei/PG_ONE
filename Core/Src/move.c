/*
 * move.c
 *
 *  Created on: Nov 27, 2024
 *      Author: lingm
 */

#include "MOTOR.h"
#include "wall.h"
#include "PID.h"
#include "PL_timer.h"
#include "PL_SENSOR.h"
#include "main.h"
#include "stdio.h"
#include "move.h"
#include <math.h>
#include "define.h"
#include "mode.h"
#include "saitan.h"
#include "wall.h"
#include "led_control.h"
#include "log.h"
#include "PL_encoder.h"
#include "motion_param.h"
#include "SPEAKER.h"

// 実行時に変更される変数
float saitan_Gv;
float straight_saitan_Gv;
float OOMWARI_R_90_omega ,OOMWARI_R_90_acc;
float OOMWARI_L_90_omega,OOMWARI_L_90_acc;
float OOMWARI_R_180_omega,OOMWARI_R_180_acc;
float OOMWARI_L_180_omega, OOMWARI_L_180_acc;
float KOMWARI_R_135_S_omega, KOMWARI_R_135_S_acc;
float KOMWARI_L_135_S_omega,KOMWARI_L_135_S_acc;
float KOMWARI_R_45_S_omega,KOMWARI_R_45_S_acc;
float KOMWARI_L_45_S_omega, KOMWARI_L_45_S_acc;
float KOMWARI_R_135_F_omega,KOMWARI_R_135_F_acc;
float KOMWARI_L_135_F_omega,KOMWARI_L_135_F_acc;
float KOMWARI_R_45_F_omega,KOMWARI_R_45_F_acc;
float KOMWARI_L_45_F_omega, KOMWARI_L_45_F_acc;
float V_90_R_omega,V_90_R_acc;
float V_90_L_omega, V_90_L_acc;
float Straight_acc;
float saitan_Gv;
float straight_saitan_Gv;

/*int motor_stop_flag;*/
//int senkai_flag;
//float Gv;
//float Gv;
float maekyori_X;
uint8_t kabegite_Threshol[5]={20,100,100,20,20};
uint16_t kabegite_Threshol_d[5]={20,400,400,20,20};
uint16_t kabegite_Threshol_d_F[5]={20,100,100,20,20};

uint16_t maekabe_Wall_Threshold[4] = {220, 70, 70, 180};

int maekabe_flag;

// 統合されたスラローム関数
void slalom(TurnDir_t dir, const SlalomParam_t *param) {
    wall_conf_flag = 1;
    Gx = 0;

    if (param->pre_dist > 0.0f) {
        while (1) {
            // --- 1. 前壁フラグが 1 の場合：センサー値判定のみで抜ける ---
            if (maekabe_flag == 1) {
                if ((g_sensor[0][0]+g_sensor[3][0]) > (maekabe_Wall_Threshold[0]+maekabe_Wall_Threshold[3]))
                     {

                    SPEAKER_PlaySE(SE_MAEKABE_BREAK);
                    maekabe_flag = 0;
                    break; // センサー閾値超えでのみターンへ
                }
            }
            // --- 2. 前壁フラグが 0 の場合：通常通り距離到達で抜ける ---
            else {
                if (Gx >= param->pre_dist) {
                    break; // 距離到達でターンへ
                }
            }
        }

        maekabe_flag = 0;
        Gx = 0;
    }
    // --- 2. スラローム旋回実行 ---
    if (dir == TURN_RIGHT) {
        omega_daikei_R(0, param->max_omega, 0, param->max_omega_acc, param->target_angle);
    } else {
        omega_daikei_L(0, param->max_omega, 0, param->max_omega_acc, param->target_angle);
    }

    // --- 3. 旋回後の制御復帰 ---
    wall_conf_flag = 1;
    Gx = 0;
    PID_turn_flag = 1;
    PID_flag = 1;

    // --- 4. 後距離（Post-Distance）の直進 ---
    if (param->post_dist > 0.0f) {
        while (Gx < param->post_dist);
    }

    Gx = 0;
    wall_conf_flag = 1;
}
// 既存のコードと互換性を保ちたい場合のラッパー関数
void slalom_R(float target_Gv) {
    const SlalomParam_t *p = (target_Gv >= 1000.0f) ? &SLALOM_PARAM_1000 : &SLALOM_PARAM_600;
    slalom(TURN_RIGHT, p);
}

void slalom_L(float target_Gv) {
    const SlalomParam_t *p = (target_Gv >= 1000.0f) ? &SLALOM_PARAM_1000 : &SLALOM_PARAM_600;
    slalom(TURN_LEFT, p);
}

// -------------------------------------------------------------
// 内部用ヘルパー: 停止してジャイロ姿勢をリセットする処理
// -------------------------------------------------------------
static void stop_and_reset_gyro(void) {
    wait_for_stable();
    v_end_flag = 1;
    PID_gyro_reset_flag = 1;
    LED_All_Off();
}

// -------------------------------------------------------------
// 内部用ヘルパー: 90度または180度の超信地旋回
// -------------------------------------------------------------
static void turn_inplace(TurnDir_t dir, float angle, float omega, float acc) {
    if (dir == TURN_RIGHT) {
        senkai_omega_daikei_R(0, omega, 0, acc, angle);
    } else {
        senkai_omega_daikei_L(0, omega, 0, acc, angle);
    }
}

// -------------------------------------------------------------
// 統合壁当てコア関数
// -------------------------------------------------------------
void kabeate_core(TurnDir_t dir, float target_Gv, float pre_dist, float post_dist, uint8_t is_goal) {

    // 1. 減速・停止
    motor_daikei(Gv, target_Gv, 0, 6000, pre_dist);

/*    motor_daikei_back(0, 200, 0, 2000, 20);
    wall_conf_flag = 0;

    // 2. 90度旋回 × 2回 (180度反転して後ろ向きにする)
   // stop_and_reset_gyro();
    turn_inplace(dir, 90.0f, 400.0f, 2000.0f);

    wait_ms(20);
    motor_daikei_back(0, 200, 0, 2000, 30);
    stop_and_reset_gyro();
    turn_inplace(dir, 90.0f, 400.0f, 2000.0f);*/
	wall_conf_flag = 0;

	// 2. 90度旋回 × 2回 (180度反転して後ろ向きにする)
   // stop_and_reset_gyro();
	turn_inplace(dir, 90.0f, 400.0f, 2000.0f);
    motor_daikei_back(0, 200, 0, 2000, 60);
    motor_daikei(Gv, target_Gv, 0, 6000,20);
	turn_inplace(dir, 90.0f, 400.0f, 2000.0f);
    motor_daikei_back(0, 200, 0, 2000, 60);
    //motor_daikei(Gv, target_Gv, 0, 6000,30);
    // 3. 壁当てバック
    if (is_goal) {
        PID_flag = 1;
        PID_turn_flag = 1;
        //motor_daikei_back(0, 300, 0, 2000, 60);
        wait_ms(50);
        motor_daikei(0, 600,0, 4000, 60);
        wait_ms(50);
        motor_daikei_back(0, 300, 0, 2000, 60);
        wait_ms(50);
    } else {
        PID_flag = 1;
        PID_turn_flag = 1;
/*        motor_daikei_back(0, 300, 0, 2000, 60);
        wait_ms(50);*/
        // 4. 前進して復帰
        wall_conf_flag = 0;
        if(target_Gv==1000){
        	motor_daikei(0, target_Gv, target_Gv, 10000, post_dist-10);
        }else{
        	motor_daikei(0, target_Gv, target_Gv, 4000, post_dist);
        }

    }

}

// =============================================================
// 各種公開関数（ラッパー）
// =============================================================

// 右回りで180度反転して壁当て
void kabeate_R(float target_Gv) {
    kabeate_core(TURN_LEFT, target_Gv, 60.0f, 110.0f, 0);
}

// 左回りで180度反転して壁当て
void kabeate_L(float target_Gv) {
    kabeate_core(TURN_RIGHT, target_Gv, 60.0f, 110.0f, 0);
}

// ゴール到達時の壁当て停止
void kabeate_GOAL_180(float target_Gv) {
    kabeate_core(TURN_RIGHT, target_Gv, 60.0f, 0.0f, 1);
}

// 連続180度旋回（簡易版）
void kabeate_180(float target_Gv) {
    motor_daikei(Gv, target_Gv, 0, 6000, 80);
/*
    wall_conf_flag = 0;
    PID_turn_flag = 3;
    wait_ms(100);
    omega_end_flag = 1;

    senkai_omega_daikei_L(0, 400, 0, 2000, 180);
*/
	wall_conf_flag = 0;

	// 2. 90度旋回 × 2回 (180度反転して後ろ向きにする)
   // stop_and_reset_gyro();
	turn_inplace(1, 90.0f, 400.0f, 2000.0f);
    //motor_daikei_back(0, 200, 0, 2000, 60);
    //motor_daikei(Gv, target_Gv, 0, 6000,30);
	turn_inplace(1, 90.0f, 400.0f, 2000.0f);
    //motor_daikei_back(0, 200, 0, 2000, 60);
    Gx = 0;
    wait_ms(10);
    motor_daikei_back(0, 300, 0, 4000, 30);
    wait_ms(50);
    wall_conf_flag = 1;
    if(target_Gv==1000){
    	motor_daikei(0, target_Gv, target_Gv, 10000, 100);
    }else{
    	motor_daikei(0, target_Gv, target_Gv, 4000, 110);
    }

}

void wait_for_stable() {
	PID_flag=4;
	PID_turn_flag=4;
    int count = 0;
    int timer_count =0;
    while(count <2000000) { // Reduced for example
        if(failsafe_flag == 1) break;
/*        if(Gomega <= 10 && Gomega >= -10 && Gv <= 20 && Gv >= -20) {
            count++;
        } else {
            count = 0;
        }*/
        if(timer_count>=2000000)break;
        timer_count++;
    }
    Gomega=0;
    Gv=0;
	omega_end_flag=1;
	v_end_flag = 1;
}
// -------------------------------------------------------------
// 内部ヘルパー: 直進制御用フラグの一括セット
// -------------------------------------------------------------
static inline void set_straight_flags(uint8_t wall_conf) {
    wall_conf_flag = wall_conf;
    PID_turn_flag  = 1;
    PID_Gyro_flag  = 1;
}

// =============================================================
// 直進・スタート・停止系関数群
// =============================================================

// スタート直進（標準壁制御あり）
void start(float target_Gv, float end_Gv, float X) {
    set_straight_flags(3);
    motor_daikei(Gv, target_Gv, end_Gv, Straight_acc, X);
}

// 汎用直進（加速度指定・吸引ファン併用）
void straight(float target_Gv, float end_Gv, float target_acc, float X) {
    Straight_acc   = target_acc;
    set_straight_flags(3);
    motor_daikei_sta(Gv, target_Gv, end_Gv, Straight_acc, X);
}

// 区画連続用直進（現在の Straight_acc で走行）
void straight_conf(float target_Gv, float end_Gv, float X) {
    set_straight_flags(3);
    motor_daikei(Gv, target_Gv, end_Gv, Straight_acc, X);
}

void straight_nokabegire(float target_Gv, float end_Gv, float target_acc, float X) {
    Straight_acc   = target_acc;
    set_straight_flags(3);
    motor_daikei_sta(Gv, target_Gv, end_Gv, Straight_acc, X);
}

// スタートダッシュ（壁制御なし・高加速度 25000）
void straight_start(float target_Gv, float end_Gv, float X) {
    set_straight_flags(0); // スタート直後は横壁がない/不安定なため壁制御OFF
    motor_daikei(Gv, target_Gv, end_Gv, 25000.0f, X);
}

// ゴール停止・フィニッシュ（指定距離で減速停止後、フェイルセーフで完全遮断）
void straight_finish(float target_Gv, float X) {
    set_straight_flags(3);
    motor_daikei(Gv, Gv, 0.0f, 20000.0f, X);

    wait_ms(10);
    v_end_flag    = 1;
    PID_Gyro_flag = 1;
  //  failsafe_flag = 1; // モーター完全停止
}

// 緊急・通常停止（壁制御OFFにして急速減速）
void move_stop(float X) {
    motor_daikei(Gv, Gv, 0.0f, 12000.0f, X);
    wall_conf_flag = 0;
    PID_turn_flag  = 0;
}
// ==========================================
// 大回り系ターン 内部コア関数
// ==========================================
static void oomwari_turn_core(TurnType type, Direction dir, float target_Gv, float angle) {
    const TrajectoryParam *param = &current_turn_param[type][dir];

    // 1. 壁切れ検出
    if (dir == DIR_R) {
        wall_kabegire_R_detection(1, 3);
    } else {
        wall_kabegire_L_detection(1, 3);
    }

    // 2. ターン進入速度への変速 & 前距離直進
    STF_Gx(param->Gcv, param->pre_dist);

    // 3. 台形角速度による旋回
    if (dir == DIR_R) {
        omega_daikei_R(0, param->omega, 0, param->acc, angle);
    } else {
        omega_daikei_L(0, param->omega, 0, param->acc, angle);
    }

    // 4. 次の動作速度(target_Gv)への変速 & 後距離直進
    wall_conf_flag = 3;
    STF_Gx(target_Gv, param->post_dist);
    LED_All_Off();
    wall_conf_flag = 3;
}

// ==========================================
// 各種公開関数（大回り 90度 / 180度）
// ==========================================

void OOMWARI_R_90(float target_Gv) {
    oomwari_turn_core(TURN_OOMWARI_90, DIR_R, target_Gv, 90.0f);
}

void OOMWARI_L_90(float target_Gv) {
    oomwari_turn_core(TURN_OOMWARI_90, DIR_L, target_Gv, 90.0f);
}

void OOMWARI_R_180(float target_Gv) {
    oomwari_turn_core(TURN_OOMWARI_180, DIR_R, target_Gv, 180.0f);
}

void OOMWARI_L_180(float target_Gv) {
    oomwari_turn_core(TURN_OOMWARI_180, DIR_L, target_Gv, 180.0f);
}
// ==========================================
// 小回り系ターン 内部コア関数
// ==========================================
static void komwari_turn_core(TurnType type, Direction dir, float target_Gv,
                              uint8_t is_dl, uint8_t kabegire_arg2, float angle,
                              uint8_t post_wall_conf) {
    const TrajectoryParam *param = &current_turn_param[type][dir];

    // 1. 壁切れ検出（DL版と通常版の切り替え）
    if (is_dl) {
        // 斜め進入用（Finish系など）
        if (dir == DIR_R) {
            wall_kabegire_R_detection_DL(1, kabegire_arg2);
        } else {
            wall_kabegire_L_detection_DL(1, kabegire_arg2);
        }
    } else {
        // 直線進入用（Start系など）
        if (dir == DIR_R) {
            wall_kabegire_R_detection(1, kabegire_arg2);
        } else {
            wall_kabegire_L_detection(1, kabegire_arg2);
        }
    }
    wall_conf_flag = 0;
    // 2. 進入速度への変速 & 前距離直進
    STF_Gx(param->Gcv, param->pre_dist);

    // 3. 台形角速度による旋回
    if (dir == DIR_R) {
        omega_daikei_R(0, param->omega, 0, param->acc, angle);
    } else {
        omega_daikei_L(0, param->omega, 0, param->acc, angle);
    }

    // 4. 脱出制御フラグのセット & 後距離直進（次の動作速度 target_Gv へ変速）
    wall_conf_flag = post_wall_conf;
    //STF_Gx(target_Gv, param->post_dist);
    LED_All_Off();
    wall_conf_flag = post_wall_conf;
}

// ==========================================
// 小回り系ターン 135度 (S / F)
// ==========================================

// 135度 Start (直線進入 -> 通常壁切れ)
void KOMWARI_R_135_S(float target_Gv) {
    komwari_turn_core(TURN_KOMWARI_135_S, DIR_R, target_Gv, 0, 3, 135.0f, 2);
}

void KOMWARI_L_135_S(float target_Gv) {
    komwari_turn_core(TURN_KOMWARI_135_S, DIR_L, target_Gv, 0, 3, 135.0f, 2);
}

// 135度 Finish (斜め進入 -> DL壁切れ)
void KOMWARI_R_135_F(float target_Gv) {
    komwari_turn_core(TURN_KOMWARI_135_F, DIR_R, target_Gv, 1, 5, 135.0f, 3);
}

void KOMWARI_L_135_F(float target_Gv) {
    komwari_turn_core(TURN_KOMWARI_135_F, DIR_L, target_Gv, 1, 4, 135.0f, 3);
}

// ==========================================
// 小回り系ターン 45度 (S / F)
// ==========================================

// 45度 Start (直線進入 -> 通常壁切れ)
void KOMWARI_R_45_S(float target_Gv) {
    komwari_turn_core(TURN_KOMWARI_45_S, DIR_R, target_Gv, 0, 3, 45.0f, 2);
}

void KOMWARI_L_45_S(float target_Gv) {
    komwari_turn_core(TURN_KOMWARI_45_S, DIR_L, target_Gv, 0, 3, 45.0f, 2);
}

// 45度 Finish (斜め進入 -> DL壁切れ)
void KOMWARI_R_45_F(float target_Gv) {
    komwari_turn_core(TURN_KOMWARI_45_F, DIR_R, target_Gv, 1, 5, 45.0f, 3);
}

void KOMWARI_L_45_F(float target_Gv) {
    komwari_turn_core(TURN_KOMWARI_45_F, DIR_L, target_Gv, 1, 4, 45.0f, 3);
}

// ==========================================
// V90 ターン
// ==========================================

// V90 ターンも komwari_turn_core (is_dl = 1, angle = 90.0f, post_wall_conf = 2) に集約可能
void V_90_R(float target_Gv) {
    komwari_turn_core(TURN_V_90, DIR_R, target_Gv, 1, 5, 90.0f, 2);
}

void V_90_L(float target_Gv) {
    komwari_turn_core(TURN_V_90, DIR_L, target_Gv, 1, 4, 90.0f, 2);
}
// ==========================================
// 斜め直線セクション
// ==========================================
void diagonal(float target_Gv, float end_Gv, float target_acc, float X) {
    Straight_acc   = target_acc;
    set_straight_flags(2);
    motor_daikei_sta(Gv, target_Gv, end_Gv, Straight_acc, X);
}

// ==========================================
// 指定区間内での変速直進 (STF: Speed Transition Forward)
// ==========================================
void STF_Gx(float target_Gcv, float turn_Gx) {
    float v0 = Gv;
    float v  = target_Gcv;
    float x  = turn_Gx;

    // 距離が微小または速度変化がない場合は等速で距離待ち
    if (x <= 0.001f || fabsf(v0 - v) < 0.001f) {
        Gx = 0;
        while (Gx < x) {
            if (failsafe_flag == 1) break; // スタック脱出ガード
        }
        LED_All_Off();
        return;
    }

    // 1. 必要な等加速度 a = (v^2 - v0^2) / (2 * x)
    float required_acc = ((v * v) - (v0 * v0)) / (2.0f * x);
    Ga = required_acc;

    // 2. 走行フラグの初期化
    Gx = 0;
    uint8_t speed_reached = 0;

    // 3. 変速・直進ループ
    while (Gx < x) {
        if (failsafe_flag == 1) break; // 異常検知時は即ループ脱出

        if (!speed_reached) {
            if (required_acc >= 0.0f) {
                // 加速フェーズ
                if (Gv >= v) {
                    Gv = v;
                    Ga = 0.0f;
                    speed_reached = 1;
                }
            } else {
                // 減速フェーズ
                if (Gv <= v) {
                    Gv = v;
                    Ga = 0.0f;
                    speed_reached = 1;
                }
            }
        }
    }

    // 4. 後処理（加速度をゼロにして安全に抜ける）
    Ga = 0.0f;
    LED_All_Off();
}
