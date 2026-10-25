/*
 * motion_param.h
 *
 * Created on: Jun 27, 2026
 * Author: 81220
 */

#ifndef INC_MOTION_PARAM_H_
#define INC_MOTION_PARAM_H_

#include <stdint.h>

// -------------------------------------------------------------
// 左右方向の定義
// -------------------------------------------------------------
typedef enum {
    DIR_L = 0,
    DIR_R = 1,
    DIR_MAX
} Direction;

typedef enum {
    TURN_LEFT  = 0,
    TURN_RIGHT = 1
} TurnDir_t;

// -------------------------------------------------------------
// 最短ターンの種類を定義
// -------------------------------------------------------------
typedef enum {
    TURN_OOMWARI_90,
    TURN_OOMWARI_180,
    TURN_KOMWARI_135_S,
    TURN_KOMWARI_45_S,
    TURN_KOMWARI_135_F,
    TURN_KOMWARI_45_F,
    TURN_V_90,
    TURN_TYPE_MAX
} TurnType;

// -------------------------------------------------------------
// 構造体定義
// -------------------------------------------------------------

// 探索用スラロームパラメータ構造体
typedef struct {
    float velocity;       // 進入速度 [mm/s]
    float pre_dist;       // 前距離 [mm]
    float post_dist;      // 後距離 [mm]
    float max_omega;      // 最高角速度 [deg/s]
    float max_omega_acc;  // 角加速度 [deg/s^2]
    float target_angle;   // 旋回角度 [deg]
} SlalomParam_t;

// ターンの軌道パラメータ構造体（最短走行用：前距離・後距離を追加）
typedef struct {
    float Gcv;        // 重心速度 [mm/s]
    float pre_dist;   // ターン前直線距離 [mm]
    float post_dist;  // ターン後直線距離 [mm]
    float omega;      // 角速度 [deg/s]
    float acc;        // 角加速度 [deg/s^2]
} TrajectoryParam;

// 直線用パラメータ構造体
typedef struct {
    float acc;   // 直線加速度 [mm/s^2]
} StraightParam;

// =============================================================
// 外部公開変数（extern 宣言）
// =============================================================

// 探索スラローム用パラメータ
extern const SlalomParam_t SLALOM_PARAM_600;
extern const SlalomParam_t SLALOM_PARAM_1000;

// 最短走行用パラメータポインタ
extern const TrajectoryParam (*current_turn_param)[DIR_MAX];
extern const StraightParam *current_straight_param;

// 各モードごとの実体テーブル
extern const TrajectoryParam turn_params_500[TURN_TYPE_MAX][DIR_MAX];
extern const TrajectoryParam turn_params_1000[TURN_TYPE_MAX][DIR_MAX];
extern const TrajectoryParam turn_params_1500[TURN_TYPE_MAX][DIR_MAX]; // ← 追加

// =============================================================
// 関数宣言
// =============================================================
float get_next_Gcv(int current_index, float default_saitan_v, float max_straight_v);
void change_para(int mode);

#endif /* INC_MOTION_PARAM_H_ */
