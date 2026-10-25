/*
 * motion_param.c
 *
 * Created on: Jun 27, 2026
 * Author: 81220
 */

#include "motion_param.h"
#include "move.h"
#include "MOTOR.h"
#include "saitan.h"

// -------------------------------------------------------------
// 探索スラローム用パラメータプリセット
// -------------------------------------------------------------
const SlalomParam_t SLALOM_PARAM_600 = {
    .velocity      = 600.0f,
    .pre_dist      = 2.0f,      // 前距離
    .post_dist     = 2.0f,      // 後距離
    .max_omega     = 600.0f,
    .max_omega_acc = 9000.0f,
    .target_angle  = 90.0f
};

const SlalomParam_t SLALOM_PARAM_1000 = {
    .velocity      = 1000.0f,
    .pre_dist      = 1.0f,
    .post_dist     = 5.0f,
    .max_omega     = 1400.0f,
    .max_omega_acc = 20000.0f,
    .target_angle  = 90.0f
};

// -------------------------------------------------------------
// 最短走行 ターンパラメータテーブル (500 mm/s)
// -------------------------------------------------------------
// Format: { Gcv, pre_dist, post_dist, omega, acc }
const TrajectoryParam turn_params_500[TURN_TYPE_MAX][DIR_MAX] = {
    [TURN_OOMWARI_90]    = {{ 500.0f, 50.0f, 60.0f, 400.0f, 2000.0f},  { 500.0f, 50.0f, 60.0f, 400.0f, 2000.0f}},
    [TURN_OOMWARI_180]   = {{ 500.0f, 30.0f, 60.0f, 300.0f, 1500.0f},  { 500.0f, 30.0f, 60.0f, 300.0f, 1500.0f}},
    [TURN_KOMWARI_135_S] = {{ 500.0f,  50.0f,  40.0f, 400.0f, 3000.0f},  { 500.0f,  50.0f,  40.0f, 400.0f, 3000.0f}},
    [TURN_KOMWARI_45_S]  = {{ 500.0f,  50.0f,  20.0f, 400.0f, 4000.0f},  { 500.0f,  50.0f,  20.0f, 400.0f, 4000.0f}},
    [TURN_KOMWARI_135_F] = {{ 500.0f,  30.0f,  5.0f,300.0f, 3500.0f},  { 500.0f,  30.0f,  5.0f, 300.0f, 3500.0f}},
    [TURN_KOMWARI_45_F]  = {{ 500.0f,  50.0f,  10.0f, 400.0f, 2000.0f},  { 500.0f,  50.0f,  10.0f, 300.0f, 2000.0f}},
    [TURN_V_90]          = {{ 500.0f,  25.0f,  5.0f, 300.0f, 2500.0f},  { 500.0f,  25.0f,  5.0f, 300.0f, 2500.0f}},
};

// -------------------------------------------------------------
// 最短走行 ターンパラメータテーブル (1000 mm/s)
// -------------------------------------------------------------
const TrajectoryParam turn_params_1000[TURN_TYPE_MAX][DIR_MAX] = {
    //                                         左 (DIR_L)                                                   右 (DIR_R)
    //                          {  Gcv,    pre,  post,   omega,     acc   },             {  Gcv,    pre,  post,   omega,     acc   }
    [TURN_OOMWARI_90]    = {{1000.0f, 35.0f, 15.0f,  600.0f,  6000.0f},    {1000.0f, 35.0f, 15.0f,  600.0f,  6000.0f}},
    [TURN_OOMWARI_180]   = {{1000.0f, 20.0f, 20.0f,  600.0f,  6500.0f},    {1000.0f, 20.0f, 20.0f,  600.0f,  6500.0f}},
    [TURN_KOMWARI_135_S] = {{ 1000.0f, 10.0f, 15.0f,  600.0f,  7000.0f},    { 1000.0f, 10.0f, 15.0f, 600.0f,  7000.0f}},
    [TURN_KOMWARI_45_S]  = {{1000.0f, 10.0f, 5.0f, 600.0f, 7000.0f},    {1000.0f, 10.0f, 5.0f, 600.0f, 7000.0f}},
    [TURN_KOMWARI_135_F] = {{ 1000.0f, 10.0f, 10.0f,  900.0f,  8000.0f},    { 1000.0f, 10.0f, 10.0f,  900.0f,  8000.0f}},
    [TURN_KOMWARI_45_F]  = {{1000.0f, 20.0f, 10.0f,  700.0f,  7000.0f},    {1000.0f, 20.0f, 10.0f,  700.0f,  7000.0f}},
    [TURN_V_90]          = {{1000.0f,  15.0f,  5.0f, 800.0f, 12000.0f},    {1000.0f,  15.0f,  5.0f, 800.0f, 12000.0f}},
};

// -------------------------------------------------------------
// 最短走行 ターンパラメータテーブル (1500 mm/s)
// -------------------------------------------------------------
const TrajectoryParam turn_params_1500[TURN_TYPE_MAX][DIR_MAX] = {
    //                                         左 (DIR_L)                                                   右 (DIR_R)
    //                          {  Gcv,    pre,  post,   omega,     acc   },             {  Gcv,    pre,  post,   omega,     acc   }
    [TURN_OOMWARI_90]    = {{1500.0f, 25.0f, 20.0f, 1200.0f, 12000.0f},    {1500.0f, 25.0f, 20.0f, 1200.0f, 12000.0f}},
    [TURN_OOMWARI_180]   = {{1500.0f, 20.0f, 20.0f, 900.0f, 11000.0f},    {1500.0f, 20.0f, 20.0f, 900.0f, 11000.0f}},
    [TURN_KOMWARI_135_S] = {{1500.0f, 8.0f, 5.0f, 1000.0f, 15000.0f},    {1500.0f, 8.0f, 5.0f, 1000.0f, 15000.0f}},
    [TURN_KOMWARI_45_S]  = {{1500.0f, 10.0f, 5.0f, 1100.0f, 14000.0f},    {1500.0f, 10.0f, 5.0f, 1100.0f, 14000.0f}},
    [TURN_KOMWARI_135_F] = {{1500.0f, 4.0f, 5.0f, 1200.0f, 17000.0f},    {1500.0f, 4.0f, 5.0f, 1200.0f, 17000.0f}},
    [TURN_KOMWARI_45_F]  = {{1500.0f, 20.0f, 20.0f, 1000.0f, 15000.0f},    {1500.0f, 20.00f, 20.0f, 1000.0f, 15000.0f}},
    [TURN_V_90]          = {{1500.0f, 5.0f,2.0f, 1300.0f, 20000.0f},    {1500.0f, 5.0f, 2.0f, 1300.0f, 20000.0f}},
};

// -------------------------------------------------------------
// 最短走行 ターンパラメータテーブル (2000 mm/s)
// -------------------------------------------------------------
const TrajectoryParam turn_params_2000[TURN_TYPE_MAX][DIR_MAX] = {
    //                                         左 (DIR_L)                                                   右 (DIR_R)
    //                          {  Gcv,    pre,  post,   omega,     acc   },             {  Gcv,    pre,  post,   omega,     acc   }
    [TURN_OOMWARI_90]    = {{2000.0f, 6.0f, 40.0f, 1500.0f, 20000.0f},    {2000.0f, 6.0f, 40.0f, 1500.0f, 20000.0f}},
    [TURN_OOMWARI_180]   = {{2000.0f, 10.0f, 60.0f, 1200.0f, 15000.0f},    {2000.0f, 10.0f, 60.0f, 1200.0f, 15000.0f}},
    [TURN_KOMWARI_135_S] = {{2000.0f,  3.0f,  15.0f, 1600.0f, 24000.0f},    {2000.0f,  3.0f,  15.0f, 1600.0f, 24000.0f}},
    [TURN_KOMWARI_45_S]  = {{2000.0f,  1.0f,  15.0f, 1600.0f, 22000.0f},    {2000.0f,  1.0f,  15.0f, 1600.0f, 22000.0f}},
    [TURN_KOMWARI_135_F] = {{2000.0f,  5.0f,35.0f, 2200.0f, 26000.0f},    {2000.0f,  5.0f, 35.0f, 2200.0f, 26000.0f}},
    [TURN_KOMWARI_45_F]  = {{2000.0f,  12.0f, 12.0f, 1300.0f, 16000.0f},    {2000.0f,  12.0f, 12.0f, 1300.0f, 16000.0f}},
    [TURN_V_90]          = {{2000.0f,  5.0f,  5.0f, 2300.0f, 35000.0f},    {2000.0f,  5.0f,  5.0f, 2300.0f, 35000.0f}},
};

// -------------------------------------------------------------
// 最短走行 ターンパラメータテーブル (2400 mm/s)
// 2000用から速度比1.2、角速度1.2倍、角加速度1.44倍で仮設定
// pre/postは2000と同値。実機で要調整
// -------------------------------------------------------------
const TrajectoryParam turn_params_2400[TURN_TYPE_MAX][DIR_MAX] = {
    //                                         左 (DIR_L)                                                   右 (DIR_R)
    //                          {  Gcv,    pre,  post,   omega,     acc   },             {  Gcv,    pre,  post,   omega,     acc   }
    [TURN_OOMWARI_90]    = {{2400.0f, 10.0f, 20.0f, 1800.0f, 25920.0f},    {2400.0f, 10.0f, 20.0f, 1800.0f, 25920.0f}},
    [TURN_OOMWARI_180]   = {{2400.0f, 10.0f, 20.0f, 1440.0f, 21600.0f},    {2400.0f, 10.0f, 20.0f, 1440.0f, 21600.0f}},
    [TURN_KOMWARI_135_S] = {{2400.0f,  3.0f,  5.0f, 1920.0f, 34560.0f},    {2400.0f,  3.0f,  5.0f, 1920.0f, 34560.0f}},
    [TURN_KOMWARI_45_S]  = {{2400.0f,  1.0f,  5.0f, 1680.0f, 44360.0f},    {2400.0f,  1.0f,  5.0f, 1680.0f, 44360.0f}},
    [TURN_KOMWARI_135_F] = {{2400.0f,  7.0f, 5.0f, 2640.0f, 37440.0f},    {2400.0f,  7.0f, 5.0f, 2640.0f, 37440.0f}},
    [TURN_KOMWARI_45_F]  = {{2400.0f,  12.0f, 12.0f, 1560.0f, 23040.0f},    {2400.0f,  12.0f, 12.0f, 1560.0f, 23040.0f}},
    [TURN_V_90]          = {{2400.0f,  2.0f,  5.0f, 2760.0f, 50400.0f},    {2400.0f,  2.0f,  5.0f, 2760.0f, 50400.0f}},
};


// 選択中のテーブルポインタ
const TrajectoryParam (*current_turn_param)[DIR_MAX] = turn_params_1000;

// -------------------------------------------------------------
// 直線用パラメータ定義
// -------------------------------------------------------------
const StraightParam straight_params_500  = { .acc = 10000.0f };
const StraightParam straight_params_1000 = { .acc = 10000.0f };
const StraightParam straight_params_1500 = { .acc = 10000.0f };
// 2000用の直線パラメータ
const StraightParam straight_params_2000 = { .acc = 30000.0f };

// 2400用の直線パラメータ（仮設定）
const StraightParam straight_params_2400 = { .acc = 30000.0f };


const StraightParam *current_straight_param = &straight_params_1000;

void change_para(int mode) {
    switch (mode) {
        case 500:
            current_turn_param     = turn_params_500;
            current_straight_param = &straight_params_500;
            break;

        case 1000:
            current_turn_param     = turn_params_1000;
            current_straight_param = &straight_params_1000;
            break;

        case 1500:
            current_turn_param     = turn_params_1500;
            current_straight_param = &straight_params_1500;
            break;

        case 2000:
            current_turn_param     = turn_params_2000; // ★ 2000用
            current_straight_param = &straight_params_2000;
            break;

        case 2400:
            current_turn_param     = turn_params_2400;
            current_straight_param = &straight_params_2400;
            break;

        default:
            current_turn_param     = turn_params_1000;
            current_straight_param = &straight_params_1000;
            break;
    }
}

/**
 * @brief 次のアクションに応じた目標重心速度(Gcv)を取得する
 */
float get_next_Gcv(int move_num, float default_saitan_v, float max_straight_v) {
    if (move_num == 0) {
        return 1000.0f;
    }

    int next_pass = move_num;

    // 次が「直線」または「斜め直線」の場合は、現在の直進速度を維持
    if ((1 <= next_pass && next_pass <= 40) || (-40 <= next_pass && next_pass <= -1)) {
        return Gv;
    }

    switch (next_pass) {
        case OOMWARI_R_90_NUM:     return current_turn_param[TURN_OOMWARI_90][DIR_R].Gcv;
        case OOMWARI_L_90_NUM:     return current_turn_param[TURN_OOMWARI_90][DIR_L].Gcv;

        case OOMWARI_R_180_NUM:    return current_turn_param[TURN_OOMWARI_180][DIR_R].Gcv;
        case OOMWARI_L_180_NUM:    return current_turn_param[TURN_OOMWARI_180][DIR_L].Gcv;

        case KOMWARI_R_135_S_NUM:  return current_turn_param[TURN_KOMWARI_135_S][DIR_R].Gcv;
        case KOMWARI_L_135_S_NUM:  return current_turn_param[TURN_KOMWARI_135_S][DIR_L].Gcv;

        case KOMWARI_R_45_S_NUM:   return current_turn_param[TURN_KOMWARI_45_S][DIR_R].Gcv;
        case KOMWARI_L_45_S_NUM:   return current_turn_param[TURN_KOMWARI_45_S][DIR_L].Gcv;

        case KOMWARI_R_135_F_NUM:  return current_turn_param[TURN_KOMWARI_135_F][DIR_R].Gcv;
        case KOMWARI_L_135_F_NUM:  return current_turn_param[TURN_KOMWARI_135_F][DIR_L].Gcv;

        case KOMWARI_R_45_F_NUM:   return current_turn_param[TURN_KOMWARI_45_F][DIR_R].Gcv;
        case KOMWARI_L_45_F_NUM:   return current_turn_param[TURN_KOMWARI_45_F][DIR_L].Gcv;

        case V_90_R_NUM:           return current_turn_param[TURN_V_90][DIR_R].Gcv;
        case V_90_L_NUM:           return current_turn_param[TURN_V_90][DIR_L].Gcv;

        case 45:
        case -45:
            return default_saitan_v;

        default:
            return default_saitan_v;
    }
}
