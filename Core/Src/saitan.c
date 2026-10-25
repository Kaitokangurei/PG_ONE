/*
 * saitan.c
 *
 *  Created on: Dec 14, 2024
 *      Author: suzuk
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "saitan.h"
#include "adati.h"
#include "move.h"
#include "MOTOR.h"
#include "wall.h"
#include "PL_sensor.h"
#include "PL_timer.h"
#include "PID.h"
#include "define.h"
#include "mode.h"
#include "motion_param.h"

#define QUEUE_MAX_SIZE 512


uint16_t STA_cost     = 8;
uint16_t DIA_cost     = 30;
uint16_t CON_STA_cost = 4;
uint16_t CON_DIA_cost = 15;
/*uint16_t CON_STA_cost = 4;
uint16_t CON_DIA_cost = 7;*/

uint16_t saitan_math_goalX = 7;
uint16_t saitan_math_goalY = 8;
int saitan_math_dir;

uint16_t GOAL_dir1 = 0;
uint16_t GOAL_dir2 = 180;
uint16_t GOAL_dir3 = 90;
uint16_t GOAL_dir4 = 270;
uint8_t GOAL_num = 2;
uint8_t slalom_acc_flag = 0;

static uint16_t head = 0, tail = 0;

uint8_t D_pos, D_X, D_Y;
int D_dir;
uint8_t D_num;

// saitan.c
DaikusutoraQueue_t queue[QUEUE_MAX_SIZE];
uint16_t DAIKUSUTORA_pass_count;
int DAIKUSUTORA_pass[258];
int DAIKUSUTORA_pass_OOMAWARI[258];
uint16_t DAIKUSUTORA_num[3][Map_Size + 1][Map_Size + 1] = {0};

uint32_t temp_wall_X[Map_Size + 2] = {0};
uint32_t temp_wall_Y[Map_Size + 2] = {0};

// -------------------------------------------------------------
// キュー操作関数
// -------------------------------------------------------------
void INPUT_DAIKUSUTORA_QUEUE(uint8_t pos, uint8_t X, uint8_t Y, int dir, uint8_t con_num) {
    queue[tail].pos     = pos;
    queue[tail].X       = X;
    queue[tail].Y       = Y;
    queue[tail].dir     = dir;
    queue[tail].con_num = con_num;
    tail = (tail + 1) % QUEUE_MAX_SIZE;
}

void OUTPUT_DAIKUSUTORA_QUEUE(void) {
    D_pos = queue[head].pos;
    D_X   = queue[head].X;
    D_Y   = queue[head].Y;
    D_dir = queue[head].dir;
    D_num = queue[head].con_num;
    head  = (head + 1) % QUEUE_MAX_SIZE;
}

void intilaize_DAIKUSUTORA_QUEUE(void) {
    for (int i = 0; i < QUEUE_MAX_SIZE; i++) {
        queue[i].pos     = 0;
        queue[i].X       = 0;
        queue[i].Y       = 0;
        queue[i].dir     = 0;
        queue[i].con_num = 0;
    }
    head = 0;
    tail = 0;
}

void intilaize_DAIKUSUTORA(void) {
    for (int i = 0; i <= Map_Size; i++) {
        for (int j = 0; j <= Map_Size; j++) {
            DAIKUSUTORA_num[1][j][i] = 9999;
            DAIKUSUTORA_num[2][j][i] = 9999;
        }
    }
}

// -------------------------------------------------------------
// コスト比較・更新の統合関数
// -------------------------------------------------------------
static void update_daikusutora_cost(uint8_t COM_pos, uint8_t COM_X, uint8_t COM_Y,
                                   int COM_dir, uint8_t COM_num,
                                   uint16_t normal_cost, uint16_t con_cost) {
    uint16_t current_cost = DAIKUSUTORA_num[D_pos][D_X][D_Y];

    // 同一方向への直進連続ボーナス
    if (COM_dir == D_dir) {
        uint16_t cost_diff = (con_cost > COM_num) ? (con_cost - COM_num) : 0;
        if (DAIKUSUTORA_num[COM_pos][COM_X][COM_Y] > current_cost + cost_diff) {
            DAIKUSUTORA_num[COM_pos][COM_X][COM_Y] = current_cost + cost_diff;
            if (cost_diff > 2) {
                COM_num++;
            }
            INPUT_DAIKUSUTORA_QUEUE(COM_pos, COM_X, COM_Y, COM_dir, COM_num);
            return;
        }
    }

    // 通常の遷移コスト
    if (DAIKUSUTORA_num[COM_pos][COM_X][COM_Y] > current_cost + normal_cost) {
        DAIKUSUTORA_num[COM_pos][COM_X][COM_Y] = current_cost + normal_cost;
        INPUT_DAIKUSUTORA_QUEUE(COM_pos, COM_X, COM_Y, COM_dir, 0);
    }
}

// -------------------------------------------------------------
// パス配列から 0（削除マーク）を詰めるヘルパー関数
// -------------------------------------------------------------
static void remove_zero_pass(void) {
    for (int i = 0; i <= DAIKUSUTORA_pass_count; i++) {
        if (DAIKUSUTORA_pass[i] == 0) {
            for (int j = i; j < DAIKUSUTORA_pass_count; j++) {
                DAIKUSUTORA_pass[j] = DAIKUSUTORA_pass[j + 1];
            }
            DAIKUSUTORA_pass_count--;
            i--;
        }
    }
    DAIKUSUTORA_pass[DAIKUSUTORA_pass_count + 1] = 0;
}

// -------------------------------------------------------------
// 最短走行実行
// -------------------------------------------------------------
void move_saitan(int mode) {
    float saitan_Gv;          // ターンの標準基準速度
    float straight_saitan_Gv; // 直線最高目標速度
    float next_Gcv;

    if (DAIKUSUTORA_pass_count >= 200) {
        return; // パス異常時は中断
    }

    // 1. モードに応じた速度設定
    if (mode == 500) {
        saitan_Gv          = 500.0f;
        straight_saitan_Gv = 3000.0f; // モード500時の直線最高速度
    } else if (mode == 1000) {
        saitan_Gv          = 1000.0f;
        straight_saitan_Gv = 6500.0f;
    } else if (mode == 1500) {
        saitan_Gv          = 1500.0f;
        straight_saitan_Gv = 6000.0f;
    }  else if (mode == 2000) {
        saitan_Gv          = 2000.0f;
        straight_saitan_Gv = 6000.0f;
    }  else if (mode == 2400) {
        saitan_Gv          = 2400.0f;
        straight_saitan_Gv = 6000.0f;
    }  else {
        saitan_Gv          = 1000.0f;
        straight_saitan_Gv = 5000.0f;
    }

    // 2. モードに応じたパラメータ（テーブル・加速度）をセット
    change_para(mode);
    float current_straight_acc = current_straight_param->acc;

    // 斜め壁切れ判定用のタイプを指定する変数（デフォルト値）
        uint8_t current_dl_kabegire_R = 5;
        uint8_t current_dl_kabegire_L = 4;

    // 3. 走行ループ（全モード共通）
    for (int i = 0; i <= DAIKUSUTORA_pass_count; i++) {
        // 次のアクションに応じた目標速度を先読み
        next_Gcv = get_next_Gcv(DAIKUSUTORA_pass[i + 1], saitan_Gv, straight_saitan_Gv);

        // ---------------------------------------------------------
        // 1. 直線区間 (1〜40枚)
        // ---------------------------------------------------------
        if (1 <= DAIKUSUTORA_pass[i] && DAIKUSUTORA_pass[i] <= 40) {
        	if(DAIKUSUTORA_pass[i + 1]==0){
        		straight_nokabegire(straight_saitan_Gv, next_Gcv, current_straight_acc, 90.0f * DAIKUSUTORA_pass[i]);
        	}
        	else if (i != 0) {
                straight(straight_saitan_Gv, next_Gcv, current_straight_acc, 90.0f * DAIKUSUTORA_pass[i]);
            } else {
                // 初動区間
                if (DAIKUSUTORA_pass[i] == 1) {
                    straight_start(straight_saitan_Gv, next_Gcv, 90.0f * DAIKUSUTORA_pass[i] - 80.0f);
                } else if (DAIKUSUTORA_pass[i] >= 3) {
                    straight(straight_saitan_Gv, next_Gcv, current_straight_acc, 90.0f * DAIKUSUTORA_pass[i] -60.0f);
                } else{
                    straight(straight_saitan_Gv, next_Gcv, current_straight_acc, 90.0f * DAIKUSUTORA_pass[i] - 60.0f);
                }
            }
        }
        // ---------------------------------------------------------
        // 2. ターン系マクロ
        // ---------------------------------------------------------
        else if (OOMWARI_R_90_NUM    == DAIKUSUTORA_pass[i]) { OOMWARI_R_90(next_Gcv); }
        else if (OOMWARI_L_90_NUM    == DAIKUSUTORA_pass[i]) { OOMWARI_L_90(next_Gcv); }
        else if (OOMWARI_R_180_NUM   == DAIKUSUTORA_pass[i]) { OOMWARI_R_180(next_Gcv); }
        else if (OOMWARI_L_180_NUM   == DAIKUSUTORA_pass[i]) { OOMWARI_L_180(next_Gcv); }
        else if (KOMWARI_R_135_S_NUM == DAIKUSUTORA_pass[i]) { KOMWARI_R_135_S(next_Gcv); }
        else if (KOMWARI_L_135_S_NUM == DAIKUSUTORA_pass[i]) { KOMWARI_L_135_S(next_Gcv); }
        else if (KOMWARI_R_45_S_NUM  == DAIKUSUTORA_pass[i]) { KOMWARI_R_45_S(next_Gcv); }
        else if (KOMWARI_L_45_S_NUM  == DAIKUSUTORA_pass[i]) { KOMWARI_L_45_S(next_Gcv); }
        else if (KOMWARI_R_135_F_NUM == DAIKUSUTORA_pass[i]) { KOMWARI_R_135_F(next_Gcv); }
        else if (KOMWARI_L_135_F_NUM == DAIKUSUTORA_pass[i]) { KOMWARI_L_135_F(next_Gcv); }
        else if (KOMWARI_R_45_F_NUM  == DAIKUSUTORA_pass[i]) { KOMWARI_R_45_F(next_Gcv); }
        else if (KOMWARI_L_45_F_NUM  == DAIKUSUTORA_pass[i]) { KOMWARI_L_45_F(next_Gcv); }
        else if (V_90_R_NUM          == DAIKUSUTORA_pass[i]) { V_90_R(next_Gcv); }
        else if (V_90_L_NUM          == DAIKUSUTORA_pass[i]) { V_90_L(next_Gcv); }
        // ---------------------------------------------------------
        // 3. 斜め直線
        // ---------------------------------------------------------
        else if (-1 >= DAIKUSUTORA_pass[i] && DAIKUSUTORA_pass[i] >= -40) {
           // straight(straight_saitan_Gv, next_Gcv, current_straight_acc/6, -90.0f * 1.414f * DAIKUSUTORA_pass[i]);
            diagonal(straight_saitan_Gv, next_Gcv, current_straight_acc/6, -90.0f * 1.414f * DAIKUSUTORA_pass[i]*1.05);
        }
        // ---------------------------------------------------------
        // 4. スラローム
        // ---------------------------------------------------------
        else if (45 == DAIKUSUTORA_pass[i]) {
            slalom_flag     = (i > 0 && (45 != DAIKUSUTORA_pass[i - 1] || -45 != DAIKUSUTORA_pass[i - 1])) ? 0 : 1;
            slalom_acc_flag = (45 != DAIKUSUTORA_pass[i + 1] || -45 != DAIKUSUTORA_pass[i + 1]) ? 1 : 0;
            slalom_R(saitan_Gv);
        }
        else if (-45 == DAIKUSUTORA_pass[i]) {
            slalom_flag     = (i > 0 && (45 != DAIKUSUTORA_pass[i - 1] || -45 != DAIKUSUTORA_pass[i - 1])) ? 0 : 1;
            slalom_acc_flag = (45 != DAIKUSUTORA_pass[i + 1] || -45 != DAIKUSUTORA_pass[i + 1]) ? 1 : 0;
            slalom_L(saitan_Gv);
        }

        if(1 <= DAIKUSUTORA_pass[i] && DAIKUSUTORA_pass[i] <= 40){//||(-1 >= DAIKUSUTORA_pass[i] && DAIKUSUTORA_pass[i] >= -40)){
        	dtkabegire_flag=1;
        }else{
        	dtkabegire_flag=0;
        }

        // フェイルセーフ検知時
        if (failsafe_flag == 1 || motor_stop_flag == 1) {
            break;
        }
    }

    // ゴール到達時の停止処理
    if (failsafe_flag != 1 && motor_stop_flag != 1) {
        straight_finish(saitan_Gv, 45);
    }
    Gv = 0;
	dtkabegire_flag=0;
}

// -------------------------------------------------------------
// コスト計算 (ダイクストラ法)
// -------------------------------------------------------------
void Calculate_DAIKUSUTORA_math(void) {
    for (int i = 0; i <= 16; i++) {
        for (int j = 0; j <= 16; j++) {
            if (!(wall_log_Y[j] & (1 << i))) wall_Y[j] |= (1 << i);
            if (!(wall_log_X[j] & (1 << i))) wall_X[j] |= (1 << i);
        }
    }

    for (int i = 0; i <= Map_Size; i++) {
        wall_X[i] |= (1 << 0) | (1 << 16);
        wall_Y[i] |= (1 << 0) | (1 << 16);
    }
    wall_Y[0] |= (1 << 1);
    wall_X[0] &= ~(1 << 1);

    intilaize_DAIKUSUTORA();
    intilaize_DAIKUSUTORA_QUEUE();

    saitan_math_goalY = GOAL_Y;
    saitan_math_goalX = GOAL_X;
/*
    printf("GOALCHANGE\n\r");
    ///GOAL座標を入り口の先に置く
    if(!(wall_Y[saitan_math_goalY] & (1 << (saitan_math_goalX+2)))){
    	printf("1\n\r");
        GOAL_num =1;
        saitan_math_goalX = saitan_math_goalX+1;
        saitan_math_goalY = saitan_math_goalY;
        Player_D = 180;
    }else if(!(wall_Y[saitan_math_goalY-1] & (1 << (saitan_math_goalX+2)))){
    	printf("2\n\r");
        GOAL_num =1;
        saitan_math_goalX = saitan_math_goalX+1;
        saitan_math_goalY = saitan_math_goalY-1;
        Player_D = 180;
    }else if(!(wall_Y[saitan_math_goalY] & (1 << (saitan_math_goalX)))){
    	printf("3\n\r");
        GOAL_num =1;
        saitan_math_goalX = saitan_math_goalX+1;
        saitan_math_goalY = saitan_math_goalY;
        Player_D = 0;
    }else if(!(wall_Y[saitan_math_goalY-1] & (1 << (saitan_math_goalX)))){
    	printf("4\n\r");
        GOAL_num =1;
        saitan_math_goalX = saitan_math_goalX+1;
        saitan_math_goalY = saitan_math_goalY-1;
        Player_D = 0;
    }else if(!(wall_X[saitan_math_goalX] & (1 << (saitan_math_goalY-1)))){
    	printf("5\n\r");
        GOAL_num =2;
        saitan_math_goalX = saitan_math_goalX;
        saitan_math_goalY = saitan_math_goalY;
        Player_D = 90;
    }else if(!(wall_X[saitan_math_goalX+1] & (1 << (saitan_math_goalY-1)))){
    	printf("6\n\r");
        GOAL_num =2;
        saitan_math_goalX = saitan_math_goalX+1;
        saitan_math_goalY = saitan_math_goalY;
        Player_D = 90;
    }else if(!(wall_X[saitan_math_goalX] & (1 << (saitan_math_goalY+1)))){
    	printf("7\n\r");
        GOAL_num =2;
        saitan_math_goalX = saitan_math_goalX;
        saitan_math_goalY = saitan_math_goalY;
        Player_D = 270;
    }else if(!(wall_X[saitan_math_goalX+1] & (1 << (saitan_math_goalY+1)))){
    	printf("8\n\r");
        GOAL_num =2;
        saitan_math_goalX = saitan_math_goalX+1;
        saitan_math_goalY = saitan_math_goalY;
        Player_D = 270;
    }
    printf("X:%d,Y:%d\n\r",saitan_math_goalX,saitan_math_goalY);
    Player_X =saitan_math_goalX;
    Player_Y =saitan_math_goalY;*/

    DAIKUSUTORA_num[GOAL_num][saitan_math_goalX][saitan_math_goalY] = 0;
    INPUT_DAIKUSUTORA_QUEUE(GOAL_num, saitan_math_goalX, saitan_math_goalY, 0, 0);

    for (int i = 0; i <= 1024 * 2; i++) {
        OUTPUT_DAIKUSUTORA_QUEUE();

        if (D_pos == 1) { // 縦の境界 (Y)
            // RIGHT
            if (!(wall_Y[D_Y] & (1 << (D_X + 1)))) update_daikusutora_cost(1, D_X + 1, D_Y, 180, D_num, STA_cost, CON_STA_cost);
            if (!(wall_X[D_X] & (1 << (D_Y + 1)))) update_daikusutora_cost(2, D_X, D_Y + 1, 45, D_num, DIA_cost, CON_DIA_cost);
            if (!(wall_X[D_X] & (1 << D_Y)))       update_daikusutora_cost(2, D_X, D_Y, 315, D_num, DIA_cost, CON_DIA_cost);

            // LEFT
            if (!(wall_Y[D_Y] & (1 << (D_X - 1))))   update_daikusutora_cost(1, D_X - 1, D_Y, 0, D_num, STA_cost, CON_STA_cost);
            if (!(wall_X[D_X - 1] & (1 << (D_Y + 1)))) update_daikusutora_cost(2, D_X - 1, D_Y + 1, 135, D_num, DIA_cost, CON_DIA_cost);
            if (!(wall_X[D_X - 1] & (1 << D_Y)))       update_daikusutora_cost(2, D_X - 1, D_Y, 225, D_num, DIA_cost, CON_DIA_cost);

        } else if (D_pos == 2) { // 横の境界 (X)
            // UP
            if (!(wall_X[D_X] & (1 << (D_Y + 1)))) update_daikusutora_cost(2, D_X, D_Y + 1, 90, D_num, STA_cost, CON_STA_cost);
            if (!(wall_Y[D_Y] & (1 << (D_X + 1)))) update_daikusutora_cost(1, D_X + 1, D_Y, 45, D_num, DIA_cost, CON_DIA_cost);
            if (!(wall_Y[D_Y] & (1 << D_X)))       update_daikusutora_cost(1, D_X, D_Y, 135, D_num, DIA_cost, CON_DIA_cost);

            // DOWN
            if (!(wall_X[D_X] & (1 << (D_Y - 1))))   update_daikusutora_cost(2, D_X, D_Y - 1, 270, D_num, STA_cost, CON_STA_cost);
            if (!(wall_Y[D_Y - 1] & (1 << (D_X + 1)))) update_daikusutora_cost(1, D_X + 1, D_Y - 1, 315, D_num, DIA_cost, CON_DIA_cost);
            if (!(wall_Y[D_Y - 1] & (1 << D_X)))       update_daikusutora_cost(1, D_X, D_Y - 1, 225, D_num, DIA_cost, CON_DIA_cost);
        }

        if (tail == head) {
            break;
        }
    }
}

// -------------------------------------------------------------
// 最短パスの抽出
// -------------------------------------------------------------
int DAIKUSUTORA_pass_Comparison(uint8_t COM_pos_STA, uint8_t COM_X_STA, uint8_t COM_Y_STA,
                                uint8_t COM_pos_RIG, uint8_t COM_X_RIG, uint8_t COM_Y_RIG,
                                uint8_t COM_pos_LEFT, uint8_t COM_X_LEFT, uint8_t COM_Y_LEFT) {
    int pass_num = 2;
    if (DAIKUSUTORA_num[COM_pos_STA][COM_X_STA][COM_Y_STA] > DAIKUSUTORA_num[COM_pos_RIG][COM_X_RIG][COM_Y_RIG]) {
        pass_num = 45;
        if (DAIKUSUTORA_num[COM_pos_RIG][COM_X_RIG][COM_Y_RIG] > DAIKUSUTORA_num[COM_pos_LEFT][COM_X_LEFT][COM_Y_LEFT]) {
            pass_num = -45;
        }
    } else if (DAIKUSUTORA_num[COM_pos_STA][COM_X_STA][COM_Y_STA] > DAIKUSUTORA_num[COM_pos_LEFT][COM_X_LEFT][COM_Y_LEFT]) {
        pass_num = -45;
    }
    return pass_num;
}

void Calculate_DAIKUSUTORA_pass(void) {
    DAIKUSUTORA_pass_count = 0;
    uint8_t D_Pass_X = 0, D_Pass_Y = 1;
    int D_Pass_dir = 90;
    DAIKUSUTORA_pass[0] = 2;

    for (int i = 1; i <= 256; i++) {
        if (DAIKUSUTORA_num[2][0][1] == 9999) {
            break;
        }

        if (D_Pass_dir == 0) {
            DAIKUSUTORA_pass[i] = DAIKUSUTORA_pass_Comparison(1, D_Pass_X + 1, D_Pass_Y,
                                                              2, D_Pass_X,     D_Pass_Y,
                                                              2, D_Pass_X,     D_Pass_Y + 1);
        } else if (D_Pass_dir == 180) {
            DAIKUSUTORA_pass[i] = DAIKUSUTORA_pass_Comparison(1, D_Pass_X - 1, D_Pass_Y,
                                                              2, D_Pass_X - 1, D_Pass_Y + 1,
                                                              2, D_Pass_X - 1, D_Pass_Y);
        } else if (D_Pass_dir == 90) {
            DAIKUSUTORA_pass[i] = DAIKUSUTORA_pass_Comparison(2, D_Pass_X,     D_Pass_Y + 1,
                                                              1, D_Pass_X + 1, D_Pass_Y,
                                                              1, D_Pass_X,     D_Pass_Y);
        } else if (D_Pass_dir == 270) {
            if (D_Pass_Y != 0) {
                DAIKUSUTORA_pass[i] = DAIKUSUTORA_pass_Comparison(2, D_Pass_X,     D_Pass_Y - 1,
                                                                  1, D_Pass_X,     D_Pass_Y - 1,
                                                                  1, D_Pass_X + 1, D_Pass_Y - 1);
            } else {
                DAIKUSUTORA_pass[i] = DAIKUSUTORA_pass_Comparison(2, 1,            0,
                                                                  1, D_Pass_X,     D_Pass_Y - 1,
                                                                  1, D_Pass_X + 1, D_Pass_Y - 1);
            }
        }

        // 座標および進行方向の更新
        if (DAIKUSUTORA_pass[i] == 2) {
            if (D_Pass_dir == 90)       D_Pass_Y++;
            else if (D_Pass_dir == 270) D_Pass_Y--;
            else if (D_Pass_dir == 0)   D_Pass_X++;
            else if (D_Pass_dir == 180) D_Pass_X--;
        } else if (DAIKUSUTORA_pass[i] == 45) {
            if (D_Pass_dir == 90)       { D_Pass_X++; D_Pass_dir = 0; }
            else if (D_Pass_dir == 270) { D_Pass_Y--; D_Pass_dir = 180; }
            else if (D_Pass_dir == 0)   { D_Pass_dir = 270; }
            else if (D_Pass_dir == 180) { D_Pass_X--; D_Pass_Y++; D_Pass_dir = 90; }
        } else if (DAIKUSUTORA_pass[i] == -45) {
            if (D_Pass_dir == 90)       { D_Pass_dir = 180; }
            else if (D_Pass_dir == 270) { D_Pass_X++; D_Pass_Y--; D_Pass_dir = 0; }
            else if (D_Pass_dir == 0)   { D_Pass_Y++; D_Pass_dir = 90; }
            else if (D_Pass_dir == 180) { D_Pass_X--; D_Pass_dir = 270; }
        }

        DAIKUSUTORA_pass_count++;
        saitan_math_dir = D_Pass_dir;

        if (GOAL_num == 2 && D_Pass_X == saitan_math_goalX && D_Pass_Y == saitan_math_goalY) {
            DAIKUSUTORA_pass_count++;
            DAIKUSUTORA_pass[DAIKUSUTORA_pass_count] = 1;
            if (D_Pass_dir == 270) saitan_math_goalY--;
            break;
        }
        if (GOAL_num == 1 && D_Pass_X == saitan_math_goalX && D_Pass_Y == saitan_math_goalY) {
            DAIKUSUTORA_pass_count++;
            DAIKUSUTORA_pass[DAIKUSUTORA_pass_count] = 1;
            if (D_Pass_dir == 180) saitan_math_goalX--;
            break;
        }
    }
}

// -------------------------------------------------------------
// 斜め走行・大回り・小回りへのパス圧縮
// -------------------------------------------------------------
void DAIKUSUTORA_pass_compression_diagoal(void) {
    // 1. 直線統合 (2 と 1 の連続をまとめる)
    for (int i = 0; i <= DAIKUSUTORA_pass_count; i++) {
        if (DAIKUSUTORA_pass[i] == 2) {
            for (int j = i + 1; j <= DAIKUSUTORA_pass_count; j++) {
                if (DAIKUSUTORA_pass[j] == 2 || DAIKUSUTORA_pass[j] == 1) {
                    DAIKUSUTORA_pass[i] += DAIKUSUTORA_pass[j];
                    DAIKUSUTORA_pass[j] = 0;
                } else {
                    break;
                }
            }
        }
    }
    remove_zero_pass();

    // 2. 大回り (90度 / 180度) 変換
    for (int i = 0; i <= DAIKUSUTORA_pass_count - 2; i++) {
        if (1 <= DAIKUSUTORA_pass[i] && DAIKUSUTORA_pass[i] <= 40) {
            if (DAIKUSUTORA_pass[i + 1] == 45 && DAIKUSUTORA_pass[i + 2] == 45 && 1 <= DAIKUSUTORA_pass[i + 3] && DAIKUSUTORA_pass[i + 3] <= 40) {
                DAIKUSUTORA_pass[i]--;
                DAIKUSUTORA_pass[i + 1] = 0;
                DAIKUSUTORA_pass[i + 2] = OOMWARI_R_180_NUM;
                DAIKUSUTORA_pass[i + 3]--;
            } else if (DAIKUSUTORA_pass[i + 1] == -45 && DAIKUSUTORA_pass[i + 2] == -45 && 1 <= DAIKUSUTORA_pass[i + 3] && DAIKUSUTORA_pass[i + 3] <= 40) {
                DAIKUSUTORA_pass[i]--;
                DAIKUSUTORA_pass[i + 1] = OOMWARI_L_180_NUM;
                DAIKUSUTORA_pass[i + 2] = 0;
                DAIKUSUTORA_pass[i + 3]--;
            } else if (DAIKUSUTORA_pass[i + 1] == 45 && 1 <= DAIKUSUTORA_pass[i + 2] && DAIKUSUTORA_pass[i + 2] <= 40) {
                DAIKUSUTORA_pass[i]--;
                DAIKUSUTORA_pass[i + 1] = OOMWARI_R_90_NUM;
                DAIKUSUTORA_pass[i + 2]--;
            } else if (DAIKUSUTORA_pass[i + 1] == -45 && 1 <= DAIKUSUTORA_pass[i + 2] && DAIKUSUTORA_pass[i + 2] <= 40) {
                DAIKUSUTORA_pass[i]--;
                DAIKUSUTORA_pass[i + 1] = OOMWARI_L_90_NUM;
                DAIKUSUTORA_pass[i + 2]--;
            }
        }
    }
    remove_zero_pass();

    for (int i = 0; i <= DAIKUSUTORA_pass_count; i++) {
        DAIKUSUTORA_pass_OOMAWARI[i] = DAIKUSUTORA_pass[i];
    }

    // 3. 小回り Start 変換 (135度 / 45度)
    for (int i = 0; i <= DAIKUSUTORA_pass_count - 1; i++) {
        if (1 <= DAIKUSUTORA_pass[i] && DAIKUSUTORA_pass[i] <= 40) {
            if (DAIKUSUTORA_pass[i + 1] == 45 && DAIKUSUTORA_pass[i + 2] == 45) {
                DAIKUSUTORA_pass[i]--;
                DAIKUSUTORA_pass[i + 1] = KOMWARI_R_135_S_NUM;
                DAIKUSUTORA_pass[i + 2] = 0;
            } else if (DAIKUSUTORA_pass[i + 1] == -45 && DAIKUSUTORA_pass[i + 2] == -45) {
                DAIKUSUTORA_pass[i]--;
                DAIKUSUTORA_pass[i + 1] = KOMWARI_L_135_S_NUM;
                DAIKUSUTORA_pass[i + 2] = 0;
            } else if (DAIKUSUTORA_pass[i + 1] == 45) {
                DAIKUSUTORA_pass[i]--;
                DAIKUSUTORA_pass[i + 1] = KOMWARI_R_45_S_NUM;
            } else if (DAIKUSUTORA_pass[i + 1] == -45) {
                DAIKUSUTORA_pass[i]--;
                DAIKUSUTORA_pass[i + 1] = KOMWARI_L_45_S_NUM;
            }
        }
    }

    // 4. 小回り Finish 変換 (135度 / 45度)
    for (int i = 0; i <= DAIKUSUTORA_pass_count - 1; i++) {
        if (DAIKUSUTORA_pass[i] == 45 && 1 <= DAIKUSUTORA_pass[i + 1] && DAIKUSUTORA_pass[i + 1] <= 40) {
            DAIKUSUTORA_pass[i] = KOMWARI_R_45_F_NUM;
            DAIKUSUTORA_pass[i + 1]--;
        } else if (DAIKUSUTORA_pass[i] == -45 && 1 <= DAIKUSUTORA_pass[i + 1] && DAIKUSUTORA_pass[i + 1] <= 40) {
            DAIKUSUTORA_pass[i] = KOMWARI_L_45_F_NUM;
            DAIKUSUTORA_pass[i + 1]--;
        }

        if (DAIKUSUTORA_pass[i] == 45 && DAIKUSUTORA_pass[i + 1] == 45 && 1 <= DAIKUSUTORA_pass[i + 2] && DAIKUSUTORA_pass[i + 2] <= 40) {
            DAIKUSUTORA_pass[i] = KOMWARI_R_135_F_NUM;
            DAIKUSUTORA_pass[i + 1] = 0;
            DAIKUSUTORA_pass[i + 2]--;
        } else if (DAIKUSUTORA_pass[i] == -45 && DAIKUSUTORA_pass[i + 1] == -45 && 1 <= DAIKUSUTORA_pass[i + 2] && DAIKUSUTORA_pass[i + 2] <= 40) {
            DAIKUSUTORA_pass[i] = KOMWARI_L_135_F_NUM;
            DAIKUSUTORA_pass[i + 1] = 0;
            DAIKUSUTORA_pass[i + 2]--;
        }
    }
    remove_zero_pass();

    // 5. V90 ターン変換
    for (int i = 0; i <= DAIKUSUTORA_pass_count - 1; i++) {
        if (DAIKUSUTORA_pass[i] == 45 && DAIKUSUTORA_pass[i + 1] == 45) {
            DAIKUSUTORA_pass[i]     = 0;
            DAIKUSUTORA_pass[i + 1] = V_90_R_NUM;
        } else if (DAIKUSUTORA_pass[i] == -45 && DAIKUSUTORA_pass[i + 1] == -45) {
            DAIKUSUTORA_pass[i]     = 0;
            DAIKUSUTORA_pass[i + 1] = V_90_L_NUM;
        }
    }
    remove_zero_pass();

    // 6. 斜め直線統合
    for (int i = 0; i <= DAIKUSUTORA_pass_count - 1; i++) {
        if (DAIKUSUTORA_pass[i] == 45 || DAIKUSUTORA_pass[i] == -45) {
            DAIKUSUTORA_pass[i] = diagonal_NUM;
        }
    }

    for (int i = 0; i <= DAIKUSUTORA_pass_count; i++) {
        if (DAIKUSUTORA_pass[i] == diagonal_NUM) {
            for (int j = i + 1; j <= DAIKUSUTORA_pass_count; j++) {
                if (DAIKUSUTORA_pass[j] == diagonal_NUM) {
                    DAIKUSUTORA_pass[i] += diagonal_NUM;
                    DAIKUSUTORA_pass[j] = 0;
                } else {
                    break;
                }
            }
        }
    }
    remove_zero_pass();
}

// -------------------------------------------------------------
// デバッグ出力
// -------------------------------------------------------------
void print_DAIKUSUTORAmap(void) {
    for (int i = 0; i < Map_Size; i++) {
        printf((i >= 10) ? " %d " : "  %d ", i);
    }
    printf("\n\r+");
    for (int j = 0; j < Map_Size; j++) {
        printf((wall_X[j] & (1 << Map_Size)) ? "---+" : "   +");
    }
    printf("\n\r");

    for (int i = Map_Size - 1; i >= 0; i--) {
        for (int j = 0; j < Map_Size; j++) {
            if (wall_Y[i] & (1 << j)) {
                printf((j == 0) ? "| " : " | ");
            } else if ((Player_D == 0 || Player_D == 180) && Player_X == j && Player_Y == i) {
                printf(" @  ");
            } else {
                if (DAIKUSUTORA_num[1][j][i] == 9999)      printf("@99 ");
                else if (DAIKUSUTORA_num[1][j][i] >= 100)  printf("%d ", DAIKUSUTORA_num[1][j][i]);
                else if (DAIKUSUTORA_num[1][j][i] >= 10)   printf(" %d ", DAIKUSUTORA_num[1][j][i]);
                else                                       printf(" %d  ", DAIKUSUTORA_num[1][j][i]);
            }
        }

        printf((wall_Y[i] & (1 << Map_Size)) ? " |" : "%d", DAIKUSUTORA_num[1][Map_Size][i]);
        printf("%2d\n\r+", i);

        for (int j = 0; j < Map_Size; j++) {
            if (wall_X[j] & (1 << i)) {
                printf("---+");
            } else if ((Player_D == 90 || Player_D == 270) && Player_X == j && Player_Y == i) {
                printf(" @ +");
            } else {
                if (DAIKUSUTORA_num[2][j][i] == 9999)      printf("@99+");
                else if (DAIKUSUTORA_num[2][j][i] >= 100)  printf("%d+", DAIKUSUTORA_num[2][j][i]);
                else if (DAIKUSUTORA_num[2][j][i] >= 10)   printf("%d +", DAIKUSUTORA_num[2][j][i]);
                else                                       printf(" %d +", DAIKUSUTORA_num[2][j][i]);
            }
        }
        printf("\n\r");
    }
}

void scanf_adati_pass(int pass_data, int num_elements) {
    DAIKUSUTORA_pass[num_elements] = pass_data;
    DAIKUSUTORA_pass_count = num_elements + 1;
}

void scanf_DAIKUSUTORA_pass(int* pass_data, int num_elements) {
    for (int i = 0; i < num_elements; i++) {
        DAIKUSUTORA_pass[i] = pass_data[i];
    }
    for (int i = num_elements; i < 20; i++) {
        DAIKUSUTORA_pass[i] = 0;
    }
    DAIKUSUTORA_pass_count = num_elements;
}

// -------------------------------------------------------------
// パス番号を文字列に変換して返すヘルパー関数
// -------------------------------------------------------------
static const char* get_pass_name(int pass_code) {
    // 直線区間 (1〜40)
    if (1 <= pass_code && pass_code <= 40) {
        return "Straight";
    }
    // 斜め直線区間 (-1〜-40)
    if (-40 <= pass_code && pass_code <= -1) {
        return "Diagonal";
    }

    // ターン系マクロ判定
    if (pass_code == OOMWARI_R_90_NUM)    return "OOMAWARI_R_90";
    if (pass_code == OOMWARI_L_90_NUM)    return "OOMAWARI_L_90";
    if (pass_code == OOMWARI_R_180_NUM)   return "OOMAWARI_R_180";
    if (pass_code == OOMWARI_L_180_NUM)   return "OOMAWARI_L_180";
    if (pass_code == KOMWARI_R_135_S_NUM) return "KOMWARI_R_135_Start";
    if (pass_code == KOMWARI_L_135_S_NUM) return "KOMWARI_L_135_Start";
    if (pass_code == KOMWARI_R_45_S_NUM)  return "KOMWARI_R_45_Start";
    if (pass_code == KOMWARI_L_45_S_NUM)  return "KOMWARI_L_45_Start";
    if (pass_code == KOMWARI_R_135_F_NUM) return "KOMWARI_R_135_Finish";
    if (pass_code == KOMWARI_L_135_F_NUM) return "KOMWARI_L_135_Finish";
    if (pass_code == KOMWARI_R_45_F_NUM)  return "KOMWARI_R_45_Finish";
    if (pass_code == KOMWARI_L_45_F_NUM)  return "KOMWARI_L_45_Finish";
    if (pass_code == V_90_R_NUM)          return "V_90_R";
    if (pass_code == V_90_L_NUM)          return "V_90_L";
    if (pass_code == 45)                  return "Slalom_R (45deg)";
    if (pass_code == -45)                 return "Slalom_L (-45deg)";

    return "Unknown";
}

// -------------------------------------------------------------
// パス出力関数 (詳細リスト ＆ C言語配列コピー用コード)
// -------------------------------------------------------------
void print_DAIKUSUTORA_pass(void) {
    printf("\n\r========== Shortest Path (Total: %d steps) ==========\n\r", DAIKUSUTORA_pass_count + 1);
    printf(" Index | Value | Action\n\r");
    printf("-------+-------+---------------------------\n\r");

    for (int i = 0; i <= DAIKUSUTORA_pass_count; i++) {
        int code = DAIKUSUTORA_pass[i];

        // 直線・斜め直線の場合はブロック数も合わせて表示
        if (1 <= code && code <= 40) {
            printf("  %3d  |  %4d | %s (%d blocks)\n\r", i, code, get_pass_name(code), code);
        } else if (-40 <= code && code <= -1) {
            printf("  %3d  |  %4d | %s (%d sections)\n\r", i, code, get_pass_name(code), -code);
        } else {
            printf("  %3d  |  %4d | %s\n\r", i, code, get_pass_name(code));
        }
    }
    printf("======================================================\n\r");

    // デバッグテーブル（PATH_TABLE等）にそのままコピペできるC言語配列形式でも出力
    printf("const int test_path[] = {");
    for (int i = 0; i <= DAIKUSUTORA_pass_count; i++) {
        printf("%d%s", DAIKUSUTORA_pass[i], (i == DAIKUSUTORA_pass_count) ? "" : ", ");
    }
    printf("};\n\r\n\r");
}
