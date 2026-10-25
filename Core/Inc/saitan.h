/*
 * saitan.h
 *
 *  Created on: Dec 14, 2024
 *      Author: suzuk
 */

#ifndef INC_SAITAN_H_
#define INC_SAITAN_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdio.h>

// -------------------------------------------------------------
// ターン・パス用のマクロ番号定義
// -------------------------------------------------------------
#define OOMWARI_R_90_NUM     55
#define OOMWARI_L_90_NUM    -55
#define OOMWARI_R_180_NUM    58
#define OOMWARI_L_180_NUM   -58

#define KOMWARI_R_135_S_NUM  65
#define KOMWARI_L_135_S_NUM -65
#define KOMWARI_R_45_S_NUM   75
#define KOMWARI_L_45_S_NUM  -75

#define KOMWARI_R_135_F_NUM  85
#define KOMWARI_L_135_F_NUM -85
#define KOMWARI_R_45_F_NUM   95
#define KOMWARI_L_45_F_NUM  -95

#define V_90_R_NUM           90
#define V_90_L_NUM          -90

#define diagonal_NUM        -1

#define Map_Size            16
#define adati_cost          1

#define QUEUE_MAX_SIZE      512

// -------------------------------------------------------------
// ダイクストラ探索用 キューノード構造体（単一構造体として定義）
// -------------------------------------------------------------
typedef struct {
    uint8_t pos;      // 1: 縦の壁境界(Y), 2: 横の壁境界(X)
    uint8_t X;        // マップX座標
    uint8_t Y;        // マップY座標
    int dir;          // 進入方向 (0, 90, 180, 270 など)
    uint8_t con_num;  // 連続直進カウント
} DaikusutoraQueue_t;

// -------------------------------------------------------------
// 外部公開変数 (extern)
// -------------------------------------------------------------
extern DaikusutoraQueue_t queue[QUEUE_MAX_SIZE];
extern uint16_t DAIKUSUTORA_pass_count;
extern int DAIKUSUTORA_pass[258];
extern int DAIKUSUTORA_pass_OOMAWARI[258];
extern uint16_t DAIKUSUTORA_num[3][Map_Size + 1][Map_Size + 1];

extern uint8_t slalom_acc_flag;
extern int NEXT_DAIKUSUTORA_pass;

// -------------------------------------------------------------
// 関数プロトタイプ宣言
// -------------------------------------------------------------
void move_saitan(int mode);
void move_saitan_OOMAWARI(void);
void move_saitan_change_acc_dec(int mode);
void DAIKUSUTORA_move(void);
void print_DAIKUSUTORAmap(void);

void Calculate_DAIKUSUTORA_math(void);
void Calculate_DAIKUSUTORA_pass(void);
void Calculate_DAIKUSUTORA_pass_move(uint8_t pos, uint8_t x, uint8_t y);
void DAIKUSUTORA_pass_compression(void);
void DAIKUSUTORA_pass_compression_diagoal(void);

int  DAIKUSUTORA_pass_Comparison(uint8_t COM_pos_STA, uint8_t COM_X_STA, uint8_t COM_Y_STA,
                                 uint8_t COM_pos_RIG, uint8_t COM_X_RIG, uint8_t COM_Y_RIG,
                                 uint8_t COM_pos_LEFT, uint8_t COM_X_LEFT, uint8_t COM_Y_LEFT);

void intilaize_DAIKUSUTORA(void);
void intilaize_DAIKUSUTORA_QUEUE(void);
void INPUT_DAIKUSUTORA_QUEUE(uint8_t pos, uint8_t X, uint8_t Y, int dir, uint8_t con_num);
void OUTPUT_DAIKUSUTORA_QUEUE(void);

void scanf_adati_pass(int pass_data, int num_elements);
void scanf_DAIKUSUTORA_pass(int* pass_data, int num_elements);
void print_DAIKUSUTORA_pass(void);
#ifdef __cplusplus
}
#endif

#endif /* INC_SAITAN_H_ */
