#ifndef __SPEAKER_H
#define __SPEAKER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "main.h"
#include "tim.h"

// 音階の周波数定義 (Hz)
#define NOTE_C4   262
#define NOTE_CS4  277
#define NOTE_D4   294
#define NOTE_DS4  311
#define NOTE_E4   330
#define NOTE_F4   349
#define NOTE_FS4  370
#define NOTE_G4   392
#define NOTE_GS4  415
#define NOTE_A4   440
#define NOTE_AS4  466
#define NOTE_B4   494
#define NOTE_C5   523
#define NOTE_D5   587
#define NOTE_E5   659
#define NOTE_G5   784
#define NOTE_A5   880   // ← 追加
#define NOTE_B5   988   // ← 追加
#define NOTE_C6   1047

// 効果音IDの列挙型
typedef enum {
    SE_NONE = 0,
    // 壁検知パターン
    SE_WALL_NONE,       // 壁なし（超短ピッ / 低音）
    SE_WALL_SIDE,       // 左右壁（通常の通路音）
    SE_WALL_FRONT,      // 前壁あり（高めの単音）
    SE_WALL_DEADEND,    // 袋小路（3面壁：下がる2音）
    SE_WALL_CUT,        // 壁切れ音
    // システム系
    SE_BUTTON_CLICK,    // ボタン押下音
    SE_MODE_CHANGE,     // モード切替音
    SE_SUCCESS,         // ゴール・探索完了
    SE_ERROR,           // フェイルセーフ・警告音
    SE_BOOT,             // 起動音
	SE_MAEKABE_BREAK // ← 追加
} SoundID_t;

// 関数プロトタイプ宣言
void SPEAKER_Init(void);
void SPEAKER_PlaySE(SoundID_t se_id);
void SPEAKER_Update_1ms(void);
void SPEAKER_Stop(void);

#ifdef __cplusplus
}
#endif

#endif /* __SPEAKER_H */
