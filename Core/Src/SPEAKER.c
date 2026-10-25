#include "SPEAKER.h"
#include "tim.h"

#define TIMER_CLK 11250000  // APB1 Timer 90MHz / 8分周 = 11.25MHz
extern TIM_HandleTypeDef htim2;

// 1つの音符・休符を表す構造体
typedef struct {
    uint16_t freq;      // 周波数 (Hz) / 0 は休符（無音）
    uint16_t duration;  // 長さ (ms)
} Tone_t;

// ------------------------------------------------------------------
// 各効果音の音符パターン定義（末尾は {0, 0} で終端）
// ------------------------------------------------------------------

// 1. 壁なし / 柱のみ（低く短い音 25ms）
static const Tone_t SE_DATA_WALL_NONE[] = {
    {NOTE_C4, 25},
    {0, 0}
};

// 2. 左右壁のみ（通常の走行音 30ms）
static const Tone_t SE_DATA_WALL_SIDE[] = {
    {NOTE_E4, 30},
    {0, 0}
};

// 3. 前壁あり（高音 40ms）
static const Tone_t SE_DATA_WALL_FRONT[] = {
    {NOTE_G4, 40},
    {0, 0}
};

// 4. 袋小路 / 3面壁（高→低の2音）
static const Tone_t SE_DATA_WALL_DEADEND[] = {
    {NOTE_G4, 30},
    {0, 10},
    {NOTE_C4, 50},
    {0, 0}
};

// 5. 壁切れ音（40ms）
static const Tone_t SE_DATA_WALL_CUT[] = {
    {NOTE_B4, 40},
    {0, 0}
};

// 6. 前壁抜け音（ピロッ！ / 15ms + 25ms の急上昇音）
static const Tone_t SE_DATA_MAEKABE_BREAK[] = {
    {NOTE_E5, 15},
    {NOTE_B5, 25},
    {0, 0}
};

// 7. ボタン押下音（30ms）
static const Tone_t SE_DATA_BUTTON[] = {
    {NOTE_C5, 30},
    {0, 0}
};

// 8. モード切替音（ピピッ）
static const Tone_t SE_DATA_MODE_CHANGE[] = {
    {NOTE_E5, 40},
    {0, 20},
    {NOTE_E5, 40},
    {0, 0}
};

// 9. ゴール・成功音（上昇アルペジオ）
static const Tone_t SE_DATA_SUCCESS[] = {
    {NOTE_C5, 60},
    {NOTE_E5, 60},
    {NOTE_G5, 60},
    {NOTE_C6, 120},
    {0, 0}
};

// 10. エラー・フェイルセーフ音（低い警告音）
static const Tone_t SE_DATA_ERROR[] = {
    {150, 100},
    {0, 50},
    {150, 100},
    {0, 0}
};

// 11. 起動音（ドミソド）
static const Tone_t SE_DATA_BOOT[] = {
    {NOTE_C4, 80},
    {NOTE_E4, 80},
    {NOTE_G4, 80},
    {NOTE_C5, 150},
    {0, 0}
};

// ------------------------------------------------------------------
// 再生制御ステート
// ------------------------------------------------------------------
static const Tone_t *current_track = NULL;
static volatile uint16_t note_index = 0;
static volatile uint16_t duration_counter = 0;
static volatile uint8_t is_playing = 0;

static void Set_Hardware_Tone(uint16_t freq) {
    if (freq == 0) {
        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);
        return;
    }

    uint32_t arr = (TIMER_CLK / freq) - 1;

    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);
    __HAL_TIM_SET_AUTORELOAD(&htim2, arr);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, arr / 2);
    HAL_TIM_GenerateEvent(&htim2, TIM_EVENTSOURCE_UPDATE);
    HAL_TIM_Base_Start(&htim2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
}

void SPEAKER_Init(void) {
    SPEAKER_Stop();
}

void SPEAKER_Stop(void) {
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);
    is_playing = 0;
    current_track = NULL;
    note_index = 0;
    duration_counter = 0;
}

// 効果音再生リクエスト
void SPEAKER_PlaySE(SoundID_t se_id) {
    switch (se_id) {
        case SE_WALL_NONE:      current_track = SE_DATA_WALL_NONE; break;
        case SE_WALL_SIDE:      current_track = SE_DATA_WALL_SIDE; break;
        case SE_WALL_FRONT:     current_track = SE_DATA_WALL_FRONT; break;
        case SE_WALL_DEADEND:   current_track = SE_DATA_WALL_DEADEND; break;
        case SE_WALL_CUT:       current_track = SE_DATA_WALL_CUT; break;
        case SE_MAEKABE_BREAK:  current_track = SE_DATA_MAEKABE_BREAK; break; // ← 追加
        case SE_BUTTON_CLICK:   current_track = SE_DATA_BUTTON; break;
        case SE_MODE_CHANGE:    current_track = SE_DATA_MODE_CHANGE; break;
        case SE_SUCCESS:        current_track = SE_DATA_SUCCESS; break;
        case SE_ERROR:          current_track = SE_DATA_ERROR; break;
        case SE_BOOT:           current_track = SE_DATA_BOOT; break;
        default:
            SPEAKER_Stop();
            return;
    }

    note_index = 0;
    duration_counter = 0;
    is_playing = 1;

    // 最初の音を鳴らす
    Set_Hardware_Tone(current_track[0].freq);
}

// TIM6 割り込みハンドラ（1ms周期）から毎周期呼び出すタスク
void SPEAKER_Update_1ms(void) {
    if (!is_playing || current_track == NULL) {
        return;
    }

    duration_counter++;

    // 現在の音符の持続時間を満たしたか判定
    if (duration_counter >= current_track[note_index].duration) {
        duration_counter = 0;
        note_index++;

        // 終端データ {0, 0} に到達したか判定
        if (current_track[note_index].duration == 0) {
            SPEAKER_Stop();
        } else {
            Set_Hardware_Tone(current_track[note_index].freq);
        }
    }
}
