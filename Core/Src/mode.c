/*
 * mode.c
 *
 *  Created on: Dec 14, 2024
 *      Author: suzuk
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "mode.h"
#include "MOTOR.h"
#include "PL_encoder.h"
#include "PID.h"
#include "main.h"
#include "wall.h"
#include "PL_timer.h"
#include "lsm6dsr.h"
#include "PL_sensor.h"
#include "led_control.h"
#include "adati.h"
#include "move.h"
#include "define.h"
#include "log.h"
#include "saitan.h"
#include "SPEAKER.h"

// マクロ・定数定義
#define MODE_MAX            7
#define SUB_MODE_MAX        3
#define THRESHOLD_COUNT     200
#define DELAY_MS            1000
#define REVERSE_THRESHOLD   -600
#define MAIN_MODE_STEP      300
#define SUB_MODE_STEP       300
#define MAX_MAIN_MODE       20
#define MAX_SUB_MODE        20
#define ENCODER_MAX         360.0f
// -------------------------------------------------------------
// モード決定用センサしきい値定義
// -------------------------------------------------------------
#define HAND_DETECT_SINGLE_TH   70    // 各センサ（左・右）の最低反応しきい値
#define HAND_DETECT_SUM_TH      100   // 両手センサの合計しきい値 (例: 40 + 40 より高めに設定)

int sub_mode_num;
Mode currentMode;

int mode_encoder_count_L = 0;
int mode_encoder_count_R = 0;
int total_negative_count_R = 0;
int current_encoder_R = 0;

int total_negative_count_L = 0;
int current_encoder_L = 0;
uint8_t mode_encoder_R_flag = 0;

int pre_encoder_R = 0;
int pre_encoder_L = 0;
uint8_t print_flag = 0;

// -------------------------------------------------------------
// デバッグ・テスト用パスデータ
// -------------------------------------------------------------
typedef struct {
    int *data;
    int size;
} PathEntry_t;

int path_data_1[]  = {1, 55, -55, -65, 85};
int path_data_2[]  ={5, 65, -90, 85, 2, 55, 2};/* {3, KOMWARI_L_135_S_NUM,KOMWARI_R_45_F_NUM};*/
int path_data_3[]  = { 3, OOMWARI_L_90_NUM, 1 };
int path_data_4[]  = { 3, KOMWARI_L_45_S_NUM,KOMWARI_R_135_F_NUM, OOMWARI_R_90_NUM,1};
int path_data_5[]  = { 3, OOMWARI_L_180_NUM, OOMWARI_R_90_NUM,1 };
int path_data_6[]  = { 3, KOMWARI_L_135_S_NUM,-1};// -2, KOMWARI_R_45_F_NUM };
int path_data_7[]  = { 1, KOMWARI_L_45_F_NUM, 1};//-90, KOMWARI_R_45_F_NUM, 1 };
int path_data_8[]  = { 3, KOMWARI_L_45_S_NUM,-1, -90};// KOMWARI_R_135_F_NUM, 1 };
int path_data_9[]  = { 3, KOMWARI_L_135_S_NUM,-2,90};
int path_data_10[] = { 3, KOMWARI_L_135_S_NUM,KOMWARI_R_135_F_NUM, OOMWARI_R_90_NUM,1 };
/*int path_data_6[]  = { 3, KOMWARI_R_135_S_NUM,-1};// -2, KOMWARI_R_45_F_NUM };
int path_data_7[]  = { 3, KOMWARI_R_135_S_NUM, -90, KOMWARI_R_45_F_NUM, 1 };
int path_data_8[]  = { 3, KOMWARI_L_45_S_NUM, 90, KOMWARI_R_135_F_NUM, 1 };
int path_data_9[]  = { -3, 90, -1 };
int path_data_10[] = { -2, -90, -3 };*/

static const PathEntry_t PATH_TABLE[] = {
    { NULL, 0 }, // 1-indexed用ダミー
    { path_data_1,  sizeof(path_data_1)  / sizeof(path_data_1[0]) },
    { path_data_2,  sizeof(path_data_2)  / sizeof(path_data_2[0]) },
    { path_data_3,  sizeof(path_data_3)  / sizeof(path_data_3[0]) },
    { path_data_4,  sizeof(path_data_4)  / sizeof(path_data_4[0]) },
    { path_data_5,  sizeof(path_data_5)  / sizeof(path_data_5[0]) },
    { path_data_6,  sizeof(path_data_6)  / sizeof(path_data_6[0]) },
    { path_data_7,  sizeof(path_data_7)  / sizeof(path_data_7[0]) },
    { path_data_8,  sizeof(path_data_8)  / sizeof(path_data_8[0]) },
    { path_data_9,  sizeof(path_data_9)  / sizeof(path_data_9[0]) },
    { path_data_10, sizeof(path_data_10) / sizeof(path_data_10[0]) },
};
#define PATH_TABLE_SIZE (sizeof(PATH_TABLE) / sizeof(PATH_TABLE[0]))

// -------------------------------------------------------------
// エンコーダ差分計算
// -------------------------------------------------------------
float get_encoder_delta(float current, float previous) {
    float delta = current - previous;
    if (delta > 180.0f) {
        delta -= ENCODER_MAX;
    } else if (delta < -180.0f) {
        delta += ENCODER_MAX;
    }
    return delta;
}

// -------------------------------------------------------------
// モード選択UIタスク
// -------------------------------------------------------------
void ModeSelection(void) {
    v_end_flag = 1;
    omega_end_flag = 1;
    motor_stop_flag = 1;
    wall_kabegire_ALL_RESET();

    float delta_R = get_encoder_delta(encoder_R, pre_encoder_R);
    float delta_L = get_encoder_delta(encoder_L, pre_encoder_L);

    pre_encoder_R = encoder_R;
    pre_encoder_L = encoder_L;

    // 右タイヤ（決定・サブモード）
    if (delta_R > 0) {
        total_negative_count_R -= delta_R;
    } else {
        mode_encoder_count_R -= delta_R;
    }

    // 左タイヤ（メインモード）
    if (mode_encoder_R_flag == 0) {
        mode_encoder_count_L += delta_L;
    } else {
        if (delta_L < 0) total_negative_count_L += delta_L;
    }

    if (total_negative_count_L <= REVERSE_THRESHOLD) {
        RESET_mode();
    }

    UpdateLEDs(currentMode.main_mode, currentMode.sub_mode);
    // センサ値の合計を計算
    uint16_t hand_sensor_sum = g_sensor[1][0] + g_sensor[2][0];

    // 両センサが反応しており、かつ合計値がしきい値を超えているかで判定
    if ((g_sensor[1][0] >= HAND_DETECT_SINGLE_TH) &&
        (g_sensor[2][0] >= HAND_DETECT_SINGLE_TH) &&
        (hand_sensor_sum >= HAND_DETECT_SUM_TH))
    {
        // 決定音を鳴らす（以前実装したサウンド関数）
        SPEAKER_PlaySE(SE_BUTTON_CLICK);

        ExecuteMode(currentMode);
        RESET_mode();
        HAL_Delay(2000);
    }

    currentMode.main_mode = mode_encoder_count_L / MAIN_MODE_STEP;
    if (currentMode.main_mode > MAX_MAIN_MODE) currentMode.main_mode = MAX_MAIN_MODE;

    if (mode_encoder_R_flag == 0 && mode_encoder_count_R >= SUB_MODE_STEP) {
        mode_encoder_R_flag = 1;
    }

    currentMode.sub_mode = mode_encoder_count_R / SUB_MODE_STEP;
    if (currentMode.sub_mode > MAX_SUB_MODE) currentMode.sub_mode = MAX_SUB_MODE;

    UpdateLEDs(currentMode.main_mode, currentMode.sub_mode);
}

void UpdateLEDs(int mainMode, int subMode) {
    int displayMainMode = (mainMode >= 0) ? mainMode : -mainMode;
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, (displayMainMode & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, (displayMainMode & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, (displayMainMode & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED9_GPIO_Port, LED9_Pin, (mainMode < 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    int displaySubMode = (subMode >= 0) ? subMode : -subMode;
    HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, (displaySubMode & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, (displaySubMode & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, (displaySubMode & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

// -------------------------------------------------------------
// 最短走行シーケンス実行ヘルパー（重複コードの集約）
// -------------------------------------------------------------
static void run_saitan_sequence(int speed_mode, uint8_t fan_mode, uint8_t need_calc) {
    if (need_calc) {
        Calculate_DAIKUSUTORA_math();
        Calculate_DAIKUSUTORA_pass();
        DAIKUSUTORA_pass_compression_diagoal();
    }

    prepare_for_saitan_run();

    if (fan_mode > 0) {
        FUN_PWM_Setup();
        FUN_PID_flag   = 1;
        FUN_MOTOR_flag = fan_mode;
        wait_ms(1000);
    }

    log_flag = 1;
    move_saitan(speed_mode);
    wait_ms(500);

    log_flag       = 0;
    motor_stop_flag = 1;
    FUN_MOTOR_flag = 0;
    FUN_PID_flag   = 0;
}

// -------------------------------------------------------------
// モード実行本体
// -------------------------------------------------------------
void ExecuteMode(Mode currentMode) {
    LED_All_On();
    mode_encoder_R_flag  = 0;
    mode_encoder_count_R = 0;
    mode_encoder_count_L = 0;
    wait_ms(1000);
    LED_All_Off();
    wait_ms(100);

    for (int i = 0; i < 300; i++) {
        UpdateLEDs(currentMode.main_mode, currentMode.sub_mode);
        if (g_sensor[2][0] >= 400 && g_sensor[1][0] >= 400) {
            currentMode.main_mode = 0;
            currentMode.sub_mode  = 0;
            wait_ms(1);
            break;
        }
        HAL_Delay(10);
    }

    LED_All_Off();
    printf("Mode %d.%d Executing\n", currentMode.main_mode, currentMode.sub_mode);

    switch (currentMode.main_mode) {
        case 1:
            if (currentMode.sub_mode != 0) {
                LED_All_On();
                prepare_for_saitan_run();
                Motor_Setup();
                log_flag = 1;
                log_flag = 1;
                sub_mode_num = currentMode.sub_mode;
                wait_ms(4000);
                log_flag = 0;
                motor_stop_flag = 1;
            } else {
                prepare_for_saitan_run();
                Motor_Setup();
                sub_mode_num = currentMode.sub_mode;
                log_flag = 1;
                wait_ms(4000);
                PID_turn_flag = 1;
                PID_Gyro_flag = 1;
                motor_daikei(0, 4000, 0, 20000, 180 * currentMode.sub_mode);
                wait_ms(10);
                log_flag = 0;
                motor_stop_flag = 1;
            }
            FUN_MOTOR_flag = 0;
            break;

        case 2: // サーキット・ファンテスト等
            if (currentMode.sub_mode == 1) {
                HAL_Delay(1000);
                wall_conf_flag = 1;
                motor_stop_flag = 0;
                LED_All_On();
                wait_ms(1000);
                lsm6dsr_reference();
                LED_All_Off();
                Gx = 0;
                PID_flag = 1;
                PID_turn_flag = 1;
                Motor_Setup();
                wall_conf_flag = 0;
                PID_turn_flag = 3;
                log_flag = 1;
                for (int i = 0; i <= 3; i++) {
                    Gv = 0;
                    senkai_omega_daikei_R(0, 400, 0, 2000, 90);
                    HAL_Delay(200);
                }
                log_flag = 0;
                motor_stop_flag = 1;
            } else if (currentMode.sub_mode == 2) {

                Motor_Setup();
                log_flag = 1;
                wall_conf_flag = 0;
            	PID_flag=4;
            	PID_turn_flag=4;
                wait_ms(4000);
            } else if (currentMode.sub_mode >= 3 && currentMode.sub_mode <= 5) {
                FUN_PWM_Setup();
                FUN_PID_flag   = 1;
                FUN_MOTOR_flag = (currentMode.sub_mode == 3) ? 1 : ((currentMode.sub_mode == 4) ? 4 : 6);
                log_flag = 1;
                wait_ms(50000);
                log_flag = 0;
                motor_stop_flag = 1;
                FUN_MOTOR_flag = 0;
            } else if (currentMode.sub_mode == 6) {
                prepare_for_saitan_run();
                log_flag = 1;
                Motor_Setup();
                sub_mode_num = currentMode.sub_mode;
                wall_conf_flag = 1;
                PID_turn_flag  = 1;
                PID_Gyro_flag  = 1;
                kabeate_R(600);
                log_flag = 0;
                motor_stop_flag = 1;
            }
            break;

        case 3: // 探索 ＋ 最短
            if (currentMode.sub_mode == 1) {
                prepare_for_saitan_run();
                Motor_Setup();
                log_flag = 1;
                adati_move(1);
                wait_ms(200);
                if (failsafe_flag != 1) {
                   // run_saitan_sequence(500, 1, 1);
                }
                log_flag = 0;
                motor_stop_flag = 1;
                wall_conf_flag  = 0;
                FUN_MOTOR_flag  = 0;
            } else if (currentMode.sub_mode == 2) {
            	print_DAIKUSUTORA_pass();
                print_adatimap();
                print_DAIKUSUTORAmap();
                Calculate_DAIKUSUTORA_math();
                Calculate_DAIKUSUTORA_pass();
                print_DAIKUSUTORA_pass();
                DAIKUSUTORA_pass_compression_diagoal();
            } else if (currentMode.sub_mode == 3) {
                run_saitan_sequence(500,  0, 1); // ★ 速度 500
            } else if (currentMode.sub_mode == 4) {
                run_saitan_sequence(1000, 0, 1); // ★ 速度 1000
            } else if (currentMode.sub_mode == 5) {
                run_saitan_sequence(1500, 1, 1); // ★ 速度 1500
            } else if (currentMode.sub_mode == 6) {
                run_saitan_sequence(2000, 2, 1); // ★ 速度 1800
            }
            break;

        case 4: // 各種最短速度テスト
            if (currentMode.sub_mode == 1) {
                prepare_for_saitan_run();
                Motor_Setup();
                log_flag = 1;
                adati_move(2);
                wait_ms(200);
                log_flag = 0;
                motor_stop_flag = 1;
                wall_conf_flag  = 0;
            } else if (currentMode.sub_mode == 2) {
                lsm6dsr_reference();
                log_flag = 1;
                FUN_PWM_Setup();
                FUN_PID_flag   = 1;
                FUN_MOTOR_flag = 1;
                wait_ms(6000);
                FUN_PID_flag   = 0;
                FUN_MOTOR_flag = 0;
                wait_ms(1000);
                log_flag = 0;
            } else if (currentMode.sub_mode == 3) {
                run_saitan_sequence(500,  2, 1); // ★ 速度 500
            } else if (currentMode.sub_mode == 4) {
                run_saitan_sequence(1000, 1, 1);
            } else if (currentMode.sub_mode == 5) {
                run_saitan_sequence(1400, 1, 1);
            } else if (currentMode.sub_mode == 6) {
                run_saitan_sequence(1450, 1, 1);
            } else if (currentMode.sub_mode == 7) {
                run_saitan_sequence(1470, 1, 1);
            } else if (currentMode.sub_mode == 8) {
                run_saitan_sequence(1500, 1, 1);
            }
            break;

        case 5: // 探索（複数回）＋ 最短
            if (currentMode.sub_mode == 1) {
                prepare_for_saitan_run();
                Motor_Setup();
                FUN_PWM_Setup();
                FUN_PID_flag   = 3;
                FUN_MOTOR_flag = 3;
                wait_ms(1000);
                log_flag = 1;
                adati_move(3);
                wait_ms(2000);
/*                if (failsafe_flag != 1) {
                    adati_move(3);
                    wait_ms(2000);
                }*/
                if (failsafe_flag != 1) {
                    run_saitan_sequence(1500, 1, 1);
                }
                if (failsafe_flag != 1) {
                    prepare_for_saitan_run();
                    Motor_Setup();
                    FUN_PWM_Setup();
                    FUN_PID_flag   = 3;
                    FUN_MOTOR_flag = 3;
                    wait_ms(1000);
                    wait_ms(10);
                    adati_move_GOTOSTART(3);
                }
            } else if (currentMode.sub_mode == 2) {
                print_adatimap();
                Calculate_DAIKUSUTORA_math();
                Calculate_DAIKUSUTORA_pass();
                DAIKUSUTORA_pass_compression_diagoal();
            } else if (currentMode.sub_mode >= 3 && currentMode.sub_mode <= 6) {
                int speeds[] = { 500, 1500, 1600, 1700 };
                uint8_t fan_m = (currentMode.sub_mode == 6) ? 2 : 1;
                run_saitan_sequence(speeds[currentMode.sub_mode - 3], fan_m, 1);
            }
            break;

        case 6:
        	if (currentMode.sub_mode == 1) {
        	                prepare_for_saitan_run();
        	                Motor_Setup();
        	                FUN_PWM_Setup();
        	                FUN_PID_flag   = 3;
        	                FUN_MOTOR_flag = 3;
        	                wait_ms(1000);
        	                log_flag = 1;
        	                adati_move(3);
        	                //wait_ms(2000);
            } else if (currentMode.sub_mode == 2) {
                log_print_2();
            }
            break;

        case 7:
            if (currentMode.sub_mode == 1) {
                log_print_2();
            } else if (currentMode.sub_mode == 2) {
                log_print();
            } else if (currentMode.sub_mode == 3) {
            	log_print_11();
            }
            break;
        default: // main_mode == 0 または 負のモード
			if (currentMode.main_mode == 0) {
                FUN_PWM_Setup();
                FUN_PID_flag   = 1;
                FUN_MOTOR_flag = (currentMode.sub_mode == 3) ? 1 : ((currentMode.sub_mode == 4) ? 4 : 6);
                log_flag = 1;
                wait_ms(50000);
                log_flag = 0;
                motor_stop_flag = 1;
                FUN_MOTOR_flag = 0;
				if (currentMode.sub_mode == 1)      change_goal();
				else if (currentMode.sub_mode == 2) { for (int i = 0; i <= 5000; i++) print_g_sensor(); }
				else if (currentMode.sub_mode == 3) { for (int i = 0; i <= 5000; i++) lsm6dsr_print(); }
				else if (currentMode.sub_mode == 4) {
					for (int i = 0; i <= 5000; i++) {
						printf("%4f, %4f\n\r", Measure_Gv_R, Measure_Gv_L);
						print_g_sensor();
					}
				}
			} else if (currentMode.main_mode <= -1) {
				// デバッグパス展開
				if (currentMode.sub_mode >= 1 && currentMode.sub_mode < (int)PATH_TABLE_SIZE) {
					execute_saitan_sequence(PATH_TABLE[currentMode.sub_mode].data, PATH_TABLE[currentMode.sub_mode].size);
				}

				int run_speed = 500;
				uint8_t fan_m = 1;

				// 負のモードに応じて速度・ファン設定を切り替え
				switch (currentMode.main_mode) {
					case -1: // 500 mm/s
						run_speed = 500;
						fan_m = 0;
						break;
					case -2: // 1000 mm/s
						run_speed = 1000;
						fan_m = 0;
						break;
					case -3: // 1500 mm/s
						run_speed = 1500;
						fan_m = 1;
						break;
					case -4: // 2000 mm/s (または 200)
						run_speed = 2000; // ※低速の200mm/sなら 200 に変更
						fan_m = 2;
						break;
					case -5: // 2000 mm/s (または 200)
						run_speed = 2400; // ※低速の200mm/sなら 200 に変更
						fan_m = 3;
						break;
					default:
						run_speed = 500;
						fan_m = 1;
						break;
				}

				run_saitan_sequence(run_speed, fan_m, 0); // need_calc=0 (手動パス指定)
			}
			break;
    }

    // 後処理
    PID_flag        = 0;
    PID_turn_flag   = 0;
    Motor_PWM_Stop();
    RESET_mode();
    wall_conf_flag  = 0;
    motor_stop_flag = 1;
    failsafe_flag   = 0;
    FUN_PID_flag    = 0;
    FUN_MOTOR_flag  = 0;
}

void RESET_mode(void) {
    encoder_R              = 0;
    encoder_L              = 0;
    mode_encoder_count_L   = 0;
    mode_encoder_count_R   = 0;
    total_negative_count_R = 0;
    current_encoder_R      = 0;
    total_negative_count_L = 0;
    current_encoder_L      = 0;
    mode_encoder_R_flag    = 0;
    pre_encoder_R          = 0;
    pre_encoder_L          = 0;

    LED_Chase_OneByOne();
}

void execute_saitan_sequence(int* path_data, int num_elements) {
    scanf_DAIKUSUTORA_pass(path_data, num_elements);
}

void prepare_for_saitan_run(void) {
    LED_All_On();
    HAL_Delay(2000);
    LED_All_Off();

    wall_conf_flag = 0;
    Measure_degree = 0;
    Gx             = 0;
    PID_flag       = 1;
    PID_turn_flag  = 1;
    Gdegree        = 0;
    Motor_Setup();
    Gdegree        = 0;
}
