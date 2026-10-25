/*
 * MOTOR.c
 *
 *  Created on: Nov 25, 2024
 *      Author: lingm
 */

#include "define.h"
#include "tim.h"
#include "stdio.h"
#include "stdlib.h"
#include "stm32f4xx_hal.h"
#include "PID.h"
#include "PL_encoder.h"

#include "SPEAKER.h"
#include "led_control.h"
#include "lsm6dsr.h"
#include "math.h"
#include "MOTOR.h"
#include "main.h"
#include "wall.h"
#include "log.h"

typedef struct {
    float v_sta;
    float v_max;
    float v_end;
    float dist;

    float acc_low;  // 低速域の加速度
    float acc_mid;  // 中速域の加速度
    float acc_high; // 高速域の加速度

    float v_thresh_1; // low -> mid に切り替わる速度
    float v_thresh_2; // mid -> high に切り替わる速度

} MotorProfile;

int senkai_flag=0;

#define Counter_Period 100

float Gv;
float Gt;
float Gx;
float Ga;
float Gomega;
float Gomega_acc;
float Gdegree;
float Measure_degree;

float Motor_PWM_L,Motor_PWM_R;
float duty_ratio_L, duty_ratio_R;
float ff_duty_ratio_L, ff_duty_ratio_R;
int motor_stop_flag;
int failsafe_flag;

int FUN_MOTOR_flag;
int v_end_flag;
int omega_end_flag;

void Motor_PWM_Generate() {
    float current_limit;
    // 浮動小数点数型の場合は fabsf()、整数の場合は abs() を使用
    float abs_Ga = fabsf((float)Ga);
    float abs_Gv = fabsf((float)Gv);
    // 1. 高速走行モード ＆ 高加減速または超高速域
    if ((FUN_MOTOR_flag == 1 || FUN_MOTOR_flag == 2) && (abs_Ga >= 20000.0f || abs_Gv >= 5100.0f)) {
        current_limit = 0.80f; // 高負荷時に許容するリミット
    }
    // 2. 高速走行モード（通常領域）
    else if (FUN_MOTOR_flag == 1 || FUN_MOTOR_flag == 2) {
        current_limit = 0.50f;
    }
    // 3. 探索モード時
    else if (FUN_MOTOR_flag == 3 || FUN_MOTOR_flag == 4) {
        current_limit = 0.35f;
    }
    // 4. ファン停止時 / デフォルト
    else {
        current_limit = 0.25f;
    }
    if (failsafe_flag == 1 || motor_stop_flag == 1) {
        duty_ratio_L = 0.0f;
        duty_ratio_R = 0.0f;
    }

    // --- 【追加】0.05以下のdutyは出力なし（カットオフ処理） ---
    // 正の値でも負の値でも判定できるように fabs() を使用します
    if (fabs(duty_ratio_L) <= 0.035f) {
        duty_ratio_L = 0.0f;
    }
    if (fabs(duty_ratio_R) <= 0.035f) {
        duty_ratio_R = 0.0f;
    }


    if (duty_ratio_L >  current_limit) duty_ratio_L =  current_limit;
    if (duty_ratio_L < -current_limit) duty_ratio_L = -current_limit;
    if (duty_ratio_R >  current_limit) duty_ratio_R =  current_limit;
    if (duty_ratio_R < -current_limit) duty_ratio_R = -current_limit;

    Motor_PWM_L = Counter_Period * duty_ratio_L;
    Motor_PWM_R = Counter_Period * duty_ratio_R;

    if (Motor_PWM_L < 0) {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, fabs(Motor_PWM_L));
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
    } else {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, fabs(Motor_PWM_L));
    }

    if (Motor_PWM_R < 0) {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, fabs(Motor_PWM_R));
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    } else {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, fabs(Motor_PWM_R));
    }
}
void Motor_Setup() {
    HAL_GPIO_WritePin(MOTOR_ENABLE_L_GPIO_Port,MOTOR_ENABLE_L_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_ENABLE_R_GPIO_Port,MOTOR_ENABLE_R_Pin, GPIO_PIN_SET);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    failsafe_flag=0;
    motor_stop_flag =0;
}

void Motor_PWM_Stop(){
    HAL_GPIO_WritePin(MOTOR_ENABLE_L_GPIO_Port,MOTOR_ENABLE_L_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_ENABLE_R_GPIO_Port,MOTOR_ENABLE_R_Pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
    motor_stop_flag =1;

    //PlayDoReMi();
}

static int32_t current_duty = 0;

void FUN_control(void) {

    static uint16_t count = 0;
    count++;

    if(count < 5) return;   // 5回に1回だけ更新
    count = 0;

    int32_t target_duty = 0;

    switch (FUN_MOTOR_flag) {
        case 0: target_duty = 0;  break;
        case 1: target_duty = 40; break;
        case 2: target_duty = 50; break;
        case 3: target_duty = 60; break;
        case 4: target_duty = 40; break;
        case 5: target_duty = 40; break;
        case 6: target_duty = 40; break;
        default: target_duty = 0; break;
    }

    if(failsafe_flag==1)target_duty = 0;

    const int32_t STEP = 1;

    if (current_duty < target_duty) {
        current_duty += STEP;
        if (current_duty > target_duty) current_duty = target_duty;
    }
    else if (current_duty > target_duty) {
        current_duty -= STEP;
        if (current_duty < target_duty) current_duty = target_duty;
    }


    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t)current_duty);
}

void FUN_PWM_Setup(void) {

        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
        FUN_MOTOR_flag = 1;
        FUN_PID_flag=0;
}

void FUN_PWM_Generate() {
    //FUN_PWM_Setup();
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 30);
}

void FUN_PWM_Stop(void) {
    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
    FUN_MOTOR_flag = 0;
    FUN_PID_flag=0;

}

void Motor_Control_Main(void)
{
    // =========================================================================
    // 1. 通常走行モード（吸引ファン OFF）
    // =========================================================================
    if (FUN_PID_flag == 0) {
        // --- 直進 / 並進制御 ---
        if (PID_flag == 1 || PID_flag == 2 || PID_flag == 3) {
            PID_Enc(Gv);
        } else if (PID_flag == 4) {
            wall_distance_move();
            PID_Turn_Enc(Gv);
        } else if (PID_flag == 5) {
            wall_distance_move();
            PID_stra = 0;
        } else {
            PID_stra = 0; // フェイルセーフ（制御OFF時のゼロクリア）
        }

        // --- 旋回 / 角速度制御 ---
        switch (PID_turn_flag) {
            case 1:
                PID_Gyro(Gomega - PID_wall);
                break;
            case 2:
                PID_SLALOM_Gyro(Gomega);
                break;
            case 3:
                PID_TURN_Gyro(Gomega);
                break;
            case 4:
                PID_TURN_Gyro(Gomega);
                break;
            default:
                PID_turn = 0;
                break;
        }
    }
    // =========================================================================
    // 2. 吸引ファン ON（二次走行・高速走行モード）
    // =========================================================================
    else if (FUN_PID_flag == 1 || FUN_PID_flag == 2) {
        // --- 直進 / 並進制御 ---
        if (PID_flag == 1) {
            PID_FUN_Enc(Gv);
        } else if (PID_flag == 2 || PID_flag == 3) {
            PID_Turn_FUN_Enc(Gv);
        } else {
            PID_stra = 0;
        }

        // --- 旋回 / 角速度制御 ---
        switch (PID_turn_flag) {
            case 1:
                PID_FUN_Gyro(Gomega - PID_wall);
                break;
            case 2:
                PID_FUN_SLALOM_Gyro(Gomega);
                break;
            case 3:
                PID_FUN_TURN_Gyro(Gomega);
                break;
            default:
                PID_turn = 0;
                break;
        }
    }
    // =========================================================================
    // 3. 吸引ファン ON（探索モード）
    // =========================================================================
    else if (FUN_PID_flag == 3 || FUN_PID_flag == 4) {
        // --- 直進 / 並進制御 ---
        if (PID_flag == 1) {
            PID_FUN_tansaku_Enc(Gv);
        } else if (PID_flag == 2 || PID_flag == 3) {
            PID_Turn_FUN_tansaku_Enc(Gv);
        } else if (PID_flag == 4) {
            wall_distance_move();
            PID_Turn_FUN_tansaku_Enc(Gv);
        } else {
            PID_stra = 0;
        }

        // --- 旋回 / 角速度制御 ---
        switch (PID_turn_flag) {
            case 1:
                PID_FUN_tansaku_Gyro(Gomega - PID_wall);
                break;
            case 2:
                PID_FUN_SLALOM_tansaku_Gyro(Gomega);
                break;
            case 3:
                PID_FUN_TURN_tansaku_Gyro(Gomega);
                break;
            case 4:
                PID_FUN_TURN_tansaku_Gyro(Gomega);
                break;
            default:
                PID_turn = 0;
                break;
        }
    }
}

void Motor_Task(void)

{

	Encorder_Speed_Calculate();
	duty_ratio_L =0;
	duty_ratio_R =0;
	Motor_Control_Main();

	float left_output = PID_stra - PID_turn; //- PID_wall;
	float right_output = PID_stra + PID_turn; //+ PID_wall;

	if(PID_turn_flag==1){//straight
		calculate_duty_ratio(left_output, right_output, Ga, Ga);
	}else if(PID_turn_flag==2){//slalom
		calculate_duty_ratio(left_output, right_output, -Gomega_acc,+Gomega_acc);
	}else if(PID_turn_flag==3){//turn
		calculate_duty_ratio(left_output, right_output, -Gomega_acc,+Gomega_acc);
	}else{
		calculate_duty_ratio(left_output, right_output, 0, 0);
	}

	Feedforward_control();
	duty_ratio_L +=ff_duty_ratio_L;
	duty_ratio_R +=ff_duty_ratio_R;
    // デューティ比の制限
    if (duty_ratio_L > 0.99f) {
        duty_ratio_L = 0.99f;
    }
    if (duty_ratio_R > 0.99f) {
        duty_ratio_R = 0.99f;
    }
    if (duty_ratio_L < -0.99f) {
        duty_ratio_L = -0.99f;
    }
    if (duty_ratio_R < -0.99f) {
        duty_ratio_R = -0.99f;
    }
	Motor_PWM_Generate();

}



void Feedforward_control(){
    // 度(deg) を ラジアン(rad) に変換
    float Gomega_rad = (Gomega) * PI / 180.0f;
    float Gomega_acc_rad = Gomega_acc * PI / 180.0f;

    float half_width_mm = (TreadWidth * 1000.0f) / 2.0f;

    // ラジアン版の角速度を使って目標速度を計算
    float target_v_L = Gv - (Gomega_rad * half_width_mm);
    float target_v_R = Gv + (Gomega_rad * half_width_mm);

    float target_a_L = Ga - (Gomega_acc_rad * half_width_mm);
    float target_a_R = Ga + (Gomega_acc_rad * half_width_mm);

    Feedforward_calculate_duty_ratio(target_v_L, target_v_R, target_a_L, target_a_R);
}



void Feedforward_calculate_duty_ratio(float v_L_mm, float v_R_mm, float acc_L_mm, float acc_R_mm) {

	const float Ks = 0.03f; // 摩擦補償（デッドゾーン）



	// 1. mm 単位から m 単位へ変換

	float v_L_m = v_L_mm / 1000.0f;

	float v_R_m = v_R_mm / 1000.0f;

	float a_L_m = acc_L_mm / 1000.0f;

	float a_R_m = acc_R_mm / 1000.0f;



	float r = Tire_dia / 2.0f; // タイヤ半径(m)



	float torque_L = (m * a_L_m / 2.0f) * r / reduction_ratio;

	float torque_R = (m * a_R_m / 2.0f) * r / reduction_ratio;



	float V_torque_L = (torque_L * R) / kT;

	float V_torque_R = (torque_R * R) / kT;



	float omega_motor_L = (v_L_m / r) * reduction_ratio;

	float omega_motor_R = (v_R_m / r) * reduction_ratio;



	float V_emf_L = kE * omega_motor_L;

	float V_emf_R = kE * omega_motor_R;



	// 4. デューティ比の計算 (出力 = 電圧 / バッテリー電圧)

	ff_duty_ratio_L = (V_torque_L + V_emf_L) / V_bat;

	ff_duty_ratio_R = (V_torque_R + V_emf_R) / V_bat;



	// 5. 静止摩擦補償 (速度の向きに合わせてゲタを履かせる)

	if (v_L_mm > 5.0f) ff_duty_ratio_L += Ks;

	else if (v_L_mm < -5.0f) ff_duty_ratio_L -= Ks;



	if (v_R_mm > 5.0f) ff_duty_ratio_R += Ks;

	else if (v_R_mm < -5.0f) ff_duty_ratio_R -= Ks;

}

void motor_daikei(float v_sta, float v_max, float v_end, float acc, float X) {
    PID_turn_flag = 1;
    PID_flag = 1;

	if(v_sta>= v_max){
		 v_max= v_sta;
	}
	if(v_end>= v_max){
		v_end =v_max ;
	}
	PID_turn_flag=1;
	PID_flag=1;
    // 各フェーズの時間と距離を計算
    float time_st = (v_max - v_sta) / acc;
    float time_end = (v_max - v_end) / acc;

    // 加速区間の移動距離
    float accel_dist = ((v_max + v_sta) * time_st) / 2;
    // 減速区間の移動距離
    float decel_dist = ((v_max + v_end) * time_end) / 2;
    // 等速区間の移動距離
    float const_dist = X - accel_dist - decel_dist;

    // 台形制御の条件を満たしているかチェック
    if (const_dist < 0) {
        // 等速区間が無い場合（すべて加減速区間）
        v_max = sqrt((2 * acc * X + v_sta * v_sta + v_end * v_end) / 2);
        time_st = (v_max - v_sta) / acc;
        time_end = (v_max - v_end) / acc;
        accel_dist = ((v_max + v_sta) * time_st) / 2;
        decel_dist = ((v_max + v_end) * time_end) / 2;
        const_dist = 0;
    }

    // 初期化
    //wall_con_flag=1;
    Gv = v_sta;
    Gx = 0;
    Ga = acc;

    // 加速フェーズ
    while (1) {
        Ga = acc;
        if (Gv >= v_max) {
            Gv = v_max;
            break;
        }
/*        if(v_max<=v_sta){
        	break;
        }*/
    }

    // 等速フェーズ
    Ga = 0;
    while (1) {
        Ga = 0;
        if (Gx >= X-decel_dist) {
            Gv = v_max;
            break;
        }
    }

    // 減速フェーズ
    Ga = -acc;
    while (1) {
        if (Gv <= v_end) {
            Gv = v_end;
            break;
        }
        if(Gx>=X){
        	Gv = v_end;
        	break;
        }

    }
    Ga = 0;
   // wall_con_flag=0;
//    if(v_end==0){
//    	v_end_flag=1;
//
//    }

    Gv = v_end;
    //Gx = 0;
    Ga = 0;

}



#define STA_ACCEL_SWITCH_SPEED 4500.0f
#define STA_HIGH_SPEED_ACC_RATIO 0.5f

static float sta_accel_distance(float v_start, float v_peak, float acc) {
    float distance = 0.0f;

    if (v_peak <= v_start) return 0.0f;

    if (v_start < STA_ACCEL_SWITCH_SPEED) {
        const float low_end = fminf(v_peak, STA_ACCEL_SWITCH_SPEED);
        distance += (low_end * low_end - v_start * v_start) / (2.0f * acc);
    }
    if (v_peak > STA_ACCEL_SWITCH_SPEED) {
        const float high_start = fmaxf(v_start, STA_ACCEL_SWITCH_SPEED);
        const float high_acc = acc * STA_HIGH_SPEED_ACC_RATIO;
        distance += (v_peak * v_peak - high_start * high_start) / (2.0f * high_acc);
    }
    return distance;
}

static float sta_decel_distance(float v_peak, float v_end, float decel) {
    if (v_peak <= v_end) return 0.0f;
    return (v_peak * v_peak - v_end * v_end) / (2.0f * decel);
}

void motor_daikei_sta(float v_sta, float v_max, float v_end, float acc, float X) {
    float peak_speed;
    float accel_dist;
    float decel_dist;
    float accel_command = acc;
    float decel_command = acc;
    int use_staged_accel = 1;

    PID_turn_flag = 1;
    PID_flag = 1;

    if (X <= 0.0f || acc <= 0.0f || v_sta < 0.0f || v_end < 0.0f) {
        Gv = v_end;
        Ga = 0.0f;
        v_end_flag = 1;
        return;
    }

    peak_speed = fmaxf(v_max, fmaxf(v_sta, v_end));
    accel_dist = sta_accel_distance(v_sta, peak_speed, acc);
    decel_dist = sta_decel_distance(peak_speed, v_end, acc);

    if (accel_dist + decel_dist > X) {
        const float minimum_peak = fmaxf(v_sta, v_end);
        const float minimum_dist = sta_accel_distance(v_sta, minimum_peak, acc)
                                 + sta_decel_distance(minimum_peak, v_end, acc);

        if (minimum_dist <= X) {
            float low = minimum_peak;
            float high = peak_speed;

            /* 可変加速度を含む三角プロファイルのピーク速度を求める。 */
            for (int i = 0; i < 24; i++) {
                const float candidate = (low + high) * 0.5f;
                const float required = sta_accel_distance(v_sta, candidate, acc)
                                     + sta_decel_distance(candidate, v_end, acc);
                if (required <= X) low = candidate;
                else high = candidate;
            }
            peak_speed = low;
            accel_dist = sta_accel_distance(v_sta, peak_speed, acc);
            decel_dist = sta_decel_distance(peak_speed, v_end, acc);
        } else if (v_sta > v_end) {
            /* 短距離でも指定終速に到達できる減速度を計算する。 */
            peak_speed = v_sta;
            accel_dist = 0.0f;
            decel_dist = X;
            decel_command = (v_sta * v_sta - v_end * v_end) / (2.0f * X);
        } else {
            /* 終速まで加速するだけで距離を使い切るケース。 */
            peak_speed = v_end;
            accel_dist = X;
            decel_dist = 0.0f;
            accel_command = (v_end * v_end - v_sta * v_sta) / (2.0f * X);
            use_staged_accel = 0;
        }
    }

    const float decel_start = X - decel_dist;

    Gv = v_sta;
    Gx = 0.0f;

    while (Gx < accel_dist && Gv < peak_speed) {
        if (use_staged_accel && Gv >= STA_ACCEL_SWITCH_SPEED) {
            Ga = acc * STA_HIGH_SPEED_ACC_RATIO;
        } else {
            Ga = accel_command;
        }
    }
    Gv = peak_speed;
    Ga = 0.0f;

    while (Gx < decel_start) {
        Ga = 0.0f;
    }

    while (Gx < X && Gv > v_end) {
        Ga = -decel_command;
    }

    /* 1 ms積分の丸めで早く終速へ着いた場合は、残距離を終速で走る。 */
    if (Gv <= v_end) {
        Gv = v_end;
        Ga = 0.0f;
        while (Gx < X && v_end > 0.0f) {
        }
    }

    Gv = v_end;
    Ga = 0.0f;
    v_end_flag = 1;
}
void motor_daikei_back(float v_sta, float v_max, float v_end, float acc, float X) {

	if(v_sta>= v_max){
		 v_max= v_sta;
	}
	if(v_end>= v_max){
		v_end =v_max ;
	}
	PID_turn_flag=1;
	PID_flag=1;
    float time_st = (v_max - v_sta) / acc;
    float time_end = (v_max - v_end) / acc;

    // 加速区間の移動距離
    float accel_dist = ((v_max + v_sta) * time_st) / 2;
    // 減速区間の移動距離
    float decel_dist = ((v_max + v_end) * time_end) / 2;
    // 等速区間の移動距離
    float const_dist = X - accel_dist - decel_dist;

    // 台形制御の条件を満たしているかチェック
    if (const_dist < 0) {
        // 等速区間が無い場合（すべて加減速区間）
        v_max = sqrt((2 * acc * X + v_sta * v_sta + v_end * v_end) / 2);
        time_st = (v_max - v_sta) / acc;
        time_end = (v_max - v_end) / acc;
        accel_dist = ((v_max + v_sta) * time_st) / 2;
        decel_dist = ((v_max + v_end) * time_end) / 2;
        const_dist = 0;
    }

    // 初期化
    //wall_con_flag=1;
    Gv = -v_sta;
    Gx = 0;
    Ga = -acc;

    // 加速フェーズ
    while (1) {
        Ga = -acc;
        if (Gx <= -(accel_dist)) {
            Gv = -v_max;
            break;
        }
    }

    // 等速フェーズ
    Ga = 0;
    while (1) {
        Ga = 0;
        if (Gx <= -(accel_dist + const_dist)) {
            Gv = -v_max;
            break;
        }
    }

    // 減速フェーズ
    Ga = acc;
    while (1) {
        if (Gv >= -v_end) {
            Gv = -v_end;
            break;
        }
    }
    Ga = 0;
   // wall_con_flag=0;
    if(v_end==0){
    	v_end_flag=1;

    }
    //Log_flag=0;

    //wall_con_flag=1;
    Gv = -v_end;
    //Gx = 0;
    Ga = 0;
    //v_end_flag=1;
    Gdegree=0;

}

void omega_daikei_R(float v_sta, float v_max, float v_end, float acc, float Daikei_degree) {
	v_end_flag=1;
	wall_conf_flag=0;
   // PID_gyro_reset_flag=1;
	PID_turn_flag=2;
	PID_flag=2;
	float X =  Daikei_degree;

    float time_st = (v_max - v_sta) / acc;
    float time_end = (v_max - v_end) / acc;

    // 加速区間の移動距離
    float accel_dist = ((v_max + v_sta) * time_st) / 2;
    // 減速区間の移動距離
    float decel_dist = ((v_max + v_end) * time_end) / 2;
    // 等速区間の移動距離
    float const_dist = X - accel_dist - decel_dist;

    // 台形制御の条件を満たしているかチェック
    if (const_dist < 0) {
        // 等速区間が無い場合（すべて加減速区間）
        v_max = sqrt((2 * acc * X + v_sta * v_sta + v_end * v_end) / 2);
        time_st = (v_max - v_sta) / acc;
        time_end = (v_max - v_end) / acc;
        accel_dist = ((v_max + v_sta) * time_st) / 2;
        decel_dist = ((v_max + v_end) * time_end) / 2;
        const_dist = 0;

    }

    // 初期化
    //wall_con_flag=1;
    Gomega = v_sta;
    Gdegree=0;

    Gomega_acc = acc;
    omega_end_flag=1;
    // 加速フェーズ
    while (1) {
        Gomega_acc = acc;
        if (Gomega >= v_max) {
            Gomega = v_max;
            break;
        }
    }

    // 等速フェーズ
    Gomega_acc = 0;
    while (1) {
    	Gomega_acc = 0;
        if (Gdegree>= X-decel_dist) {
        	Gomega = v_max;
            break;
        }
    }

    Gomega_acc = -acc;
    while (1) {
        if (Gomega <= v_end) {
        	Gomega = v_end;
            break;
        }
    }
    Gomega_acc = 0;
   // wall_con_flag=0;
    if(v_end==0){
    	omega_end_flag=1;

    }
    //PID_gyro_reset_flag=1;
    Gomega = v_end;

    Gomega_acc = 0;
    Gdegree=0;
    Measure_degree=0;
    PID_flag=1;
	PID_turn_flag=1;
}


void senkai_omega_daikei_R(float v_sta, float v_max, float v_end, float acc, float Daikei_degree) {
	v_end_flag=1;
	wall_conf_flag=0;
	PID_flag=3;
	PID_turn_flag=3;
	float X =  TreadWidth*2000*M_PI*Daikei_degree /360;
   // Log_flag=1;
    // 各フェーズの時間と距離を計算
    float time_st = (v_max - v_sta) / acc;
    float time_end = (v_max - v_end) / acc;

    // 加速区間の移動距離
    float accel_dist = ((v_max + v_sta) * time_st) / 2;
    // 減速区間の移動距離
    float decel_dist = ((v_max + v_end) * time_end) / 2;
    // 等速区間の移動距離
    float const_dist = X - accel_dist - decel_dist;

    // 台形制御の条件を満たしているかチェック
    if (const_dist < 0) {
        // 等速区間が無い場合（すべて加減速区間）
        v_max = sqrt((2 * acc * X + v_sta * v_sta + v_end * v_end) / 2);
        time_st = (v_max - v_sta) / acc;
        time_end = (v_max - v_end) / acc;
        accel_dist = ((v_max + v_sta) * time_st) / 2;
        decel_dist = ((v_max + v_end) * time_end) / 2;
        const_dist = 0;
    }

    // 初期化
    //wall_con_flag=1;
    Gomega = v_sta;
    Gdegree=0;
    Gomega_acc = acc;
    omega_end_flag=1;
    // 加速フェーズ
    while (1) {
        Gomega_acc = acc;
        if (Gomega >= v_max) {
            Gomega = v_max;
            break;
        }
    }

    // 等速フェーズ
    Gomega_acc = 0;
    while (1) {
    	Gomega_acc = 0;
        if (Gdegree>= 360*(accel_dist + const_dist)/(TreadWidth*2000*M_PI)) {
        	Gomega = v_max;
            break;
        }
    }

    // 減速フェーズ
    Gomega_acc = -acc;
    while (1) {
        if (Gomega <= v_end) {
        	Gomega = v_end;
            break;
        }
    }
    Gomega_acc = 0;
   // wall_con_flag=0;
    //Log_flag=0;

    //wall_con_flag=1;
    Gomega = v_end;
    //Gx = 0;
    Gomega_acc = 0;
    Gdegree=0;
    Measure_degree=0;
    senkai_flag=0;
    //motor_stop_flag=0;
	PID_turn_flag=1;
}



void omega_daikei_L(float v_sta, float v_max, float v_end, float acc, float Daikei_degree) {
	v_end_flag=1;
	wall_conf_flag=0;
   // PID_gyro_reset_flag=1;
	PID_turn_flag=2;
	PID_flag=2;
	float X = Daikei_degree ;
   // Log_flag=1;
    // 各フェーズの時間と距離を計算
    float time_st = (v_max - v_sta) / acc;
    float time_end = (v_max - v_end) / acc;

    // 加速区間の移動距離
    float accel_dist = ((v_max + v_sta) * time_st) / 2;
    // 減速区間の移動距離
    float decel_dist = ((v_max + v_end) * time_end) / 2;
    // 等速区間の移動距離
    float const_dist = X - accel_dist - decel_dist;

    // 台形制御の条件を満たしているかチェック
    if (const_dist < 0) {
        // 等速区間が無い場合（すべて加減速区間）
        v_max = sqrt((2 * acc * X + v_sta * v_sta + v_end * v_end) / 2);
        time_st = (v_max - v_sta) / acc;
        time_end = (v_max - v_end) / acc;
        accel_dist = ((v_max + v_sta) * time_st) / 2;
        decel_dist = ((v_max + v_end) * time_end) / 2;
        const_dist = 0;
    }

    // 初期化
    //wall_con_flag=1;
    Gomega = -v_sta;
    Gdegree=0;

    Gomega_acc = -acc;
    omega_end_flag=1;
    // 加速フェーズ
    while (1) {
        Gomega_acc = -acc;
        if (Gomega <= -v_max) {
            Gomega = -v_max;
            break;
        }
    }

    // 等速フェーズ
    Gomega_acc = 0;
    while (1) {
    	Gomega_acc = 0;
        if (Gdegree<= -X+decel_dist) {
        	Gomega = -v_max;
            break;
        }
    }

    // 減速フェーズ
    Gomega_acc = acc;
    while (1) {
        if (Gomega >= -v_end) {
        	Gomega = -v_end;
            break;
        }
    }
    Gomega_acc = 0;
   // wall_con_flag=0;
    if(v_end==0){
    	omega_end_flag=1;

    }
    //PID_gyro_reset_flag=1;
    Gomega = -v_end;

    Gomega_acc = 0;
    Gdegree=0;
    Measure_degree=0;
    PID_flag=1;
	PID_turn_flag=1;
}


void senkai_omega_daikei_L(float v_sta, float v_max, float v_end, float acc, float Daikei_degree) {
	v_end_flag=1;
	wall_conf_flag=0;
	PID_flag=3;
	PID_turn_flag=3;
	float X =  TreadWidth*2000*M_PI*Daikei_degree /360;
   // Log_flag=1;
    // 各フェーズの時間と距離を計算
    float time_st = (v_max - v_sta) / acc;
    float time_end = (v_max - v_end) / acc;

    // 加速区間の移動距離
    float accel_dist = ((v_max + v_sta) * time_st) / 2;
    // 減速区間の移動距離
    float decel_dist = ((v_max + v_end) * time_end) / 2;
    // 等速区間の移動距離
    float const_dist = X - accel_dist - decel_dist;

    // 台形制御の条件を満たしているかチェック
    if (const_dist < 0) {
        // 等速区間が無い場合（すべて加減速区間）
        v_max = sqrt((2 * acc * X + v_sta * v_sta + v_end * v_end) / 2);
        time_st = (v_max - v_sta) / acc;
        time_end = (v_max - v_end) / acc;
        accel_dist = ((v_max + v_sta) * time_st) / 2;
        decel_dist = ((v_max + v_end) * time_end) / 2;
        const_dist = 0;
    }

    // 初期化
    //wall_con_flag=1;
    Gomega = -v_sta;
    Gdegree=0;
    Gomega_acc = -acc;
    omega_end_flag=1;
    // 加速フェーズ
    while (1) {
        Gomega_acc = -acc;
        if (Gomega <= -v_max) {
            Gomega = -v_max;
            break;
        }
    }

    // 等速フェーズ
    Gomega_acc = 0;
    while (1) {
    	Gomega_acc = 0;
        if (Gdegree<= -360*(accel_dist + const_dist)/(TreadWidth*2000*M_PI)) {
        	Gomega = -v_max;
            break;
        }
    }

    // 減速フェーズ
    Gomega_acc = acc;
    while (1) {
        if (Gomega >= v_end) {
        	Gomega = -v_end;
            break;
        }
    }
    Gomega_acc = 0;

    Gomega = -v_end;
    Gomega_acc = 0;
    Gdegree=0;
    Measure_degree=0;
    senkai_flag=0;
	PID_turn_flag=1;
}


void calculate_duty_ratio(float v_L, float v_R, float acc_L, float acc_R) {

    // 左右のトルク計算
    float T_L = m * Tire_dia / 2.0f * acc_L / (2.0f  * reduction_ratio);
    float T_R = m * Tire_dia / 2.0f * acc_R / (2.0f  * reduction_ratio);

    // 目標角速度の計算（rad/ms）
    float Target_omega_L = 60.0f * v_L / 1000.0f * reduction_ratio / (M_PI * Tire_dia);
    float Target_omega_R = 60.0f * v_R / 1000.0f * reduction_ratio / (M_PI * Tire_dia);

/*    // PID制御による角速度補正
    float PID_omega_L = calculate_PID_omega_L(Target_omega_L);
    float PID_omega_R = calculate_PID_omega_R(Target_omega_R);*/

    // バックEMFの計算
    float E_L = kE * (Target_omega_L);// + PID_omega_L);
    float E_R = kE * (Target_omega_R);// + PID_omega_R);



    // デューティ比の計算 (グローバル変数に代入)
    duty_ratio_L = ((T_L * R / kT) + E_L) / V_bat;
    duty_ratio_R = ((T_R * R / kT) + E_R) / V_bat;

    // デューティ比の制限
    if (duty_ratio_L > 0.99f) {
        duty_ratio_L = 0.99f;
    }
    if (duty_ratio_R > 0.99f) {
        duty_ratio_R = 0.99f;
    }
    if (duty_ratio_L < -0.99f) {
        duty_ratio_L = -0.99f;
    }
    if (duty_ratio_R < -0.99f) {
        duty_ratio_R = -0.99f;
    }
}

void failsafe(){
	if(failsafe_flag==1){
		LED_All_On();
	}
}

int check_lifted_condition(float accel_y, float gyro_z) {
    static uint16_t lifted_counter = 0;
    const uint16_t THRESHOLD_TIME = 200; // 50ms -> 80ms など少し長めに設定

    // 1. 理論上の遠心加速度を計算 (v[m/s] * omega[rad/s])
    // ※ 単位を加速度センサの単位 (m/s^2 など) に合わせる
    float omega_rad = gyro_z * (3.14159265f / 180.0f); // deg/s -> rad/s
    float v_m_s = Gv / 1000.0f;                 // mm/s -> m/s
    float centrifugal_acc = v_m_s * omega_rad;         // [m/s^2]

    // 2. 遠心力を差し引いた正味の横加速度
    float net_accel_y = accel_y - centrifugal_acc;

    // 3. 正味の加速度で判定 (走行による遠心力以外の異常な外力・傾き)
    if (fabsf(net_accel_y) > 60.0f) {
        lifted_counter++;
    } else {
        if (lifted_counter > 0) lifted_counter--;
    }

    if (lifted_counter > THRESHOLD_TIME) {
        log_data_2(3, (Measure_Gv_R + Measure_Gv_L) * 1000 / 2.0f);
        return 1;
    }
    return 0;
}

// 角度偏差によるフェールセーフ判定関数
int check_angle_error_condition(float target_degree, float current_degree) {
    static uint16_t error_counter = 0;
    const uint16_t ERROR_THRESHOLD_TIME = 100; // 50ms（ループ周期が1msの場合）
    const float MAX_ALLOWABLE_ERROR = 90.0f;    // 許容できる最大誤差（30度）

    float angle_error = fabs(target_degree - current_degree);

    if (angle_error > MAX_ALLOWABLE_ERROR) {
        error_counter++;
    } else {
        if (error_counter > 0) error_counter--;
    }

    if (error_counter > ERROR_THRESHOLD_TIME) {
    	log_data_2(2,(Measure_Gv_R+ Measure_Gv_L)*1000 / 2.0f);
        return 1; // 異常あり
    }
    return 0; // 正常
}

// 速度偏差によるフェールセーフ判定関数
int check_velocity_error_condition(float theory_v, float meas_v_r, float meas_v_l) {
    static uint16_t v_error_counter = 0;
    const uint16_t V_ERROR_THRESHOLD_TIME = 100; // 150ms（環境に合わせて調整）
    const float MAX_V_DIFF = 500.0f;             // 許容できる最大速度差（単位に合わせて調整）

    // 実測速度の平均を算出
    float measured_v_avg = (meas_v_r + meas_v_l) * 1000.0f / 2.0f;

    // 理論速度と実測速度の差の絶対値を計算
    float velocity_diff = fabs(theory_v - measured_v_avg);

    if (velocity_diff > MAX_V_DIFF) {
        v_error_counter++;
    } else {
        if (v_error_counter > 0) v_error_counter--;
    }

    if (v_error_counter > V_ERROR_THRESHOLD_TIME) {
    	log_data_2(1,(Measure_Gv_R+ Measure_Gv_L)*1000 / 2.0f);
        return 1; // 速度異常（スリップまたはスタック）
    }
    return 0;
}
