/*
 * PID.c
 *
 *  Created on: Nov 25, 2024
 *      Author: lingm
 */

#include "PL_encoder.h"
#include "MOTOR.h"
#include "lsm6dsr.h"
#include "PID.h"
#include "wall.h"
#include "log.h"
//#include "led_control.h"

/*int v_end_flag;
int omega_end_flag;*/

PID_Control Enc = {0};  // 初期化
PID_Control Gyro = {0}; // 初期化

float PID_stra = 0.0f;
float PID_turn = 0.0f;
int FUN_PID_flag;
/*int motor_stop_flag;*/
int PID_flag;
int PID_turn_flag;
int PID_gyro_reset_flag;
int PID_Gyro_flag=0;

void PID_Enc(float target_v) {
    Enc.error = target_v - (Measure_Gv_R+ Measure_Gv_L)*1000 / 2.0f;
    Enc.delta_error = Enc.error - Enc.old_error;
    Enc.old_error = Enc.error;
    Enc.sigma_error += Enc.error;
    PID_stra = Kp_enc * Enc.error + Ki_enc * Enc.sigma_error + Kd_enc * Enc.delta_error;


    if (Enc.sigma_error > 2000.0f)  Enc.sigma_error = 2000.0f;
    if (Enc.sigma_error < -2000.0f) Enc.sigma_error = -2000.0f;
    if (v_end_flag == 1) {
    	Enc.delta_error =0;
        Enc.sigma_error = 0;
        v_end_flag = 0;
    }

	//log_data(PID_wall,Enc.delta_error,Enc.sigma_error,Gv,(Measure_Gv_R+ Measure_Gv_L)*1000 / 2.0f)	;

}
void PID_Turn_Enc(float target_v) {
    Enc.error = target_v - (Measure_Gv_R+ Measure_Gv_L)*1000 / 2.0f;
    Enc.delta_error = Enc.error - Enc.old_error;
    Enc.old_error = Enc.error;
    Enc.sigma_error += Enc.error;
    PID_stra = Ktp_enc * Enc.error + Kti_enc * Enc.sigma_error + Ktd_enc * Enc.delta_error;

    if (v_end_flag == 1) {
    	Enc.delta_error =0;
        Enc.sigma_error = 0;
        v_end_flag = 0;
    }
}

void PID_Gyro(float target_omega) {
    Gyro.error = target_omega - G_Gyro_Z;// + PID_wall;
    Gyro.delta_error = Gyro.error - Gyro.old_error;
    Gyro.old_error = Gyro.error;
    Gyro.sigma_error += Gyro.error;

    PID_turn = Kp_gyro * Gyro.error + Ki_gyro * Gyro.sigma_error + Kd_gyro * Gyro.delta_error;

    if (omega_end_flag == 1||PID_gyro_reset_flag==1) {
        Gyro.sigma_error = 0;
    	PID_gyro_reset_flag=0;
    	omega_end_flag=0;
    }
}

void PID_SLALOM_Gyro(float target_omega) {
    Gyro.error = target_omega - G_Gyro_Z;
    Gyro.delta_error = Gyro.error - Gyro.old_error;
    Gyro.old_error = Gyro.error;
    Gyro.sigma_error += Gyro.error;
    PID_turn = Ksp_gyro * Gyro.error + Ksi_gyro * Gyro.sigma_error + Ksd_gyro * Gyro.delta_error;
    if (omega_end_flag == 1||PID_gyro_reset_flag==1) {
        Gyro.sigma_error = 0;
    	PID_gyro_reset_flag=0;
    	omega_end_flag=0;
    }
}

void PID_TURN_Gyro(float target_omega) {
    Gyro.error = target_omega - G_Gyro_Z;
    Gyro.delta_error = Gyro.error - Gyro.old_error;
    Gyro.old_error = Gyro.error;
    Gyro.sigma_error += Gyro.error;
    PID_turn = Ktp_gyro * Gyro.error + Kti_gyro * Gyro.sigma_error + Ktd_gyro * Gyro.delta_error;

    if(omega_end_flag==1||PID_gyro_reset_flag==1){
    	Gyro.sigma_error=0;
    	Gyro.delta_error=0;
    	 Gyro.error=0;
    	    Gyro.old_error=0;
    	//Enc.sigma_error=0;
    	PID_gyro_reset_flag=0;
    	omega_end_flag=0;
    }

    if (Gyro.sigma_error  > 2000.0f)  Gyro.sigma_error  = 2000.0f;
    if (Gyro.sigma_error  < -2000.0f) Gyro.sigma_error  = -2000.0f;
}

void PID_FUN_Enc(float target_v) {
    // 1. エラーの計算
    Enc.error = target_v - (Measure_Gv_R + Measure_Gv_L) * 1000 / 2.0f;

    // 2. 積分項の計算（ここがポイント！）
    Enc.sigma_error += Enc.error;

    if (Enc.sigma_error > 3000.0f)  Enc.sigma_error = 3000.0f;
    if (Enc.sigma_error < -3000.0f) Enc.sigma_error = -3000.0f;
    // --------------------------------

    Enc.delta_error = Enc.error - Enc.old_error;
    Enc.old_error = Enc.error;

    PID_stra = (Kp_FUN_enc * Enc.error)
             + (Ki_FUN_enc * Enc.sigma_error)
             + (Kd_FUN_enc * Enc.delta_error);

    if(Gv==0&&(Measure_Gv_R + Measure_Gv_L) * 1000 / 2.0f<=0){
    	 v_end_flag = 1;
    }

    if (v_end_flag == 1) {
        Enc.delta_error = 0;
        Enc.sigma_error = 0;
        Enc.old_error = 0; // old_errorもリセットしておくと次回の始動がスムーズです
        v_end_flag = 0;
    }

    	// log_data(PID_wall,Enc.delta_error,Enc.sigma_error,Gv,(Measure_Gv_R+ Measure_Gv_L)*1000 / 2.0f)	;

}

void PID_Turn_FUN_Enc(float target_v) {
    // 1. エラーの計算
    Enc.error = target_v - (Measure_Gv_R + Measure_Gv_L) * 1000 / 2.0f;

    // 2. 積分項の計算（ここがポイント！）
    Enc.sigma_error += Enc.error;

    if (Enc.sigma_error > 1000.0f)  Enc.sigma_error = 1000.0f;
    if (Enc.sigma_error < -1000.0f) Enc.sigma_error = -1000.0f;
    // --------------------------------

    Enc.delta_error = Enc.error - Enc.old_error;
    Enc.old_error = Enc.error;

    PID_stra = (Ktp_FUN_enc * Enc.error)
             + (Kti_FUN_enc * Enc.sigma_error)
             + (Ktd_FUN_enc * Enc.delta_error);

    if (v_end_flag == 1) {
        Enc.delta_error = 0;
        Enc.sigma_error = 0;
        Enc.old_error = 0; // old_errorもリセットしておくと次回の始動がスムーズです
        v_end_flag = 0;
    }
}


void PID_FUN_Gyro(float target_omega) {
    // 終了フラグ処理


    Gyro.error = target_omega - G_Gyro_Z;// + PID_wall;
    Gyro.delta_error = Gyro.error - Gyro.old_error;
    Gyro.old_error = Gyro.error;
    Gyro.sigma_error += Gyro.error;

    if (Gyro.sigma_error  >1000.0f)  Gyro.sigma_error  =100.0f;
    if (Gyro.sigma_error  < -1000.0f) Gyro.sigma_error  = -1000.0f;
        PID_turn = Kp_FUN_gyro * Gyro.error + Ki_FUN_gyro * Gyro.sigma_error + Kd_FUN_gyro * Gyro.delta_error;


    if (omega_end_flag == 1||PID_gyro_reset_flag==1) {
        Gyro.sigma_error = 0;
    	PID_gyro_reset_flag=0;
    	omega_end_flag=0;
    }

  //  log_data(PID_wall,G_Gyro_Z,Gyro.sigma_error ,Gv,(Measure_Gv_R+ Measure_Gv_L)*1000 / 2.0f);
}




void PID_FUN_SLALOM_Gyro(float target_omega) {
    Gyro.error = target_omega - G_Gyro_Z;
    Gyro.delta_error = Gyro.error - Gyro.old_error;
    Gyro.old_error = Gyro.error;
    Gyro.sigma_error += Gyro.error;
    PID_turn = Ksp_FUN_gyro * Gyro.error + Ksi_FUN_gyro * Gyro.sigma_error + Ksd_FUN_gyro * Gyro.delta_error;
    if ((omega_end_flag == 1 || PID_gyro_reset_flag == 1) && (G_Gyro_Z <= 20 && G_Gyro_Z >= -20)) {
    	Gyro.sigma_error=0;
    	Gyro.delta_error=0;
    	 Gyro.error=0;
    	    Gyro.old_error=0;
    	PID_gyro_reset_flag=0;
    	omega_end_flag=0;
    }



}



void PID_FUN_TURN_Gyro(float target_omega) {
    if(omega_end_flag==1){
    	Gyro.sigma_error=0;
    	//Enc.sigma_error=0;

    	omega_end_flag=0;
    }
    Gyro.error = target_omega - G_Gyro_Z;
    Gyro.delta_error = Gyro.error - Gyro.old_error;
    Gyro.old_error = Gyro.error;
    Gyro.sigma_error += Gyro.error;
    PID_turn = Ktp_FUN_gyro * Gyro.error + Kti_FUN_gyro * Gyro.sigma_error + Ktd_FUN_gyro * Gyro.delta_error;



}



void PID_FUN_tansaku_Enc(float target_v) {
    Enc.error = target_v - (Measure_Gv_R+ Measure_Gv_L)*1000 / 2.0f;
    Enc.delta_error = Enc.error - Enc.old_error;
    Enc.old_error = Enc.error;
    Enc.sigma_error += Enc.error;
    PID_stra = Kp_FUN_tansaku_enc * Enc.error + Ki_FUN_tansaku_enc * Enc.sigma_error + Kd_FUN_tansaku_enc * Enc.delta_error;

    // 終了フラグ処理
    if (v_end_flag == 1) {
    	Enc.delta_error =0;
        Enc.sigma_error = 0;
        v_end_flag = 0;
    }
}

void PID_Turn_FUN_tansaku_Enc(float target_v) {
    Enc.error = target_v - (Measure_Gv_R+ Measure_Gv_L)*1000 / 2.0f;
    Enc.delta_error = Enc.error - Enc.old_error;
    Enc.old_error = Enc.error;
    Enc.sigma_error += Enc.error;
    PID_stra = Ktp_FUN_tansaku_enc * Enc.error + Kti_FUN_tansaku_enc * Enc.sigma_error + Ktd_FUN_tansaku_enc * Enc.delta_error;

    if (v_end_flag == 1) {
    	Enc.delta_error =0;
        Enc.sigma_error = 0;
        v_end_flag = 0;
    }

//	  log_data(Gomega,G_Gyro_Z,PID_wall,Gv,(Measure_Gv_R+ Measure_Gv_L)*1000 / 2.0f);
}

void PID_FUN_tansaku_Gyro(float target_omega) {



    Gyro.error = target_omega - G_Gyro_Z;// + PID_wall;
    Gyro.delta_error = Gyro.error - Gyro.old_error;
    Gyro.old_error = Gyro.error;
    Gyro.sigma_error += Gyro.error;
        PID_turn = Kp_FUN_tansaku_gyro * Gyro.error + Ki_FUN_tansaku_gyro * Gyro.sigma_error + Kd_FUN_tansaku_gyro * Gyro.delta_error;


    if (omega_end_flag == 1||PID_gyro_reset_flag==1) {
        Gyro.sigma_error = 0;
    	PID_gyro_reset_flag=0;
    	omega_end_flag=0;
    }


}




void PID_FUN_SLALOM_tansaku_Gyro(float target_omega) {
    Gyro.error = target_omega - G_Gyro_Z;
    Gyro.delta_error = Gyro.error - Gyro.old_error;
    Gyro.old_error = Gyro.error;
    Gyro.sigma_error += Gyro.error;
    PID_turn = Ksp_FUN_tansaku_gyro * Gyro.error + Ksi_FUN_tansaku_gyro * Gyro.sigma_error + Ksd_FUN_tansaku_gyro * Gyro.delta_error;

    if(omega_end_flag==1||PID_gyro_reset_flag==1){
    	Gyro.sigma_error=0;
    	Gyro.delta_error=0;
    	 Gyro.error=0;
    	    Gyro.old_error=0;
    	//Enc.sigma_error=0;
    	PID_gyro_reset_flag=0;
    	omega_end_flag=0;
    }


}



void PID_FUN_TURN_tansaku_Gyro(float target_omega) {
    if(omega_end_flag==1){
    	Gyro.sigma_error=0;
    	//Enc.sigma_error=0;

    	omega_end_flag=0;
    }
    Gyro.error = target_omega - G_Gyro_Z;
    Gyro.delta_error = Gyro.error - Gyro.old_error;
    Gyro.old_error = Gyro.error;
    Gyro.sigma_error += Gyro.error;
    PID_turn = Ktp_FUN_tansaku_gyro * Gyro.error + Kti_FUN_tansaku_gyro * Gyro.sigma_error + Ktd_FUN_tansaku_gyro * Gyro.delta_error;



}
