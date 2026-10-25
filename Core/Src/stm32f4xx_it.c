/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "lsm6dsr.h"
#include "led_control.h"
#include "PL_timer.h"
#include "PL_sensor.h"
#include "MOTOR.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"
#include "adc.h"

#include "MOTOR.h"
#include "PL_encoder.h"
#include "PID.h"
#include "define.h"
//#include "wall.h

#include "log.h"
#include "math.h"
/*
#include "adc.h"*/

#include "stdio.h"

#include "mode.h"
#include "SPEAKER.h"
#include "wall.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim6;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles ADC1, ADC2 and ADC3 interrupts.
  */
void ADC_IRQHandler(void)
{
  /* USER CODE BEGIN ADC_IRQn 0 */

  /* USER CODE END ADC_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc1);
  /* USER CODE BEGIN ADC_IRQn 1 */

  /* USER CODE END ADC_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt and DAC1, DAC2 underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  Gv += Ga * 0.001;
  Gt += 0.001;
  Gx += Gv * 0.001;
  Gomega += Gomega_acc * 0.001f;
  Gdegree += Gomega * 0.001f;

  pl_timer_count();
  pl_interupt_getSensor();
  get_wall() ;
  wall_control();
  lsm6dsr_get();


  Motor_Task();

 FUN_control();
 get_sensor_smoothed_diff();
 wall_kabegire_control();

 SPEAKER_Update_1ms();

 if(motor_stop_flag == 0){
     if (check_angle_error_condition(Gdegree,  Measure_degree )) {
    	 LED_All_Off();
         LED_Right_On();
         failsafe_flag = 1;
     }

     if(check_lifted_condition(G_Accel_Y,G_Gyro_Z)) {
    	 LED_All_Off();
         LED_Left_On();
         failsafe_flag = 1;
     }


     if (check_velocity_error_condition(Gv, Measure_Gv_R, Measure_Gv_L)) {
    	 LED_All_Off();
    	 LED_All_On();
              failsafe_flag = 1;
          }

     if (failsafe_flag == 1) {
         //LED_All_On();
    	 //SPEAKER_PlaySE(SE_ERROR);
         Motor_PWM_Stop();
         __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
     } else {
         // 通常のモーター制御処理など
     }
 }


/*  Measure_Gv_R_sum+=Measure_Gv_R;
  Measure_Gv_L_sum+=Measure_Gv_L;*/
  Measure_degree += G_Gyro_Z*0.001f;

  if(log_flag==1 &&failsafe_flag != 1){//&&motor_stop_flag != 1){
 //log_data(duty_ratio_L,duty_ratio_R,Gv,Measure_Gv_L*1000,Measure_Gv_R*1000);
//	  log_data(Gomega,g_sensor[0][0],g_sensor[3][0],Gv,(Measure_Gv_R+ Measure_Gv_L)*1000 / 2.0f);
//	  log_data(Gomega,//g_sensor[1][0],g_sensors_smoothed_diff[1],g_sensor[2][0],g_sensors_smoothed_diff[2]);
//	   log_data(Gomega,G_Gyro_Z, Measure_degree,Gv,(Measure_Gv_R+ Measure_Gv_L)*1000 / 2.0f);
//	   log_data(Gomega,G_Gyro_Z,   Measure_degree ,g_sensor[2][0],(Measure_Gv_R+ Measure_Gv_L)*1000 / 2.0f);
	//  log_data(Gomega, Gv,g_sensor[0][0],g_sensor[3][0],G_Gyro_Z);

	//  log_data(G_Gyro_Z,Gv,Measure_Gv_L*1000,Measure_Gv_R*1000,(Measure_Gv_R+ Measure_Gv_L)*1000 / 2.0f);

	 log_data(PID_wall,g_sensor[1][0],g_sensor[2][0],Gv,(Measure_Gv_R+ Measure_Gv_L)*1000 / 2.0f)	;

	/*  log_data_11(duty_ratio_L,duty_ratio_R,g_sensors_smoothed_diff[1],g_sensors_smoothed_diff[2],PID_wall,g_sensor[1][0],g_sensor[2][0],Gomega,G_Gyro_Z,Gv,(Measure_Gv_R+ Measure_Gv_L)*1000 / 2.0f);
*/

	 // log_data_11(duty_ratio_L,duty_ratio_R,PID_wall,g_sensor[0][0],g_sensor[1][0],g_sensor[2][0],g_sensor[3][0],Gomega,G_Gyro_Z,Gv,(Measure_Gv_R+ Measure_Gv_L)*1000 / 2.0f);

	//   log_data(Gomega,g_sensor[1][0],g_sensor[2][0],Gv,(Measure_Gv_R+ Measure_Gv_L)*1000 / 2.0f);
//log_data(Measure_Gv_L*1000,Measure_Gv_R*1000,(Measure_Gv_R+ Measure_Gv_L)*1000 / 2.0f,Gv,PID_wall);
//	  log_data(G_Gyro_Z,Measure_Gv_L*1000,Measure_Gv_R*1000,g_sensor[0][0],g_sensor[3][0])	;
//	  log_data(G_Gyro_Z,Measure_Gv_L*1000,Measure_Gv_R*1000,Gv,(Measure_Gv_R+ Measure_Gv_L)*1000 / 2.0f)	;


 	 // log_data(Gomega,G_Gyro_Z,PID_wall,Gv,(Measure_Gv_R+ Measure_Gv_L)*1000 / 2.0f)	;
// 	  log_data(duty_ratio_L,G_Gyro_Z,PID_wall,Gv,(Measure_Gv_R+ Measure_Gv_L)*1000 / 2.0f)	;


//  	  log_data(Gomega,g_sensor[1][0],g_sensor[2][0],Gv,(Measure_Gv_R+ Measure_Gv_L)*1000 / 2.0f)	;
//	  log_data(g_sensor[0][0], g_sensor[1][0], g_sensor[2][0], g_sensor[3][0],(Measure_Gv_R+ Measure_Gv_L)*1000 / 2.0f);//PID_wall);
//	log_data(g_sensor[1][0], g_sensor[2][0],g_sensor_diff[1],g_sensor_diff[2],Gx);//PID_wall);
//  	  //log_data_2(encoder_L, encoder_R);
 // 	 log_data_2(Gv,(Measure_Gv_R+ Measure_Gv_L)*1000 / 2.0f);
  }



  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream0 global interrupt.
  */
void DMA2_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */

  /* USER CODE END DMA2_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */

  /* USER CODE END DMA2_Stream0_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
