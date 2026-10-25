/*
 * led_control.c
 *
 *  Created on: Jun 21, 2025
 *      Author: suzuk
 */

#include "main.h"
#include "led_control.h"
#include "PL_timer.h"

// LEDのGPIOポートとピンを配列で管理（LED1〜10）
GPIO_TypeDef* LED_GPIO_PORTS[] = {
    LED1_GPIO_Port, LED2_GPIO_Port, LED3_GPIO_Port,
    LED4_GPIO_Port, LED5_GPIO_Port, LED6_GPIO_Port,
    LED7_GPIO_Port, LED8_GPIO_Port, LED9_GPIO_Port,
	LED10_GPIO_Port
};

uint16_t LED_GPIO_PINS[] = {
    LED1_Pin, LED2_Pin, LED3_Pin,
    LED4_Pin, LED5_Pin, LED6_Pin,
    LED7_Pin, LED8_Pin, LED9_Pin,
	LED10_Pin
};

// 範囲チェック付きのON/OFF/TOGGLE関数
void LED_On(uint8_t n) {
    if (n >= 1 && n <= 10)
        HAL_GPIO_WritePin(LED_GPIO_PORTS[n-1], LED_GPIO_PINS[n-1], GPIO_PIN_SET); // アクティブLow
}

void LED_Off(uint8_t n) {
    if (n >= 1 && n <= 10)
        HAL_GPIO_WritePin(LED_GPIO_PORTS[n-1], LED_GPIO_PINS[n-1], GPIO_PIN_RESET); // アクティブLow
}

void LED_Left_On() {
    for (uint8_t i = 1; i <= 3; i++) {
        LED_On(i);
    }
}

void LED_Left_Off() {
    for (uint8_t i = 1; i <= 3; i++) {
        LED_Off(i);
    }
}

void LED_Right_On() {
    for (uint8_t i = 4; i <=6; i++) {
        LED_On(i);
    }
}

void LED_Right_Off() {
    for (uint8_t i = 4; i <=6; i++) {
        LED_Off(i);
    }
}

void LED_Toggle(uint8_t n) {
    if (n >= 1 && n <= 10)
        HAL_GPIO_TogglePin(LED_GPIO_PORTS[n-1], LED_GPIO_PINS[n-1]);
}
void LED_All_On(void) {
    for (uint8_t i = 1; i <= 10; i++) {
        LED_On(i);
    }
}
void LED_All_Off(void) {
    for (uint8_t i = 1; i <= 10; i++) {
        LED_Off(i);
    }
}
void LED_Chase_OneByOne(void) {
    for (uint8_t i = 1; i <=10; i++) {
        // LED ON（アクティブLow）
    	LED_On(i);
       HAL_Delay(100);
        LED_Off(i);
    }
}
static uint8_t current_led_state_for_toggle = 0;
static uint8_t current_led_state_for_toggle_count = 0;
void Toggle_LED5_and_LED4()
{
    if (current_led_state_for_toggle_count<=500) {
        // 現在LED5が点灯している（または初期状態）ので、LED4を点灯させる
        LED_Off(5); // LED5を消す
        LED_On(4);  // LED4を点灯
        current_led_state_for_toggle = 1; // 状態をLED4点灯に更新
        current_led_state_for_toggle_count++;
    } else {
        // 現在LED4が点灯しているので、LED5を点灯させる
        LED_Off(4); // LED4を消す
        LED_On(5);  // LED5を点灯
        current_led_state_for_toggle = 0; // 状態をLED5点灯に更新
        current_led_state_for_toggle_count++;
        if (current_led_state_for_toggle_count>=1000){
        	current_led_state_for_toggle_count=0;
        }
    }
}

//use method
/*
#include "led_control.h"

void some_function(void) {
    LED_On(1);     // LED1をON（消灯なら点灯）
    LED_Off(8);    // LED8をOFF（点灯なら消灯）
    LED_Toggle(3); // LED3の状態を反転
}

*/
