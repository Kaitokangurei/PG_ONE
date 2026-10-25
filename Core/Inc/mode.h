/*
 * mode.h
 *
 *  Created on: Dec 14, 2024
 *      Author: suzuk
 */

#ifndef INC_MODE_H_
#define INC_MODE_H_

typedef struct {
    int main_mode;  // メインモード
    int sub_mode;   // サブモード
    int adjust_mode; //調整
} Mode;


void ModeSelection(void);
void UpdateLEDs(int mainMode, int subMode) ;
void ExecuteMode(Mode currentMode);

void RESET_mode();
void prepare_for_saitan_run(void);
void execute_saitan_sequence(int* , int);
void checkMotorStop();
extern uint8_t print_flag;

extern int sub_mode_num;

#endif /* INC_MODE_H_ */
