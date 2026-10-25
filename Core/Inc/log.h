/*
 * log.h
 *
 *  Created on: Jun 30, 2025
 *      Author: suzuk
 */

#ifndef INC_LOG_H_
#define INC_LOG_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define LOG_BUFFER_SIZE   5000 // ログの最大サンプル数（RAM容量に応じて調整）
#define LOG_CHANNELS_11   5   // ログの種類数を 11 に変更

extern uint32_t log_index;
extern uint8_t log_flag;

// 初期化
void log_init(void);

// 11チャンネル用
void log_data_11(float d1, float d2, float d3, float d4, float d5,
                 float d6, float d7, float d8, float d9, float d10,
                 float d11);
void log_data_array(const float data[LOG_CHANNELS_11]);
void log_print_11(void);

// 互換用（5チャンネル / 2チャンネル）
void log_data(float data1, float data2, float data3, float data4, float data5);
void log_print(void);

void log_data_2(float data1, float data2);
void log_print_2(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_LOG_H_ */
