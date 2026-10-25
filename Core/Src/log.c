/*
 * log.c
 *
 *  Created on: Jun 30, 2025
 *      Author: suzuk
 */

#include "log.h"
#include <stdio.h>

uint32_t log_index = 0;
uint8_t log_flag = 0;

// 11種類 × サンプル数の2次元バッファ [チャンネル番号 0~10][インデックス]
static float log_buf[LOG_CHANNELS_11][LOG_BUFFER_SIZE];

void log_init(void) {
    log_index = 0;
    log_flag = 0;
    for (uint8_t ch = 0; ch < LOG_CHANNELS_11; ch++) {
        for (uint32_t i = 0; i < LOG_BUFFER_SIZE; i++) {
            log_buf[ch][i] = 0.0f;
        }
    }
}

// ------------------------------------------------------------------
// 11チャンネル 記録・出力
// ------------------------------------------------------------------
void log_data_11(float d1, float d2, float d3, float d4, float d5,
                 float d6, float d7, float d8, float d9, float d10, float d11) {
    if (log_flag == 0) {
        return;
    }

    if (log_index < LOG_BUFFER_SIZE) {
        log_buf[0][log_index] = d1;
        log_buf[1][log_index] = d2;
        log_buf[2][log_index] = d3;
        log_buf[3][log_index] = d4;
        log_buf[4][log_index] = d5;
        log_buf[5][log_index] = d6;
        log_buf[6][log_index] = d7;
        log_buf[7][log_index] = d8;
        log_buf[8][log_index] = d9;
        log_buf[9][log_index] = d10;
        log_buf[10][log_index] = d11;
        log_index++;
    } else {
        log_flag = 0;
    }
}

void log_data_array(const float data[LOG_CHANNELS_11]) {
    if (log_flag == 0) {
        return;
    }

    if (log_index < LOG_BUFFER_SIZE) {
        for (uint8_t ch = 0; ch < LOG_CHANNELS_11; ch++) {
            log_buf[ch][log_index] = data[ch];
        }
        log_index++;
    } else {
        log_flag = 0;
    }
}

void log_print_11(void) {
    log_flag = 0;

    printf("--- Log Data Start (11ch) ---\n\r");
    printf("index,d1,d2,d3,d4,d5,d6,d7,d8,d9,d10,d11\n\r"); // d11を追加

    for (uint32_t i = 0; i < log_index; i++) {
        printf("%lu, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n\r",
               (unsigned long)i,
               log_buf[0][i], log_buf[1][i], log_buf[2][i], log_buf[3][i], log_buf[4][i],
               log_buf[5][i], log_buf[6][i], log_buf[7][i], log_buf[8][i], log_buf[9][i],
               log_buf[10][i]);
    }
    printf("--- Log Data End ---\n\r");
}

// ------------------------------------------------------------------
// 既存互換用: 5チャンネル（0〜4番スロットを使用）
// ------------------------------------------------------------------
void log_data(float data1, float data2, float data3, float data4, float data5) {
    if (log_flag == 0) {
        return;
    }

    if (log_index < LOG_BUFFER_SIZE) {
        log_buf[0][log_index] = data1;
        log_buf[1][log_index] = data2;
        log_buf[2][log_index] = data3;
        log_buf[3][log_index] = data4;
        log_buf[4][log_index] = data5;
        log_index++;
    } else {
        log_flag = 0;
    }
}

void log_print(void) {
    log_flag = 0;

    printf("--- Log Data Start (5ch) ---\n\r");
    for (uint32_t i = 0; i < log_index; i++) {
        printf("%lu, %f, %f, %f, %f, %f\n\r",
               (unsigned long)i,
               log_buf[0][i], log_buf[1][i], log_buf[2][i], log_buf[3][i], log_buf[4][i]);
    }
    printf("--- Log Data End ---\n\r");
}

// ------------------------------------------------------------------
// 既存互換用: 2チャンネル（0〜1番スロットを使用）
// ------------------------------------------------------------------
void log_data_2(float data1, float data2) {
    if (log_flag == 0) {
        return;
    }

    if (log_index < LOG_BUFFER_SIZE) {
        log_buf[0][log_index] = data1;
        log_buf[1][log_index] = data2;
        log_index++;
    } else {
        log_flag = 0;
    }
}

void log_print_2(void) {
    log_flag = 0;

    printf("--- Log Data Start (2ch) ---\n\r");
    for (uint32_t i = 0; i < log_index; i++) {
        printf("%lu, %f, %f\n\r", (unsigned long)i, log_buf[0][i], log_buf[1][i]);
    }
    printf("--- Log Data End ---\n\r");
}
