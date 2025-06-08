/**
 ******************************************************************************
 * @file           : ws2812.c
 * @brief          : Ws2812 library source
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 - 2025 Lars Boegild Thomsen <lbthomsen@gmail.com>
 * All rights reserved
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

/**
 * Notice, a timer with a DMA driven PWM output will need to be configured
 * before this library is initialized.
 */

#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include "main.h"
#include "ws2812.h"


/*
 * Update next 24 bits in the dma buffer - assume dma_buffer_pointer is pointing
 * to the buffer that is safe to update.  The dma_buffer_pointer and the call to
 * this function is handled by the dma callbacks.
 */
// Khai báo biến, kích thước dữ liệu là 24* số đèn LED
uint16_t WS2812_RGB_Buff[LED_NUM * DATA_LEN + WS2812_RST_NUM] = {0};

/*
 * Định nghĩa: WS2812 sử dụng giao thức 24-bit để điều khiển mỗi đèn LED
 * Sử dụng PWM để điều khiển từng bit
 * Tần số PWM là 800kHz
 * Độ rộng xung là 0-1250ns (0), 625ns (1)
 */

void WS2812_Set(uint16_t t_num, uint8_t R, uint8_t G, uint8_t B)
{
    uint32_t t_indexx = (t_num * 3);
    for (uint8_t i = 0; i < 8; i++)
    {
        // Gửi dữ liệu
        WS2812_RGB_Buff[t_indexx + i] = ((G << i) & (0x80)) ? WS_H : WS_L;
        WS2812_RGB_Buff[t_indexx + 8 + i] = ((R << i) & (0x80)) ? WS_H : WS_L;
        WS2812_RGB_Buff[t_indexx + 16 + i] = ((B << i) & (0x80)) ? WS_H : WS_L;
    }
}

/* WS2812 Khởi tạo */
void WS2812_Init()
{
    // Cấu hình PWM
    for(int i=0;i<8;i++)
    {
        WS2812_Set(i, 0, 20, 0);
    }

/* Chuyển HAL_TIM_PWM_Start_DMA */

}
