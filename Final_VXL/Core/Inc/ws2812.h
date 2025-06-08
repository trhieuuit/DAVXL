/**
 ******************************************************************************
 * @file           : ws2812.h
 * @brief          : Ws2812 library header
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 - 2025 Lars Boegild Thomsen <lbthomsen@gmail.com>
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#ifndef __WS2812_H
#define __WS2812_H

#include "main.h"

#define WS_H       60    // 1 tương ứng với độ rộng xung là 625ns
#define WS_L       29    // 0 tương ứng với độ rộng xung là 375ns
#define WS_REST    49    // Thời gian reset là 50us
#define LED_NUM    64     // Số lượng LED WS2812
#define DATA_LEN   24    // WS2812 sử dụng 24-bit để điều khiển
#define WS2812_RST_NUM 50 // Thời gian reset là 50us

extern uint16_t WS2812_RGB_Buff[LED_NUM * DATA_LEN + WS2812_RST_NUM];


void WS2812_Init(void);
void WS2812_Set(uint16_t t_num, uint8_t R, uint8_t G, uint8_t B);
#endif // _WS2812_H
/*
 * vim: ts=4 nowrap
 */
