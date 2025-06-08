/**
 ******************************************************************************
 * @file           : ws2812_demos.h
 * @brief          : Ws2812b demos source
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 Lars Boegild Thomsen <lbthomsen@gmail.com>.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include "main.h"
#include "ws2812.h"
#include "ws2812_demos.h"


uint8_t active_demo = 0;

const uint8_t led_line_colors[][3] = {
        { 10, 0, 0 },
        { 0, 10, 0 },
        { 0, 0, 10 },
        { 10, 10, 0 },
        { 0, 10, 10 },
        { 10, 0, 10 },
        { 10, 10, 10 }
};

void ws2812_demos_set(ws2812_handleTypeDef *ws2812, uint8_t demo) {
    active_demo = demo;
}

void ws2812_demos_tick(ws2812_handleTypeDef *ws2812) {};



void ws2812_demo_task(ws2812_handleTypeDef *ws2812, uint32_t now, uint32_t *next_led, uint8_t *ledMatrix) {
    static uint32_t active_demo = WS2812_DEMO_SPECTRUM;
    static const uint32_t led_interval = 50; // 50ms

    switch (active_demo) {
    case WS2812_DEMO_SPECTRUM:
        if (now >= *next_led) {
            // Đặt tất cả LED về 0 trước khi cập nhật
            zeroLedValues(ws2812);

            // Ánh xạ ledMatrix thành ma trận 8x8 với màu xanh lá
            for (int col = 0; col < 8; col++) {
                int numLedsOn = ledMatrix[col]; // Số LED sáng trong cột (0-8)
                if (numLedsOn > 8) numLedsOn = 8;

                for (int row = 0; row < numLedsOn; row++) {
                    // Mapping theo dạng "row * 8 + col" (theo hàng)
                    int ledIndex = row * 8 + col;

                    // Tính độ sáng theo chiều cao cột (tùy chọn)
                    uint8_t brightness = (uint8_t)(255.0f * ((float)numLedsOn / 8.0f));

                    // Gán màu xanh lá cây
                    setLedValues(ws2812, ledIndex, 0, brightness, 0); // R=0, G=brightness, B=0
                }
            }

            // Đánh dấu buffer DMA cần cập nhật
            ws2812->is_dirty = 1;

            // Cập nhật thời gian cho lần lặp tiếp theo
            *next_led = now + led_interval;
        }
        break;
        default:
            // Không làm gì nếu không phải demo spectrum
            break;
    }
}

