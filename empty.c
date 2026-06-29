/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ti_msp_dl_config.h"
#include "usart.h"
#include "grayscale.h"
#include "motor.h"
#include "pid.h"
#include "delay.h"

int main(void)
{
    /* 系统 & 外设初始化 System & peripheral initialization */
    SYSCFG_DL_init();
    USART_Init();
    Motor_Init();

    /*
     * PID 循迹控制器初始化
     * Kp=1.0: 比例增益 — 响应快慢
     * Ki=0.0: 积分增益 — 0=暂不启用 (弯道时再开启)
     * Kd=0.0: 微分增益 — 抑制震荡
     * base_speed=40: 基础速度 40%
     */
    PID_Init(1.2f, 0.0f, 0.3f, 35.0f);

    /*
     * 可选: 通过串口输出调参/调试信息
     * 连接串口助手 (115200 baud) 可观察滤波位置和PID输出
     */
    printf("PID Line-Following Start\r\n");

    while (1) {
        /* 读取12路灰度传感器 (16-bit bitmap) */
        uint16_t sensor_values = Grayscale_ReadAll();

        /* PID 循迹更新: 传感器→加权位置→EMA滤波→PID→差速电机 */
        PID_Update(sensor_values);

        /*
         * 调试输出 (每100次循环 ≈ 每500ms 打印一次)
         * 可观察: 滤波位置, PID输出, 是否丢线
         */
        static uint16_t debug_cnt = 0;
        if (++debug_cnt >= 100) {
            debug_cnt = 0;
            printf("Pos:%.2f Out:%.2f Err:%.2f Int:%.2f Lost:%d\r\n",
                   PID_GetFilteredPosition(),
                   PID_GetOutput(),
                   PID_GetError(),
                   PID_GetIntegral(),
                   PID_IsLineLost());
        }

        /* 控制周期 ~5ms (200Hz), 由 delay_us 提供 */
        delay_us(5000);
    }
}
