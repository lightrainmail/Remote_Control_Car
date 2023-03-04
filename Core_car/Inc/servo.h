//
// Created by Lightrain on 2023/3/1.
//舵机控制线（橙色）接stm32TIM2_CH1(PA0)通道输出的PWM波
//其中，舵机对PWM波的周期要求不是很高，但对高电平持续时间要求很高
//网上买的单片机输出PWM波形的周期是19.95ms，在计算占空比时要带入19.95ms,而不是20ms来计算
//

#ifndef MXPROJECT_SERVO_H
#define MXPROJECT_SERVO_H

#include "main.h"
#include "tim.h"

void servo_init(void);
void servo_setangle(uint16_t ccr);

#endif //MXPROJECT_SERVO_H
