//
// Created by Lightrain on 2023/3/1.
//
#include "servo.h"


//启动舵机，我把它叫舵机初始化代码
void servo_init(void){
    HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
}

//为了不损失精度，就不用中间变量角度，直接传入16位的ccr值.取值范围[1000,2000]对应角度[0,pi]
void servo_setangle(uint16_t ccr){
    __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,ccr);
}
