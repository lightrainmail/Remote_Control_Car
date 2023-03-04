//
// Created by Lightrain on 2023/3/1.
//这个代码用于驱动电机
//两个电机我采用两路PWM来独立驱动，PWM频率是100KHz
//TIM3用于控制右侧电机，TIM4用于控制左侧电机
//其他硬件连接：
//单片机----------电机驱动
//PA6------------ENA
//PA5------------IN2
//PA4------------IN1
//PB6------------ENB
//PB7------------IN4
//PB8------------IN3
//
//
#include "motor.h"


//这个函数用于启动控制两个电机的两个定时器
void motor_init(void){
    HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
}

//这个函数用于控制小车前进和其前进速度，speed取值范围[0,720],值越大速度越快.
//speedA控制右侧电机转速;speedB用于控制左侧电机转速
void motor_goahead(uint16_t speedA,uint16_t speedB){

    //控制输出PWM波的占空比来调速
    __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,speedA);
    __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,speedB);

    //控制右侧电机正转
    HAL_GPIO_WritePin(IN1_GPIO_Port,IN1_Pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN2_GPIO_Port,IN2_Pin,GPIO_PIN_SET);

    //控制左侧电机正转
    HAL_GPIO_WritePin(IN3_GPIO_Port,IN3_Pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN4_GPIO_Port,IN4_Pin,GPIO_PIN_SET);
}


//控制小车后退函数
void motor_stepback(uint16_t speedA,uint16_t speedB){

    //控制输出PWM波的占空比来调速
    __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,speedA);
    __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,speedB);

    //控制右侧电机反转
    HAL_GPIO_WritePin(IN1_GPIO_Port,IN1_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(IN2_GPIO_Port,IN2_Pin,GPIO_PIN_RESET);

    //控制左侧电机反转
    HAL_GPIO_WritePin(IN3_GPIO_Port,IN3_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(IN4_GPIO_Port,IN4_Pin,GPIO_PIN_RESET);
}

//小车电动刹车，这是电机驱动自带功能
//参数barke用来衡量刹车的轻重，取值范围在[0,720],值越大，刹车越重
void motor_barke(uint16_t barkeA,uint16_t barkeB){
    //一个周期内刹车时间占整个周期的分量等效刹车轻重
    __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,barkeA);
    __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,barkeB);

    //控制右侧电机刹车
    HAL_GPIO_WritePin(IN1_GPIO_Port,IN1_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(IN2_GPIO_Port,IN2_Pin,GPIO_PIN_SET);

    //控制左侧电机刹车
    HAL_GPIO_WritePin(IN3_GPIO_Port,IN3_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(IN4_GPIO_Port,IN4_Pin,GPIO_PIN_SET);
}