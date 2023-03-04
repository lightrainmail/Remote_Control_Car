# Remote_Control_Car
就是一个小玩具，遥控车。
![IMG_20230304_010733](https://user-images.githubusercontent.com/89727667/222783478-8eef1f79-2593-46db-a06d-619b3e284f1e.jpg)

## 功能说明
他有那些功能呢。

他可以前进，后退，左转右转，而且每一个电机都有一个独立TIM定时器输出的PWM控制。小车的速度是可以调节的，转弯的角度是可以调节的。本质上就是一个玩具小车，只不过升级了些。

## 硬件部分
### 遥控器部分
stm32f103c8t6最小系统板+摇杆模块+NRF24L01模块
![IMG_20230304_012459](https://user-images.githubusercontent.com/89727667/222787181-b207340d-6486-45ab-a252-3dae144af038.jpg)

### 小车部分
stm32f103c8t6最小系统板+L298N电机驱动模块+两个稳压模块+两个电机+一个舵机+NRF24L01+两节18650电池+亚克力板车架

### 注意
stm32f103c8t6这款芯片最高主频是72MHz,我使用的频率是72MHz。

所有这些都是在淘宝上买的

## 软件部分
### 小车部分
#### 用于使小车转向的舵机
这个小车前面转向舵机型号是MG995，工作电压在5-7.2V。

转动角度和输入信号的关系参考知乎一个人的图
![v2-53bc14d137a7a06b963c3ab0220c73ef_r](https://user-images.githubusercontent.com/89727667/222798974-6f44f906-3bbf-42b6-a9b7-4b99ee3cd933.jpg)

我使用定时器TIM2的CH1通道输出PWM来控制舵机
![image](https://user-images.githubusercontent.com/89727667/222799235-610a9e67-8952-45bf-9038-bb697069efde.png)

设置PSC为72-1，ARR为20000-1，Pulse（即为CCR）为1500，其他默认。

接下来就可以直接写代码了

servo.h文件

```c
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
```

servo.c文件

```c
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

```
#### 用于使小车前进的电机控制
我使用了一个电机驱动，电机驱动上有6个输入口：
![IMG_20230305_030056](https://user-images.githubusercontent.com/89727667/222924050-3d2d8c43-4181-4744-af43-569d1f8ccf3d.jpg)
其中，ENA接单片机定时器TIM3的PWM输出通道CH1（PA6）用于控制右侧电机转速
TIM3设置：
![image](https://user-images.githubusercontent.com/89727667/222924297-07e6bd8c-4d69-438f-96d0-0c0ea7c5d3f6.png)


ENB接单片机定时器TIM4的PWM输出通道CH1（PB6）用于控制左侧电机转速
TIM4设置：
![image](https://user-images.githubusercontent.com/89727667/222924318-cad93d94-12e3-406d-b759-a9431495baf3.png)


IN1----PA4
IN2----PA5
IN1、IN2共同控制右侧电机正反转

IN3----PB8
IN4----PB7
IN3、IN4共同控制左侧点击正反转

IN1、IN2、IN3、IN4设置都一样：
![image](https://user-images.githubusercontent.com/89727667/222924360-627000cf-8acb-41fa-ba5e-d2f5432e45a2.png)

接下来就可以写代码了：

motor.h
```c
//
// Created by Lightrain on 2023/3/1.
//

#ifndef MXPROJECT_MOTOR_H
#define MXPROJECT_MOTOR_H

#include "main.h"
#include "tim.h"

void motor_init(void);
void motor_goahead(uint16_t speedA,uint16_t speedB);
void motor_stepback(uint16_t speedA,uint16_t speedB);
void motor_barke(uint16_t barkeA,uint16_t barkeB);

#endif //MXPROJECT_MOTOR_H
```

motor.c
```c
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
```






