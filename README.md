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
### 用于使小车转向的舵机
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

```
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

