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

#### 控制小车怎么动的无线模块
单片机的SPI2口与无线模块进行通讯，我在这里是用的mokhwasomssi的驱动

SPI2设置：
![image](https://user-images.githubusercontent.com/89727667/222924587-b7b17ecb-a1be-4e9a-b9df-9dde3dc14298.png)

无线模块的使能CE引脚设置:
![image](https://user-images.githubusercontent.com/89727667/222924672-90b4d34a-5a18-4933-aa77-e5d541a520cd.png)

无线模块片选CSN设置:
![image](https://user-images.githubusercontent.com/89727667/222924709-c2647838-2fe1-4d54-958e-0b5087190fd3.png)

无线模块输出中断IRQ设置:
![image](https://user-images.githubusercontent.com/89727667/222924740-41bef19f-d1e6-4841-80d7-e6e30f2d5c33.png)

设置好以后就可以些代码了，其驱动代码包含两个文件:

nrf24l01p.h
```c
  
/*
 *  nrf24l01_plus.h
 *
 *  Created on: 2021. 7. 20.
 *      Author: mokhwasomssi
 * 
 */


#ifndef __NRF24L01P_H__
#define __NRF24L01P_H__


#include "spi.h"    // header from stm32cubemx code generate
#include <stdbool.h>


/* User Configurations */
#define NRF24L01P_SPI                     (&hspi2)

#define NRF24L01P_SPI_CS_PIN_PORT         GPIOB 
#define NRF24L01P_SPI_CS_PIN_NUMBER       GPIO_PIN_11

#define NRF24L01P_CE_PIN_PORT             GPIOB
#define NRF24L01P_CE_PIN_NUMBER           GPIO_PIN_12

#define NRF24L01P_IRQ_PIN_PORT            GPIOA
#define NRF24L01P_IRQ_PIN_NUMBER          GPIO_PIN_8

#define NRF24L01P_PAYLOAD_LENGTH          8     // 1 - 32bytes


/* nRF24L01+ typedefs */
typedef uint8_t count;
typedef uint8_t widths;
typedef uint8_t length;
typedef uint16_t delay;
typedef uint16_t channel;

typedef enum
{
    _250kbps = 2,
    _1Mbps   = 0,
    _2Mbps   = 1
} air_data_rate;

typedef enum
{
    _0dBm  = 3,
    _6dBm  = 2,
    _12dBm = 1,
    _18dBm = 0
} output_power;


/* Main Functions */
void nrf24l01p_rx_init(channel MHz, air_data_rate bps);
void nrf24l01p_tx_init(channel MHz, air_data_rate bps);

void nrf24l01p_rx_receive(uint8_t* rx_payload);
void nrf24l01p_tx_transmit(uint8_t* tx_payload);

// Check tx_ds or max_rt
void nrf24l01p_tx_irq();  


/* Sub Functions */
void nrf24l01p_reset();

void nrf24l01p_prx_mode();
void nrf24l01p_ptx_mode();

void nrf24l01p_power_up();
void nrf24l01p_power_down();

uint8_t nrf24l01p_get_status();
uint8_t nrf24l01p_get_fifo_status();

// Static payload lengths
void nrf24l01p_rx_set_payload_widths(widths bytes);

uint8_t nrf24l01p_read_rx_fifo(uint8_t* rx_payload);
uint8_t nrf24l01p_write_tx_fifo(uint8_t* tx_payload);

void nrf24l01p_flush_rx_fifo();
void nrf24l01p_flush_tx_fifo();

// Clear IRQ pin. Change LOW to HIGH
void nrf24l01p_clear_rx_dr();
void nrf24l01p_clear_tx_ds();
void nrf24l01p_clear_max_rt();

void nrf24l01p_set_rf_channel(channel MHz);
void nrf24l01p_set_rf_tx_output_power(output_power dBm);
void nrf24l01p_set_rf_air_data_rate(air_data_rate bps);

void nrf24l01p_set_crc_length(length bytes);
void nrf24l01p_set_address_widths(widths bytes);
void nrf24l01p_auto_retransmit_count(count cnt);
void nrf24l01p_auto_retransmit_delay(delay us);


/* nRF24L01+ Commands */
#define NRF24L01P_CMD_R_REGISTER                  0b00000000
#define NRF24L01P_CMD_W_REGISTER                  0b00100000
#define NRF24L01P_CMD_R_RX_PAYLOAD                0b01100001
#define NRF24L01P_CMD_W_TX_PAYLOAD                0b10100000
#define NRF24L01P_CMD_FLUSH_TX                    0b11100001
#define NRF24L01P_CMD_FLUSH_RX                    0b11100010
#define NRF24L01P_CMD_REUSE_TX_PL                 0b11100011
#define NRF24L01P_CMD_R_RX_PL_WID                 0b01100000
#define NRF24L01P_CMD_W_ACK_PAYLOAD               0b10101000
#define NRF24L01P_CMD_W_TX_PAYLOAD_NOACK          0b10110000
#define NRF24L01P_CMD_NOP                         0b11111111    

/* nRF24L01+ Registers */
#define NRF24L01P_REG_CONFIG            0x00
#define NRF24L01P_REG_EN_AA             0x01
#define NRF24L01P_REG_EN_RXADDR         0x02
#define NRF24L01P_REG_SETUP_AW          0x03
#define NRF24L01P_REG_SETUP_RETR        0x04
#define NRF24L01P_REG_RF_CH             0x05
#define NRF24L01P_REG_RF_SETUP          0x06
#define NRF24L01P_REG_STATUS            0x07
#define NRF24L01P_REG_OBSERVE_TX        0x08    // Read-Only
#define NRF24L01P_REG_RPD               0x09    // Read-Only
#define NRF24L01P_REG_RX_ADDR_P0        0x0A
#define NRF24L01P_REG_RX_ADDR_P1        0x0B
#define NRF24L01P_REG_RX_ADDR_P2        0x0C
#define NRF24L01P_REG_RX_ADDR_P3        0x0D
#define NRF24L01P_REG_RX_ADDR_P4        0x0E
#define NRF24L01P_REG_RX_ADDR_P5        0x0F
#define NRF24L01P_REG_TX_ADDR           0x10
#define NRF24L01P_REG_RX_PW_P0          0x11
#define NRF24L01P_REG_RX_PW_P1          0x12
#define NRF24L01P_REG_RX_PW_P2          0x13
#define NRF24L01P_REG_RX_PW_P3          0x14
#define NRF24L01P_REG_RX_PW_P4          0x15
#define NRF24L01P_REG_RX_PW_P5          0x16
#define NRF24L01P_REG_FIFO_STATUS       0x17
#define NRF24L01P_REG_DYNPD             0x1C
#define NRF24L01P_REG_FEATURE           0x1D


#endif /* __NRF24L01P_H__ */

```

nrf24l01p.c
```c
/*
 *  nrf24l01_plus.c
 *
 *  Created on: 2021. 7. 20.
 *      Author: mokhwasomssi
 * 
 */


#include "nrf24l01p.h"


static void cs_high()
{
    HAL_GPIO_WritePin(NRF24L01P_SPI_CS_PIN_PORT, NRF24L01P_SPI_CS_PIN_NUMBER, GPIO_PIN_SET);
}

static void cs_low()
{
    HAL_GPIO_WritePin(NRF24L01P_SPI_CS_PIN_PORT, NRF24L01P_SPI_CS_PIN_NUMBER, GPIO_PIN_RESET);
}

static void ce_high()
{
    HAL_GPIO_WritePin(NRF24L01P_CE_PIN_PORT, NRF24L01P_CE_PIN_NUMBER, GPIO_PIN_SET);
}

static void ce_low()
{
    HAL_GPIO_WritePin(NRF24L01P_CE_PIN_PORT, NRF24L01P_CE_PIN_NUMBER, GPIO_PIN_RESET);
}

static uint8_t read_register(uint8_t reg)
{
    uint8_t command = NRF24L01P_CMD_R_REGISTER | reg;
    uint8_t status;
    uint8_t read_val;

    cs_low();
    HAL_SPI_TransmitReceive(NRF24L01P_SPI, &command, &status, 1, 2000);
    HAL_SPI_Receive(NRF24L01P_SPI, &read_val, 1, 2000);
    cs_high();

    return read_val;
}

static uint8_t write_register(uint8_t reg, uint8_t value)
{
    uint8_t command = NRF24L01P_CMD_W_REGISTER | reg;
    uint8_t status;
    uint8_t write_val = value;

    cs_low();
    HAL_SPI_TransmitReceive(NRF24L01P_SPI, &command, &status, 1, 2000);
    HAL_SPI_Transmit(NRF24L01P_SPI, &write_val, 1, 2000);
    cs_high();

    return write_val;
}


/* nRF24L01+ Main Functions */
void nrf24l01p_rx_init(channel MHz, air_data_rate bps)
{
    nrf24l01p_reset();

    nrf24l01p_prx_mode();
    nrf24l01p_power_up();

    nrf24l01p_rx_set_payload_widths(NRF24L01P_PAYLOAD_LENGTH);

    nrf24l01p_set_rf_channel(MHz);
    nrf24l01p_set_rf_air_data_rate(bps);
    nrf24l01p_set_rf_tx_output_power(_0dBm);

    nrf24l01p_set_crc_length(1);
    nrf24l01p_set_address_widths(5);

    nrf24l01p_auto_retransmit_count(3);
    nrf24l01p_auto_retransmit_delay(250);
    
    ce_high();
}

void nrf24l01p_tx_init(channel MHz, air_data_rate bps)
{
    nrf24l01p_reset();

    nrf24l01p_ptx_mode();
    nrf24l01p_power_up();

    nrf24l01p_set_rf_channel(MHz);
    nrf24l01p_set_rf_air_data_rate(bps);
    nrf24l01p_set_rf_tx_output_power(_0dBm);

    nrf24l01p_set_crc_length(1);
    nrf24l01p_set_address_widths(5);

    nrf24l01p_auto_retransmit_count(3);
    nrf24l01p_auto_retransmit_delay(250);

    ce_high();
}

void nrf24l01p_rx_receive(uint8_t* rx_payload)
{
    nrf24l01p_read_rx_fifo(rx_payload);
    nrf24l01p_clear_rx_dr();

    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
}

void nrf24l01p_tx_transmit(uint8_t* tx_payload)
{
    nrf24l01p_write_tx_fifo(tx_payload);
}

void nrf24l01p_tx_irq()
{
    uint8_t tx_ds = nrf24l01p_get_status();
    tx_ds &= 0x20;

    if(tx_ds)
    {   
        // TX_DS
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        nrf24l01p_clear_tx_ds();
    }

    else
    {
        // MAX_RT
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, SET);
        nrf24l01p_clear_max_rt();
    }
}

/* nRF24L01+ Sub Functions */
void nrf24l01p_reset()
{
    // Reset pins
    cs_high();
    ce_low();

    // Reset registers
    write_register(NRF24L01P_REG_CONFIG, 0x08);
    write_register(NRF24L01P_REG_EN_AA, 0x3F);
    write_register(NRF24L01P_REG_EN_RXADDR, 0x03);
    write_register(NRF24L01P_REG_SETUP_AW, 0x03);
    write_register(NRF24L01P_REG_SETUP_RETR, 0x03);
    write_register(NRF24L01P_REG_RF_CH, 0x02);
    write_register(NRF24L01P_REG_RF_SETUP, 0x07);
    write_register(NRF24L01P_REG_STATUS, 0x7E);
    write_register(NRF24L01P_REG_RX_PW_P0, 0x00);
    write_register(NRF24L01P_REG_RX_PW_P0, 0x00);
    write_register(NRF24L01P_REG_RX_PW_P1, 0x00);
    write_register(NRF24L01P_REG_RX_PW_P2, 0x00);
    write_register(NRF24L01P_REG_RX_PW_P3, 0x00);
    write_register(NRF24L01P_REG_RX_PW_P4, 0x00);
    write_register(NRF24L01P_REG_RX_PW_P5, 0x00);
    write_register(NRF24L01P_REG_FIFO_STATUS, 0x11);
    write_register(NRF24L01P_REG_DYNPD, 0x00);
    write_register(NRF24L01P_REG_FEATURE, 0x00);

    // Reset FIFO
    nrf24l01p_flush_rx_fifo();
    nrf24l01p_flush_tx_fifo();
}

void nrf24l01p_prx_mode()
{
    uint8_t new_config = read_register(NRF24L01P_REG_CONFIG);
    new_config |= 1 << 0;

    write_register(NRF24L01P_REG_CONFIG, new_config);
}

void nrf24l01p_ptx_mode()
{
    uint8_t new_config = read_register(NRF24L01P_REG_CONFIG);
    new_config &= 0xFE;

    write_register(NRF24L01P_REG_CONFIG, new_config);
}

uint8_t nrf24l01p_read_rx_fifo(uint8_t* rx_payload)
{
    uint8_t command = NRF24L01P_CMD_R_RX_PAYLOAD;
    uint8_t status;

    cs_low();
    HAL_SPI_TransmitReceive(NRF24L01P_SPI, &command, &status, 1, 2000);
    HAL_SPI_Receive(NRF24L01P_SPI, rx_payload, NRF24L01P_PAYLOAD_LENGTH, 2000);
    cs_high();

    return status;
}

uint8_t nrf24l01p_write_tx_fifo(uint8_t* tx_payload)
{
    uint8_t command = NRF24L01P_CMD_W_TX_PAYLOAD;
    uint8_t status;

    cs_low();
    HAL_SPI_TransmitReceive(NRF24L01P_SPI, &command, &status, 1, 2000);
    HAL_SPI_Transmit(NRF24L01P_SPI, tx_payload, NRF24L01P_PAYLOAD_LENGTH, 2000);
    cs_high(); 

    return status;
}

void nrf24l01p_flush_rx_fifo()
{
    uint8_t command = NRF24L01P_CMD_FLUSH_RX;
    uint8_t status;

    cs_low();
    HAL_SPI_TransmitReceive(NRF24L01P_SPI, &command, &status, 1, 2000);
    cs_high();
}

void nrf24l01p_flush_tx_fifo()
{
    uint8_t command = NRF24L01P_CMD_FLUSH_TX;
    uint8_t status;

    cs_low();
    HAL_SPI_TransmitReceive(NRF24L01P_SPI, &command, &status, 1, 2000);
    cs_high();
}

uint8_t nrf24l01p_get_status()
{
    uint8_t command = NRF24L01P_CMD_NOP;
    uint8_t status;

    cs_low();
    HAL_SPI_TransmitReceive(NRF24L01P_SPI, &command, &status, 1, 2000);
    cs_high(); 

    return status;
}

uint8_t nrf24l01p_get_fifo_status()
{
    return read_register(NRF24L01P_REG_FIFO_STATUS);
}

void nrf24l01p_rx_set_payload_widths(widths bytes)
{
    write_register(NRF24L01P_REG_RX_PW_P0, bytes);
}

void nrf24l01p_clear_rx_dr()
{
    uint8_t new_status = nrf24l01p_get_status();
    new_status |= 0x40;

    write_register(NRF24L01P_REG_STATUS, new_status);
}

void nrf24l01p_clear_tx_ds()
{
    uint8_t new_status = nrf24l01p_get_status();
    new_status |= 0x20;

    write_register(NRF24L01P_REG_STATUS, new_status);     
}

void nrf24l01p_clear_max_rt()
{
    uint8_t new_status = nrf24l01p_get_status();
    new_status |= 0x10;

    write_register(NRF24L01P_REG_STATUS, new_status); 
}

void nrf24l01p_power_up()
{
    uint8_t new_config = read_register(NRF24L01P_REG_CONFIG);
    new_config |= 1 << 1;

    write_register(NRF24L01P_REG_CONFIG, new_config);
}

void nrf24l01p_power_down()
{
    uint8_t new_config = read_register(NRF24L01P_REG_CONFIG);
    new_config &= 0xFD;

    write_register(NRF24L01P_REG_CONFIG, new_config);
}

void nrf24l01p_set_crc_length(length bytes)
{
    uint8_t new_config = read_register(NRF24L01P_REG_CONFIG);
    
    switch(bytes)
    {
        // CRCO bit in CONFIG resiger set 0
        case 1:
            new_config &= 0xFB;
            break;
        // CRCO bit in CONFIG resiger set 1
        case 2:
            new_config |= 1 << 2;
            break;
    }

    write_register(NRF24L01P_REG_CONFIG, new_config);
}

void nrf24l01p_set_address_widths(widths bytes)
{
    write_register(NRF24L01P_REG_SETUP_AW, bytes - 2);
}

void nrf24l01p_auto_retransmit_count(count cnt)
{
    uint8_t new_setup_retr = read_register(NRF24L01P_REG_SETUP_RETR);
    
    // Reset ARC register 0
    new_setup_retr |= 0xF0;
    new_setup_retr |= cnt;
    write_register(NRF24L01P_REG_SETUP_RETR, new_setup_retr);
}

void nrf24l01p_auto_retransmit_delay(delay us)
{
    uint8_t new_setup_retr = read_register(NRF24L01P_REG_SETUP_RETR);

    // Reset ARD register 0
    new_setup_retr |= 0x0F;
    new_setup_retr |= ((us / 250) - 1) << 4;
    write_register(NRF24L01P_REG_SETUP_RETR, new_setup_retr);
}

void nrf24l01p_set_rf_channel(channel MHz)
{
	uint16_t new_rf_ch = MHz - 2400;
    write_register(NRF24L01P_REG_RF_CH, new_rf_ch);
}

void nrf24l01p_set_rf_tx_output_power(output_power dBm)
{
    uint8_t new_rf_setup = read_register(NRF24L01P_REG_RF_SETUP) & 0xF9;
    new_rf_setup |= (dBm << 1);

    write_register(NRF24L01P_REG_RF_SETUP, new_rf_setup);
}

void nrf24l01p_set_rf_air_data_rate(air_data_rate bps)
{
    // Set value to 0
    uint8_t new_rf_setup = read_register(NRF24L01P_REG_RF_SETUP) & 0xD7;
    
    switch(bps)
    {
        case _1Mbps: 
            break;
        case _2Mbps: 
            new_rf_setup |= 1 << 3;
            break;
        case _250kbps:
            new_rf_setup |= 1 << 5;
            break;
    }
    write_register(NRF24L01P_REG_RF_SETUP, new_rf_setup);
}
```

到此为止，小车上所有的硬件驱动代码都说完了，接下来看小车的主函数有什么吧

####main函数部分
```c
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "nrf24l01p.h"
#include "servo.h"
#include "motor.h"
#include "retarget.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//data array to be read

uint8_t rx_data[NRF24L01P_PAYLOAD_LENGTH]={0,};

/*定义接受数据是干嘛的ra_data[0]
 * rx_data[0]用于确定电机
 *
 * rx_data[1],ra_data[2]组成16位数据，
 *
 * rx_data[3],rx_data[4]组成16位数据，
 *
 * rx_data[5].rx_data[6]组成16位数据，
 *
 */

 void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  RetargetInit(&huart1);

    nrf24l01p_rx_init(2500,_1Mbps);

    servo_init();
    motor_init();

    uint16_t speedA,speedB,angle=1500;
    float A,B;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      //更新遥控器发来的数据,并将8为数据合�??????16位数�??????
      angle=(rx_data[2]<<8)|rx_data[1];
      A=(rx_data[4]<<8)|rx_data[3];
      B=(rx_data[6]<<8)|rx_data[5];

      servo_setangle(angle);

      if(rx_data[0]==2){
          speedA=(uint16_t)(0.2591*A-320);
          speedB=(uint16_t)(0.2591*B-320);
          motor_stepback( speedA,speedB);
      }
      if(rx_data[0]==1){
          speedA=(uint16_t)(-0.2641*A+720);
          speedB=(uint16_t)(-0.2641*B+720);
          motor_goahead(speedA,speedB);
      }

      if(rx_data[0]=0){
          motor_goahead(0,0);
      }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
    if(GPIO_Pin==NRF24L01P_IRQ_PIN_NUMBER){
        nrf24l01p_rx_receive(rx_data);//read data when data ready flag is set
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
```

OK，到此为止，小车的整个代码说完了

### 遥控器部分
![IMG_20230305_032547](https://user-images.githubusercontent.com/89727667/222925075-914d837b-a12a-429b-9ec9-eac1f5ff6546.jpg)
遥控器就三个硬件，单片机本体+摇杆+nrf24l01无线模块

原理也很简单，摇杆用ADC采样，采样后的数据稍微处理一下就直接通过无线模块发出去

#### 摇杆部分
摇杆有X、Y两个方向，反正单片机资源够用，这里用了两个ADC











































