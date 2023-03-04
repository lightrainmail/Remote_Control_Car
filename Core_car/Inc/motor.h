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
