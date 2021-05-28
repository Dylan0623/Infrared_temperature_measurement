#ifndef __CONTROL_H
#define __CONTROL_H

#include "main.h"
#include "mlx90614.h"
#include "stdio.h"
#include "motor.h"
#include "linefinder.h"

extern int speed_set;

int Position_PID_X (float value, float Target);
int Position_PID_Y (float value, float Target);
void Track(void);
void Control_Loop_A(void);     						//模式一：体温测量    
void Control_Loop_B(void);								//模式二：水温测量
void Control_Loop_C(void);								//人脸识别测温
void Control_Loop_D(void);								//识别是否佩戴口罩
float human_temp(float temp);

#endif 
