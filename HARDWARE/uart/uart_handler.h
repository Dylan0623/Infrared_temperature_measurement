#ifndef UART_HANDLER_H_
#define UART_HANDLER_H_

#include "main.h"
#include "usart.h"
#include "stdio.h"
#include "stdlib.h"

void Camera_Get(uint8_t	RxBuffer[]);
void Uart_DataProcess(void);
void Get_Value(unsigned char* ucFlag);//获取VL53L0数据
void angle_rec(unsigned char ucData);
void angle_dateprocess(void);
void Screen_get(uint8_t	RxBuffer[]);		//获取屏幕数据
void Screnn_trans_int(int value,int screen_tx);
void Screnn_trans_char(uint8_t value,char screen_tx[]);
void Screnn_trans_float(int value,float screen_tx);
void Camera_temperature(float temp);
void Camera_normal(void);
void Camera_entering(void);
void Camera_Learn(void);

#endif
