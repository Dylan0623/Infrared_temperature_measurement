#include "control.h"
#include "motor.h"
#include "BPSim.h"
#include "uart_handler.h"
#include "gpio.h"

extern void Turn_Right(void);
extern void Turn_Left(void);

float Position_KP=2,Position_KI=0,Position_KD=0;  			//X控制PID参数
float Position_Kp=2,Position_Ki=-0,Position_Kd=0;  			//Y控制PID参数
extern uint8_t Temperature_limit;
extern uint16_t Distance;
extern float Temperature,skin_temp;
extern double data1[],data[];
extern unsigned char ucRxFinish;
extern float BP_in[2],BP_out[1];
extern float	Temperature;

void Control_Loop_A(void)													//模式一：体温测量
{ 
	int i;
	
	MLX90614_ReadObjectTemperature(&Temperature);
	BP_in[0] = Temperature;
	BP_in[1] = Distance;
	sim(BP_in, BP_out);
	skin_temp = human_temp(BP_out[0]-2);
	if(skin_temp > Temperature_limit && Temperature_limit != 0)
	{
		Beep();
	}
	Laser_on();
	Get_Value(&ucRxFinish);
	Screnn_trans_float(0,skin_temp);		//屏幕显示温度
	Screnn_trans_int(6,Distance);
//	uartx_printf(huart1,"体温%.2f   距离 %dmm\n",skin_temp,Distance);
}

void Control_Loop_B(void)     													//模式二：物体表面温度 测量              
{
	int i;

	Laser_on();
	Get_Value(&ucRxFinish);
	
	MLX90614_ReadObjectTemperature(&Temperature);
	
	BP_in[0] = Temperature;
	BP_in[1] = Distance;
	sim(BP_in, BP_out);
	if(BP_out[0] > Temperature_limit && Temperature_limit != 0)
	{
		Beep();
	}
	Screnn_trans_float(0,BP_out[0]);
//	Screnn_trans_int(0,BP_out[0]);		//屏幕显示温度
//	uartx_printf(huart1,"%.2ftemp:%.1f\n",Temperature,BP_out[0]);
//	uartx_printf(huart1,"物体表面温度%.2f   距离 %dmm\n",Temperature,Distance);
}

void Control_Loop_C(void)											//模式三：人脸识别测，显示对应ID
{
	extern int ID,learn;
	Screnn_trans_int(6,ID);
	if(learn == 1)
	{
		learn = 0;
		Camera_Learn();
	}
	MLX90614_ReadObjectTemperature(&Temperature);
	BP_in[0] = Temperature;
	BP_in[1] = Distance;
	sim(BP_in, BP_out);
	skin_temp = human_temp(BP_out[0]);
	Get_Value(&ucRxFinish);
	Camera_temperature(skin_temp);
}

void Control_Loop_D(void)											//模式四：识别是否佩戴口罩
{
	extern uint16_t mask;
	
	Control_Loop_A();
	Camera_temperature(skin_temp);
	if(mask == 1)
	{
		Screnn_trans_char(8,"是");
		mask = 0;
	}
	else Screnn_trans_char(8,"否");
}

int Position_PID_X (float value, float Target)
{
    static float Bias, Pwm, Integral_bias, Last_Bias;
    Bias = value - Target;                                //计算偏差
    Integral_bias += Bias;	                             //求出偏差的积分
    Pwm = Position_KP * Bias +                            //PID控制器比例项
          Position_KI * Integral_bias +                     //PID控制器积分项
          Position_KD * (Bias - Last_Bias);                 //PID控制器微分项
    Last_Bias = Bias;                                     //保存上一次偏差
    return Pwm;                                           //增量输出
}

int Position_PID_Y (float value, float Target)
{
    static float Bias, Pwm, Integral_bias, Last_Bias;
    Bias = value - Target;                                //计算偏差
    Integral_bias += Bias;	                             //求出偏差的积分
    Pwm = Position_Kp * Bias +                            //PID控制器比例项
          Position_Ki * Integral_bias +                     //PID控制器积分项
          Position_Kd * (Bias - Last_Bias);                 //PID控制器微分项
    Last_Bias = Bias;                                     //保存上一次偏差
    return Pwm;                                           //增量输出
}

float human_temp(float temp)
{
	float body_temp;
	if(temp <= 32)
	{
		body_temp = temp + 2.4;
	}
	else if(temp > 32 && temp <= 35)
	{
		body_temp = temp * 0.85 + 7.266;
	}
	else if(temp > 35 && temp <= 36.4)
	{
		body_temp = temp * 0.75 + 10.815;
	}
	else if(temp > 36.4)
	{
		body_temp = temp + 1.5;
	}
	return body_temp;
}
