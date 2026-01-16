#ifndef _KEY_H
#define _KEY_H
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 

//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////

#define KEY1        HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_10)  //KEY1按键
#define KEY2        HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_11)  //KEY2按键
#define KEY3        HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_12) //KEY3按键


#define KEY1_PRES 1  	//KEY1按下后返回值
#define KEY2_PRES	2	//KEY2按下后返回值
#define KEY3_PRES	3	//KEY3按下后返回值


void KEY_Init(void);  //按键IO初始化函数
u8 KEY_Scan(void); //按键扫描函数
#endif
