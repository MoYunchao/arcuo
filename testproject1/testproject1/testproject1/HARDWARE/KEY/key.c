#include "key.h"
#include "delay.h"
//////////////////////////////////////////////////////////////////////////////////	 

//////////////////////////////////////////////////////////////////////////////////

//按键初始化函数
void KEY_Init(void)
{
    GPIO_InitTypeDef GPIO_Initure;
	
    __HAL_RCC_GPIOE_CLK_ENABLE();           //开启GPIOH时钟

    GPIO_Initure.Pin=GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12; //PA0
    GPIO_Initure.Mode=GPIO_MODE_INPUT; //输入
    GPIO_Initure.Pull=GPIO_PULLUP;    //上拉
    GPIO_Initure.Speed=GPIO_SPEED_FREQ_VERY_HIGH; //高速
    HAL_GPIO_Init(GPIOE,&GPIO_Initure);  
    
}

//按键处理函数
//返回按键值
u8 KEY_Scan(void)
{    
    if((KEY1==0||KEY2==0||KEY3==0))
    {
        delay_ms(20);        
        if(KEY1==0) 
				{ while(KEY1==0); 
					return KEY1_PRES;
				}
        else if(KEY2==0)
        {	while(KEY2==0); 
					return KEY2_PRES;
				}
				else if(KEY3==0)
        {	while(KEY3==0); 
					return KEY3_PRES;
				}                      
    }
    return 0;   //无按键按下
}
