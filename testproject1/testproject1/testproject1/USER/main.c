#define ARM_MATH_CM7
#define __FPU_PRESENT 1
#define __TARGET_FPU_VFP
#define ARM_MATH_MATRIX_CHECK
#include "sys.h"
#include "delay.h"
#include "usart.h" 
#include "led.h"
#include "key.h"
#include "fdcan.h"
#include "lkmotor.h"
#include "main.h"
#include <math.h>
#include "Kinematics.h"
#include "timer.h"
/************************************************
 
************************************************/

#ifndef M_PI
#define M_PI 3.14159265358979323846f  // ?????
#endif
//电机信息结构体
volatile MotorArray_t motorinfo;

//循环标志
u8 flag = 0;


	
int main(void)
{
  u8 key;
	u8 t=0;	
	u8 canbuf[8]={0x92,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
	u8 res;
	u32 i = 0;
	u32 n =10;
	
	int8_t respdata[16];
	
	u8 loop = 0;
	
	int32_t P = 1;
	int32_t I = 1;
	int32_t D = 1;
	
	
	u8 mode=1;  
	u8 len;
	u8 rx_buffer[64];
	Cache_Enable();                			//打开L1-Cache
	HAL_Init();				        		//初始化HAL库
	Stm32_Clock_Init(160,5,2,4);  		    //设置时钟,400Mhz
	delay_init(400);						//延时初始化
	uart_init(115200);						//串口初始化	
	LED_Init();								//初始化LED
	KEY_Init();								//初始化按键	
	
	//控制频率40ms
	
	TIM3_Init(300-1,20000-1);      	//定时器3初始化，定时器时钟为200M，分频系数为20000-1，
										//所以定时器3的频率为200M/20000=10K，自动重装载为5000-1，那么定时器周期就是500ms
	
  FDCAN1_Mode_Init(10,8,31,8,FDCAN_MODE_NORMAL);  //正常模式
	
	//读取电机初始位置
	get_position(1);
	//delay_ms(1);
	delay_us(500);
	get_position(2);
	//delay_ms(1);
	delay_us(500);
	get_position(3);
	//delay_ms(1);
	delay_us(500);
	get_position(4);
	delay_us(500);
	
	//电机无刹车，保持
	closed_loop_position(1,motorinfo.motors[1].angle,360);
	delay_us(500);
	closed_loop_position(2,motorinfo.motors[2].angle,360);
	delay_us(500);
	closed_loop_position(3,motorinfo.motors[3].angle,360);
	delay_us(500);
	closed_loop_position(4,motorinfo.motors[4].angle,360);
	delay_us(500);
	
	
	//记录电机初始位置		
	int64_t angle_0[4] = {motorinfo.motors[1].angle,motorinfo.motors[2].angle,motorinfo.motors[3].angle,motorinfo.motors[4].angle};
	
	
	//控制角度
	int64_t angle_c[4] = {motorinfo.motors[1].angle,motorinfo.motors[2].angle,motorinfo.motors[3].angle,motorinfo.motors[4].angle};
	
	//动平台初始位置
	float position_0[3] = {0,0,-0.51};
	
	//动平台当前位置
	float position_1[3] = {0,0,-0.51};
	
	//动平台目标位置
	float position_2[3] = {-0.25,-0.25,-0.4};
	
	//动平台控制位置
	float position_c[3] = {0,0,-0.51};
	
	//初始绳长
	float cable_length_0[4];
	get_cable_length(cable_length_0,position_0);
	
	//当前绳长
	float cable_length_1[4];
	
	//控制绳长
	float cable_length_c[4];
	
	//控制角度
	int64_t delta_angle[4];
	

	
  //printf("LoopBack Mode\r\n");	
    while(1)
    {
        key=KEY_Scan();
		//if(key==KEY1_PRES)//KEY1按下,发送一次数据
			if(1)
		{
			loop = 1;
			closed_loop_position(1,angle_0[0],360);
			delay_us(500);
			closed_loop_position(2,angle_0[1],360);
			delay_us(500);
			closed_loop_position(3,angle_0[2],360);
			delay_us(500);
			closed_loop_position(4,angle_0[3],360);
			delay_us(500);
			
			if(USART_RX_STA&0x8000)
			{					   
				//清空接收区
				for(int i = 0; i < 16; i++)
				{
					respdata[i] = 0;
				}
				
				//得到此次接收到的数据长度
				len=USART_RX_STA&0x3fff;
				
				//打印接收到的数据长度
				printf("Received %d bytes:\r\n", len);
				
				//写入接收区（使用int类型）
				for(int i = 0; i < len; i++)
				{
					respdata[i] = (int8_t)USART_RX_BUF[i];
				}
				
				//printf("\r\n您发送的消息为:\r\n");
//			HAL_UART_Transmit(&UART1_Handler,(uint8_t*)USART_RX_BUF,len,1000);	//发送接收到的数据
//			while(__HAL_UART_GET_FLAG(&UART1_Handler,UART_FLAG_TC)!=SET);		//等待发送结束
				
				//若数据长度为8，则可能只看到一个，也可能什么都没看到
				if (len == 8)
				{
					//如果看到tagid是0
					if (respdata[0] == 0)
					{
						//写入动平台位置
						position_1[0] = respdata[1]/100.0f;
						position_1[1] = respdata[2]/100.0f;
						position_1[2] = respdata[3]/100.0f;
						
						//打印动平台坐标
						printf("position_1:%f  ", position_1[0]);
						printf("%f  ", position_1[1]);
						printf("%f", position_1[2]);
						printf("\r\n");
						
					}
					
					//如果看到tagid是1
					else if (respdata[0] == 1)
					{
						//写入目标位置
						position_2[0] = respdata[1]/100.0f;
						position_2[1] = respdata[2]/100.0f;
						position_2[2] = respdata[3]/100.0f;
						
						//打印目标坐标
						printf("position_2:%f  ", position_2[0]);
						printf("%f  ", position_2[1]);
						printf("%f", position_2[2]);
						printf("\r\n");
					}
					
					//如果什么都没看到
					else
					{
						printf("nothing8:");
						for(int i = 0; i < len; i++)
						{
							printf("%d ", USART_RX_BUF[i]);
						}
						printf("\r\n");
					}
				}
				
				
				//如果数据长度为16，则可能看到两个
				else if (len == 16)
				{
					//如果看到第一个tagid是0
					if (respdata[0] == 0)
					{
						//写入动平台位置
						position_1[0] = respdata[1]/100.0f;
						position_1[1] = respdata[2]/100.0f;
						position_1[2] = respdata[3]/100.0f;
						
						//打印动平台坐标
						printf("position_1:%f  ", position_1[0]);
						printf("%f  ", position_1[1]);
						printf("%f", position_1[2]);
						printf("\r\n");
					}
					
					//如果看到第二个tagid是1
					if (respdata[8] == 1)
					{
						//写入目标位置
						position_2[0] = respdata[9]/100.0f;
						position_2[1] = respdata[10]/100.0f;
						position_2[2] = respdata[11]/100.0f;
						//打印目标坐标
						printf("position_2:%f  ", position_2[0]);
						printf("%f  ", position_2[1]);
						printf("%f", position_2[2]);
						printf("\r\n");
					}
					
					//如果什么都没看到
					if ((respdata[8] != 1)&&(respdata[0] != 0))
					{
						printf("nothing16:");
						for(int i = 0; i < len; i++)
						{
							printf("%d ", USART_RX_BUF[i]);
						}
						printf("\r\n");
					}
				}
				
				//如果数据长度不为8或16
				else
				{
					printf("error:");
					for(int i = 0; i < len; i++)
					{
						printf("%d ", USART_RX_BUF[i]);
					}
					printf("\r\n");
				}
			
	
				//重置接收标志
				USART_RX_STA=0;
				printf("\r\n");
	}
			
		}
		if(flag == 1&&loop == 2)
		{
			
			//重置控制周期状态
			flag = 0;
			
			//led1闪烁标识
			LED1_Toggle;
			
			//重置电机通讯状态
			motorinfo.motors[1].can = 0;
			motorinfo.motors[2].can = 0;
			motorinfo.motors[3].can = 0;
			motorinfo.motors[4].can = 0;
			
			
//		  if (i<=n)
//			{
//				move_point(position_0, position_2,n,i);
//				
//				i++;
//			}
			
			get_position(1);
			//delay_ms(1);
			delay_us(500);
			get_position(2);
			//delay_ms(1);
			delay_us(500);
			get_position(3);
			//delay_ms(1);
			delay_us(500);
			get_position(4);
			delay_us(500);
			
			float delta_x = position_2[0]-position_1[0];
			
			float delta_y = position_2[1]-position_1[1];
			float delta_z = position_2[2]-position_1[2];
			
			position_c[0] = position_1[0] + delta_x*P*0.04;
			position_c[1] = position_1[1] + delta_y*P*0.04;
			position_c[2] = position_1[2] + delta_z*P*0.04;
			
			get_cable_length( cable_length_c,position_c);
			get_cable_length( cable_length_1,position_1);
			
			delta_angle[0] = (cable_length_c[0] - cable_length_1[0])*36000/(0.02*M_PI);
			delta_angle[1] = (cable_length_c[1] - cable_length_1[1])*36000/(0.02*M_PI);
			delta_angle[2] = (cable_length_c[2] - cable_length_1[2])*36000/(0.02*M_PI);
			delta_angle[3] = (cable_length_c[3] - cable_length_1[3])*36000/(0.02*M_PI);
			
			angle_c[0] = angle_c[0] + delta_angle[0];
			angle_c[1] = angle_c[1] - delta_angle[1];
			angle_c[2] = angle_c[2] + delta_angle[2];
			angle_c[3] = angle_c[3] - delta_angle[3];
			
			closed_loop_position(1,angle_c[0],3600);
			delay_us(500);
			closed_loop_position(2,angle_c[1],3600);
			delay_us(500);
			closed_loop_position(3,angle_c[2],3600);
			delay_us(500);
			closed_loop_position(4,angle_c[3],3600);
			delay_us(500);
			
			position_1[0] = position_c[0];
			position_1[1] = position_c[1];
			position_1[2] = position_c[2];
			
			
			
			
			
			
//			closed_loop_position(1,motorinfo.motors[1].angle+28647,180);
//			delay_us(500);
//			closed_loop_position(2,motorinfo.motors[2].angle+28647,180);
//			delay_us(500);
////			closed_loop_position(3,motorinfo.motors[3].angle-22918,180);
////			delay_us(500);
//			closed_loop_position(4,motorinfo.motors[4].angle+45836,180);
//			delay_us(500);
//			printf("发送数据：\r\n");
//			
//			for(i=0;i<8;i++)   //此处的循环次数为发送数据字节数，发送8字节填8，发送64字节填64
//      {
//				printf("txdata[%d]:0x%x\r\n",i,canbuf[i]);
//			}
//			
//			res=FDCAN1_Send_Msg(0x141,canbuf,FDCAN_DLC_BYTES_8);//发送8个字节 
//			if(res)
//			{
//				printf("发送失败\r\n");//提示发送失败
//			}
//			else 
//			{
//				printf("发送成功\r\n");//提示发送成功			
//			}				
//			
			
			
//			get_status_1(1);
//			delay_ms(1);
			//closed_loop_position(2,100000,180);
//			delay_ms(1);
//			get_position(1);
//			
//			get_status_1(2);
//			delay_ms(1);
//			//closed_loop_position(2,360000,180);
//			delay_ms(1);
//			get_position(2);
								
//				double p[3] = {-0.163,-0.28,-0.49};
//				
//				double cable_length[4];
//				
//				

//				get_cable_length(cable_length,p);
//				
//				for(int i = 0; i < 4; i++)
//    {
//        printf("%lf\r\n", cable_length[i]);
//        
//    }
			
			
			
//			printf ("motor 1 temperature: %d degree \r\n",motorinfo.motors[1].temp);
//			
//			printf("motor 1 voltage: %.2f V \r\n", motorinfo.motors[1].vol / 100.0f);
//			
//			printf("motor 1 current: %.2f A \r\n", motorinfo.motors[1].cur / 100.0f);
//			
//			switch(motorinfo.motors[1].motorstate) 
//				{
//				case 0x00:
//					printf("status: open (0x00) \r\n");
//        break;
//				case 0x10:
//					printf("status: shut down (0x10) \r\n");
//        break;
//				default:
//					printf("status: unknown (0x%02X) \r\n", motorinfo.motors[1].motorstate);
//        break;
//				}
//				
//			printf("motor 1 power: %d W \r\n", motorinfo.motors[1].power);

//			printf("motor 1 speed: %d degree/s \r\n", motorinfo.motors[1].speed);

//			printf("motor 1 encoder: %u \r\n", motorinfo.motors[1].encoder);

//			printf("motor 1 angle: %.2f degree \r\n", motorinfo.motors[1].angle / 100.0f);
				
//				get_position(1);
//				get_position(2);
//				get_position(3);
//				get_position(4);
				//delay_ms(5);
//				get_position(1);
//				//delay_ms(1);
//				delay_us(500);
//				get_position(2);
//				//delay_ms(1);
//				delay_us(500);
//				get_position(3);
//				//delay_ms(1);
//				delay_us(500);
//				get_position(4);


				print_motor_msg(1);
				print_motor_msg(2);
				print_motor_msg(3);
				print_motor_msg(4);
				
			
		}
		
			
		
		
		
		
//		t++; 
//		
//		if(t==200)
//		{
//			LED1_Toggle;//提示系统正在运行	
//			t=0;			
//		}	
	}	     
}

