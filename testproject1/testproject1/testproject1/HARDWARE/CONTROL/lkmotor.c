#include "sys.h"
#include "fdcan.h"
#include "main.h"
#include "lkmotor.h"


//读取电机状态1
void get_status_1(u32 id)
{
	u8 msg[8];
	msg[0] = 0x9a;
	msg[1] = 0x00;
	msg[2] = 0x00;
	msg[3] = 0x00;
	msg[4] = 0x00;
	msg[5] = 0x00;
	msg[6] = 0x00;
	msg[7] = 0x00;
	FDCAN1_Send_Msg(0x140+id,msg,FDCAN_DLC_BYTES_8);
}

//读取电机状态2
void get_status_2(u32 id)
{
	u8 msg[8];
	msg[0] = 0x9c;
	msg[1] = 0x00;
	msg[2] = 0x00;
	msg[3] = 0x00;
	msg[4] = 0x00;
	msg[5] = 0x00;
	msg[6] = 0x00;
	msg[7] = 0x00;
	FDCAN1_Send_Msg(0x140+id,msg,FDCAN_DLC_BYTES_8);
}

//清除电机错误
void clear_error(u32 id)
{
	u8 msg[8];
	msg[0] = 0x9B;
	msg[1] = 0x00;
	msg[2] = 0x00;
	msg[3] = 0x00;
	msg[4] = 0x00;
	msg[5] = 0x00;
	msg[6] = 0x00;
	msg[7] = 0x00;
	FDCAN1_Send_Msg(0x140+id,msg,FDCAN_DLC_BYTES_8);
}

//电机关闭
void shut_down(u32 id)
{
	u8 msg[8];
	msg[0] = 0x80;
	msg[1] = 0x00;
	msg[2] = 0x00;
	msg[3] = 0x00;
	msg[4] = 0x00;
	msg[5] = 0x00;
	msg[6] = 0x00;
	msg[7] = 0x00;
	FDCAN1_Send_Msg(0x140+id,msg,FDCAN_DLC_BYTES_8);
}

//电机运行
void motor_run(u32 id)
{
	u8 msg[8];
	msg[0] = 0x88;
	msg[1] = 0x00;
	msg[2] = 0x00;
	msg[3] = 0x00;
	msg[4] = 0x00;
	msg[5] = 0x00;
	msg[6] = 0x00;
	msg[7] = 0x00;
	FDCAN1_Send_Msg(0x140+id,msg,FDCAN_DLC_BYTES_8);
}

//电机停止
void motor_stop(u32 id)
{
	u8 msg[8];
	msg[0] = 0x81;
	msg[1] = 0x00;
	msg[2] = 0x00;
	msg[3] = 0x00;
	msg[4] = 0x00;
	msg[5] = 0x00;
	msg[6] = 0x00;
	msg[7] = 0x00;
	FDCAN1_Send_Msg(0x140+id,msg,FDCAN_DLC_BYTES_8);
}

//位置闭环控制
//可读取电机状态2
//id:1~4
//deg：单位0.01°
//speed：单位1°/s
void closed_loop_position(u32 id, int32_t deg, u16 speed)
{
	u8 msg[8];
	msg[0] = 0xa4;
	msg[1] = 0x00;
	msg[2] = speed & 0xFF;
	msg[3] = (speed >> 8) & 0xFF;
	msg[4] = deg & 0xFF;
	msg[5] = (deg >> 8) & 0xFF;
	msg[6] = (deg >> 16) & 0xFF;
	msg[7] = (deg >> 24) & 0xFF;
	FDCAN1_Send_Msg(0x140+id,msg,FDCAN_DLC_BYTES_8);
}

	
//电机角度读取
void get_position(u32 id)
{
	u8 msg[8];
	msg[0] = 0x92;
	msg[1] = 0x00;
	msg[2] = 0x00;
	msg[3] = 0x00;
	msg[4] = 0x00;
	msg[5] = 0x00;
	msg[6] = 0x00;
	msg[7] = 0x00;
	FDCAN1_Send_Msg(0x140+id,msg,FDCAN_DLC_BYTES_8);
}

//接收电机状态1
void read_status_1(u32 id, u8 msg[8])
{
	motorinfo.motors[id].can = 1;
	
	motorinfo.motors[id].temp = (int8_t)msg[1];
	
	motorinfo.motors[id].vol = (int16_t)((msg[3] << 8) | msg[2]);
	
	motorinfo.motors[id].cur = (int16_t)((msg[5] << 8) | msg[4]);
	
	motorinfo.motors[id].motorstate = msg[6];
	
	motorinfo.motors[id].errorstate = msg[7];
}

//接收电机状态2
void read_status_2(u32 id, u8 msg[8])
{
	
	motorinfo.motors[id].can = 1;
	
	motorinfo.motors[id].temp = (int8_t)msg[1];
	
	motorinfo.motors[id].power = (int16_t)((uint16_t)(msg[3] << 8) | msg[2]);
	
	motorinfo.motors[id].speed = (int16_t)((uint16_t)(msg[5] << 8) | msg[4]);
	
	motorinfo.motors[id].encoder = ((msg[7] << 8) | msg[6]);
}

void read_position(u32 id, u8 msg[8])
{
	
	motorinfo.motors[id].can = 1;
	
	motorinfo.motors[id].angle = (int64_t)(((uint64_t)msg[1] << 0)   |
                          ((uint64_t)msg[2] << 8)   |
                          ((uint64_t)msg[3] << 16)  |
                          ((uint64_t)msg[4] << 24)  |
                          ((uint64_t)msg[5] << 32)  |
                          ((uint64_t)msg[6] << 40)  |
                          ((uint64_t)msg[7] << 48));
	

	uint64_t raw_angle = ((uint64_t)msg[1] << 0)   |
											 ((uint64_t)msg[2] << 8)   |
											 ((uint64_t)msg[3] << 16)  |
											 ((uint64_t)msg[4] << 24)  |
											 ((uint64_t)msg[5] << 32)  |
											 ((uint64_t)msg[6] << 40)  |
											 ((uint64_t)msg[7] << 48);


	if (raw_angle & (1ULL << 55)) 
		{  
			raw_angle |= (0xFFULL << 56);
	}
		else 
			{
			
	}


int64_t motorAngle = (int64_t)raw_angle;

motorinfo.motors[id].angle = motorAngle;
}
		

void print_motor_msg(u32 id)
{
	printf("motor %d can: %d \r\n", id, motorinfo.motors[id].can);
	printf("motor %d temperature: %d degree \r\n", id, motorinfo.motors[id].temp);
	printf("motor %d voltage: %.2f V \r\n", id, motorinfo.motors[id].vol / 100.0f);
	printf("motor %d current: %.2f A \r\n", id, motorinfo.motors[id].cur / 100.0f);

	switch(motorinfo.motors[id].motorstate) {
		case 0x00:
				printf("motor %d status: open (0x00) \r\n", id);
				break;
		case 0x10:
				printf("motor %d status: shut down (0x10) \r\n", id);
				break;
		default:
				printf("motor %d status: unknown (0x%02X) \r\n", id, motorinfo.motors[id].motorstate);
				break;
	}

	printf("motor %d power: %d W \r\n", id, motorinfo.motors[id].power);
	printf("motor %d speed: %d degree/s \r\n", id, motorinfo.motors[id].speed);
	printf("motor %d encoder: %u \r\n", id, motorinfo.motors[id].encoder);
	printf("motor %d angle: %.2f degree \r\n", id, motorinfo.motors[id].angle / 100.0f);
			
}