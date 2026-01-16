#ifndef _LKMOTOR_H
#define _LKMOTOR_H
#include "sys.h"
#include "fdcan.h"
//FDCAN1接收RX0中断使能
#define FDCAN1_RX0_INT_ENABLE	1		//0,不使能;1,使能.

//读取电机状态1
void get_status_1(u32 id);

//读取电机状态2
void get_status_2(u32 id);

//清除电机错误
void clear_error(u32 id);

//电机关闭
void shut_down(u32 id);

//电机运行
void motor_run(u32 id);

//电机停止
void motor_stop(u32 id);

//位置闭环控制
void closed_loop_position(u32 id, int32_t deg, u16 speed);

//角度读取
void get_position(u32 id);

//接收电机状态1
void read_status_1(u32 id, u8 msg[8]);

//接收电机状态2
void read_status_2(u32 id, u8 msg[8]);

//接收电机角度
void read_position(u32 id, u8 msg[8]);

//串口打印电机信息
void print_motor_msg(u32 id);

#endif