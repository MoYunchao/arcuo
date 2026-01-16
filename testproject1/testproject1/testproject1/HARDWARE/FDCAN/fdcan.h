#ifndef _FDCAN_H
#define _FDCAN_H
#include "sys.h"
//FDCAN1接收RX0中断使能
#define FDCAN1_RX0_INT_ENABLE	1		//0,不使能;1,使能.

u8 FDCAN1_Mode_Init(u16 presc,u8 ntsjw,u16 ntsg1,u8 ntsg2,u32 mode);
u8 FDCAN1_Send_Msg(u32 id,u8* msg,u32 len);
u8 FDCAN1_Receive_Msg(u8 *buf);
#endif
