#include "sys.h"
#include <math.h>
#include "Kinematics.h"
#include "lkmotor.h"
#include "delay.h"
#include "main.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f  // ?????
#endif

void get_cable_length(float cable_length[], float p[])
{
	cable_length[0] = sqrt(pow(0.715-p[0]-0.05,2)+pow(p[1],2)+pow(p[2],2));
	cable_length[1] = sqrt(pow(p[0],2)+pow(0.715-p[1]-0.05,2)+pow(p[2],2));
	cable_length[2] = sqrt(pow(-0.715-p[0]+0.05,2)+pow(p[1],2)+pow(p[2],2));
	cable_length[3] = sqrt(pow(p[0],2)+pow(-0.715-p[1]+0.05,2)+pow(p[2],2));
}

	
//void move_point(double point1[], double point2[],u32 n,u32 i)
//{
//	if(i == 0)
//	{
//		get_position(1);
//				//delay_ms(1);
//				delay_us(500);
//				get_position(2);
//				//delay_ms(1);
//				delay_us(500);
//				get_position(3);
//				//delay_ms(1);
//				delay_us(500);
//				get_position(4);
//			delay_us(500);
//		
//		int64_t angle_1[4] = {motorinfo.motors[1].angle,motorinfo.motors[2].angle,motorinfo.motors[3].angle,motorinfo.motors[4].angle};
//	}
//		
//	
//	
//	double point_now[3];
//	point_now[0] = ((n-i)*point1[0] + i*point2[0])/n;
//	point_now[1] = ((n-i)*point1[1] + i*point2[1])/n;
//	point_now[2] = ((n-i)*point1[2] + i*point2[2])/n;
//	
//	double cable_length_1[4];
//		
//	double cable_length_now[4];
//	
//	get_cable_length(cable_length_1,point_now);
//	
//	get_cable_length(cable_length_now,point_now);
//	
//	double delta_cable_length[4];
//	
//	delta_cable_length[0] = cable_length_now[0] - cable_length_1[0];
//	delta_cable_length[1] = cable_length_now[1] - cable_length_1[1];
//	delta_cable_length[2] = cable_length_now[2] - cable_length_1[2];
//	delta_cable_length[3] = cable_length_now[3] - cable_length_1[3];
//	
//	int64_t delta_angle[4];
//	
//	delta_angle[0] = delta_cable_length[0]*36000/(0.02*M_PI);
//	delta_angle[1] = delta_cable_length[1]*36000/(0.02*M_PI);
//	delta_angle[2] = delta_cable_length[2]*36000/(0.02*M_PI);
//	delta_angle[3] = delta_cable_length[3]*36000/(0.02*M_PI);
//	
//	closed_loop_position(1,delta_angle[0],180);
//	delay_us(500);
//	closed_loop_position(2,delta_angle[1],180);
//	delay_us(500);
//	closed_loop_position(3,delta_angle[2],180);
//	delay_us(500);
//	closed_loop_position(4,delta_angle[3],180);
//	delay_us(500);
//	
//}
	
	
	
	
	
	
	
	
	