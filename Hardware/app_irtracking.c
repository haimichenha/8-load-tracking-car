#include "app_irtracking.h"

#define IRTrack_Trun_KP (500)
#define IRTrack_Trun_KI (0) 
#define IRTrack_Trun_KD (0) 

int pid_output_IRR = 0;
u8 trun_flag = 0;

#define IRR_SPEED 			  300  //Ѳ���ٶ�   Patrol speed

float PID_IR_Calc(int8_t actual_value)
{

	float IRTrackTurn = 0;
	int8_t error;
	static int8_t error_last=0;
	static float IRTrack_Integral;

	error=actual_value;
	
	IRTrack_Integral +=error;
	
	//λ��ʽpid    Positional pid
	IRTrackTurn=error*IRTrack_Trun_KP
							+IRTrack_Trun_KI*IRTrack_Integral
							+(error - error_last)*IRTrack_Trun_KD;
	return IRTrackTurn;
}

//x1-x8 ����������   x1-x8 count from left to right
void LineWalking(void)
{
	static int8_t err = 0;
	static u8 x1,x2,x3,x4,x5,x6,x7,x8;
	uint8_t sensor = Tracking_Read();
	
	x1 = (sensor >> 7) & 0x01;
	x2 = (sensor >> 6) & 0x01;
	x3 = (sensor >> 5) & 0x01;
	x4 = (sensor >> 4) & 0x01;
	x5 = (sensor >> 3) & 0x01;
	x6 = (sensor >> 2) & 0x01;
	x7 = (sensor >> 1) & 0x01;
	x8 = (sensor >> 0) & 0x01;
	
	
	//debug
//	printf("%d\t %d\t %d\t %d\t %d\t %d\t %d\t %d\t \r\n",x1,x2,x3,x4,x5,x6,x7,x8);

    ///L1ΪX1���׵����ʱΪ1��������ʱΪ0     L1 is X1, 1 when the white background is off, 0 when the black line is on///
	
//�����ж�  Priority judgment
    //1100 0011
//	if(x1 == 1 && x2 == 1 &&x3 == 0 &&  x4 == 0  && x5 == 0 && x6  == 0 && x7 == 1 && x8 == 1 ) //�����   transverse acute angle
//	{
//		err = 15; 
//	}
//	else if(x1 == 1 && x2 == 1 &&x3 == 1 &&  x4 == 1  && x5 == 1 && x6  == 1 && x7 == 1 && x8 == 1 ) //�����  transverse acute angle
//	{
//		if(trun_flag == 0) //������    out of the line
//		{
//			err = 15; 
//			trun_flag = 1;
//		}
//		//������������ϸ�״̬    Otherwise, the situation remains the same as before.
//	}

//�����ж��Ƿ�ֱ�ǻ����  Prioritize whether to right angles or acute angles
	 if(x1 == 0 && x2 == 0  && x3 == 0&& x4 == 0 && x5 == 0 && x6 == 1  && x7 == 1 && x8 == 1) // 0000 0111
	{
		err = -15;
        delay_ms(100);
	}
    else if(x1 == 1 && x2 == 1  && x3 == 1&& x4 == 0 && x5 == 0 && x6 == 0  && x7 == 0 && x8 == 0) // 1110 0000
	{
		err = 15;
        delay_ms(100);
	}

  else if(x1 == 0 &&  x2 == 0  && x7 == 0 && x8 == 0 ) //���߶�����ֱ��    Both sides are lit. Run straight.
	{
		err = 0;
		if(trun_flag == 1)
		{
			trun_flag = 0;//�ߵ�Ȧ��    Walking in circles.
		}
	}
	
// else if(x1 == 0 &&  x3 == 0 && x4 == 0 && x5 == 0 && x8 == 0 )
//	{
//		err = 0;
//	}
//	//����ֱ��  Add Right Angle
//	else if((x1 == 0 || x2 == 0 ) && x8 == 1) 
//	{
//		err = -15; 
//	}
//	//����ֱ��  Add Right Angle
//	else if((x7 == 0 ||  x8 == 0) && x1 == 1) 
//	{
//		err = 15 ;
//	}
	


	else if(x1 == 1 && x2 == 1  && x3 == 1&& x4 == 0 && x5 == 1 && x6 == 1  && x7 == 1 && x8 == 1) // 1110 1111
	{
		err = -1;
	}
	else if(x1 == 1 && x2 == 1  && x3 == 0&& x4 == 0 && x5 == 1 && x6 == 1  && x7 == 1 && x8 == 1) // 1100 1111
	{
		err = -2;
	}
//	else if(x1 == 1 && x2 == 1  && x3 == 0&& x4 == 1 && x5 == 1 && x6 == 1  && x7 == 1 && x8 == 1) // 1101 1111
//	{
//		err = -2;
//	}
	
//	else if(x1 == 1 && x2 == 0  && x3 == 1&& x4 == 1 && x5 == 1 && x6 == 1  && x7 == 1 && x8 == 1) // 1011 1111
//	{
//		err = -3;
//	}
	else if(x1 == 1 && x2 == 0  && x3 == 0&& x4 == 1 && x5 == 1 && x6 == 1  && x7 == 1 && x8 == 1) // 1001 1111
	{
		err = -8;
	}
    
//		else if(x1 == 0 && x2 == 0  && x3 == 1&& x4 == 1 && x5 == 1 && x6 == 1  && x7 == 1 && x8 == 1) // 0011 1111
//	{
//		err = -4;   //ע�ͣ�����ֱ�Ǵ��� Note, when treated as a right angle
//	}
	else if(x1 == 0 && x2 == 1  && x3 == 1&& x4 == 1 && x5 == 1 && x6 == 1  && x7 == 1 && x8 == 1) // 0111 1111
	{
		err = -10; 
	}

	

	
	
	
	else if(x1 == 1 && x2 == 1  && x3 == 1&& x4 == 1 && x5 == 0 && x6 == 1  && x7 == 1 && x8 == 1) // 1111 0111
	{
		err = 1;
	} 
	else if(x1 == 1 && x2 == 1  && x3 == 1&& x4 == 1 && x5 == 0 && x6 == 0  && x7 == 1 && x8 == 1) // 1111 0011
	{
		err = 2;
	}
//	else if(x1 == 1 && x2 == 1  && x3 == 1&& x4 == 1 && x5 == 1 && x6 == 0  && x7 == 1 && x8 == 1) // 1111 1011
//	{
//		err = 2;
//	}
	else if(x1 == 1 && x2 == 1  && x3 == 1&& x4 == 1 && x5 == 1 && x6 == 0  && x7 == 0 && x8 == 1) // 1111 1001
	{
		err = 8;
	}
	
//	else if(x1 == 1 && x2 == 1  && x3 == 1&& x4 == 1 && x5 == 1 && x6 == 1  && x7 == 0 && x8 == 1) // 1111 1101
//	{
//		err = 3;
//	}
//	else if(x1 == 1 && x2 == 1  && x3 == 1&& x4 == 1 && x5 == 1 && x6 == 1  && x7 == 0 && x8 == 0) // 1111 1100
//	{
//		err = 4; ///����ֱ�Ǵ���  treat as a right angle
//	}
		else if(x1 == 1 && x2 == 1  && x3 == 1&& x4 == 1 && x5 == 1 && x6 == 1  && x7 == 1 && x8 == 0) // 1111 1110
	{
		err = 10;
	}
	

	
 
	else if(x1 == 1 &&x2 == 1 &&x3 == 1 && x4 == 0 && x5 == 0 && x6 == 1 && x7 == 1&& x8 == 1) //ֱ�� go straight
	{
		err = 0;
	}
    
	//ʣ�µľͱ�����һ��״̬	    The rest will stay the same.
    
    
	pid_output_IRR = (int)(PID_IR_Calc(err));

	Motion_Car_Control(IRR_SPEED, 0, pid_output_IRR);	

}

