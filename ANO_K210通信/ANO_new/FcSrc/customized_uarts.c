#include "customized_uarts.h"
#include "SysConfig.h"

s16 color_identify_data[3] = {0}; 	//色块识别xyz误差量*10000

s16 follow_line_data[2] = {0}; 			//巡线角度*100，截距*10000

s16 detour_pole_data[2] = {0}; 			//绕杆误差量*10000

s16 K210_Identify_data[4] = {0};					//K210识别数据(看头文件)

s16 K210_learn_state = 0;						//K210学习状态

s16 QR_data[3] = {0}; 							//二维码数据


u8 U1_re_temp[26] = {0};		//自定义uart1接收缓存
				
u8 U2_re_temp[26] = {0};		//自定义uart2接收缓存

u8 U3_re_temp[26] = {0};		//自定义uart3接收缓存


void UART1_Receive_Prepare(u8 data) 	//串口1接收处理
{
	static u8 _data_len = 0;
	static u8 rxstate = 0;


	if (rxstate == 0 && data == 'O')
	{
		rxstate = 1;
	}
	
	else if (rxstate == 1)		//判断帧头是否是openmv的
	{
		if(data == 'P') rxstate = 2;
		else rxstate = 0;		
	}
//数据标识,D绕杆	
	else if (rxstate == 2)		
	{
		if(data == 'D' )
		{
			rxstate = 3;
			U1_re_temp[0] = data;
			_data_len = 0;
		}
		else rxstate = 0;
	}
	//接收第一个数据高位第一位
	else if (rxstate == 3)
	{
		if(data != '$')
		{
			if(data > 47 && data < 58)
			{				
				rxstate = 4;
				U1_re_temp[1] = data - 48;
			}
			else rxstate = 0;
		}
		else
		{	
			U1_re_temp[25] = _data_len;
			rxstate = 0;
			UART1_data_handle(); 
		}
	}
			
	//接收第一个数据高位第二位
	else if (rxstate == 4)
	{
		if(data != '$')
		{
			if(data > 47 && data < 58)
			{				
				rxstate = 5;
				U2_re_temp[2] = data - 48;
			}
			else rxstate = 0;
		}
		else
		{	
			U1_re_temp[25] = _data_len;
			rxstate = 0;
			UART1_data_handle(); 
		}
	}
	//接收第一个数据高位第三位
	else if (rxstate == 5)
	{
		if(data != '$')
		{
			if(data > 47 && data < 58)
			{				
				rxstate = 6;
				U1_re_temp[3] = data - 48;				
			}
			else rxstate = 0;
		}
		else
		{	
			U1_re_temp[25] = _data_len;
			rxstate = 0;
			UART1_data_handle(); 
		}
	}
	//接收第一个数据低位第一位
	else if (rxstate == 6)
	{
		if(data != '$')
		{
			if(data > 47 && data < 58)
			{				
				rxstate = 7;
				U1_re_temp[4] = data - 48;
			}
			else rxstate = 0;
		}
		else
		{	
			U1_re_temp[25] = _data_len;
			rxstate = 0;
			UART1_data_handle(); 
		}
	}
	//接收第一个数据低位第二位
	else if (rxstate == 7)
	{
		if(data != '$')
		{
			if(data > 47 && data < 58)
			{				
				rxstate = 8;
				U1_re_temp[5] = data - 48;
			}
			else rxstate = 0;
		}
		else
		{	
			U1_re_temp[25] = _data_len;
			rxstate = 0;
			UART1_data_handle(); 
		}
	}	
	//接收第一个数据低位第三位
	else if (rxstate == 8)
	{
		if(data != '$')
		{
			if(data > 47 && data < 58)
			{				
				rxstate = 9;
				U1_re_temp[6] = data - 48;
				_data_len++;
			}
			else rxstate = 0;
		}
		else
		{	
			U1_re_temp[25] = _data_len;
			rxstate = 0;
			UART1_data_handle(); 
		}
	}
	//接收第二个数据高位第一位
	else if (rxstate == 9)
	{
		if(data != '$')
		{
			if(data > 47 && data < 58)
			{				
				rxstate = 10;
				U1_re_temp[7] = data - 48;
				
			}
			else rxstate = 0;
		}
		else
		{	
			U1_re_temp[25] = _data_len;
			rxstate = 0;
			UART1_data_handle(); 
		}
	}
	//接收接收第二个数据高位第二位
	else if (rxstate == 10)
	{
		if(data != '$')
		{
			if(data > 47 && data < 58)
			{				
				rxstate = 11;
				U1_re_temp[8] = data - 48;
			}
			else rxstate = 0;
		}
		else
		{	
			U1_re_temp[25] = _data_len;
			rxstate = 0;
			UART1_data_handle(); 
		}
	}	
	//接收接收第二个数据高位第三位
	else if (rxstate == 11)
	{
		if(data != '$')
		{
			if(data > 47 && data < 58)
			{				
				rxstate = 12;
				U1_re_temp[9] = data - 48;
			}
			else rxstate = 0;
		}
		else
		{	
			U1_re_temp[25] = _data_len;
			rxstate = 0;
			UART1_data_handle(); 
		}
	}
	//接收第二个数据低位第一位
	else if (rxstate == 12)
	{
		if(data != '$')
		{
			if(data > 47 && data < 58)
			{				
				rxstate = 13;
				U1_re_temp[10] = data - 48;				
			}
			else rxstate = 0;
		}
		else
		{	
			U1_re_temp[25] = _data_len;
			rxstate = 0;
			UART1_data_handle(); 
		}
	}
	//接收第二个数据低位第二位
	else if (rxstate == 13)
	{
		if(data != '$')
		{
			if(data > 47 && data < 58)
			{				
				rxstate = 14;
				U1_re_temp[11] = data - 48;
			}
			else rxstate = 0;
		}
		else
		{	
			U1_re_temp[25] = _data_len;
			rxstate = 0;
			UART1_data_handle(); 
		}
	}	
	//接收第二个数据低位第三位
	else if (rxstate == 14)
	{
		if(data != '$')
		{
			if(data > 47 && data < 58)
			{				
				rxstate = 15;
				U1_re_temp[12] = data - 48;
				_data_len++;
			}
			else rxstate = 0;
		}
		else
		{	
			U1_re_temp[25] = _data_len;
			rxstate = 0;
			UART1_data_handle(); 
		}
	}

	//接收第三个数据高位第一位
	else if (rxstate == 15)
	{
		if(data != '$')
		{
			if(data > 47 && data < 58)
			{				
				rxstate = 16;
				U1_re_temp[13] = data - 48;
				
			}
			else rxstate = 0;
		}
		else
		{	
			U1_re_temp[25] = _data_len;
			rxstate = 0;
			UART1_data_handle(); 
		}
	}
	//接收第三个数据高位第二位
	else if (rxstate == 16)
	{
		if(data != '$')
		{
			if(data > 47 && data < 58)
			{				
				rxstate = 17;
				U1_re_temp[14] = data - 48;
			}
			else rxstate = 0;
		}
		else
		{	
			U1_re_temp[25] = _data_len;
			rxstate = 0;
			UART1_data_handle(); 
		}
	}	
	//接收第三个数据高位第三位
	else if (rxstate == 17)
	{
		if(data != '$')
		{
			if(data > 47 && data < 58)
			{				
				rxstate = 18;
				U1_re_temp[15] = data - 48;
			}
			else rxstate = 0;
		}
		else
		{	
			U1_re_temp[25] = _data_len;
			rxstate = 0;
			UART1_data_handle(); 
		}
	}
	//接收第三个数据低位第一位
	else if (rxstate == 18)
	{
		if(data != '$')
		{
			if(data > 47 && data < 58)
			{				
				rxstate = 19;
				U1_re_temp[16] = data - 48;				
			}
			else rxstate = 0;
		}
		else
		{	
			U1_re_temp[25] = _data_len;
			rxstate = 0;
			UART1_data_handle(); 
		}
	}
	//接收第三个数据低位第二位
	else if (rxstate == 19)
	{
		if(data != '$')
		{
			if(data > 47 && data < 58)
			{				
				rxstate = 20;
				U1_re_temp[17] = data - 48;
			}
			else rxstate = 0;
		}
		else
		{	
			U1_re_temp[25] = _data_len;
			rxstate = 0;
			UART1_data_handle(); 
		}
	}	
	//接收第三个数据低位第三位
	else if (rxstate == 20)
	{
		if(data != '$')
		{
			if(data > 47 && data < 58)
			{				
				rxstate = 21;
				U1_re_temp[18] = data - 48;
				_data_len++;
			}
			else rxstate = 0;
		}
		else
		{	
			U1_re_temp[25] = _data_len;
			rxstate = 0;
			UART1_data_handle(); 
		}
	}
	//接收第四个数据高位第一位
	else if (rxstate == 21)
	{
		if(data != '$')
		{
			if(data > 47 && data < 58)
			{				
				rxstate = 22;
				U1_re_temp[19] = data - 48;
				
			}
			else rxstate = 0;
		}
		else
		{	
			U1_re_temp[25] = _data_len;
			rxstate = 0;
			UART1_data_handle(); 
		}
	}
	//接收第四个数据高位第二位
	else if (rxstate == 22)
	{
		if(data != '$')
		{
			if(data > 47 && data < 58)
			{				
				rxstate = 23;
				U1_re_temp[20] = data - 48;
			}
			else rxstate = 0;
		}
		else
		{	
			U1_re_temp[25] = _data_len;
			rxstate = 0;
			UART1_data_handle(); 
		}
	}	
	//接收第四个数据高位第三位
	else if (rxstate == 23)
	{
		if(data != '$')
		{
			if(data > 47 && data < 58)
			{				
				rxstate = 24;
				U1_re_temp[21] = data - 48;
			}
			else rxstate = 0;
		}
		else
		{	
			U1_re_temp[25] = _data_len;
			rxstate = 0;
			UART1_data_handle(); 
		}
	}
	//接收第四个数据低位第一位
	else if (rxstate == 24)
	{
		if(data != '$')
		{
			if(data > 47 && data < 58)
			{				
				rxstate = 25;
				U1_re_temp[22] = data - 48;				
			}
			else rxstate = 0;
		}
		else
		{	
			U1_re_temp[25] = _data_len;
			rxstate = 0;
			UART1_data_handle(); 
		}
	}
	//接收第四个数据低位第二位
	else if (rxstate == 25)
	{
		if(data != '$')
		{
			if(data > 47 && data < 58)
			{				
				rxstate = 26;
				U1_re_temp[23] = data - 48;
			}
			else rxstate = 0;
		}
		else
		{	
			U1_re_temp[25] = _data_len;
			rxstate = 0;
			UART1_data_handle(); 
		}
	}	
	//接收第四个数据低位第三位
	else if (rxstate == 26)
	{
		if(data != '$')
		{
			if(data > 47 && data < 58)
			{				
				rxstate = 0;
				U1_re_temp[24] = data - 48;
				_data_len++;
				U1_re_temp[25] = _data_len;				
				UART1_data_handle(); 
			}
			else rxstate = 0;
		}
		else
		{	
			U1_re_temp[25] = _data_len;
			rxstate = 0;
			UART1_data_handle(); 
		}
	}
}

void UART2_Receive_Prepare(u8 data) 	//串口2接收处理
{
	static u8 _data_len = 0;
	static u8 rxstate = 0;

	
	if (rxstate == 0 && data == 'O')
	{
		rxstate = 1;
	}
	
	else if (rxstate == 1)		//判断帧头是否是openmv的
	{
		if(data == 'P') rxstate = 2;
		else rxstate = 0;		
	}
//数据标识,A发姿态角，C色块，F巡线，D绕杆	
	else if (rxstate == 2)		
	{
		if(data == 'A' || data == 'C' || data == 'D' || data == 'F' )
		{
			rxstate = 3;
			U2_re_temp[0] = data;
			_data_len = 0;
		}
		else rxstate = 0;
	}
	//接收第一个数据高位第一位
	else if (rxstate == 3)
	{
		if(data != '$')
		{
			if(data > 47 && data < 58)
			{				
				rxstate = 4;
				U2_re_temp[1] = data - 48;
			}
			else rxstate = 0;
		}
		else
		{	
			U2_re_temp[25] = _data_len;
			rxstate = 0;
			UART2_data_handle(); 
		}
	}
			
	//接收第一个数据高位第二位
	else if (rxstate == 4)
	{
		if(data != '$')
		{
			if(data > 47 && data < 58)
			{				
				rxstate = 5;
				U2_re_temp[2] = data - 48;
			}
			else rxstate = 0;
		}
		else
		{	
			U2_re_temp[25] = _data_len;
			rxstate = 0;
			UART2_data_handle(); 
		}
	}
	//接收第一个数据高位第三位
	else if (rxstate == 5)
	{
		if(data != '$')
		{
			if(data > 47 && data < 58)
			{				
				rxstate = 6;
				U2_re_temp[3] = data - 48;				
			}
			else rxstate = 0;
		}
		else
		{	
			U2_re_temp[25] = _data_len;
			rxstate = 0;
			UART2_data_handle(); 
		}
	}
	//接收第一个数据低位第一位
	else if (rxstate == 6)
	{
		if(data != '$')
		{
			if(data > 47 && data < 58)
			{				
				rxstate = 7;
				U2_re_temp[4] = data - 48;
			}
			else rxstate = 0;
		}
		else
		{	
			U2_re_temp[25] = _data_len;
			rxstate = 0;
			UART2_data_handle(); 
		}
	}
	//接收第一个数据低位第二位
	else if (rxstate == 7)
	{
		if(data != '$')
		{
			if(data > 47 && data < 58)
			{				
				rxstate = 8;
				U2_re_temp[5] = data - 48;
			}
			else rxstate = 0;
		}
		else
		{	
			U2_re_temp[25] = _data_len;
			rxstate = 0;
			UART2_data_handle(); 
		}
	}	
	//接收第一个数据低位第三位
	else if (rxstate == 8)
	{
		if(data != '$')
		{
			if(data > 47 && data < 58)
			{				
				rxstate = 9;
				U2_re_temp[6] = data - 48;
				_data_len++;
			}
			else rxstate = 0;
		}
		else
		{	
			U2_re_temp[25] = _data_len;
			rxstate = 0;
			UART2_data_handle(); 
		}
	}
	//接收第二个数据高位第一位
	else if (rxstate == 9)
	{
		if(data != '$')
		{
			if(data > 47 && data < 58)
			{				
				rxstate = 10;
				U2_re_temp[7] = data - 48;
				
			}
			else rxstate = 0;
		}
		else
		{	
			U2_re_temp[25] = _data_len;
			rxstate = 0;
			UART2_data_handle(); 
		}
	}
	//接收接收第二个数据高位第二位
	else if (rxstate == 10)
	{
		if(data != '$')
		{
			if(data > 47 && data < 58)
			{				
				rxstate = 11;
				U2_re_temp[8] = data - 48;
			}
			else rxstate = 0;
		}
		else
		{	
			U2_re_temp[25] = _data_len;
			rxstate = 0;
			UART2_data_handle(); 
		}
	}	
	//接收接收第二个数据高位第三位
	else if (rxstate == 11)
	{
		if(data != '$')
		{
			if(data > 47 && data < 58)
			{				
				rxstate = 12;
				U2_re_temp[9] = data - 48;
			}
			else rxstate = 0;
		}
		else
		{	
			U2_re_temp[25] = _data_len;
			rxstate = 0;
			UART2_data_handle(); 
		}
	}
	//接收第二个数据低位第一位
	else if (rxstate == 12)
	{
		if(data != '$')
		{
			if(data > 47 && data < 58)
			{				
				rxstate = 13;
				U2_re_temp[10] = data - 48;				
			}
			else rxstate = 0;
		}
		else
		{	
			U2_re_temp[25] = _data_len;
			rxstate = 0;
			UART2_data_handle(); 
		}
	}
	//接收第二个数据低位第二位
	else if (rxstate == 13)
	{
		if(data != '$')
		{
			if(data > 47 && data < 58)
			{				
				rxstate = 14;
				U2_re_temp[11] = data - 48;
			}
			else rxstate = 0;
		}
		else
		{	
			U2_re_temp[25] = _data_len;
			rxstate = 0;
			UART2_data_handle(); 
		}
	}	
	//接收第二个数据低位第三位
	else if (rxstate == 14)
	{
		if(data != '$')
		{
			if(data > 47 && data < 58)
			{				
				rxstate = 15;
				U2_re_temp[12] = data - 48;
				_data_len++;
			}
			else rxstate = 0;
		}
		else
		{	
			U2_re_temp[25] = _data_len;
			rxstate = 0;
			UART2_data_handle(); 
		}
	}

	//接收第三个数据高位第一位
	else if (rxstate == 15)
	{
		if(data != '$')
		{
			if(data > 47 && data < 58)
			{				
				rxstate = 16;
				U2_re_temp[13] = data - 48;
				
			}
			else rxstate = 0;
		}
		else
		{	
			U2_re_temp[25] = _data_len;
			rxstate = 0;
			UART2_data_handle(); 
		}
	}
	//接收第三个数据高位第二位
	else if (rxstate == 16)
	{
		if(data != '$')
		{
			if(data > 47 && data < 58)
			{				
				rxstate = 17;
				U2_re_temp[14] = data - 48;
			}
			else rxstate = 0;
		}
		else
		{	
			U2_re_temp[25] = _data_len;
			rxstate = 0;
			UART2_data_handle(); 
		}
	}	
	//接收第三个数据高位第三位
	else if (rxstate == 17)
	{
		if(data != '$')
		{
			if(data > 47 && data < 58)
			{				
				rxstate = 18;
				U2_re_temp[15] = data - 48;
			}
			else rxstate = 0;
		}
		else
		{	
			U2_re_temp[25] = _data_len;
			rxstate = 0;
			UART2_data_handle(); 
		}
	}
	//接收第三个数据低位第一位
	else if (rxstate == 18)
	{
		if(data != '$')
		{
			if(data > 47 && data < 58)
			{				
				rxstate = 19;
				U2_re_temp[16] = data - 48;				
			}
			else rxstate = 0;
		}
		else
		{	
			U2_re_temp[25] = _data_len;
			rxstate = 0;
			UART2_data_handle(); 
		}
	}
	//接收第三个数据低位第二位
	else if (rxstate == 19)
	{
		if(data != '$')
		{
			if(data > 47 && data < 58)
			{				
				rxstate = 20;
				U2_re_temp[17] = data - 48;
			}
			else rxstate = 0;
		}
		else
		{	
			U2_re_temp[25] = _data_len;
			rxstate = 0;
			UART2_data_handle(); 
		}
	}	
	//接收第三个数据低位第三位
	else if (rxstate == 20)
	{
		if(data != '$')
		{
			if(data > 47 && data < 58)
			{				
				rxstate = 21;
				U2_re_temp[18] = data - 48;
				_data_len++;
			}
			else rxstate = 0;
		}
		else
		{	
			U2_re_temp[25] = _data_len;
			rxstate = 0;
			UART2_data_handle(); 
		}
	}
	//接收第四个数据高位第一位
	else if (rxstate == 21)
	{
		if(data != '$')
		{
			if(data > 47 && data < 58)
			{				
				rxstate = 22;
				U2_re_temp[19] = data - 48;
				
			}
			else rxstate = 0;
		}
		else
		{	
			U2_re_temp[25] = _data_len;
			rxstate = 0;
			UART2_data_handle(); 
		}
	}
	//接收第四个数据高位第二位
	else if (rxstate == 22)
	{
		if(data != '$')
		{
			if(data > 47 && data < 58)
			{				
				rxstate = 23;
				U2_re_temp[20] = data - 48;
			}
			else rxstate = 0;
		}
		else
		{	
			U2_re_temp[25] = _data_len;
			rxstate = 0;
			UART2_data_handle(); 
		}
	}	
	//接收第四个数据高位第三位
	else if (rxstate == 23)
	{
		if(data != '$')
		{
			if(data > 47 && data < 58)
			{				
				rxstate = 24;
				U2_re_temp[21] = data - 48;
			}
			else rxstate = 0;
		}
		else
		{	
			U2_re_temp[25] = _data_len;
			rxstate = 0;
			UART2_data_handle(); 
		}
	}
	//接收第四个数据低位第一位
	else if (rxstate == 24)
	{
		if(data != '$')
		{
			if(data > 47 && data < 58)
			{				
				rxstate = 25;
				U2_re_temp[22] = data - 48;				
			}
			else rxstate = 0;
		}
		else
		{	
			U2_re_temp[25] = _data_len;
			rxstate = 0;
			UART2_data_handle(); 
		}
	}
	//接收第四个数据低位第二位
	else if (rxstate == 25)
	{
		if(data != '$')
		{
			if(data > 47 && data < 58)
			{				
				rxstate = 26;
				U2_re_temp[23] = data - 48;
			}
			else rxstate = 0;
		}
		else
		{	
			U2_re_temp[25] = _data_len;
			rxstate = 0;
			UART2_data_handle(); 
		}
	}	
	//接收第四个数据低位第三位
	else if (rxstate == 26)
	{
		if(data != '$')
		{
			if(data > 47 && data < 58)
			{				
				rxstate = 0;
				U2_re_temp[24] = data - 48;
				_data_len++;
				U2_re_temp[25] = _data_len;				
				UART2_data_handle(); 
			}
			else rxstate = 0;
		}
		else
		{	
			U2_re_temp[25] = _data_len;
			rxstate = 0;
			UART2_data_handle(); 
		}
	}
}

void UART3_Receive_Prepare(u8 data) 	//串口3接收处理
{
	static u8 _data_len = 0;
	static u8 rxstate = 0;

	
	if (rxstate == 0 && data == 'K')
	{
		rxstate = 1;
	}
	
	else if (rxstate == 1)		//判断帧头是否是K210的
	{
		if(data == 'R') rxstate = 2;
		else rxstate = 0;		
	}
/*数据标识
	D识别颜色和形状数据
	标识后第一二位依次为存储的目标形状和颜色
				第二三位依次为存储的目标形状和颜色
	形状：0无效，1圆，2正方形，3三角形
	颜色：0无效，1红，2蓝
	
	L为K210视觉学习状态，仅第一位
	0未开始，1正在学习处理，2完成，3已失败
	
	Q二维码数据*/
	else if (rxstate == 2)		
	{
		if(data == 'D' || data == 'L' || data == 'Q')
		{
			rxstate = 3;
			U3_re_temp[0] = data;
			_data_len = 0;
		}
		else rxstate = 0;
	}
	//接收第一个数据高位第一位
	else if (rxstate == 3)
	{
		if(data != '$')
		{
			if(data > 47 && data < 58)
			{				
				rxstate = 4;
				U3_re_temp[1] = data - 48;
			}
			else rxstate = 0;
		}
		else
		{	
			U3_re_temp[25] = _data_len;
			rxstate = 0;
			UART3_data_handle(); 
		}
	}
			
	//接收第一个数据高位第二位
	else if (rxstate == 4)
	{
		if(data != '$')
		{
			if(data > 47 && data < 58)
			{				
				rxstate = 5;
				U3_re_temp[2] = data - 48;
			}
			else rxstate = 0;
		}
		else
		{	
			U3_re_temp[25] = _data_len;
			rxstate = 0;
			UART3_data_handle(); 
		}
	}
	//接收第一个数据高位第三位
	else if (rxstate == 5)
	{
		if(data != '$')
		{
			if(data > 47 && data < 58)
			{				
				rxstate = 6;
				U3_re_temp[3] = data - 48;				
			}
			else rxstate = 0;
		}
		else
		{	
			U3_re_temp[25] = _data_len;
			rxstate = 0;
			UART3_data_handle(); 
		}
	}
	//接收第一个数据低位第一位
	else if (rxstate == 6)
	{
		if(data != '$')
		{
			if(data > 47 && data < 58)
			{				
				rxstate = 7;
				U3_re_temp[4] = data - 48;
			}
			else rxstate = 0;
		}
		else
		{	
			U3_re_temp[25] = _data_len;
			rxstate = 0;
			UART3_data_handle(); 
		}
	}
	//接收第一个数据低位第二位
	else if (rxstate == 7)
	{
		if(data != '$')
		{
			if(data > 47 && data < 58)
			{				
				rxstate = 8;
				U3_re_temp[5] = data - 48;
			}
			else rxstate = 0;
		}
		else
		{	
			U3_re_temp[25] = _data_len;
			rxstate = 0;
			UART3_data_handle(); 
		}
	}	
	//接收第一个数据低位第三位
	else if (rxstate == 8)
	{
		if(data != '$')
		{
			if(data > 47 && data < 58)
			{				
				rxstate = 9;
				U3_re_temp[6] = data - 48;
				_data_len++;
			}
			else rxstate = 0;
		}
		else
		{	
			U3_re_temp[25] = _data_len;
			rxstate = 0;
			UART3_data_handle(); 
		}
	}
	//接收第二个数据高位第一位
	else if (rxstate == 9)
	{
		if(data != '$')
		{
			if(data > 47 && data < 58)
			{				
				rxstate = 10;
				U3_re_temp[7] = data - 48;
				
			}
			else rxstate = 0;
		}
		else
		{	
			U3_re_temp[25] = _data_len;
			rxstate = 0;
			UART3_data_handle(); 
		}
	}
	//接收接收第二个数据高位第二位
	else if (rxstate == 10)
	{
		if(data != '$')
		{
			if(data > 47 && data < 58)
			{				
				rxstate = 11;
				U3_re_temp[8] = data - 48;
			}
			else rxstate = 0;
		}
		else
		{	
			U3_re_temp[25] = _data_len;
			rxstate = 0;
			UART3_data_handle(); 
		}
	}	
	//接收接收第二个数据高位第三位
	else if (rxstate == 11)
	{
		if(data != '$')
		{
			if(data > 47 && data < 58)
			{				
				rxstate = 12;
				U3_re_temp[9] = data - 48;
			}
			else rxstate = 0;
		}
		else
		{	
			U3_re_temp[25] = _data_len;
			rxstate = 0;
			UART3_data_handle(); 
		}
	}
	//接收第二个数据低位第一位
	else if (rxstate == 12)
	{
		if(data != '$')
		{
			if(data > 47 && data < 58)
			{				
				rxstate = 13;
				U3_re_temp[10] = data - 48;				
			}
			else rxstate = 0;
		}
		else
		{	
			U3_re_temp[25] = _data_len;
			rxstate = 0;
			UART3_data_handle(); 
		}
	}
	//接收第二个数据低位第二位
	else if (rxstate == 13)
	{
		if(data != '$')
		{
			if(data > 47 && data < 58)
			{				
				rxstate = 14;
				U3_re_temp[11] = data - 48;
			}
			else rxstate = 0;
		}
		else
		{	
			U3_re_temp[25] = _data_len;
			rxstate = 0;
			UART3_data_handle(); 
		}
	}	
	//接收第二个数据低位第三位
	else if (rxstate == 14)
	{
		if(data != '$')
		{
			if(data > 47 && data < 58)
			{				
				rxstate = 15;
				U3_re_temp[12] = data - 48;
				_data_len++;
			}
			else rxstate = 0;
		}
		else
		{	
			U3_re_temp[25] = _data_len;
			rxstate = 0;
			UART3_data_handle(); 
		}
	}

	//接收第三个数据高位第一位
	else if (rxstate == 15)
	{
		if(data != '$')
		{
			if(data > 47 && data < 58)
			{				
				rxstate = 16;
				U3_re_temp[13] = data - 48;
				
			}
			else rxstate = 0;
		}
		else
		{	
			U3_re_temp[25] = _data_len;
			rxstate = 0;
			UART3_data_handle(); 
		}
	}
	//接收第三个数据高位第二位
	else if (rxstate == 16)
	{
		if(data != '$')
		{
			if(data > 47 && data < 58)
			{				
				rxstate = 17;
				U3_re_temp[14] = data - 48;
			}
			else rxstate = 0;
		}
		else
		{	
			U3_re_temp[25] = _data_len;
			rxstate = 0;
			UART3_data_handle(); 
		}
	}	
	//接收第三个数据高位第三位
	else if (rxstate == 17)
	{
		if(data != '$')
		{
			if(data > 47 && data < 58)
			{				
				rxstate = 18;
				U3_re_temp[15] = data - 48;
			}
			else rxstate = 0;
		}
		else
		{	
			U3_re_temp[25] = _data_len;
			rxstate = 0;
			UART3_data_handle(); 
		}
	}
	//接收第三个数据低位第一位
	else if (rxstate == 18)
	{
		if(data != '$')
		{
			if(data > 47 && data < 58)
			{				
				rxstate = 19;
				U3_re_temp[16] = data - 48;				
			}
			else rxstate = 0;
		}
		else
		{	
			U3_re_temp[25] = _data_len;
			rxstate = 0;
			UART3_data_handle(); 
		}
	}
	//接收第三个数据低位第二位
	else if (rxstate == 19)
	{
		if(data != '$')
		{
			if(data > 47 && data < 58)
			{				
				rxstate = 20;
				U3_re_temp[17] = data - 48;
			}
			else rxstate = 0;
		}
		else
		{	
			U3_re_temp[25] = _data_len;
			rxstate = 0;
			UART3_data_handle(); 
		}
	}	
	//接收第三个数据低位第三位
	else if (rxstate == 20)
	{
		if(data != '$')
		{
			if(data > 47 && data < 58)
			{				
				rxstate = 21;
				U3_re_temp[18] = data - 48;
				_data_len++;
			}
			else rxstate = 0;
		}
		else
		{	
			U3_re_temp[25] = _data_len;
			rxstate = 0;
			UART3_data_handle(); 
		}
	}
	//接收第四个数据高位第一位
	else if (rxstate == 21)
	{
		if(data != '$')
		{
			if(data > 47 && data < 58)
			{				
				rxstate = 22;
				U3_re_temp[19] = data - 48;
				
			}
			else rxstate = 0;
		}
		else
		{	
			U3_re_temp[25] = _data_len;
			rxstate = 0;
			UART3_data_handle(); 
		}
	}
	//接收第四个数据高位第二位
	else if (rxstate == 22)
	{
		if(data != '$')
		{
			if(data > 47 && data < 58)
			{				
				rxstate = 23;
				U3_re_temp[20] = data - 48;
			}
			else rxstate = 0;
		}
		else
		{	
			U3_re_temp[25] = _data_len;
			rxstate = 0;
			UART3_data_handle(); 
		}
	}	
	//接收第四个数据高位第三位
	else if (rxstate == 23)
	{
		if(data != '$')
		{
			if(data > 47 && data < 58)
			{				
				rxstate = 24;
				U3_re_temp[21] = data - 48;
			}
			else rxstate = 0;
		}
		else
		{	
			U3_re_temp[25] = _data_len;
			rxstate = 0;
			UART3_data_handle(); 
		}
	}
	//接收第四个数据低位第一位
	else if (rxstate == 24)
	{
		if(data != '$')
		{
			if(data > 47 && data < 58)
			{				
				rxstate = 25;
				U3_re_temp[22] = data - 48;				
			}
			else rxstate = 0;
		}
		else
		{	
			U3_re_temp[25] = _data_len;
			rxstate = 0;
			UART3_data_handle(); 
		}
	}
	//接收第四个数据低位第二位
	else if (rxstate == 25)
	{
		if(data != '$')
		{
			if(data > 47 && data < 58)
			{				
				rxstate = 26;
				U3_re_temp[23] = data - 48;
			}
			else rxstate = 0;
		}
		else
		{	
			U3_re_temp[25] = _data_len;
			rxstate = 0;
			UART3_data_handle(); 
		}
	}	
	//接收第四个数据低位第三位
	else if (rxstate == 26)
	{
		if(data != '$')
		{
			if(data > 47 && data < 58)
			{				
				rxstate = 0;
				U3_re_temp[24] = data - 48;
				_data_len++;
				U3_re_temp[25] = _data_len;				
				UART3_data_handle(); 
			}
			else rxstate = 0;
		}
		else
		{	
			U3_re_temp[25] = _data_len;
			rxstate = 0;
			UART3_data_handle(); 
		}
	}
}
