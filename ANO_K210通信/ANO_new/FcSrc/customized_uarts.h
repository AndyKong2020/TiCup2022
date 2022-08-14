#include "SysConfig.h"

#ifndef _CUSTOMIZED_UARTS_H_
#define _CUSTOMIZED_UARTS_H_

extern s16 color_identify_data[3]; 	//openmv色块识别xyz误差量*10000

extern s16 follow_line_data[2]; 		//openmv巡线角度*100，截距*10000

extern s16 detour_pole_data[2]; 		//openmv绕杆误差量*10000

extern s16 K210_Identify_data[4];		//K210识别数据
/*第一二位依次为存储的目标形状和颜色
	第二三位依次为目前识别的形状和颜色
	形状：0无效，1圆，2正方形，3三角形
	颜色：0无效，1红，2蓝*/

extern s16 K210_learn_state;			//K210学习状态
/*L为K210视觉学习状态，仅第一位
	0未开始，1正在学习处理，2完成，3已失败*/
	
extern s16 QR_data[3]; 							//二维码数据

void UART1_Receive_Prepare(u8 data);
void UART1_data_handle(void);				

void UART2_Receive_Prepare(u8 data);
void UART2_data_handle(void);					

void UART3_Receive_Prepare(u8 data);
void UART3_data_handle(void);					//自定义串口数据处理

extern u8 U1_re_temp[26];							//自定义uart2接收缓存

extern u8 U2_re_temp[26];							//自定义uart2接收缓存

extern u8 U3_re_temp[26];							//自定义uart2接收缓存

#endif
