#ifndef __USER_TASK_H
#define __USER_TASK_H

#include "SysConfig.h" 

void UserTask_OneKeyCmd(void);

void follow_line(void);
void find_target_test(void);
void find_target(void);

void mission_start(void);

void control(s16 yawdps,s16 pitch,s16 roll,s16 height);

s16 height_speed(u8 target_height);

s16 pitch_speed(void);

s16 roll_speed(void);

extern u16 i;

extern s16 yaw_1,position;

extern s16 xa,ya;

extern s16 a,b;

extern u8 mission_step;
	
#endif
