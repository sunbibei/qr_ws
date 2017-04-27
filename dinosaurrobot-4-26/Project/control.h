#ifndef __CONTROL_H
#define __CONTROL_H

#define Joint_Num 3
#define Knee_Maxangle 242
#define Knee_Minangle 147
#define Hip_Maxangle 187
#define Hip_Minangle 87
#define Yaw_Maxangle 137
#define Yaw_Minangle 75

void PID_control(int current_command,int current_coder,int ID);
void PID_init(float P,float I,float D,int ID);
void ControlProcess();

struct PID_Struct
{
	float Kp;
	float Kd;
	float Ki;
	float err;
	float last_err;
	float SetPos;
	float ActualPos;
	float result;
};
struct Ctl_Msg_Struct
{
	int frameType;	
	int length;
	int jointangle;
};

extern struct Ctl_Msg_Struct Ctl_Msgs[Joint_Num];
extern struct PID_Struct PID[Joint_Num];
#endif
