#ifndef __MAIN_H
#define __MAIN_H

#include "stm32f10x.h"
#include "stm32f10x_it.h"
#include "sys.h"
#include "stdio.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "can.h"
#include "motor.h"
#include "jointcoder.h"
#include "iwdg.h"
#include "control.h"//TODO

extern uint16_t timeCountMs,iwdgFeedTime,ctlPeroidTime;
extern uint8_t driverCheckPeriod,can1SendPeriod;
extern u16 kneeAngle,hipAngle,yawAngle;//TODO
extern int16_t knee_pos_target;
extern int16_t hip_pos_target;
extern int16_t yaw_pos_target;
//extern int16_t pos_current;
#define abs(x) ((x)>0? (x):(-(x))

#endif
