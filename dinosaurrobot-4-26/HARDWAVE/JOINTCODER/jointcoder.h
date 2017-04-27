#ifndef __JOINTCODER_H
#define __JOINTCODER_H	
#include "sys.h"

void jointCoderInit(void);
u16  Get_Adc(u8 ch); 
u16 getJointCoder(u8 ch);
 
#endif 
