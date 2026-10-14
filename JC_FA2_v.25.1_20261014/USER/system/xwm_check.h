#ifndef XWM_CHECK_H
#define XWM_CHECK_H
#include "sys_def.h"
// 
#define HALL_XWM_PORT       PortB
#define HALL_XWM_PIN        Pin01

#define EN_XWM_PORT         PortA
#define EN_XWM_PIN          Pin11


#define XWM_HIGH_LEVEL      1
#define XWM_LOW_LEVEL       0



typedef enum
{
    SYMBOL_UNKNOW,
    SYMBOL_HIGH,
    SYMBOL_LOW
} XWM_state;

void XWM_Check_Process(void);
void WXM_Sample(void);
static bool xwm_window_check(void);
void XWM_Send_EN(XWM_state symbol);
void XWM_NotTriggered(void);
void XWM_Triggered(void);

#endif
