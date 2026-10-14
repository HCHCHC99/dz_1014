#ifndef MOTOR_CMD_H
#define MOTOR_CMD_H

#include "sys_def.h"
#include "hz_timer.h"

// 硬件IO定义（电机模块专用）
#define MOTOR_PWM_PORT   GPIO_PORT_C
#define MOTOR_PWM_PIN    GPIO_PIN_0  
#define MOTOR_DIR_PORT   GPIO_PORT_C
#define MOTOR_DIR_PIN    GPIO_PIN_1    

#define MOTOR_SLOWSTART_TIME        0
#define MOTOR_POSITIVE_TIME         0
#define MOTOR_NEGATIVE_TIME         0
#define MOTOR_POSITIVE_CUR_VALID_TIME 50  //ms
#define MOTOR_NEGATIVE_CUR_VALID_TIME 50

#define MOTOR_MAX_DUTY      100.0f   // 最大运行占空比

extern NonBlockingDelay_t Motor_slowstart_Delay;

// 电压锁相关接口（使用 g_sys_ctx 中的电压锁变量）
void MotorCmd_VoltageLock_Set(VoltageLimitState fault_type);
void MotorCmd_VoltageLock_Clear(void);
bool MotorCmd_VoltageLock_GetStatus(void);
VoltageLimitState MotorCmd_VoltageLock_GetFaultType(void);

// 接口函数
void MotorCmd_Init(void);
SystemError MotorCmd_Execute(PowerPolarity power_input, RodPosition pos_input, 
                             CurrentLimitState current_input, VoltageLimitState voltage_input);
MotorCmdState MotorCmd_GetState(bool *executed);
void MotorCmd_StopImmediately(void);
CurrentLimitValidState MotorCurLimit_GetState(bool *executed);

#endif // MOTOR_CMD_H