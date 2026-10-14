#include "motor_cmd.h"
#include "hardware.h"
#include "gpio_adapter.h"
#include "hc32f46x_gpio.h"
#include "sys_def.h"
#include "main.h"
#include "adapter_pwm.h"
#include "hz_nopwm.h"
#include "hz_timer.h"
#include "motor_adc.h"
#include "dev_pwm.h"

extern SystemContext g_sys_ctx;

// 模块内部静态变量
static MotorCmdState s_motor_cmd = MOTOR_STOP;
static bool s_motor_executed = false;
static MotorCmdState s_last_motor_cmd = MOTOR_STOP;
static float s_current_duty = 0.0f;

// 延时相关
static NonBlockingDelay_t Motor_slowstart_Delay;
static NonBlockingDelay_t Motor_Positive_Delay;
static NonBlockingDelay_t Motor_Negative_Delay;
static NonBlockingDelay_t Motor_positive_cur_valid_Delay;
static NonBlockingDelay_t Motor_negative_cur_valid_Delay;
static CurrentLimitValidState s_cur_limit_valid = CURRENT_LIMIT_VALID_ON;

// ========== 电压锁接口实现 ==========
void MotorCmd_VoltageLock_Set(VoltageLimitState fault_type)
{
    g_sys_ctx.voltage_lock = true;
    g_sys_ctx.voltage_fault_type = fault_type;
    
    // 上锁的同时立即停止电机
    Stop();
    s_cur_limit_valid = CURRENT_LIMIT_VALID_ON;
    s_motor_cmd = MOTOR_STOP;
    s_last_motor_cmd = MOTOR_STOP;
    s_current_duty = 0.0f;
}

void MotorCmd_VoltageLock_Clear(void)
{
    g_sys_ctx.voltage_lock = false;
    g_sys_ctx.voltage_fault_type = VOLTAGE_LIMIT_NONE;
}

bool MotorCmd_VoltageLock_GetStatus(void)
{
    return g_sys_ctx.voltage_lock;
}

VoltageLimitState MotorCmd_VoltageLock_GetFaultType(void)
{
    return g_sys_ctx.voltage_fault_type;
}

void MotorCmd_Init(void) {
    s_motor_cmd = MOTOR_STOP;
    s_last_motor_cmd = MOTOR_STOP;
    s_motor_executed = false;
    s_current_duty = 0.0f;
    s_cur_limit_valid = CURRENT_LIMIT_VALID_ON;
    
    nbDelay_Init(&Motor_slowstart_Delay, MOTOR_SLOWSTART_TIME*1000);
    nbDelay_Init(&Motor_Positive_Delay, MOTOR_POSITIVE_TIME);
    nbDelay_Init(&Motor_Negative_Delay, MOTOR_NEGATIVE_TIME);    
    nbDelay_Init(&Motor_positive_cur_valid_Delay, MOTOR_POSITIVE_CUR_VALID_TIME);
    nbDelay_Init(&Motor_negative_cur_valid_Delay, MOTOR_NEGATIVE_CUR_VALID_TIME);    
}

// 执行电机控制 - 包含电压锁检查
SystemError MotorCmd_Execute(PowerPolarity power_input, RodPosition pos_input, 
                             CurrentLimitState current_input, VoltageLimitState voltage_input) 
{
    SystemError err = ERROR_NONE;
    s_motor_executed = false;

    // ========== 第一步：检查电压锁状态 ==========
    if (g_sys_ctx.voltage_lock) {
        // 电压锁定时，电机强制停止
        Stop();
        s_cur_limit_valid = CURRENT_LIMIT_VALID_ON;
        s_motor_cmd = MOTOR_STOP;
        s_last_motor_cmd = MOTOR_STOP;
        s_current_duty = 0.0f;
        
        // 根据故障类型返回对应错误码
        if (g_sys_ctx.voltage_fault_type == VOLTAGE_LIMIT_OVER) {
            return ERROR_OVER_VOLTAGE;
        } else if (g_sys_ctx.voltage_fault_type == VOLTAGE_LIMIT_UNDER) {
            return ERROR_UNDER_VOLTAGE;
        }
        return ERROR_CMD_INVALID;
    }

    // ========== 第二步：检查传入的电压限位状态 ==========
    if (voltage_input == VOLTAGE_LIMIT_OVER) {
        MotorCmd_VoltageLock_Set(VOLTAGE_LIMIT_OVER);
        return ERROR_OVER_VOLTAGE;
    }
    
    if (voltage_input == VOLTAGE_LIMIT_UNDER) {
        MotorCmd_VoltageLock_Set(VOLTAGE_LIMIT_UNDER);
        return ERROR_UNDER_VOLTAGE;
    }

    // ========== 第三步：位置错误处理 ==========
    if (pos_input == ROD_POS_ERROR) {
        Stop();
        s_cur_limit_valid = CURRENT_LIMIT_VALID_ON;          
        s_motor_cmd = MOTOR_STOP;
        s_last_motor_cmd = MOTOR_STOP;
        s_current_duty = 0.0f;
        return ERROR_HALL_SAMPLE_FAIL;
    }
    
    // ========== 第四步：正常控制逻辑 ==========
    switch (power_input) {
        case POWER_POSITIVE:
            if (pos_input != ROD_POS_UPPER && current_input != CURRENT_LIMITING_POSITIVE) {
                if (s_last_motor_cmd != MOTOR_RUN_EXTEND) {
                    Positive_Run();             
                    s_cur_limit_valid = CURRENT_LIMIT_VALID_OFF_POSITIVE;                           
                    nbDelay_Start(&Motor_positive_cur_valid_Delay);
                }
                if(nbDelay_IsComplete(&Motor_positive_cur_valid_Delay)) {
                    s_cur_limit_valid = CURRENT_LIMIT_VALID_ON;                    
                }
                All_DevicePwm_Update();    
                s_motor_cmd = MOTOR_RUN_EXTEND;
            } else {
                s_cur_limit_valid = CURRENT_LIMIT_VALID_ON;                  
                Stop();              
                s_motor_cmd = MOTOR_STOP;
                s_current_duty = 0.0f;
            }
            break;
            
        case POWER_NEGATIVE:
            if (pos_input != ROD_POS_LOWER && current_input != CURRENT_LIMITING_NEGATIVE) {
                if (s_last_motor_cmd != MOTOR_RUN_RETRACT) {
                    s_cur_limit_valid = CURRENT_LIMIT_VALID_OFF_NEGATIVE;                    
                    Negative_Run();                    
                    nbDelay_Start(&Motor_negative_cur_valid_Delay);
                }
                if(nbDelay_IsComplete(&Motor_negative_cur_valid_Delay)) {
                    s_cur_limit_valid = CURRENT_LIMIT_VALID_ON;
                }
                All_DevicePwm_Update();
                s_motor_cmd = MOTOR_RUN_RETRACT;
            } else {
                Stop();     
                s_cur_limit_valid = CURRENT_LIMIT_VALID_ON;                
                s_motor_cmd = MOTOR_STOP;
                s_current_duty = 0.0f;
            }
            break;
        case POWER_DOWN:
        case POWER_POSITIVE_TO_NEGATIVE:
        case POWER_NEGATIVE_TO_POSITIVE:
            Stop();
            s_cur_limit_valid = CURRENT_LIMIT_VALID_ON;            
            s_motor_cmd = MOTOR_STOP;
            s_current_duty = 0.0f;
            break;
        default:
            Stop();      
            s_cur_limit_valid = CURRENT_LIMIT_VALID_ON;                
            s_motor_cmd = MOTOR_STOP;
            s_current_duty = 0.0f;
            err = (power_input >= POWER_MAX) ? ERROR_CMD_INVALID : ERROR_NONE;
            break;
    }
    
    // 更新状态记录
    s_last_motor_cmd = s_motor_cmd;
    if (s_motor_cmd != MOTOR_STOP) {
        s_current_duty = (s_current_duty < MOTOR_MAX_DUTY) ? 
                         s_current_duty + 0.5f : MOTOR_MAX_DUTY;
    }
    
    s_motor_executed = (err == ERROR_NONE);
    return err;
}

MotorCmdState MotorCmd_GetState(bool *executed) {
    if (executed) *executed = s_motor_executed;
    return s_motor_cmd;
}

CurrentLimitValidState MotorCurLimit_GetState(bool *executed) {
    if (executed) {
        *executed = true;
    }
    return s_cur_limit_valid;
}

void MotorCmd_StopImmediately(void) {
    // 如果电压锁已锁定，不执行停止（电机已经停了）
    if (g_sys_ctx.voltage_lock) {
        return;
    }
    
    Stop(); 
    s_cur_limit_valid = CURRENT_LIMIT_VALID_ON;        
    s_motor_cmd = MOTOR_STOP;
    s_last_motor_cmd = MOTOR_STOP;
    s_current_duty = 0.0f;
}
