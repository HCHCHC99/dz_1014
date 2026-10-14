#include "sys_main_state.h"
#include "power.h"
#include "hall_position.h"
#include "motor_cmd.h"
#include "system.h"
#include "main.h"
#include "hall_position.h"
#include "hz_timer.h"
#include "hardware.h"
#include "adapter_pwm.h"
#include "main.h"
#include "adc_adapter.h"
#include "sys_def.h"
#include "usart_usb.h"
#include "motor_adc.h"
#include "dev_pwm.h"
#include "device_manager.h"

// 全局系统上下文定义
SystemContext g_sys_ctx;
static int x =0;
#if hz_usart1_debug
NonBlockingDelay_t Usart1_Usb_Send_Delay;
        static uint8_t firstSend = 1;
#endif

volatile int fault_num = 0;

// 欠压阻塞相关变量
volatile int undervoltage_num = 0;
static NonBlockingDelay_t UnderVoltage_Block_Delay;

// ========== 电压限位状态（用于传递给 MotorCmd_Execute） ==========
static VoltageLimitState s_voltage_limit_state = VOLTAGE_LIMIT_NONE;

// ========== 电流校准相关变量（移到这里，方便状态机访问） ==========
static float s_calibration_offset = 0.0f;
static uint16_t s_calibration_samples[SAMPLE_BUFFER_SIZE_cur] = {0};
static uint32_t s_calibration_count = 0;

// 辅助函数：处理断电状态
static void handle_power_down_state(void) {
    switch (g_sys_ctx.power_sample_state) {
        case SAMPLE_POSITIVE:
            g_sys_ctx.power_state = POWER_POSITIVE;
            g_sys_ctx.power_valid = 1;
            g_sys_ctx.main_state = SYS_STATE_HALL_SAMPLE;
            break;
        case SAMPLE_NEGATIVE:
            g_sys_ctx.power_state = POWER_NEGATIVE;
            g_sys_ctx.power_valid = 1;
            g_sys_ctx.main_state = SYS_STATE_HALL_SAMPLE;
            break;
        case SAMPLE_POWER_DOWN:
            g_sys_ctx.main_state = SYS_STATE_HALL_SAMPLE;
            break;
        case SAMPLE_ERROR:
            g_sys_ctx.power_state = POWER_ERROR;
            g_sys_ctx.main_state = SYS_STATE_HALL_SAMPLE;
            break;
        default:
            break;
    }
}

// 辅助函数：处理未确定状态
static void handle_undetermined_state(void) {
    switch (g_sys_ctx.power_sample_state) {
        case SAMPLE_POSITIVE:
            g_sys_ctx.power_state = POWER_POSITIVE;
            g_sys_ctx.power_valid = 1;
            g_sys_ctx.main_state = SYS_STATE_HALL_SAMPLE;
            break;
        case SAMPLE_NEGATIVE:
            g_sys_ctx.power_state = POWER_NEGATIVE;
            g_sys_ctx.power_valid = 1;
            g_sys_ctx.main_state = SYS_STATE_HALL_SAMPLE;
            break;
        case SAMPLE_UNDETERMINED:
            break;
        case SAMPLE_ERROR:
            g_sys_ctx.power_state = POWER_ERROR;
            g_sys_ctx.main_state = SYS_STATE_HALL_SAMPLE;
            break;
        default:
            break;
    }
}

// 辅助函数：处理稳定电源状态（正/负）
static void handle_stable_power_state(void) {
    if (g_sys_ctx.power_sample_state == SAMPLE_ERROR) {
        g_sys_ctx.power_state = POWER_ERROR;
        g_sys_ctx.main_state = SYS_STATE_HALL_SAMPLE;
        return;
    }

    if (g_sys_ctx.power_sample_state == SAMPLE_POWER_DOWN) {
        g_sys_ctx.power_state = POWER_DOWN;
        g_sys_ctx.main_state = SYS_STATE_HALL_SAMPLE;
        return;
    }

    switch (g_sys_ctx.power_sample_state) {
        case SAMPLE_POSITIVE:
            g_sys_ctx.power_state = POWER_POSITIVE;
            g_sys_ctx.power_valid = 1;
            break;
        case SAMPLE_NEGATIVE:
            g_sys_ctx.power_state = POWER_NEGATIVE;
            g_sys_ctx.power_valid = 1;
            break;
        default:
            break;
    }
    g_sys_ctx.main_state = SYS_STATE_HALL_SAMPLE;
}

void SysMainState_Init(void) {
    memset(&g_sys_ctx, 0, sizeof(SystemContext));
    g_sys_ctx.main_state = SYS_STATE_INIT;
    g_sys_ctx.Motor_Current = 0.0f;
    g_sys_ctx.Motor_Voltage = 0.0f;
    g_sys_ctx.Voltage_Limit_Upper = THRESHOLD_VOLTAGE_OVER;
    g_sys_ctx.Voltage_Limit_Lower = THRESHOLD_VOLTAGE_UNDER;
    g_sys_ctx.Current_Limit_Upper = THRESHOLD_CURRENT_OVER;
    g_sys_ctx.Current_Limit_Recovery = THRESHOLD_CURRENT_RECOVERY;
    
    // ========== 初始化电流校准 ==========
    g_sys_ctx.current_offset = 0.0f;
    g_sys_ctx.current_calib_done = false;
    s_calibration_offset = 0.0f;
    s_calibration_count = 0;
    memset(s_calibration_samples, 0, sizeof(s_calibration_samples));
    
    // ========== 初始化电压锁 ==========
    g_sys_ctx.voltage_lock = false;
    g_sys_ctx.voltage_fault_type = VOLTAGE_LIMIT_NONE;
    s_voltage_limit_state = VOLTAGE_LIMIT_NONE;
    
    nbDelay_Init(&UnderVoltage_Block_Delay, 2000);
    
#if hz_usart1_debug
    nbDelay_Init(&Usart1_Usb_Send_Delay, USART1_USB_SEND_TIME*1000);
#endif
}

// 主状态机处理（核心流程控制）
void SysMainState_Process(void) {
    switch (g_sys_ctx.main_state) {
        case SYS_STATE_INIT:
            MotorCmd_Init();
            ADC_Init_hz();  // 确保ADC已初始化
            OC_Init();      // start 50us OC sampling (2026-08-22)
            g_sys_ctx.main_state = SYS_STATE_CURRENT_CALIB;
            break;
            
            case SYS_STATE_CURRENT_CALIB: {
                // ========== 使用 motor_adc.c 中的校准接口 ==========
                
                // 1. 启动校准
                ADC_CURRENT_Calibration_Start();
                
                // 2. 阻塞等待校准完成
                while (1) {
                    bool calib_done = ADC_CURRENT_Calibration_Process();
                    if (calib_done) {
                        break;
                    }
                    // 如果不需要完全阻塞，可以加入小延时防止看门狗
                    // tickTimer_DelayMs(1);
                }
                
                // 3. 获取校准结果
                g_sys_ctx.current_offset = ADC_CURRENT_GetOffset();
                g_sys_ctx.current_calib_done = true;
                OC_UpdateThresholds();   // update OC thresholds after calibration (2026-08-22)
                g_sys_ctx.main_state = SYS_STATE_POWER_SAMPLE;
                break;
            }
            
        case SYS_STATE_POWER_SAMPLE: {
            // ========== 检查电压锁：如果 power_state == POWER_DOWN，清除电压锁 ==========
            if (g_sys_ctx.voltage_lock) {
                if (g_sys_ctx.power_state == POWER_DOWN) {
                    // 清除电压锁
                    g_sys_ctx.voltage_lock = false;
                    g_sys_ctx.voltage_fault_type = VOLTAGE_LIMIT_NONE;
                    s_voltage_limit_state = VOLTAGE_LIMIT_NONE;
                }
            }
            
            SystemError err = Power_IOSample_NonBlocking();
            if (err == ERROR_IO_SAMPLE_FAIL || err == ERROR_POWER_CONFLICT) {
                g_sys_ctx.main_state = SYS_STATE_FAULT;
                fault_num = 1;
                x = 1;
                Power_ResetSampleState();
                break;
            }

            if (!Power_HasValidData()) {
                break;
            }

            bool sampling_power_valid;
            PowerSampleState sampling_power_state = Power_GetState(&sampling_power_valid);
            if (sampling_power_state == SAMPLE_UNDETERMINED || !sampling_power_valid) {
                break;
            }

            g_sys_ctx.power_sample_state = sampling_power_state;
            g_sys_ctx.power_sample_valid = sampling_power_valid;
            g_sys_ctx.power_valid = 0;

            switch (g_sys_ctx.power_state) {
                case POWER_DOWN:
                    handle_power_down_state();
                    break;
                case POWER_UNDETERMINED:
                    handle_undetermined_state();
                    break;
                case POWER_POSITIVE:
                case POWER_NEGATIVE:
                    handle_stable_power_state();
                    break;
                default:
                    g_sys_ctx.main_state = SYS_STATE_POWER_SAMPLE;
                    break;
            }
            break;
        }
        
        case SYS_STATE_HALL_SAMPLE: {
            SystemError err = HallPosition_Sample();
            if (err != ERROR_NONE) {
                g_sys_ctx.main_state = SYS_STATE_FAULT;
                fault_num = 2;
                x = 2;
                break;
            }

            g_sys_ctx.rod_pos = HallPosition_GetState(&g_sys_ctx.rod_pos_valid);

            if (g_sys_ctx.rod_pos != ROD_POS_UNKNOWN && g_sys_ctx.rod_pos_valid) {
                if (g_sys_ctx.rod_pos == ROD_POS_ERROR) {
                    g_sys_ctx.main_state = SYS_STATE_FAULT;
                    fault_num = 3;
                    x = 3;
                    break;
                }
                g_sys_ctx.main_state = SYS_STATE_MOTOR_CMD;
            }
            break;
        }
        
        case SYS_STATE_MOTOR_CMD: {
            SystemError err = MotorCmd_Execute(g_sys_ctx.power_state, g_sys_ctx.rod_pos, 
                                               g_sys_ctx.current_limit_state, s_voltage_limit_state);
            if (err != ERROR_NONE) {
                g_sys_ctx.main_state = SYS_STATE_POWER_SAMPLE;
                break;
            }
            g_sys_ctx.motor_cmd = MotorCmd_GetState(&g_sys_ctx.motor_cmd_executed);
            
            bool cur_limit_executed;
            g_sys_ctx.Cur_Limit_valid = MotorCurLimit_GetState(&cur_limit_executed);
            g_sys_ctx.main_state = SYS_STATE_MOTOR_PROTECT;
            break;
        }
        
        case SYS_STATE_MOTOR_PROTECT: {
            if(g_sys_ctx.Cur_Limit_valid != CURRENT_LIMIT_VALID_ON) {
                g_sys_ctx.main_state = SYS_STATE_POWER_SAMPLE;
                break;
            }
            
            // 欠压阻塞处理
            if (undervoltage_num == 1) {
                if (nbDelay_IsComplete(&UnderVoltage_Block_Delay)) {
                    undervoltage_num = 0;
                    MotorProtect_ResetVoltageMin();
                    g_sys_ctx.main_state = SYS_STATE_POWER_SAMPLE;
                    break;
                }
                break;
            }
            
            SystemError err = MotorProtect_Generate(g_sys_ctx.motor_cmd, g_sys_ctx.Cur_Limit_valid);
            MotorProtect_GetMeasure(&g_sys_ctx.Motor_Current, &g_sys_ctx.Motor_Voltage, 
                                    &g_sys_ctx.Protect_valid, &g_sys_ctx.current_limit_valid);

            // ========== 过压检测 ==========
            if (!g_sys_ctx.voltage_lock) {
                if (g_sys_ctx.power_state == POWER_POSITIVE || g_sys_ctx.power_state == POWER_NEGATIVE) {
#ifdef ENABLE_OVER_VOLTAGE_PROTECTION
                    if (err == ERROR_OVER_VOLTAGE) {
                        s_voltage_limit_state = VOLTAGE_LIMIT_OVER;
                        g_sys_ctx.voltage_lock = true;
                        g_sys_ctx.voltage_fault_type = VOLTAGE_LIMIT_OVER;
                        g_sys_ctx.main_state = SYS_STATE_POWER_SAMPLE;
                        break;
                    }
#endif

#ifdef ENABLE_UNDER_VOLTAGE_PROTECTION
                    if (err == ERROR_UNDER_VOLTAGE) {
                        s_voltage_limit_state = VOLTAGE_LIMIT_UNDER;
                        g_sys_ctx.voltage_lock = true;
                        g_sys_ctx.voltage_fault_type = VOLTAGE_LIMIT_UNDER;
                        g_sys_ctx.main_state = SYS_STATE_POWER_SAMPLE;
                        break;
                    }
#endif
                }
            }

            // 限位状态设置
            if (g_sys_ctx.current_limit_valid) {
                if (g_sys_ctx.motor_cmd == MOTOR_RUN_EXTEND) {
                    g_sys_ctx.current_limit_state = CURRENT_LIMITING_POSITIVE;
                } else if (g_sys_ctx.motor_cmd == MOTOR_RUN_RETRACT) {
                    g_sys_ctx.current_limit_state = CURRENT_LIMITING_NEGATIVE;
                }
            } else if (!g_sys_ctx.current_limit_valid) {
                if ((g_sys_ctx.current_limit_state == CURRENT_LIMITING_POSITIVE && g_sys_ctx.motor_cmd == MOTOR_RUN_RETRACT) ||
                    (g_sys_ctx.current_limit_state == CURRENT_LIMITING_NEGATIVE && g_sys_ctx.motor_cmd == MOTOR_RUN_EXTEND)) {
                    g_sys_ctx.current_limit_state = CURRENT_LIMIT_NONE;
                }
            }

#if hz_usart1_debug
            uint8_t Volatge_string[] = "Volatge:";
            uint8_t Current_string[] = "Current:";

            if(firstSend==1) {
                nbDelay_Start(&Usart1_Usb_Send_Delay);
            }
            firstSend++;
            if(nbDelay_IsComplete(&Usart1_Usb_Send_Delay)) {
                firstSend = 0;
                USART1_Send_Data_IT(Space,sizeof(Space)-1);
                USART1_Send_Data_IT(Volatge_string,sizeof(Volatge_string)-1);
                USART1_Send_Float_IT(g_sys_ctx.Motor_Voltage,3);
                USART1_Send_Data_IT(Space,sizeof(Space)-1);
                USART1_Send_Data_IT(Current_string,sizeof(Current_string)-1);
                USART1_Send_Float_IT(g_sys_ctx.Motor_Current,3);
                USART1_Send_Data_IT(Enter,sizeof(Enter)-1);
            }
#endif   
            if (g_sys_ctx.Protect_valid) {
                // 有效保护状态，不做处理
            } else {
                if(err == ERROR_OVER_CURRENT) {
                    g_sys_ctx.Time_Current_Over++;
                }
            }
            
            // 错误处理
            if(err == ERROR_OVER_CURRENT) {
                g_sys_ctx.main_state = SYS_STATE_POWER_SAMPLE;
                break;
            } else if(err != ERROR_NONE) {
                g_sys_ctx.main_state = SYS_STATE_FAULT;
                fault_num = 5;
                x = 5;
                break;
            }

            g_sys_ctx.main_state = SYS_STATE_POWER_SAMPLE;
            break;
        }
        
        case SYS_STATE_FAULT: {
            MotorCmd_StopImmediately();
            g_sys_ctx.main_state = SYS_STATE_POWER_SAMPLE;
            break;
        }
        
        default:
            break;
    }
}
