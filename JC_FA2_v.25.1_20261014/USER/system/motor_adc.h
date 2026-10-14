#ifndef MOTOR_H
#define MOTOR_H
#include "adc_adapter.h"
#include "system.h"
#include "sys_def.h"
#include "hz_timer.h"
/******************************应用层业务宏定义*******************************/
#define THRESHOLD_CURRENT_OVER    15.0f   // 过流阈值，单位A
#define THRESHOLD_VOLTAGE_UNDER   19.0f   // 欠压阈值，单位V
#define THRESHOLD_VOLTAGE_OVER    28.0f   // 过压阈值，单位V
#define THRESHOLD_CURRENT_RECOVERY 4.0f  // 过流恢复阈值

#define CALIBRATION_TIME    0 //校准时间

#define hz_adc_debug        1  // 调试开关
#define hz_cur_watch        1
#define TIME_SAMPLE_CURRENT_OVER   1     // 过流判断连续采样次数
#define VM_R1               10           // 电压分压电阻1
#define VM_R2               150          // 电压分压电阻2
// 注意：原来的 hz_IVM_OFFSET 已移除，改用动态校准偏移

// 采样参数（应用层专属）
#define SampleTime_vm               50
#define SampleIntervalTime_vm       1
#define SampleTime_cur              20
#define SampleIntervalTime_cur      1

/****************************** Over-current sampling optimization (rev. 2026-08-22) *******************
 * 50us TMR02 chB ISR (Int003, prio 0): ADC1 dual-channel sampling, NO ADC2
 *   - current (IN5) sampled EVERY tick            -> 20kHz, OC ring buffer + window counter (integer)
 *   - voltage (IN6) sampled every OC_VM_DIV ticks -> 10kHz, moving average (integer)
 * 1ms gate (Timer01B_CallBack -> OC_Gate_Tick): only sets a flag
 * Main loop (MotorProtect_Generate -> OC_Process): consumes gate, float percentage judgment (>=30%)
 *****************************************************************************************************/
#define ADC_CUR_SAMPLE_INTERVAL_US  300              /* current sampling interval: 50us = 20kHz */
#define OC_TIMER_US                 ADC_CUR_SAMPLE_INTERVAL_US   /* 50us timer period (TMR02 chB) */
#define OC_WINDOW_MS                1              /* over-current judging window: 10ms */
#define OC_WINDOW_SAMPLES           ((OC_WINDOW_MS * 1000u) / ADC_CUR_SAMPLE_INTERVAL_US)  /* =200 */
#define OC_TRIP_PERCENT             100.0f           /* trip percentage of above-threshold points in window */
#define OC_TRIP_SAMPLES             ((uint16_t)(OC_WINDOW_SAMPLES * OC_TRIP_PERCENT / 100.0f)) /* =60 */
#define OC_ADC1_SAMPLE_TIME         (0x14)          /* ADC1 sample time: 21 cyc (~5.25us @4MHz PCLK2).
                                                       Old polling code used 0x50(81cyc); shortened so two
                                                       conversions fit inside the 50us ISR. VERIFY on hw. */
#define OC_VM_DIV                   2               /* voltage sampled every N ticks (2 -> 10kHz) */
#define OC_VM_WINDOW_SAMPLES        50              /* voltage moving-average window (10kHz -> 5ms) */

/* 50us sampling timer IRQ: TMR02 chB compare match -> Int003 (TMR01 chA is async-clock-only in DDL, so TMR02 chB is used) */
#define OC_TIMER_IRQN               (Int003_IRQn)
#define OC_TIMER_IRQ_PRIORITY       (DDL_IRQ_PRIORITY_00)   /* higher than 1ms timer to keep sampling cadence */

/* Over-current sampling optimization interfaces */
void OC_Init(void);                 /* init 50us timer + OC ring buffer (called in SYS_STATE_INIT) */
void OC_UpdateThresholds(void);     /* update over-current threshold ADC codes after current calibration */
void OC_50us_CallBack(void);        /* 50us timer ISR: sample current (+voltage) -> ring buffer -> window counter */
void OC_Gate_Tick(void);            /* called by 1ms timer ISR: set the gate flag */
bool OC_Get_TripFlag(void);         /* current over-current verdict (used by MotorProtect_Generate) */

// ====================== 欠压检测配置 ======================
#define UNDER_VOLTAGE_DEBOUNCE_MS       50      // 欠压持续判定时间(ms)
                                                // 连续50ms都检测到欠压才触发错误

// ====================== 电压保护独立开关 ======================
// 注释掉下面对应的宏定义即可关闭对应的保护功能
#define ENABLE_UNDER_VOLTAGE_PROTECTION     // 使能欠压保护
#define ENABLE_OVER_VOLTAGE_PROTECTION      // 使能过压保护// 电流采样比较结果（应用层业务枚举）
typedef enum {
    ADC_COMPARE_ALL_ABOVE = 1,         // 所有采样点超过阈值
    ADC_COMPARE_NOT_ALL_ABOVE = 2,     // 不是所有采样点超过阈值
    ADC_COMPARE_NOT_ENOUGH_SAMPLES = 0,// 未达到采样个数
    ADC_COMPARE_NOT_READY = -1,        // 未到采样间隔
    ADC_COMPARE_DISABLED = -2,         // 功能禁用
    ADC_COMPARE_INITIALIZING = 3       // 初始化状态
} ADC_Compare_Result;

/******************************应用层变量声明*******************************/
// 电机电压、电流采样句柄（原adc_adapter.c中的全局变量移到应用层）
extern ADC_CH_CTRL_t    hADCChannalMBV; // 母线电压采样句柄
extern ADC_CH_CTRL_t    hADCChannalIVM; // 电流采样句柄

// 采样缓冲区（应用层专属）
#define SAMPLE_BUFFER_SIZE_vm                   32
extern uint16_t sample_buffer_vm[SAMPLE_BUFFER_SIZE_vm];
#define SAMPLE_BUFFER_SIZE_cur                  32
extern uint16_t sample_buffer_cur[SAMPLE_BUFFER_SIZE_cur];

/******************************应用层接口声明*******************************/
// 电机ADC初始化（包含电流、电压通道）
void ADC_Init_hz(void);

// 电机保护测量结果获取
void MotorProtect_GetMeasure(float *current, float *voltage, bool *valid, bool *current_over_valid);

// 电机保护逻辑处理
SystemError MotorProtect_Generate(MotorCmdState motor_cmd, CurrentLimitValidState cur_limit_valid);

// 电流采样初始化
void ADC_CUR_Init(void);

// 电流采样数组比较（过流判断）
ADC_Compare_Result ADC_CUR_ARRAY_COMPARE(float* CUR_Voltage_ptr);
ADC_Compare_Result ADC_CUR_ARRAY_COMPARE_OPAMP(float* CUR_Voltage_ptr);
// 电压采样初始化
void ADC_VM_Init(void);

// 电压采样处理
void ADC_VM(float* VM_Voltage_ptr);

// 新增：获取电压最低值
float MotorProtect_GetVoltageMin(void);

// 新增：重置电压最低值记录
void MotorProtect_ResetVoltageMin(void);

// ========== 电流校准接口（新增） ==========
void ADC_CURRENT_Calibration_Start(void);      // 开始电流校准
bool ADC_CURRENT_Calibration_Process(void);    // 执行校准采样（状态机中周期性调用）
bool ADC_CURRENT_Calibration_IsDone(void);     // 检查校准是否完成
float ADC_CURRENT_GetOffset(void);             // 获取校准偏移值
void ADC_CURRENT_SetOffset(float offset);      // 设置偏移值（用于外部注入）

#endif
