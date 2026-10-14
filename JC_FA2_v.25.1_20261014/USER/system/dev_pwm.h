#ifndef MOTOR_PWM_H_
#define MOTOR_PWM_H_

#include "main.h"
#include "hc32f46x_timera.h"
#include "gpio_adapter.h"
#include "adapter_pwm.h"  // 包含操作集定义
#include "device_manager.h"  // 包含设备管理器定义


#define MOTORPWM_BLOCK_TIME             3    //3s
#define MAX_PWM_CHANNELS                16   // 支持的最大PWM通道数
#define PWM_SLOWSTART_TIME              1.2
#define PWM_SLOWSTART_START_DUTY        35
#define PWM_SLOWSTART_TARGET_DUTY       2
#define PWM_Duty_100                    100
#define PWM_Duty_0                      0

#define PWM_OUT_POSITIVE_PORT       PortB
#define PWM_OUT_POSITIVE_PIN        Pin07
#define PWM_OUT_POSITIVE_TIMERA     M4_TMRA4
#define PWM_OUT_POSITIVE_CHANNEL    TimeraCh2

#define PWM_OUT_NEGATIVE_PORT       PortB
#define PWM_OUT_NEGATIVE_PIN        Pin09
#define PWM_OUT_NEGATIVE_TIMERA     M4_TMRA4
#define PWM_OUT_NEGATIVE_CHANNEL    TimeraCh4

// PWM设备参数结构
typedef struct {
    float duty_cycle;
    float change_duty_time;
    float slowstart_start_duty;
    float slowstart_target_duty;
} PwmDutyParams_t;

typedef struct {
    uint16_t frequency;
} PwmFreqParams_t;

// PWM设备数据结构
typedef struct {
    float duty_cycle;
    uint16_t frequency;
    uint8_t state;
} PwmDeviceData_t;

typedef enum {
    PWM_IDLE = 0,
    MOTOR_PWM_RAMPING,
    MOTOR_PWM_COMPLETE
} PWM_State_t;

// PWM通道句柄
typedef void* PWM_Handle_t;
extern PWM_Handle_t phu;
extern PWM_Handle_t phv;


// PWM通道控制结构
typedef struct {
    // 硬件配置
    en_port_t port;
    en_pin_t pin;
    M4_TMRA_TypeDef* Timer;
    en_timera_channel_t timerChannel;
    
    // 操作集
    pwm_ops_t ops;  // 新增：函数指针集合
    
    // 状态控制
    PWM_State_t state;
    float startDuty;
    float targetDuty;
    float currentDuty;

    float frequency;
    bool polarity;
    uint16_t totalSteps;
    uint16_t currentStep;
    uint32_t lastUpdateTime;
    uint16_t stepDelayMs;

}  PWM_Channel_t;

// 原有控制层函数
void PWM_InitAllChannels(void);
PWM_Handle_t MotorPWM_Create(en_port_t port, en_pin_t pin, 
                                 M4_TMRA_TypeDef* Timer, 
                                 en_timera_channel_t timerChannel,
                                 bool polarity);
void PWM_Destroy(PWM_Handle_t handle);
void PWM_SlowStart(PWM_Handle_t handle,float slowstart_start_duty,float slowstart_target_duty, float time);
void PWM_Stop(PWM_Handle_t handle);
void PWM_StopRamp(PWM_Handle_t handle);
bool PWM_IsRampingComplete(PWM_Handle_t handle);
en_result_t PWM_setDuty_noblock(PWM_Handle_t handle, float targetDuty, float time);
void PWM_DutyUpdate(PWM_Handle_t handle);
en_result_t PWM_setDuty_block(PWM_Handle_t handle, float targetDuty);
en_result_t PWM_setDutyInstant(PWM_Handle_t handle, float duty);
en_result_t PWM_setFreq(PWM_Handle_t handle, uint16_t freqHz);
PWM_State_t PWM_GetState(PWM_Handle_t handle);

// 设备管理层接口函数
en_result_t PWM_DeviceRead(void* handle, void* data, uint32_t size);
en_result_t PWM_DeviceWrite(void* handle, const void* data, uint32_t size);
en_result_t PWM_DeviceInit(void* handle);
en_result_t PWM_DeviceControl(void* handle, DeviceCommandData_t* cmd);
en_result_t PWM_DeviceUpdate(void* handle);

void SlowStart_PWMDuty(PWM_Handle_t device_name);
void SetDutyInstant_PWMDuty(PWM_Handle_t device_name, float target_duty);
void Positive_Run(void);
void Negative_Run(void);
void Stop(void);
void All_DevicePwm_Update(void);
// PWM设备注册宏
#define REGISTER_PWM_DEVICE(dev_id, dev_name, pwm_handle) \
    DeviceManager_RegisterDevice(dev_id, dev_name, DEVICE_TYPE_PWM, pwm_handle, \
    (DeviceOps_t){ \
        .read = PWM_DeviceRead, \
        .write = PWM_DeviceWrite, \
        .init = PWM_DeviceInit, \
        .control = PWM_DeviceControl, \
        .update = PWM_DeviceUpdate \
    })

#endif
