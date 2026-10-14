#ifndef POWER_H
#define POWER_H

#include "sys_def.h"
#include "hc32f46x_gpio.h"

// 硬件IO定义（电源模块专用）
#define POWER_POSITIVE_IO_PORT    PortB
#define POWER_POSITIVE_IO_PIN     Pin13    // 电源极性检测IO
#define POWER_NEGATIVE_IO_PORT    PortB
#define POWER_NEGATIVE_IO_PIN     Pin12  

// #define POWER_POSITIVE_IO_PORT    PortA
// #define POWER_POSITIVE_IO_PIN     Pin03    // 电源极性检测IO
// #define POWER_NEGATIVE_IO_PORT    PortA
// #define POWER_NEGATIVE_IO_PIN     Pin04   

#define POWER_HIGH_LEVEL  1             // 高电平
#define POWER_LOW_LEVEL   0             // 低电平

#define WINDOWS_NUM     10      //窗口点位个数

// ====================== 电源采样超时配置 ======================
#define POWER_STABLE_TIMEOUT_MS     80      // 状态稳定超时时间(ms)
                                            // 用于区分"切换中"和"真正掉电"
                                            // 机械开关弹跳通常10-50ms，电子开关更快
                                            // 80ms是个保守值，兼顾可靠性和响应速度

// 电源采样状态机状态定义
typedef enum {
    POWER_SAMPLE_IDLE = 0,    // 空闲状态，等待开始采样
    POWER_SAMPLE_READING,     // 正在读取IO状态
    POWER_SAMPLE_WAITING,     // 等待采样间隔
    POWER_SAMPLE_COMPLETE,    // 采样完成，等待处理
    POWER_SAMPLE_PROCESSING   // 正在处理采样结果
} PowerSamplingState;

// 接口声明
void Power_Init(void);
void Power_ResetSampleState(void);
static void Power_AddSampleToWindow(uint8_t level1, uint8_t level2);
SystemError Power_IOSample_NonBlocking(void);
PowerSampleState Power_GetState(bool *valid);

bool Power_HasValidData(void);


#endif // POWER_H
