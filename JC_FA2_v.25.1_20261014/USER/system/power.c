#include "power.h"
#include "hardware.h"
#include "gpio_adapter.h"
#include "hz_timer.h"

// 模块内部静态变量
static PowerSampleState s_power_state = SAMPLE_UNDETERMINED;
static bool s_power_valid = false;
static uint8_t s_power_pos_sample_window[WINDOWS_NUM];
static uint8_t s_power_neg_sample_window[WINDOWS_NUM];

// 新增：状态保持 + 超时机制相关变量
static PowerSampleState s_last_valid_power_state = SAMPLE_UNDETERMINED;  // 上次有效状态
static NonBlockingDelay_t s_power_timeout_delay;   // 超时计时器
static bool s_power_timeout_running = false;       // 计时器运行标志
static bool s_power_was_down = false;              // 是否曾经进入掉电状态（用于恢复检测）

// 新增：非阻塞采样相关变量
static PowerSamplingState s_sample_state = POWER_SAMPLE_IDLE;
static uint8_t s_samples_collected = 0;        // 已收集的采样点数量
static NonBlockingDelay_t s_sample_delay;
static bool s_first_sample = true;             // 是否是第一次采样


void Power_Init(void) {
    s_power_state = SAMPLE_UNDETERMINED;
    s_power_valid = false;
    s_last_valid_power_state = SAMPLE_UNDETERMINED;
    s_power_timeout_running = false;
    s_power_was_down = false;
    
    memset(s_power_pos_sample_window, 0, sizeof(s_power_pos_sample_window));
    memset(s_power_neg_sample_window, 0, sizeof(s_power_neg_sample_window));
    
    // 初始化非阻塞采样状态
    s_sample_state = POWER_SAMPLE_IDLE;
    s_samples_collected = 0;
    s_first_sample = true;
    nbDelay_Init(&s_sample_delay, 1);
    
    // 初始化超时计时器
    nbDelay_Init(&s_power_timeout_delay, POWER_STABLE_TIMEOUT_MS);
}

void Power_ResetSampleState(void) {
    s_sample_state = POWER_SAMPLE_IDLE;
    s_samples_collected = 0;
    s_first_sample = true;
    s_power_timeout_running = false;
    s_power_was_down = false;
    nbDelay_Stop(&s_sample_delay);
    nbDelay_Stop(&s_power_timeout_delay);
}


/**
 * @brief 添加新采样点并滑动窗口
 */
static void Power_AddSampleToWindow(uint8_t level1, uint8_t level2) {
    if (s_samples_collected < WINDOWS_NUM) {
        // 初始填充阶段：直接添加到末尾
        s_power_pos_sample_window[s_samples_collected] = level1;
        s_power_neg_sample_window[s_samples_collected] = level2;
        s_samples_collected++;
    } else {
        // 窗口已满：滑动窗口（移除最早的点，添加新点）
        for (uint8_t i = 0; i < WINDOWS_NUM - 1; i++) {
            s_power_pos_sample_window[i] = s_power_pos_sample_window[i + 1];
            s_power_neg_sample_window[i] = s_power_neg_sample_window[i + 1];
        }
        s_power_pos_sample_window[WINDOWS_NUM - 1] = level1;
        s_power_neg_sample_window[WINDOWS_NUM - 1] = level2;
    }
}


/**
 * @brief 辅助函数：检查缓冲区中所有样本是否都等于目标值
 * @param buf 采样缓冲区
 * @param target 目标电平值
 * @param count 要检查的样本数量
 * @return 所有样本都等于目标值返回true，否则返回false
 */
static bool checkAllSamples(const uint8_t* buf, uint8_t target, uint8_t count) {
    for (uint8_t i = 0; i < count; i++) {
        if (buf[i] != target) {
            return false;
        }
    }
    return true;
}

/**
 * @brief 检查窗口是否处于掉电状态（正负窗口全低）
 */
static bool checkPowerDown(const uint8_t* pos_buf, const uint8_t* neg_buf, uint8_t count) {
    return checkAllSamples(pos_buf, POWER_LOW_LEVEL, count) && 
           checkAllSamples(neg_buf, POWER_LOW_LEVEL, count);
}

/**
 * @brief 检查窗口是否处于错误状态（正负窗口全高）
 */
static bool checkPowerError(const uint8_t* pos_buf, const uint8_t* neg_buf, uint8_t count) {
    return checkAllSamples(pos_buf, POWER_HIGH_LEVEL, count) && 
           checkAllSamples(neg_buf, POWER_HIGH_LEVEL, count);
}

/**
 * @brief 重置采样窗口（用于掉电恢复时重新采样）
 */
static void Power_ResetWindow(void) {
    memset(s_power_pos_sample_window, 0, sizeof(s_power_pos_sample_window));
    memset(s_power_neg_sample_window, 0, sizeof(s_power_neg_sample_window));
    s_samples_collected = 0;
    s_sample_state = POWER_SAMPLE_IDLE;
    s_first_sample = true;
}

/**
 * @brief 电源窗口滤波与状态检测（带超时机制）
 * @param sample_count 采样点数
 * @return 检测有效返回true，否则返回false
 */
static bool Power_CheckWindow(uint8_t sample_count) {
    // 检查各采样窗口的电平一致性
    const bool pos_window_all_high = checkAllSamples(s_power_pos_sample_window, POWER_HIGH_LEVEL, sample_count);
    const bool pos_window_all_low  = checkAllSamples(s_power_pos_sample_window, POWER_LOW_LEVEL, sample_count);
    const bool neg_window_all_high = checkAllSamples(s_power_neg_sample_window, POWER_HIGH_LEVEL, sample_count);
    const bool neg_window_all_low  = checkAllSamples(s_power_neg_sample_window, POWER_LOW_LEVEL, sample_count);

    // 判断窗口是否稳定（正极或负极窗口全部一致）
    bool pos_window_stable = pos_window_all_high || pos_window_all_low;
    bool neg_window_stable = neg_window_all_high || neg_window_all_low;
    bool window_stable = pos_window_stable && neg_window_stable;
    
    // ==================== 情况1：窗口稳定 ====================
    if (window_stable) {
        // 停止超时计时器
        s_power_timeout_running = false;
        nbDelay_Stop(&s_power_timeout_delay);
        s_power_was_down = false;
        
        // 正向电源：正极窗口全高且负极窗口全低
        if (pos_window_all_high && neg_window_all_low) {
            s_power_state = SAMPLE_POSITIVE;
            s_last_valid_power_state = SAMPLE_POSITIVE;
            s_power_valid = true;
            return true;
        }

        // 负向电源：正极窗口全低且负极窗口全高
        if (pos_window_all_low && neg_window_all_high) {
            s_power_state = SAMPLE_NEGATIVE;
            s_last_valid_power_state = SAMPLE_NEGATIVE;
            s_power_valid = true;
            return true;
        }

        // 掉电状态：正负极窗口均全低
        if (pos_window_all_low && neg_window_all_low) {
            s_power_state = SAMPLE_POWER_DOWN;
            s_last_valid_power_state = SAMPLE_POWER_DOWN;
            s_power_valid = true;
            s_power_was_down = true;
            return true;
        }

        // 电源冲突：正负极窗口均全高
        if (pos_window_all_high && neg_window_all_high) {
            s_power_state = SAMPLE_ERROR;
            s_last_valid_power_state = SAMPLE_ERROR;
            s_power_valid = true;
            return true;
        }

        // 理论上不会到这里，但以防万一
        s_power_state = SAMPLE_UNDETERMINED;
        s_power_valid = false;
        return false;
    }
    
    // ==================== 情况2：窗口不稳定 ====================
    
    // 检查是否是掉电状态（两个窗口都全低）
    bool is_power_down = checkPowerDown(s_power_pos_sample_window, s_power_neg_sample_window, sample_count);
    
    // 检查是否是错误状态（两个窗口都全高）
    bool is_power_error = checkPowerError(s_power_pos_sample_window, s_power_neg_sample_window, sample_count);
    
    // ==================== 情况2a：两个都为高（硬件故障） ====================
    if (is_power_error) {
        // 硬件冲突，立即报错，无需等待
        s_power_timeout_running = false;
        nbDelay_Stop(&s_power_timeout_delay);
        s_power_state = SAMPLE_ERROR;
        s_last_valid_power_state = SAMPLE_ERROR;
        s_power_valid = true;
        s_power_was_down = false;
        return true;
    }
    
    // ==================== 情况2b：两个都为低（可能是掉电或切换中） ====================
    if (is_power_down) {
        // 如果之前处于掉电状态且现在还是掉电，继续保持
        if (s_power_was_down) {
            s_power_state = SAMPLE_POWER_DOWN;
            s_power_valid = true;
            return true;
        }
        
        // 首次进入不稳定状态，启动超时计时器
        if (!s_power_timeout_running) {
            nbDelay_Start(&s_power_timeout_delay);
            s_power_timeout_running = true;
        }
        
        // 检查是否超时
        if (nbDelay_IsComplete_noclose(&s_power_timeout_delay)) {
            // 超时了还是两个都低 -> 判定为掉电
            s_power_timeout_running = false;
            s_power_state = SAMPLE_POWER_DOWN;
            s_last_valid_power_state = SAMPLE_POWER_DOWN;
            s_power_valid = true;
            s_power_was_down = true;
            return true;
        } else {
            // 未超时：保持上次有效状态
            if (s_last_valid_power_state != SAMPLE_UNDETERMINED && 
                s_last_valid_power_state != SAMPLE_ERROR) {
                s_power_state = s_last_valid_power_state;
                s_power_valid = true;  // 保持有效，让电机继续运行
                return true;
            } else {
                // 没有有效状态可保持
                s_power_state = SAMPLE_UNDETERMINED;
                s_power_valid = false;
                return false;
            }
        }
    }
    
    // ==================== 情况2c：其他不稳定（一个高一个低，但不满足稳定条件） ====================
    // 如果之前处于掉电状态，但检测到信号变化（不再是两个都低），说明电源恢复
    if (s_power_was_down) {
        // 电源恢复，重置采样窗口重新开始
        s_power_was_down = false;
        s_power_timeout_running = false;
        nbDelay_Stop(&s_power_timeout_delay);
        Power_ResetWindow();
        s_power_state = SAMPLE_UNDETERMINED;
        s_power_valid = false;
        return false;
    }
    
    // 其他不稳定情况：保持上次有效状态
    if (s_last_valid_power_state != SAMPLE_UNDETERMINED && 
        s_last_valid_power_state != SAMPLE_ERROR) {
        s_power_state = s_last_valid_power_state;
        s_power_valid = true;  // 保持有效，让电机继续运行
        return true;
    } else {
        // 没有有效状态可保持
        s_power_state = SAMPLE_UNDETERMINED;
        s_power_valid = false;
        return false;
    }
}


SystemError Power_IOSample_NonBlocking(void) {
    SystemError err = ERROR_NONE;
    uint8_t pos_level, neg_level;
    
    switch (s_sample_state) {
        case POWER_SAMPLE_IDLE:
            // 第一次采样或重置后：立即开始采样
            if (s_first_sample) {
                s_sample_state = POWER_SAMPLE_READING;
                s_first_sample = false;
            } else {
                // 非第一次采样：等待采样间隔
                nbDelay_Start(&s_sample_delay);
                s_sample_state = POWER_SAMPLE_WAITING;
            }
            break;
            
        case POWER_SAMPLE_WAITING:
            // 检查是否到达采样间隔
            if (nbDelay_IsComplete(&s_sample_delay)) {
                s_sample_state = POWER_SAMPLE_READING;
            }
            break;
            
        case POWER_SAMPLE_READING:
            // 读取当前IO电平
            pos_level = PORT_GetBit(POWER_POSITIVE_IO_PORT, POWER_POSITIVE_IO_PIN) ? 
                            POWER_HIGH_LEVEL : POWER_LOW_LEVEL;
            neg_level = PORT_GetBit(POWER_NEGATIVE_IO_PORT, POWER_NEGATIVE_IO_PIN) ? 
                            POWER_HIGH_LEVEL : POWER_LOW_LEVEL;
            
            // 添加到滑动窗口
            Power_AddSampleToWindow(pos_level, neg_level);
            
            // 如果有足够的数据点（至少WINDOWS_NUM个），立即处理
            if (s_samples_collected >= WINDOWS_NUM) {
                s_sample_state = POWER_SAMPLE_PROCESSING;
            } else {
                // 数据不足，等待下一个采样周期
                nbDelay_Start(&s_sample_delay);
                s_sample_state = POWER_SAMPLE_WAITING;
            }
            break;
            
        case POWER_SAMPLE_PROCESSING:
            // 处理采样结果
            s_power_valid = Power_CheckWindow(WINDOWS_NUM); 
            
            if (!s_power_valid) {
                // err = ERROR_IO_SAMPLE_FAIL;  //不能说无效就ERROR
            } else if (s_power_state == SAMPLE_ERROR) {
                // err = ERROR_POWER_CONFLICT;
            }
            
            // 处理完成后，立即开始下一个采样周期
            nbDelay_Start(&s_sample_delay);
            s_sample_state = POWER_SAMPLE_WAITING;
            break;
            
        default:
            s_sample_state = POWER_SAMPLE_IDLE;
            break;
    }
    
    return err;
}


PowerSampleState Power_GetState(bool *valid) {
    // 只要有足够的数据点就返回当前状态（即使还在持续采样）
    if (s_samples_collected < WINDOWS_NUM) {
        if (valid) *valid = false;
        return SAMPLE_UNDETERMINED;
    }
    
    if (valid) *valid = s_power_valid;
    return s_power_state;
}

// 新增：检查是否有有效的采样数据
bool Power_HasValidData(void) {
    return (s_samples_collected >= WINDOWS_NUM) && s_power_valid;
}
