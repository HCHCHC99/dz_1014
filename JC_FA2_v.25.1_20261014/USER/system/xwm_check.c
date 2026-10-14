#include "hardware.h"
#include "xwm_check.h"
#include "gpio_adapter.h"

static XWM_state symbol_xwm =SYMBOL_UNKNOW;
static bool s_xwm_valid = false;
static uint8_t xwm_window[10];  // upper的10点采样缓存
static bool checkHallStable(const uint8_t* buf, uint8_t target) {
    for (uint8_t i = 0; i < 10; i++) {
        if (buf[i] != target) {
            return false;
        }
    }
    return true;
}

void XWM_Check_Process(void)
{
    WXM_Sample();
}

static bool xwm_window_check(void)
{
    // 检查上限位的稳定状态
    bool xwm_high = checkHallStable(xwm_window, XWM_HIGH_LEVEL);
    bool xwm_low  = checkHallStable(xwm_window, XWM_LOW_LEVEL);
    
    if(1==xwm_high)
    {
        symbol_xwm = SYMBOL_HIGH;
        return true;
    }
    else if(1 ==xwm_low)
    {
        symbol_xwm = SYMBOL_LOW;
        return true;
    }
    symbol_xwm = SYMBOL_UNKNOW;
    return false;
}

void WXM_Sample(void)
{
    uint8_t level_sample;
    static bool xwm_window_ready = false;
    static uint8_t s_xwm_sample_idx = 0;  // 需要设为static
    
    // 如果窗口未就绪，继续填充
    if (!xwm_window_ready) {
        level_sample = PORT_GetBit(HALL_XWM_PORT, HALL_XWM_PIN) ? XWM_HIGH_LEVEL : XWM_LOW_LEVEL;
        xwm_window[s_xwm_sample_idx] = level_sample;
        
        s_xwm_sample_idx++;
        if (s_xwm_sample_idx >= 10) {
            xwm_window_ready = true;
            s_xwm_sample_idx = 0;  // 重置索引
        }
    } 
    else {  // 窗口已就绪，进行滑动窗口更新
        // 移除最旧的数据，添加新数据
        for (uint8_t i = 0; i < 9; i++) {
            xwm_window[i] = xwm_window[i+1];
        }
        level_sample = PORT_GetBit(HALL_XWM_PORT, HALL_XWM_PIN) ? XWM_HIGH_LEVEL : XWM_LOW_LEVEL;
        xwm_window[9] = level_sample;
        
        // 检查窗口状态
        s_xwm_valid = xwm_window_check();
        if (s_xwm_valid) {
            XWM_Send_EN(symbol_xwm);
        }
    }
}
void XWM_Send_EN(XWM_state symbol)
{
    if(SYMBOL_HIGH==symbol)
    {
        XWM_NotTriggered();
    }
    else if(SYMBOL_LOW==symbol)
    {
        XWM_Triggered();
    }
    else
    {

    }
}

void XWM_NotTriggered(void)
{
    PORT_ResetBits(EN_XWM_PORT, EN_XWM_PIN);
}

void XWM_Triggered(void)
{
    PORT_SetBits(EN_XWM_PORT, EN_XWM_PIN);
}
