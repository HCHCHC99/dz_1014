 /**
 *******************************************************************************
 * @file  main.c
 * @brief Main program template.
 @verbatim
   Change Logs:
   Date             Author          Notes
   2020-06-30        CDT         First version
 @endverbatim
 *******************************************************************************
 * Copyright (C) 2020, Huada Semiconductor Co., Ltd. All rights reserved.
 *
 * This software component is licensed by HDSC under BSD 3-Clause license
 * (the "License"); You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                    opensource.org/licenses/BSD-3-Clause
 */
/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "hc32_ddl.h"
#include "main.h"
#include "system.h"
#include "mc_config.h"
#include "delay_adapter.h"
#include "gpio_adapter.h"
// 在文件顶部添加这些声明
#include <stdbool.h>

#include "uart_adapter.h"
#include "hz_timer.h"

#include "mc_cur.h"
#include "mc_app.h"
#include "adc_adapter.h"
#include "sys_main_state.h"
#include "hardware.h"
#include "hz_timer.h"
#include "adapter_pwm.h"
#include "usart_usb.h"
#include "hz_nopwm.h"
#include "hc32_common.h"
#include "ring_buffer.h"
#include "dev_pwm.h"
#include "device_manager.h"
#include "xwm_check.h"
/*******************************************************************************
 * Local type definitions ('typedef')
 ******************************************************************************/

/*******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/

/*定时器相关计算*/
/*! Macro to convert a microsecond period to raw count value */
#define USEC_TO_COUNT(us, clockFreqInHz) (uint16_t)(((uint64_t)(us) * (clockFreqInHz)) / 1000000U)
/*! Macro to convert a raw count value to microsecond */
#define COUNT_TO_USEC(count, clockFreqInHz) (uint64_t)((uint64_t)(count) * 1000000U / (clockFreqInHz))
/*! Macro to convert a millisecond period to raw count value */
#define MSEC_TO_COUNT(ms, clockFreqInHz) (uint64_t)((uint64_t)(ms) * (clockFreqInHz) / 1000U)
/*! Macro to convert a raw count value to millisecond */
#define COUNT_TO_MSEC(count, clockFreqInHz) (uint64_t)((uint64_t)(count) * 1000U / (clockFreqInHz))

//hz
volatile uint8_t g_pwm_level = 0;

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/

/**
 *******************************************************************************
 ** \brief System clock init function
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
/*__DEBUG,HC32F460,USE_DEVICE_DRIVER_LIB*/
static void SystemClk_Init(void)
{
    stc_clk_sysclk_cfg_t    stcSysClkCfg;
    stc_clk_xtal_cfg_t      stcXtalCfg;
    stc_clk_mpll_cfg_t      stcMpllCfg;

    MEM_ZERO_STRUCT(stcSysClkCfg);
    MEM_ZERO_STRUCT(stcXtalCfg);
    MEM_ZERO_STRUCT(stcMpllCfg);

    /* Set bus clk div. */
    stcSysClkCfg.enHclkDiv = ClkSysclkDiv1;
    stcSysClkCfg.enExclkDiv = ClkSysclkDiv2;
    stcSysClkCfg.enPclk0Div = ClkSysclkDiv1;
    stcSysClkCfg.enPclk1Div = ClkSysclkDiv2;
    stcSysClkCfg.enPclk2Div = ClkSysclkDiv4;
    stcSysClkCfg.enPclk3Div = ClkSysclkDiv4;
    stcSysClkCfg.enPclk4Div = ClkSysclkDiv2;
    CLK_SysClkConfig(&stcSysClkCfg);

    /* Switch system clock source to MPLL. */
    /* Use Xtal as MPLL source. */
    stcXtalCfg.enMode = ClkXtalModeOsc;
    stcXtalCfg.enDrv = ClkXtalLowDrv;
    stcXtalCfg.enFastStartup = Enable;
    
    //CLK_XtalConfig(&stcXtalCfg);  //hz 关闭外部晶振
    //CLK_XtalCmd(Enable);
    CLK_XtalCmd(Disable);   //hz 关闭外部晶振
    CLK_HrcCmd(Enable);     //hz 开启内部时钟源

    /* MPLL config. */
    stcMpllCfg.pllmDiv = 1u;
    stcMpllCfg.plln = 32u;
    stcMpllCfg.PllpDiv = 2u;
    stcMpllCfg.PllqDiv = 2u;
    stcMpllCfg.PllrDiv = 2u;
    //CLK_SetPllSource(ClkPllSrcXTAL); //hz 关闭源为外部晶振
    CLK_SetPllSource(ClkPllSrcHRC); //hz 把时钟源设置为HRC  
    CLK_MpllConfig(&stcMpllCfg);

    /* flash read wait cycle setting */
    EFM_Unlock();
    EFM_SetLatency(EFM_LATENCY_4);
    EFM_Lock();

    /* Enable MPLL. */
    CLK_MpllCmd(Enable);

    /* Wait MPLL ready. */
    while (Set != CLK_GetFlagStatus(ClkFlagMPLLRdy))
    {
    }

    /* Switch system clock source to MPLL. */
    //CLK_SetSysClkSource(CLKSysSrcMPLL); //hz 使用内部时钟源并PLL倍频之后为128Mhz  MPLL输出频率 = (HRC频率 / pllmDiv) × plln / PllpDiv  = (8/1) × 32 / 2 =128Mhz
    CLK_SetSysClkSource(ClkSysSrcHRC);
}
/**
 *******************************************************************************
 ** \brief  Configure pins
 **
 ** \param  None
 **
 ** \retval
 **
 ******************************************************************************/
void gpio_Init(void)
{
    stc_port_init_t     stcPortInit;
    stc_port_pub_set_t  stcPortPubSet;
    stc_exint_config_t  stcExintCfg;
    stc_irq_regi_conf_t stcPortIrqCfg;

    MEM_ZERO_STRUCT(stcPortInit);
    MEM_ZERO_STRUCT(stcPortPubSet);
    MEM_ZERO_STRUCT(stcExintCfg);
    MEM_ZERO_STRUCT(stcPortIrqCfg);

    // /*特殊引脚初始化*/
    // PORT_DebugPortSetting(TDI|TDO_SWO|TRST,Disable);    //PB3,PB4,PA13,PA14,PA15默认是烧录接口,需要禁止烧录功能才能用作GPIO

    /*GPIO-输入引脚初始化-不触发外部中断*/
    stcPortInit.enPinMode = Pin_Mode_In;
    stcPortInit.enPullUp = Disable;
    stcPortInit.enExInt = Disable;
    PORT_Init(GPIO_IN1_KUP_PORT, GPIO_IN1_KUP_PIN, &stcPortInit);
    PORT_Init(GPIO_IN2_KDOWN_PORT, GPIO_IN2_KDOWN_PIN, &stcPortInit);
	PORT_Init(GPIO_IN3_PORT, GPIO_IN3_PIN, &stcPortInit);
//    PORT_Init(GPIO_K3_PORT, GPIO_K3_PIN, &stcPortInit);
//    PORT_Init(GPIO_K4_PORT, GPIO_K4_PIN, &stcPortInit);
    PORT_Init(GPIO_M1_LIMIT_BTM_PORT, GPIO_M1_LIMIT_BTM_PIN, &stcPortInit);
    PORT_Init(GPIO_M1_LIMIT_TOP_PORT, GPIO_M1_LIMIT_TOP_PIN, &stcPortInit);

    /*GPIO-输入引脚初始化-触发外部中断*/
    stcPortInit.enPinMode = Pin_Mode_In;
    stcPortInit.enPullUp = Disable;
    stcPortInit.enExInt = Enable;
    PORT_Init(EXINT_M1_HALLA_PORT, EXINT_M1_HALLA_PIN, &stcPortInit);
    PORT_Init(EXINT_M1_HALLB_PORT, EXINT_M1_HALLB_PIN, &stcPortInit);
//    PORT_Init(EXINT_M1_HALLC_PORT, EXINT_M1_HALLC_PIN, &stcPortInit);

    /*GPIO-输出引脚初始化*/
    MEM_ZERO_STRUCT(stcPortInit);
    stcPortInit.enPinMode = Pin_Mode_Out;
    stcPortInit.enPinDrv = Pin_Drv_H;
    stcPortInit.enPullUp = Disable;
    stcPortInit.enExInt = Disable;
    stcPortInit.enPinOType = Pin_OType_Cmos;
    /*初始电平设置*/
	PORT_ResetBits(GPIO_LED_PORT, GPIO_LED_PIN);
    PORT_ResetBits(GPIO_SLEEP_PORT, GPIO_SLEEP_PIN);
    PORT_ResetBits(GPIO_OUT3_PORT, GPIO_OUT3_PIN);
	PORT_ResetBits(GPIO_OUT4_PORT, GPIO_OUT4_PIN);
//#if (PROTOCOL_TYPE == MODBUS_PROTOCOL)
    PORT_ResetBits(UART_485_DIR_PORT, UART_485_DIR_PIN);
//#elif (PROTOCOL_TYPE == CAN_PROTOCOL) 
//#endif
#if (DRV_OUTPUT_TYPE == DO_H_PWM_L_NPWM)	//初始化打开下桥
	PORT_SetBits(GPIO_PLU_PORT, GPIO_PLU_PIN);
	PORT_SetBits(GPIO_PLV_PORT, GPIO_PLV_PIN);
	PORT_SetBits(GPIO_PLW_PORT, GPIO_PLW_PIN);
#elif (DRV_OUTPUT_TYPE == DO_H_PWM_L_IO)
	PORT_ResetBits(GPIO_PLU_PORT, GPIO_PLU_PIN);
	PORT_ResetBits(GPIO_PLV_PORT, GPIO_PLV_PIN);
	PORT_ResetBits(GPIO_PLW_PORT, GPIO_PLW_PIN);
#endif
	PORT_ResetBits(AT_TOP_GPIO_Port, AT_TOP_Pin);
    PORT_ResetBits(AT_BTM_GPIO_Port, AT_BTM_Pin);
    /*使能输出*/
	PORT_OE(GPIO_LED_PORT, GPIO_LED_PIN, Enable);
    PORT_OE(GPIO_SLEEP_PORT, GPIO_SLEEP_PIN, Enable);
    PORT_OE(GPIO_OUT3_PORT, GPIO_OUT3_PIN, Enable);
	PORT_OE(GPIO_OUT4_PORT, GPIO_OUT4_PIN, Enable);
//#if (PROTOCOL_TYPE == MODBUS_PROTOCOL)
    PORT_OE(UART_485_DIR_PORT, UART_485_DIR_PIN, Enable);
//#elif (PROTOCOL_TYPE == CAN_PROTOCOL)  
//#endif
	PORT_OE(GPIO_PLU_PORT, GPIO_PLU_PIN, Enable);
	PORT_OE(GPIO_PLV_PORT, GPIO_PLV_PIN, Enable);
	PORT_OE(GPIO_PLW_PORT, GPIO_PLW_PIN, Enable);
	PORT_OE(AT_TOP_GPIO_Port, AT_TOP_Pin, Enable);
    PORT_OE(AT_BTM_GPIO_Port, AT_BTM_Pin, Enable);
    /*端口设置*/
	PORT_Init(GPIO_LED_PORT, GPIO_LED_PIN, &stcPortInit);
    PORT_Init(GPIO_SLEEP_PORT, GPIO_SLEEP_PIN, &stcPortInit);
    PORT_Init(GPIO_OUT3_PORT, GPIO_OUT3_PIN, &stcPortInit);
	PORT_Init(GPIO_OUT4_PORT, GPIO_OUT4_PIN, &stcPortInit);
//#if (PROTOCOL_TYPE == MODBUS_PROTOCOL)
    PORT_Init(UART_485_DIR_PORT, UART_485_DIR_PIN, &stcPortInit);
//#elif (PROTOCOL_TYPE == CAN_PROTOCOL)
//#endif
	PORT_Init(GPIO_PLU_PORT, GPIO_PLU_PIN, &stcPortInit);
	PORT_Init(GPIO_PLV_PORT, GPIO_PLV_PIN, &stcPortInit);
	PORT_Init(GPIO_PLW_PORT, GPIO_PLW_PIN, &stcPortInit);
	PORT_Init(AT_TOP_GPIO_Port, AT_TOP_Pin, &stcPortInit);
    PORT_Init(AT_BTM_GPIO_Port, AT_BTM_Pin, &stcPortInit);
}
/**
 *******************************************************************************
 ** \brief SW2 init function
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
void intc_exint_Init(void)
{
    stc_exint_config_t stcExtiConfig;
    stc_irq_regi_conf_t stcIrqRegiConf;
    stc_port_init_t stcPortInit;

    /* 1 */
    /* configuration structure initialization */
    MEM_ZERO_STRUCT(stcExtiConfig);
    MEM_ZERO_STRUCT(stcIrqRegiConf);
    MEM_ZERO_STRUCT(stcPortInit);

    stcExtiConfig.enExitCh = M1_HALLA_EIRQ_CH;          /* Set External Int Ch */
    stcExtiConfig.enFilterEn = Enable;                  /* Filter setting */
    stcExtiConfig.enFltClk = Pclk3Div8;
    stcExtiConfig.enExtiLvl = ExIntBothEdge;            /* Triger edge setting */
    EXINT_Init(&stcExtiConfig);

    /* Set GPIO as External Int Ch input */
    stcPortInit.enExInt = Enable;
    PORT_Init(EXINT_M1_HALLA_PORT, EXINT_M1_HALLA_PIN, &stcPortInit);

    stcIrqRegiConf.enIntSrc = M1_HALLA_EIRQ_NUM;        /*  */
    stcIrqRegiConf.enIRQn = M1_HALLA_EIRQ_IRQN;         /* Register External Int to Vect.No.XXX */
    stcIrqRegiConf.pfnCallback = &ExtInt_Callback;      /* register Callback function */
    enIrqRegistration(&stcIrqRegiConf);

    EXINT_IrqFlgClr(M1_HALLA_EIRQ_CH);
//    NVIC_ClearPendingIRQ(M1_HALLA_EIRQ_IRQN);                    /* Clear pending */
//    NVIC_SetPriority(M1_HALLA_EIRQ_IRQN, M1_HALL_EIRQ_PRIORITY);   /* Set priority */
//    NVIC_EnableIRQ(M1_HALLA_EIRQ_IRQN);                          /* Enable NVIC */

    /* 2 */
    /* configuration structure initialization */
    MEM_ZERO_STRUCT(stcExtiConfig);
    MEM_ZERO_STRUCT(stcIrqRegiConf);
    MEM_ZERO_STRUCT(stcPortInit);

    stcExtiConfig.enExitCh = M1_HALLB_EIRQ_CH;          /* Set External Int Ch */
    stcExtiConfig.enFilterEn = Enable;                  /* Filter setting */
    stcExtiConfig.enFltClk = Pclk3Div8;
    stcExtiConfig.enExtiLvl = ExIntBothEdge;            /* Triger edge setting */
    EXINT_Init(&stcExtiConfig);

    /* Set GPIO as External Int Ch input */
    stcPortInit.enExInt = Enable;
    PORT_Init(EXINT_M1_HALLB_PORT, EXINT_M1_HALLB_PIN, &stcPortInit);

    stcIrqRegiConf.enIntSrc = M1_HALLB_EIRQ_NUM;        /*  */
    stcIrqRegiConf.enIRQn = M1_HALLB_EIRQ_IRQN;         /* Register External Int to Vect.No.XXX */
    stcIrqRegiConf.pfnCallback = &ExtInt_Callback;      /* register Callback function */
    enIrqRegistration(&stcIrqRegiConf);

    EXINT_IrqFlgClr(M1_HALLB_EIRQ_CH);
//    NVIC_ClearPendingIRQ(M1_HALLB_EIRQ_IRQN);                    /* Clear pending */
//    NVIC_SetPriority(M1_HALLB_EIRQ_IRQN, M1_HALL_EIRQ_PRIORITY);   /* Set priority */
//    NVIC_EnableIRQ(M1_HALLB_EIRQ_IRQN);                          /* Enable NVIC */


  /* 3 */
    /* configuration structure initialization */
    MEM_ZERO_STRUCT(stcExtiConfig);
    MEM_ZERO_STRUCT(stcIrqRegiConf);
    MEM_ZERO_STRUCT(stcPortInit);

    stcExtiConfig.enExitCh = M1_HALLC_EIRQ_CH;            /* Set External Int Ch */
    stcExtiConfig.enFilterEn = Enable;                    /* Filter setting */
    stcExtiConfig.enFltClk = Pclk3Div8;
    stcExtiConfig.enExtiLvl = ExIntBothEdge;          /* Triger edge setting */
    EXINT_Init(&stcExtiConfig);

    /* Set GPIO as External Int Ch input */
    stcPortInit.enExInt = Enable;
//    PORT_Init(EXINT_M1_HALLC_PORT, EXINT_M1_HALLC_PIN, &stcPortInit);

    stcIrqRegiConf.enIntSrc = M1_HALLC_EIRQ_NUM;      /*  */
    stcIrqRegiConf.enIRQn = M1_HALLC_EIRQ_IRQN;           /* Register External Int to Vect.No.XXX */
    stcIrqRegiConf.pfnCallback = &ExtInt_Callback;        /* register Callback function */
    enIrqRegistration(&stcIrqRegiConf);

  EXINT_IrqFlgClr(M1_HALLC_EIRQ_CH);
//    NVIC_ClearPendingIRQ(M1_HALLC_EIRQ_IRQN);                  /* Clear pending */
//    NVIC_SetPriority(M1_HALLC_EIRQ_IRQN, M1_HALL_EIRQ_PRIORITY); /* Set priority */
//    NVIC_EnableIRQ(M1_HALLC_EIRQ_IRQN);                            /* Enable NVIC */
}
/**
 *******************************************************************************
 ** \brief  Main function of example project
 **
 ** \param  None
 **
 ** \retval int32_t return value, if needed
 **
 ******************************************************************************/
void timer0_Init(uint32_t us)
{
    stc_tim0_base_init_t stcTimerCfg;
    stc_irq_regi_conf_t stcIrqRegiConf;
    stc_tim0_trigger_init_t StcTimer0TrigInit;

    MEM_ZERO_STRUCT(stcTimerCfg);
    MEM_ZERO_STRUCT(stcIrqRegiConf);
    MEM_ZERO_STRUCT(StcTimer0TrigInit);

    uint32_t u32Pclk1;
    stc_clk_freq_t stcClkTmp;

    /* Get pclk1 */
    CLK_GetClockFreq(&stcClkTmp);
    u32Pclk1 = stcClkTmp.pclk1Freq;
    
    /* Timer0 peripheral enable */
    PWC_Fcg2PeriphClockCmd(PWC_FCG2_PERIPH_TIM01, Enable);

    /* Clear CNTAR register for channel A */
    TIMER0_WriteCntReg(M4_TMR01, Tim0_ChannelB, 0u);

    /*config register for channel B */
    stcTimerCfg.Tim0_CounterMode = Tim0_Sync;//Sync 同步时钟源  ； Async异步时钟源
    stcTimerCfg.Tim0_SyncClockSource = Tim0_Pclk1;
    //  stcTimerCfg.Tim0_AsyncClockSource = Tim0_XTAL32;
    stcTimerCfg.Tim0_ClockDivision = Tim0_ClkDiv64;
    stcTimerCfg.Tim0_CmpValue = USEC_TO_COUNT(us,u32Pclk1/64) - 1;//(us*pclk1/Timdiv)-1 <= 65535
    TIMER0_BaseInit(M4_TMR01,Tim0_ChannelB,&stcTimerCfg);


    /* Clear compare flag */
    TIMER0_ClearFlag(M4_TMR01, Tim0_ChannelB);
    TIMER0_IntCmd(M4_TMR01,Tim0_ChannelB,Enable);

    /* Register TMR_INI_GCMB Int to Vect.No.009 */
    stcIrqRegiConf.enIRQn = TIME0_IRQN;
    /* Select I2C Error or Event interrupt function */
    stcIrqRegiConf.enIntSrc = TIME_GCM_NUM;
    /* Callback function */
    stcIrqRegiConf.pfnCallback = &Timer01B_CallBack;
    /* Registration IRQ */
    enIrqRegistration(&stcIrqRegiConf);
//    /* Clear Pending */
   NVIC_ClearPendingIRQ(TIME0_IRQN);
//    /* Set priority */
   NVIC_SetPriority(TIME0_IRQN, TIME0_IRQ_PRIORITY);
//    /* Enable NVIC */
   NVIC_EnableIRQ(TIME0_IRQN);

    /*start timer0*/
    TIMER0_Cmd(M4_TMR01,Tim0_ChannelB,Enable);

    tickTimer_Init();  // 初始化滴答计数    
}
/**
 *******************************************************************************
 ** \brief Configure Timera peripheral function
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
void Timera_Pwm_Config(void)  //hz PWM TIMERA
{
#if (DRV_OUTPUT_TYPE != DO_H_PWM_L_NPWM)	//无刷: 用timera上桥pwm,下桥io; 有刷: 用timer4互补PWM输出
    stc_timera_base_init_t stcTimeraInit;
    stc_timera_compare_init_t stcTimerCompareInit;

    /* configuration structure initialization */
    MEM_ZERO_STRUCT(stcTimeraInit);
    MEM_ZERO_STRUCT(stcTimerCompareInit);

    /* Configuration peripheral clock */
    PWC_Fcg2PeriphClockCmd(PWMCH_M1_UH_TIMERCLOCK , Enable);
    PWC_Fcg2PeriphClockCmd(PWMCH_M1_VH_TIMERCLOCK , Enable);
//    PWC_Fcg2PeriphClockCmd(PWMCH_M1_WH_TIMERCLOCK , Enable);

    /* Configuration TIMERA compare pin */
    PORT_SetFunc(PWMCH_M1_UH_PORT, PWMCH_M1_UH_PIN, PWMCH_M1_UH_FUNC_PWM, Disable);
    PORT_SetFunc(PWMCH_M1_VH_PORT, PWMCH_M1_VH_PIN, PWMCH_M1_VH_FUNC_PWM, Disable);
//    PORT_SetFunc(PWMCH_M1_WH_PORT, PWMCH_M1_WH_PIN, PWMCH_M1_WH_FUNC_PWM, Disable);

    /* Configuration timera unit 1 base structure */
    stcTimeraInit.enClkDiv = TimeraPclkDiv2;    //fCLK = 64MHz / Div
    stcTimeraInit.enCntMode = TimeraCountModeSawtoothWave;
    stcTimeraInit.enCntDir = TimeraCountDirUp;
    stcTimeraInit.enSyncStartupEn = Disable;
    stcTimeraInit.u16PeriodVal = 2000;        //fPWM = fCLK / Period = 16kHz
    TIMERA_BaseInit(PWMCH_M1_UH_TIMER, &stcTimeraInit);
    TIMERA_BaseInit(PWMCH_M1_VH_TIMER, &stcTimeraInit);
//    TIMERA_BaseInit(PWMCH_M1_WH_TIMER, &stcTimeraInit);

    /* Configuration timera unit 1 compare structure */
    stcTimerCompareInit.u16CompareVal = 0;
    stcTimerCompareInit.enStartCountOutput = TimeraCountStartOutputLow;
    stcTimerCompareInit.enStopCountOutput = TimeraCountStopOutputLow;
    stcTimerCompareInit.enCompareMatchOutput = TimeraCompareMatchOutputLow;
    stcTimerCompareInit.enPeriodMatchOutput = TimeraPeriodMatchOutputHigh;
    stcTimerCompareInit.enSpecifyOutput = TimeraSpecifyOutputInvalid;
    stcTimerCompareInit.enCacheEn = Disable;
    stcTimerCompareInit.enTriangularTroughTransEn = Disable;
    stcTimerCompareInit.enTriangularCrestTransEn = Disable;
    stcTimerCompareInit.u16CompareCacheVal = stcTimerCompareInit.u16CompareVal;
    /* Configure Channel 1 */
    TIMERA_CompareInit(PWMCH_M1_UH_TIMER, PWMCH_M1_UH_CH, &stcTimerCompareInit);
    TIMERA_CompareInit(PWMCH_M1_VH_TIMER, PWMCH_M1_VH_CH, &stcTimerCompareInit);
//    TIMERA_CompareInit(PWMCH_M1_WH_TIMER, PWMCH_M1_WH_CH, &stcTimerCompareInit);
    /* Configure channel 3 */
    TIMERA_Cmd(PWMCH_M1_UH_TIMER, Enable);
    TIMERA_Cmd(PWMCH_M1_VH_TIMER, Enable);
//    TIMERA_Cmd(PWMCH_M1_WH_TIMER, Enable);
#endif
}

/**
 *******************************************************************************
 ** \brief Configure Timer4 peripheral function
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
#define TEST1	1
#define TEST2	2
#define TEST	TEST2
uint16_t g_PWMDuty = 0;
/* Timer4 CNT */
#define TIMER4_UNIT                     (M4_TMR43)
#define TIMER4_CNT_CYCLE_VAL            (2000)        /* Timer4 counter cycle value */

/* Timer4 OCO */
#define TIMER4_OCO_HIGH_CH              (Timer4OcoOuh)  /* only Timer4OcoOuh  Timer4OcoOvh  Timer4OcoOwh */
#define TIMER4_OCO_LOW_CH				(Timer4OcoOul)  /* only Timer4OcoOuh  Timer4OcoOvh  Timer4OcoOwh */

/* Timer4 PWM */
#define TIMER4_PWM_CH                   (Timer4PwmU)    /* only Timer4PwmU  Timer4PwmV  Timer4PwmW */

/* Define port and pin for Timer4Pwm */
#define TIMER4_PWM_H_PORT               (PortB)         /* TIM4_1_OUH_B:PE9   TIM4_1_OVH_B:PE11   TIM4_1_OWH_B:PE13 */
#define TIMER4_PWM_H_PIN                (Pin09)
#define TIMER4_PWM_L_PORT               (PortB)         /* TIM4_1_OUL_B:PE8   TIM4_1_OVL_B:PE10   TIM4_1_OWL_B:PE12 */
#define TIMER4_PWM_L_PIN                (Pin08)
void hc32f460_timer4_complementary_pwm_cfg(void)
{
#if (DRV_OUTPUT_TYPE == DO_H_PWM_L_NPWM)	//无刷: 用timera上桥pwm,下桥io; 有刷: 用timer4互补PWM输出
#if (TEST == TEST1)		//H,L独立输出
    stc_timer4_cnt_init_t stcCntInit;
    stc_timer4_oco_init_t stcOcoInit;
    stc_timer4_pwm_init_t stcPwmInit;
    stc_oco_low_ch_compare_mode_t stcLowChCmpMode;
    stc_oco_high_ch_compare_mode_t stcHighChCmpMode;

    /* Clear structures */
    MEM_ZERO_STRUCT(stcCntInit);
    MEM_ZERO_STRUCT(stcOcoInit);
    MEM_ZERO_STRUCT(stcPwmInit);
    MEM_ZERO_STRUCT(stcLowChCmpMode);
    MEM_ZERO_STRUCT(stcHighChCmpMode);

    /* Enable peripheral clock */
    PWC_Fcg2PeriphClockCmd(PWC_FCG2_PERIPH_TIM43, Enable);

    /* Timer4 CNT : Initialize CNT configuration structure */
    stcCntInit.enBufferCmd = Disable;
    stcCntInit.enClk = Timer4CntPclk;
    stcCntInit.enClkDiv = Timer4CntPclkDiv2;  /* CNT clock divide */
    stcCntInit.u16Cycle = TIMER4_CNT_CYCLE_VAL;
    stcCntInit.enCntMode = Timer4CntSawtoothWave;
    stcCntInit.enZeroIntCmd = Disable;
    stcCntInit.enPeakIntCmd = Disable;
    stcCntInit.enZeroIntMsk = Timer4CntIntMask0;
    stcCntInit.enPeakIntMsk = Timer4CntIntMask0;
    TIMER4_CNT_Init(TIMER4_UNIT, &stcCntInit); /* Initialize CNT */

    /* Timer4 OCO : Initialize OCO configuration structure */
    stcOcoInit.enOccrBufMode = OccrBufDisable;
    stcOcoInit.enOcmrBufMode = OcmrBufDisable;
    stcOcoInit.enPortLevel = OcPortLevelLow;
    stcOcoInit.enOcoIntCmd = Disable;
    TIMER4_OCO_Init(TIMER4_UNIT, TIMER4_OCO_HIGH_CH, &stcOcoInit);	/* Initialize OCO high channel */
    TIMER4_OCO_Init(TIMER4_UNIT, TIMER4_OCO_LOW_CH, &stcOcoInit);	/* Initialize OCO low channel */

    /* OCMR[31:0] Ox 0FF0 0FFF    0000 1111 1111 0000   0000 1111 1111 1111 */
    stcHighChCmpMode.enCntZeroMatchOpState = OcoOpOutputLow;			//bit[11:10]<-->(0==CNT==OCCR) 		通道值==0的特例
    stcHighChCmpMode.enCntZeroNotMatchOpState = OcoOpOutputHigh;		//bit[15:14]<-->(0==CNT!=OCCR)		CNT自加的过程
    stcHighChCmpMode.enCntUpCntMatchOpState = OcoOpOutputLow;			//bit[9:8]<-->(CNT↑==OCCR!=0)		CNT自加到通道值的时刻
    stcHighChCmpMode.enCntPeakMatchOpState = OcoOpOutputHigh;			//bit[7:6]<-->(CNT==OCCR==CPSR)		通道值==模值的特例
    stcHighChCmpMode.enCntPeakNotMatchOpState = OcoOpOutputHold;		//bit[13:12]<-->(CNT==CPSR!=OCCR)	通道值>模值的特例
	TIMER4_OCO_SetHighChCompareMode(TIMER4_UNIT, TIMER4_OCO_HIGH_CH, &stcHighChCmpMode);  /* Set OCO high channel compare mode */

    /*************Timer4 OCO ocmr1[31:0] = 0x0FF0 0FFF*****************************/
	/* OCMR[31:0] Ox 0FF0 0FFF    0000 1111 1111 0000   0000 1111 1111 1111 */
	stcLowChCmpMode.enCntZeroLowMatchHighNotMatchLowChOpState = OcoOpOutputLow;		//bit[11:10]<-->(0==CNT==OCCR) 		通道值==0的特例
	stcLowChCmpMode.enCntZeroLowNotMatchHighNotMatchLowChOpState = OcoOpOutputHigh;	//bit[15:14]<-->(0==CNT!=OCCR)		CNT自加的过程
	stcLowChCmpMode.enCntUpCntLowMatchHighNotMatchLowChOpState = OcoOpOutputLow; 	//bit[9:8]<-->(CNT↑==OCCR!=0)		CNT自加到通道值的时刻
	stcLowChCmpMode.enCntPeakLowMatchHighNotMatchLowChOpState = OcoOpOutputHigh;	//bit[7:6]<-->(CNT==OCCR==CPSR)		通道值==模值的特例
	stcLowChCmpMode.enCntPeakLowNotMatchHighNotMatchLowChOpState = OcoOpOutputHold;	//bit[13:12]<-->(CNT==CPSR!=OCCR)	通道值>模值的特例
	//stcLowChCmpMode.enMatchConditionExtendCmd = Enable;
	TIMER4_OCO_SetLowChCompareMode(TIMER4_UNIT, TIMER4_OCO_LOW_CH, &stcLowChCmpMode);  /* Set OCO low channel compare mode */

    /* Enable OCO */
    TIMER4_OCO_OutputCompareCmd(TIMER4_UNIT, TIMER4_OCO_HIGH_CH, Enable);
    TIMER4_OCO_OutputCompareCmd(TIMER4_UNIT, TIMER4_OCO_LOW_CH, Enable);

    /* Initialize PWM I/O */
    PORT_SetFunc(TIMER4_PWM_H_PORT, TIMER4_PWM_H_PIN, Func_Tim4, Disable);
    PORT_SetFunc(TIMER4_PWM_L_PORT, TIMER4_PWM_L_PIN, Func_Tim4, Disable);



    /* Timer4 PWM: Initialize PWM configuration structure */
    stcPwmInit.enRtIntMaskCmd = Enable;
    stcPwmInit.enClkDiv = PwmPlckDiv2;
    stcPwmInit.enOutputState = PwmHPwmLHold;//PwmHHoldPwmLReverse;
    stcPwmInit.enMode = PwmThroughMode;
    TIMER4_PWM_Init(TIMER4_UNIT, TIMER4_PWM_CH, &stcPwmInit); /* Initialize timer4 pwm */

    /* Clear && Start CNT */
    TIMER4_CNT_ClearCountVal(TIMER4_UNIT);
    TIMER4_CNT_Start(TIMER4_UNIT);
	
    while (1)
    {
		TIMER4_OCO_WriteOccr(M4_TMR43, TIMER4_OCO_HIGH_CH, g_PWMDuty); /* Set OCO low channel compare value */
		TIMER4_OCO_WriteOccr(M4_TMR43, TIMER4_OCO_LOW_CH, g_PWMDuty+1); /* Set OCO low channel compare value */
		if(g_PWMDuty < TIMER4_CNT_CYCLE_VAL)
			g_PWMDuty += TIMER4_CNT_CYCLE_VAL / 5;
		else
			g_PWMDuty = 0;
		Ddl_Delay1ms(200);
		SWDT_RefreshCounter();		/*看门狗喂狗函数*/
    }
#elif (TEST == TEST2)	//H,L互补输出
    stc_timer4_cnt_init_t stcCntInit;
    stc_timer4_oco_init_t stcOcoInit;
    stc_timer4_pwm_init_t stcPwmInit;
    stc_oco_low_ch_compare_mode_t stcLowChCmpMode;
	
    /* Clear structures */
    MEM_ZERO_STRUCT(stcCntInit);
    MEM_ZERO_STRUCT(stcOcoInit);
    MEM_ZERO_STRUCT(stcPwmInit);
    MEM_ZERO_STRUCT(stcLowChCmpMode);
	
    /* Enable peripheral clock */
    PWC_Fcg2PeriphClockCmd(PWC_FCG2_PERIPH_TIM43, Enable);

    /* Timer4 CNT : Initialize CNT configuration structure */
    stcCntInit.enBufferCmd = Disable;
    stcCntInit.enClk = Timer4CntPclk;
    stcCntInit.enClkDiv = Timer4CntPclkDiv2;  //fDR_CNT = fPCLK1/2 = 64MHz/2 = 32MHz
    stcCntInit.u16Cycle = TIMER4_CNT_CYCLE_VAL;
    stcCntInit.enCntMode = Timer4CntSawtoothWave;
    stcCntInit.enZeroIntCmd = Disable;
    stcCntInit.enPeakIntCmd = Disable;
    stcCntInit.enZeroIntMsk = Timer4CntIntMask0;
    stcCntInit.enPeakIntMsk = Timer4CntIntMask0;
    TIMER4_CNT_Init(M4_TMR43, &stcCntInit); /* Initialize CNT */

    /* Timer4 OCO : Initialize OCO configuration structure */
    stcOcoInit.enOcoIntCmd = Disable;
    stcOcoInit.enPortLevel = OcPortLevelLow;
    stcOcoInit.enOccrBufMode = OccrBufDisable;
    stcOcoInit.enOcmrBufMode = OcmrBufDisable;
    TIMER4_OCO_Init(M4_TMR43, Timer4OcoOul, &stcOcoInit);  /* Initialize OCO low channel */
    TIMER4_OCO_Init(M4_TMR43, Timer4OcoOvl, &stcOcoInit);  /* Initialize OCO low channel */
    TIMER4_OCO_Init(M4_TMR43, Timer4OcoOwl, &stcOcoInit);  /* Initialize OCO low channel */
	
    /*************Timer4 OCO ocmr1[31:0] = 0x0FF0 0FFF*****************************/
	/* OCMR[31:0] Ox 0FF0 0FFF    0000 1111 1111 0000   0000 1111 1111 1111 */
	stcLowChCmpMode.enCntZeroLowMatchHighNotMatchLowChOpState = OcoOpOutputLow;		//bit[11:10]<-->(0==CNT==OCCR) 		通道值==0的特例
	stcLowChCmpMode.enCntZeroLowNotMatchHighNotMatchLowChOpState = OcoOpOutputHigh;	//bit[15:14]<-->(0==CNT!=OCCR)		CNT自加的过程
	stcLowChCmpMode.enCntUpCntLowMatchHighNotMatchLowChOpState = OcoOpOutputLow; 	//bit[9:8]<-->(CNT↑==OCCR!=0)		CNT自加到通道值的时刻
	stcLowChCmpMode.enCntPeakLowMatchHighNotMatchLowChOpState = OcoOpOutputHigh;	//bit[7:6]<-->(CNT==OCCR==CPSR)		通道值==模值的特例
	stcLowChCmpMode.enCntPeakLowNotMatchHighNotMatchLowChOpState = OcoOpOutputHold;	//bit[13:12]<-->(CNT==CPSR!=OCCR)	通道值>模值的特例
	TIMER4_OCO_SetLowChCompareMode(M4_TMR43, Timer4OcoOul, &stcLowChCmpMode);  /* Set OCO low channel compare mode */
	TIMER4_OCO_SetLowChCompareMode(M4_TMR43, Timer4OcoOvl, &stcLowChCmpMode);  /* Set OCO low channel compare mode */
	TIMER4_OCO_SetLowChCompareMode(M4_TMR43, Timer4OcoOwl, &stcLowChCmpMode);  /* Set OCO low channel compare mode */
	
	/* Enable OCO */
    TIMER4_OCO_OutputCompareCmd(M4_TMR43, Timer4OcoOul, Enable);
    TIMER4_OCO_OutputCompareCmd(M4_TMR43, Timer4OcoOvl, Enable);
    TIMER4_OCO_OutputCompareCmd(M4_TMR43, Timer4OcoOwl, Enable);
	
    /* Initialize PWM I/O */    
	PORT_SetFunc(PortB, Pin09, Func_Tim4, Disable);	//为什么是Disable的解释：Disable的是引脚的双周边(副)功能,Enable则引脚就同时配置为2个功能!
    PORT_SetFunc(PortB, Pin08, Func_Tim4, Disable);
	PORT_SetFunc(PortB, Pin07, Func_Tim4, Disable);	//为什么是Disable的解释：Disable的是引脚的双周边(副)功能,Enable则引脚就同时配置为2个功能!
    PORT_SetFunc(PortB, Pin06, Func_Tim4, Disable);
	PORT_SetFunc(PortB, Pin05, Func_Tim4, Disable);	//为什么是Disable的解释：Disable的是引脚的双周边(副)功能,Enable则引脚就同时配置为2个功能!
    PORT_SetFunc(PortB, Pin04, Func_Tim4, Disable);


	
    /* Timer4 PWM: Initialize PWM configuration structure */
    stcPwmInit.enRtIntMaskCmd = Enable;
    stcPwmInit.enClkDiv = PwmPlckDiv2;	//fDR_CNT = fPCLK1/2 = 64MHz/2 = 32MHz
    stcPwmInit.enOutputState = PwmHPwmLHold;	//OXH、OXL输出不反转
    stcPwmInit.enMode = PwmDeadTimerMode;
    TIMER4_PWM_Init(M4_TMR43, Timer4PwmU, &stcPwmInit); /* Initialize timer4 pwm */
    TIMER4_PWM_Init(M4_TMR43, Timer4PwmV, &stcPwmInit); /* Initialize timer4 pwm */
    TIMER4_PWM_Init(M4_TMR43, Timer4PwmW, &stcPwmInit); /* Initialize timer4 pwm */
    TIMER4_PWM_WriteDeadRegionValue(M4_TMR43, Timer4PwmV, 10u, 10);	//左右死区时间 = 1/32MHz * 10 = 0.3125us
    TIMER4_PWM_WriteDeadRegionValue(M4_TMR43, Timer4PwmU, 10u, 10);	//左右死区时间 = 1/32MHz * 10 = 0.3125us
    TIMER4_PWM_WriteDeadRegionValue(M4_TMR43, Timer4PwmW, 10u, 10);	//左右死区时间 = 1/32MHz * 10 = 0.3125us
	
    /* Clear && Start CNT */
    TIMER4_CNT_ClearCountVal(M4_TMR43);
    TIMER4_CNT_Start(M4_TMR43);
//	//test
//    while (1)
//    {
//		TIMER4_OCO_WriteOccr(M4_TMR43, Timer4OcoOuh, g_PWMDuty+1); /* Set OCO low channel compare value */
//		TIMER4_OCO_WriteOccr(M4_TMR43, Timer4OcoOul, g_PWMDuty); /* Set OCO low channel compare value */
//		TIMER4_OCO_WriteOccr(M4_TMR43, Timer4OcoOvh, g_PWMDuty+1); /* Set OCO low channel compare value */
//		TIMER4_OCO_WriteOccr(M4_TMR43, Timer4OcoOvl, g_PWMDuty); /* Set OCO low channel compare value */
//		TIMER4_OCO_WriteOccr(M4_TMR43, Timer4OcoOwh, g_PWMDuty+1); /* Set OCO low channel compare value */
//		TIMER4_OCO_WriteOccr(M4_TMR43, Timer4OcoOwl, g_PWMDuty); /* Set OCO low channel compare value */		
//		if(g_PWMDuty < TIMER4_CNT_CYCLE_VAL)
//			g_PWMDuty += TIMER4_CNT_CYCLE_VAL / 5;
//		else
//			g_PWMDuty = 0;
//		Ddl_Delay1ms(200);
//		SWDT_RefreshCounter();		/*看门狗喂狗函数*/
//    }
#endif
#endif
}
/**
 *******************************************************************************
 ** \brief  ADC configuration, including clock configuration, initial configuration,
 **         channel configuration and trigger source configuration.
 **
 ** \param  None.
 **
 ** \retval None.
 **
 ******************************************************************************/
void AdcConfig(void)
{
    stc_adc_init_t stcAdcInit;

    /* configuration structure initialization */
    MEM_ZERO_STRUCT(stcAdcInit);

    CLK_SetPeriClkSource(ClkPeriSrcPclk);

    /* Configuration peripheral clock */
    PWC_Fcg3PeriphClockCmd(PWC_FCG3_PERIPH_ADC1, Enable);

    stcAdcInit.enResolution = AdcResolution_12Bit;
    stcAdcInit.enDataAlign  = AdcDataAlign_Right;
    stcAdcInit.enAutoClear  = AdcClren_Disable;
    stcAdcInit.enScanMode   = AdcMode_SAOnce;

    
    ADC_Init(M4_ADC1, &stcAdcInit);
    // ADC_Init(M4_ADC2, &stcAdcInit);

    // AdcChannelConfig();
}
/**
 *******************************************************************************
 ** \brief  Main function of example project
 **
 ** \param  None
 **
 ** \retval int32_t return value, if needed
 **
 ******************************************************************************/
void timer0_uart_Init(void) //HC32F460的空闲中断基于timer0实现
{
    stc_clk_freq_t stcClkTmp;
    stc_tim0_base_init_t stcTimerCfg;
    stc_tim0_trigger_init_t StcTimer0TrigInit;

    MEM_ZERO_STRUCT(stcClkTmp);
    MEM_ZERO_STRUCT(stcTimerCfg);
    MEM_ZERO_STRUCT(StcTimer0TrigInit);

    /* Timer0 peripheral enable */
    PWC_Fcg2PeriphClockCmd(PWC_FCG2_PERIPH_TIM02, Enable);

    /* Clear CNTAR register for channel A */
    TIMER0_WriteCntReg(M4_TMR02, Tim0_ChannelA, 0u);
    TIMER0_WriteCntReg(M4_TMR02, Tim0_ChannelB, 0u);

    /* Config register for channel A */
    stcTimerCfg.Tim0_CounterMode = Tim0_Async;
    stcTimerCfg.Tim0_AsyncClockSource = Tim0_XTAL32;
    stcTimerCfg.Tim0_ClockDivision = Tim0_ClkDiv8;
    stcTimerCfg.Tim0_CmpValue = 16;
    TIMER0_BaseInit(M4_TMR02, Tim0_ChannelA, &stcTimerCfg);

    /* Clear compare flag */
    TIMER0_ClearFlag(M4_TMR02, Tim0_ChannelA);

    /* Config timer0 hardware trigger */
    StcTimer0TrigInit.Tim0_InTrigEnable = false;
    StcTimer0TrigInit.Tim0_InTrigClear = true;
    StcTimer0TrigInit.Tim0_InTrigStart = true;
    StcTimer0TrigInit.Tim0_InTrigStop = false;
    TIMER0_HardTriggerInit(M4_TMR02, Tim0_ChannelA, &StcTimer0TrigInit);
}
/**
 *******************************************************************************
 ** \brief Configure uart peripheral function
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
void Uart1_Init(uint32_t bdr)
{
    stc_irq_regi_conf_t stcIrqRegiCfg;
    stc_usart_uart_init_t stcInitCfg;

    MEM_ZERO_STRUCT(stcIrqRegiCfg);
    MEM_ZERO_STRUCT(stcInitCfg);

    /* Enable peripheral clock */
    PWC_Fcg1PeriphClockCmd(PWC_FCG1_PERIPH_USART1, Enable);
    /* Initialize USART IO */
    PORT_SetFunc(UART_MCU_RX_PORT, UART_MCU_RX_PIN, UART_MCU_RX_FUNC, Disable);
    PORT_SetFunc(UART_MCU_TX_PORT, UART_MCU_TX_PIN, UART_MCU_TX_FUNC, Disable);

    /* Initialize UART */
    stcInitCfg.enClkMode = UsartIntClkCkNoOutput;
    if(bdr <= 2400)
        stcInitCfg.enClkDiv = UsartClkDiv_64;       //1200bps
    else if(bdr <= 19200)
        stcInitCfg.enClkDiv = UsartClkDiv_16;       //9600bps
    else
        stcInitCfg.enClkDiv = UsartClkDiv_4;        //115200bps
    stcInitCfg.enDataLength = UsartDataBits8;
    stcInitCfg.enDirection = UsartDataLsbFirst;
    stcInitCfg.enStopBit = UsartOneStopBit;
    stcInitCfg.enParity = UsartParityNone;
    stcInitCfg.enSampleMode = UsartSamleBit8;
    stcInitCfg.enDetectMode = UsartStartBitFallEdge;
    stcInitCfg.enHwFlow = UsartRtsEnable;
    if(Ok != USART_UART_Init(UART_MCU, &stcInitCfg))
    {
        while (1);
    }

    /* Set baudrate */
    if(Ok != USART_SetBaudrate(UART_MCU, bdr))
    {
        while (1);
    }

    /* Set USART RX IRQ */
    stcIrqRegiCfg.enIRQn = UART_MCU_RI_IRQN;
    stcIrqRegiCfg.pfnCallback = &Uart_MCU_Callback_Rx;
    stcIrqRegiCfg.enIntSrc = UART_MCU_RI_NUM;
    enIrqRegistration(&stcIrqRegiCfg);
//    NVIC_SetPriority(UART_485_RI_IRQN, UART_IRQ_PRIORITY);
//    NVIC_ClearPendingIRQ(UART_485_RI_IRQN);
//    NVIC_EnableIRQ(UART_485_RI_IRQN);
    /* Set USART RX error IRQ */
    stcIrqRegiCfg.enIRQn = UART_MCU_EI_IRQN;
    stcIrqRegiCfg.pfnCallback = &Uart_MCU_Callback_Err;
    stcIrqRegiCfg.enIntSrc = UART_MCU_EI_NUM;
    enIrqRegistration(&stcIrqRegiCfg);
//    NVIC_SetPriority(UART_485_EI_IRQN, UART_IRQ_PRIORITY);
//    NVIC_ClearPendingIRQ(UART_485_EI_IRQN);
//    NVIC_EnableIRQ(UART_485_EI_IRQN);
#if 0	//HC32的rxtimeout中断不好用
    /* Set USART RX timeout error IRQ */
    stcIrqRegiCfg.enIRQn = UART_MCU_RTOI_IRQN;
    stcIrqRegiCfg.pfnCallback = &Uart_MCU_Callback_Idle;
    stcIrqRegiCfg.enIntSrc = UART_485_RTOI_NUM;
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(UART_485_RTOI_IRQN, UART_IRQ_PRIORITY);
    NVIC_ClearPendingIRQ(UART_485_RTOI_IRQN);
    NVIC_EnableIRQ(UART_485_RTOI_IRQN);
#endif
    /* Set USART TX IRQ */
    stcIrqRegiCfg.enIRQn = UART_MCU_TI_IRQN;
    stcIrqRegiCfg.pfnCallback = &Uart_MCU_Callback_Tx;
    stcIrqRegiCfg.enIntSrc = UART_MCU_TI_NUM;
    enIrqRegistration(&stcIrqRegiCfg);
//    NVIC_SetPriority(UART_485_TI_IRQN, UART_IRQ_PRIORITY);
//    NVIC_ClearPendingIRQ(UART_485_TI_IRQN);
//    NVIC_EnableIRQ(UART_485_TI_IRQN);
    /* Set USART TX complete IRQ */
    stcIrqRegiCfg.enIRQn = UART_MCU_TCI_IRQN;
    stcIrqRegiCfg.pfnCallback = &Uart_MCU_Callback_TC;
    stcIrqRegiCfg.enIntSrc = UART_MCU_TCI_NUM;
    enIrqRegistration(&stcIrqRegiCfg);
//    NVIC_SetPriority(UART_485_TCI_IRQN, UART_IRQ_PRIORITY);
//    NVIC_ClearPendingIRQ(UART_485_TCI_IRQN);
//    NVIC_EnableIRQ(UART_485_TCI_IRQN);

    /*Enable RX && TX function*/
    USART_FuncCmd(UART_MCU, UsartRx, Enable);
    USART_FuncCmd(UART_MCU, UsartTx, Enable);
    USART_FuncCmd(UART_MCU, UsartRxInt, Enable);
    USART_FuncCmd(UART_MCU, UsartTimeOut, Enable);
    USART_FuncCmd(UART_MCU, UsartTimeOutInt, Enable);
}
/**
 *******************************************************************************
 ** \brief Configure can peripheral function
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
void can_bdr_Init(uint32_t bdr, stc_can_bt_t *pInit)
{
	if(!pInit)
		return;
	//HC32F460的can_clk == 外部晶振时钟频率(这块板子是8MHz)
	//公式1: fTQ = can_clk / (PRESC + 1) --> TQ = (PRESC + 1) * (1 / can_clk)
	//公式2: BT = tSEG1 + tSEG2 = ((SEG_1 + 2) + (SEG_2 + 1)) * TQ
	//规则: SEG_1 >= SEG_2 + 1 && SEG_2 >= SJW
	//范围：SEG_1 ∈[0,63]  SEG_2 ∈[0,7]  SJW ∈[0,7]
	if(bdr == 1000000)			//1M
	{
		pInit->PRESC = 1u-1u;	//1分频
		pInit->SEG_1 = 5u-2u;
		pInit->SEG_2 = 3u-1u;
		pInit->SJW   = 3u-1u;		
	}else if(bdr == 500000)		//500k
	{
		pInit->PRESC = 2u-1u;	//2分频
		pInit->SEG_1 = 5u-2u;
		pInit->SEG_2 = 3u-1u;
		pInit->SJW   = 3u-1u;	
	}else						//default == 250k
	{
		pInit->PRESC = 4u-1u;	//4分频
		pInit->SEG_1 = 5u-2u;
		pInit->SEG_2 = 3u-1u;
		pInit->SJW   = 3u-1u;	
	}
}

/**
 *******************************************************************************
 ** \brief Configure uart peripheral function
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
//#if (PROTOCOL_TYPE == MODBUS_PROTOCOL)
void Uart3_Init(uint32_t bdr)
{
    stc_irq_regi_conf_t stcIrqRegiCfg;
    stc_usart_uart_init_t stcInitCfg;

    MEM_ZERO_STRUCT(stcIrqRegiCfg);
    MEM_ZERO_STRUCT(stcInitCfg);

    /* Enable peripheral clock */
    PWC_Fcg1PeriphClockCmd(PWC_FCG1_PERIPH_USART3, Enable);
    /* Initialize USART IO */
    PORT_SetFunc(UART_485_RX_PORT, UART_485_RX_PIN, UART_485_RX_FUNC, Disable);
    PORT_SetFunc(UART_485_TX_PORT, UART_485_TX_PIN, UART_485_TX_FUNC, Disable);

    /* Initialize UART */
    //stcInitCfg.enClkMode = UsartIntClkCkNoOutput;
    stcInitCfg.enClkMode = UsartIntClkCkOutput;     //想用IDLE(接收超时)功能需要选UsartIntClkCkOutput
    if(bdr <= 2400)
        stcInitCfg.enClkDiv = UsartClkDiv_64;       //1200bps
    else if(bdr <= 19200)
        stcInitCfg.enClkDiv = UsartClkDiv_16;       //9600bps
    else
        stcInitCfg.enClkDiv = UsartClkDiv_4;        //115200bps
    stcInitCfg.enDataLength = UsartDataBits8;
    stcInitCfg.enDirection = UsartDataLsbFirst;
    stcInitCfg.enStopBit = UsartOneStopBit;
//#if (PROTOCOL_TYPE == MODBUS_PROTOCOL)
    stcInitCfg.enParity = UsartParityEven;
//#elif (PROTOCOL_TYPE == CAN_PROTOCOL)
//    stcInitCfg.enParity = UsartParityNone;
//#endif
    stcInitCfg.enSampleMode = UsartSamleBit8;//UsartSampleBit8;
    stcInitCfg.enDetectMode = UsartStartBitFallEdge;
    stcInitCfg.enHwFlow = UsartRtsEnable;
    if(Ok != USART_UART_Init(UART_485, &stcInitCfg))
    {
        while (1);
    }

    /* Set baudrate */
    if(Ok != USART_SetBaudrate(UART_485, bdr))
    {
        while (1);
    }

    /* Set USART RX IRQ */
    stcIrqRegiCfg.enIRQn = UART_485_RI_IRQN;
    stcIrqRegiCfg.pfnCallback = &Uart_Callback_Rx;
    stcIrqRegiCfg.enIntSrc = UART_485_RI_NUM;
    enIrqRegistration(&stcIrqRegiCfg);
//    NVIC_SetPriority(UART_485_RI_IRQN, UART_IRQ_PRIORITY);
//    NVIC_ClearPendingIRQ(UART_485_RI_IRQN);
//    NVIC_EnableIRQ(UART_485_RI_IRQN);
    /* Set USART RX error IRQ */
    stcIrqRegiCfg.enIRQn = UART_485_EI_IRQN;
    stcIrqRegiCfg.pfnCallback = &Uart_Callback_Err;
    stcIrqRegiCfg.enIntSrc = UART_485_EI_NUM;
    enIrqRegistration(&stcIrqRegiCfg);
//    NVIC_SetPriority(UART_485_EI_IRQN, UART_IRQ_PRIORITY);
//    NVIC_ClearPendingIRQ(UART_485_EI_IRQN);
//    NVIC_EnableIRQ(UART_485_EI_IRQN);
#if 0	//HC32的rxtimeout中断不好用
    /* Set USART RX timeout error IRQ */
    stcIrqRegiCfg.enIRQn = UART_485_RTOI_IRQN;
    stcIrqRegiCfg.pfnCallback = &Uart_Callback_Idle;
    stcIrqRegiCfg.enIntSrc = UART_485_RTOI_NUM;
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(UART_485_RTOI_IRQN, UART_IRQ_PRIORITY);
    NVIC_ClearPendingIRQ(UART_485_RTOI_IRQN);
    NVIC_EnableIRQ(UART_485_RTOI_IRQN);
#endif
    /* Set USART TX IRQ */
    stcIrqRegiCfg.enIRQn = UART_485_TI_IRQN;
    stcIrqRegiCfg.pfnCallback = &Uart_Callback_Tx;
    stcIrqRegiCfg.enIntSrc = UART_485_TI_NUM;
    enIrqRegistration(&stcIrqRegiCfg);
//    NVIC_SetPriority(UART_485_TI_IRQN, UART_IRQ_PRIORITY);
//    NVIC_ClearPendingIRQ(UART_485_TI_IRQN);
//    NVIC_EnableIRQ(UART_485_TI_IRQN);
    /* Set USART TX complete IRQ */
    stcIrqRegiCfg.enIRQn = UART_485_TCI_IRQN;
    stcIrqRegiCfg.pfnCallback = &Uart_Callback_TC;
    stcIrqRegiCfg.enIntSrc = UART_485_TCI_NUM;
    enIrqRegistration(&stcIrqRegiCfg);
//    NVIC_SetPriority(UART_485_TCI_IRQN, UART_IRQ_PRIORITY);
//    NVIC_ClearPendingIRQ(UART_485_TCI_IRQN);
//    NVIC_EnableIRQ(UART_485_TCI_IRQN);

    /*Enable RX && TX function*/
    USART_FuncCmd(UART_485, UsartRx, Enable);
    USART_FuncCmd(UART_485, UsartTx, Enable);
    USART_FuncCmd(UART_485, UsartRxInt, Enable);
    USART_FuncCmd(UART_485, UsartTimeOut, Enable);
    USART_FuncCmd(UART_485, UsartTimeOutInt, Enable);
}    
//#elif (PROTOCOL_TYPE == CAN_PROTOCOL)
void can_Init(uint32_t bdr)
{
	stc_pwc_ram_cfg_t       stcRamCfg;
    stc_can_init_config_t   stcCanInitCfg;
    stc_can_filter_t        stcFilter;
    stc_can_txframe_t       stcTxFrame;

    stc_irq_regi_conf_t     stcIrqRegiConf;

    MEM_ZERO_STRUCT(stcRamCfg);
    MEM_ZERO_STRUCT(stcCanInitCfg);
    MEM_ZERO_STRUCT(stcFilter);
    MEM_ZERO_STRUCT(stcTxFrame);
	
	//<<Enable can peripheral clock and buffer(ram)
#if 0	//RAM配置? 干嘛用的?
    stcRamCfg.enRamOpMd = HighSpeedMd;
    stcRamCfg.enCan = DynamicCtl;
    PWC_RamCfg(&stcRamCfg);
#endif
    PWC_Fcg1PeriphClockCmd(PWC_FCG1_PERIPH_CAN, Enable);
	
	//<<Can filter config
    stcFilter.enFilterSel = CanFilterSel1;	//禁止所有ID的接收(f460上电默认开启Filter1关闭Filter2-Filter8,只要禁止Filter1就行)
    CAN_FilterConfig(&stcFilter, Disable);
	
    //<<CAN GPIO config
    PORT_SetFunc(CAN_RX_PORT, CAN_RX_PIN, CAN_RX_FUNC, Disable);
    PORT_SetFunc(CAN_TX_PORT, CAN_TX_PIN, CAN_TX_FUNC, Disable);
	
    //<<Can bit time config
	can_bdr_Init(bdr, &stcCanInitCfg.stcCanBt);
	
	//err config
	stcCanInitCfg.stcWarningLimit.CanErrorWarningLimitVal = 10u;	//
    stcCanInitCfg.stcWarningLimit.CanWarningLimitVal = 16-1u;	//(N+1)*8
	
	//rx config
    stcCanInitCfg.enCanRxBufAll  = CanRxNormal;			//CanRxNormal: 不存储错误数据; CanRxAll: 错误数据也存储
    stcCanInitCfg.enCanRxBufMode = CanRxBufOverwritten;	//CanRxBufNotStored: 接收buf溢出时,不存储新的数据! CanRxBufOverwritten: 接收buf溢出时,新数据覆盖最早收到的数据!
    stcCanInitCfg.enCanSAck      = CanSelfAckEnable;	//CanSelfAckEnable: LBME=1时,使能自应答功能
    stcCanInitCfg.enCanSTBMode   = CanSTBFifoMode;		//CanSTBPrimaryMode: 优先级模式根据ID自动判断,ID越小优先级越高; CanSTBFifoMode: FIFO模式根据数据帧写入的先后顺序发送

	CAN_Init(&stcCanInitCfg);
	
	//<<Can Irq Enable
    CAN_IrqCmd(CanRxIrqEn, Enable);
    CAN_IrqCmd(CanRxOverIrqEn, Enable);
    //CAN_IrqCmd(CanRxBufFullIrqEn, Enable);
	//CAN_IrqCmd(CanRxBufAlmostFullIrqEn, Enable);
	//CAN_IrqCmd(CanTxPrimaryIrqEn, Enable);
	//CAN_IrqCmd(CanTxSecondaryIrqEn, Enable);
    CAN_IrqCmd(CanErrorIrqEn, Enable);
	CAN_IrqCmd(CanBusErrorIrqEn, Enable);
	//CAN_IrqCmd(CanErrorPassiveIrqEn, Enable);
	CAN_IrqCmd(CanArbiLostIrqEn, Enable);
	
	CAN_IrqFlgClr(CanRxIrqFlg);
	CAN_IrqFlgClr(CanRxBufAlmostFullIrqFlg);
	CAN_IrqFlgClr(CanErrorIrqFlg);
	CAN_IrqFlgClr(CanBusErrorIrqFlg);
	CAN_IrqFlgClr(CanErrorWarningIrqFlg);
	CAN_IrqFlgClr(CanErrorPassivenodeIrqFlg);
	CAN_IrqFlgClr(CanErrorPassiveIrqFlg);
	CAN_IrqFlgClr(CanArbiLostIrqFlg);
	CAN_IrqFlgClr(CanRxBufFullIrqFlg);
	CAN_IrqFlgClr(CanRxOverIrqFlg);

	stcIrqRegiConf.enIRQn = CAN_RX_IRQN;			//中断号
    stcIrqRegiConf.enIntSrc = INT_CAN_INT;			//源
    stcIrqRegiConf.pfnCallback = &CAN_RxIrqCallBack;//回调函数
    enIrqRegistration(&stcIrqRegiConf);
//    NVIC_SetPriority(CAN_RX_IRQN, CAN_IRQ_PRIORITY);//优先级
//    NVIC_ClearPendingIRQ(CAN_RX_IRQN);
//    NVIC_EnableIRQ(CAN_RX_IRQN);
}


/**
 *******************************************************************************
 ** \brief  Main function of example project
 **
 ** \param  None
 **
 ** \retval int32_t return value, if needed
 **
 ******************************************************************************/
void flash_Init(void)
{
    /* Unlock EFM. */
    EFM_Unlock();

    /* Enable flash. */
    EFM_FlashCmd(Enable);
    /* Wait flash ready. */
    while(Set != EFM_GetFlagStatus(EFM_FLAG_RDY))
    {
        ;
    }
    /* Lock EFM. */
    EFM_Lock();
}

void Interrupt_Enable(void)
{
	/*所有外设中断放在进入mainLoop前最后开启(尽量避免外部通讯口中断挂起)*/
#if (MOTOR_TYPE == MOTOR_TYPE_BLDC)
	/*边沿中断 or 输入捕捉中断*/
    NVIC_ClearPendingIRQ(M1_HALLA_EIRQ_IRQN);                    /* Clear pending */
    NVIC_SetPriority(M1_HALLA_EIRQ_IRQN, M1_HALL_EIRQ_PRIORITY);   /* Set priority */
    NVIC_EnableIRQ(M1_HALLA_EIRQ_IRQN);                          /* Enable NVIC */

    NVIC_ClearPendingIRQ(M1_HALLB_EIRQ_IRQN);                    /* Clear pending */
    NVIC_SetPriority(M1_HALLB_EIRQ_IRQN, M1_HALL_EIRQ_PRIORITY);   /* Set priority */
    NVIC_EnableIRQ(M1_HALLB_EIRQ_IRQN);                          /* Enable NVIC */

    NVIC_ClearPendingIRQ(M1_HALLC_EIRQ_IRQN);                  /* Clear pending */
    NVIC_SetPriority(M1_HALLC_EIRQ_IRQN, M1_HALL_EIRQ_PRIORITY); /* Set priority */
    NVIC_EnableIRQ(M1_HALLC_EIRQ_IRQN);                            /* Enable NVIC */
#endif

	/*定时器中断*/
    NVIC_SetPriority(TIME0_IRQN, TIME0_IRQ_PRIORITY);
    NVIC_ClearPendingIRQ(TIME0_IRQN);
    NVIC_EnableIRQ(TIME0_IRQN);
    
	 /*UART中断*/
    NVIC_SetPriority(UART_MCU_RI_IRQN, UART_IRQ_PRIORITY);
    NVIC_ClearPendingIRQ(UART_MCU_RI_IRQN);
    NVIC_EnableIRQ(UART_MCU_RI_IRQN);

    NVIC_SetPriority(UART_MCU_EI_IRQN, UART_IRQ_PRIORITY);
    NVIC_ClearPendingIRQ(UART_MCU_EI_IRQN);
    NVIC_EnableIRQ(UART_MCU_EI_IRQN);

    NVIC_SetPriority(UART_MCU_TI_IRQN, UART_IRQ_PRIORITY);
    NVIC_ClearPendingIRQ(UART_MCU_TI_IRQN);
    NVIC_EnableIRQ(UART_MCU_TI_IRQN);

    NVIC_SetPriority(UART_MCU_TCI_IRQN, UART_IRQ_PRIORITY);
    NVIC_ClearPendingIRQ(UART_MCU_TCI_IRQN);
    NVIC_EnableIRQ(UART_MCU_TCI_IRQN);
	
//#if (PROTOCOL_TYPE == MODBUS_PROTOCOL)
    /*UART中断*/
    NVIC_SetPriority(UART_485_RI_IRQN, UART_IRQ_PRIORITY);
    NVIC_ClearPendingIRQ(UART_485_RI_IRQN);
    NVIC_EnableIRQ(UART_485_RI_IRQN);

    NVIC_SetPriority(UART_485_EI_IRQN, UART_IRQ_PRIORITY);
    NVIC_ClearPendingIRQ(UART_485_EI_IRQN);
    NVIC_EnableIRQ(UART_485_EI_IRQN);

    NVIC_SetPriority(UART_485_TI_IRQN, UART_IRQ_PRIORITY);
    NVIC_ClearPendingIRQ(UART_485_TI_IRQN);
    NVIC_EnableIRQ(UART_485_TI_IRQN);

    NVIC_SetPriority(UART_485_TCI_IRQN, UART_IRQ_PRIORITY);
    NVIC_ClearPendingIRQ(UART_485_TCI_IRQN);
    NVIC_EnableIRQ(UART_485_TCI_IRQN);
//#elif (PROTOCOL_TYPE == CAN_PROTOCOL)
	/*CAN中断*/
    NVIC_SetPriority(CAN_RX_IRQN, CAN_IRQ_PRIORITY);//优先级
    NVIC_ClearPendingIRQ(CAN_RX_IRQN);
    NVIC_EnableIRQ(CAN_RX_IRQN);    
//#endif
}

/**
 * @brief  Main function of template project
 * @param  None
 * @retval int32_t return value, if needed
 */

void bor_swdt_Init(void)
{
	/*f460的BOR配置见hc32f46x_icg.h中的宏ICG1_VDU0_REG_CONFIG*/
	
	/*f460的SWDT配置见hc32f46x_icg.h中的宏ICG0_SWDT_REG_CONFIG*/
	/*
	SWDT实用内部专用的RC时钟源 10KHZ
	复位时间=ICG0_SWDT_PERI*ICG0_SWDT_CKS/10k(s)
	目前使用看门狗复位定时800ms
	*/
}

//hz  pwm init
void hz_init(void)
{
    // 使能外设时钟
    PWC_Fcg2PeriphClockCmd(PWC_FCG2_PERIPH_TIMA2, Enable);
    PWC_Fcg1PeriphClockCmd(PWC_FCG0_PERIPH_AOS, Enable);  // 确认GPIO时钟使能方式

    // 配置PA03为TIMA2_CH4功能（修正复用功能号）
    PORT_SetFunc(PortA, Pin03, Func_Tima0, Disable);     // 确认Func_Tima2_Ch4是否为正确功能号

    // TIMA2时基参数配置
    stc_timera_base_init_t stcTimeraInit;
    MEM_ZERO_STRUCT(stcTimeraInit);
    stcTimeraInit.enClkDiv = TimeraPclkDiv1;
    stcTimeraInit.enCntMode = TimeraCountModeSawtoothWave;
    stcTimeraInit.enCntDir = TimeraCountDirUp;
    stcTimeraInit.enSyncStartupEn = Disable;
    stcTimeraInit.u16PeriodVal = 4000;
    TIMERA_BaseInit(M4_TMRA2, &stcTimeraInit);

    // 通道4比较输出配置（修正极性和占空比计算）
    stc_timera_compare_init_t stcTimerCompareInit;
    MEM_ZERO_STRUCT(stcTimerCompareInit);
    stcTimerCompareInit.u16CompareVal = 1500;      // 50%占空比（根据新极性配置）
    stcTimerCompareInit.enStartCountOutput = TimeraCountStartOutputHigh;
    stcTimerCompareInit.enStopCountOutput = TimeraCountStopOutputLow;
    stcTimerCompareInit.enCompareMatchOutput = TimeraCompareMatchOutputHigh; // 比较匹配时变高
    stcTimerCompareInit.enPeriodMatchOutput = TimeraPeriodMatchOutputLow;    // 周期匹配时变低
    stcTimerCompareInit.enCacheEn = Enable;
    stcTimerCompareInit.enTriangularTroughTransEn = Enable; // 三角波谷点更新缓存
    
    // 初始化通道4比较输出
    TIMERA_CompareInit(M4_TMRA2, TimeraCh4, &stcTimerCompareInit);
    TIMERA_CompareCmd(M4_TMRA2, TimeraCh4, Enable);  // 使能通道4输出
    
    // 启动TIMA2
    TIMERA_Cmd(M4_TMRA2, Enable);
}

// 全局环形缓冲区实例，供ADC采样函数访问
RingBuffer current_buffer;
void Application_Run(void)
{
    // 使用统一接口控制PWM设备
    DeviceCommandData_t cmd;
    PwmDutyParams_t duty_params;
    // PwmFreqParams_t freq_params;
    
    // 设置PWM1占空比为75%
    duty_params.duty_cycle = 75.0f;
    cmd.cmd = CMD_PWM_SET_DUTY_SLOW;
    cmd.device_id = 0;
    cmd.params = &duty_params;
    cmd.param_size = sizeof(PwmDutyParams_t);
    Device_Control(0, &cmd);
    
}

void Application_Run1(void)
{
    // 使用统一接口控制PWM设备
    DeviceCommandData_t cmd;
    PwmDutyParams_t duty_params;
    // PwmFreqParams_t freq_params;
    
    // 设置PWM1占空比为75%
    duty_params.duty_cycle = 15.0f;
    cmd.cmd = CMD_PWM_SET_DUTY_SLOW;
    cmd.device_id = 0;
    cmd.params = &duty_params;
    cmd.param_size = sizeof(PwmDutyParams_t);
    Device_Control(0, &cmd);
}
void Application_Run2(void)
{
    // 使用统一接口控制PWM设备
    DeviceCommandData_t cmd;
    PwmDutyParams_t duty_params;
    // PwmFreqParams_t freq_params;
    
    // 设置PWM1占空比为75%
    duty_params.duty_cycle = 50.0f;
    cmd.cmd = CMD_PWM_SET_DUTY_INSTANT;
    cmd.device_id = 0;
    cmd.params = &duty_params;
    cmd.param_size = sizeof(PwmDutyParams_t);
    Device_Control(0, &cmd);
}

void Application_Run3(void)
{
    // 使用统一接口控制PWM设备
    DeviceCommandData_t cmd;
    PwmDutyParams_t duty_params;
    // PwmFreqParams_t freq_params;
    
    // 设置PWM1占空比为75%
    duty_params.change_duty_time = 7;
    duty_params.slowstart_start_duty = 10;
    duty_params.slowstart_target_duty= 95;
    cmd.cmd = CMD_PWM_SLOWSTART;
    cmd.device_id = 0;
    cmd.params = &duty_params;
    cmd.param_size = sizeof(PwmDutyParams_t);
    Device_Control(0, &cmd);
}

void Application_Run4(void)
{
    // 使用统一接口控制PWM设备
    DeviceCommandData_t cmd;
    PwmDutyParams_t duty_params;
    // PwmFreqParams_t freq_params;
    
    memset(&cmd, 0, sizeof(DeviceCommandData_t));
    memset(&duty_params, 0, sizeof(PwmDutyParams_t));
    // 设置PWM1占空比为75%
    duty_params.change_duty_time = PWM_SLOWSTART_TIME;
    duty_params.slowstart_start_duty = PWM_SLOWSTART_START_DUTY;
    duty_params.slowstart_target_duty = PWM_SLOWSTART_TARGET_DUTY;
    cmd.cmd = CMD_PWM_SLOWSTART;
    cmd.device_id = 0;
    cmd.params = &duty_params;
    cmd.param_size = sizeof(PwmDutyParams_t);
    Device_Control(cmd.device_id, &cmd);
}
void Application_Run5(void)
{
    DeviceCommandData_t cmd;
    PwmFreqParams_t freq_params;

    freq_params.frequency = 15000;
    cmd.cmd = CMD_PWM_SET_FREQUENCY;
    cmd.device_id = 0;
    cmd.params = &freq_params;
    cmd.param_size = sizeof(PwmFreqParams_t);
    Device_Control(0,&cmd);
}
void Application_Run6(void)
{
    DeviceCommandData_t cmd;
    PwmFreqParams_t freq_params;

    freq_params.frequency = 12000;
    cmd.cmd = CMD_PWM_SET_FREQUENCY;
    cmd.device_id = 0;
    cmd.params = &freq_params;
    cmd.param_size = sizeof(PwmFreqParams_t);
    Device_Control(0,&cmd);
}
void Application_Run7(void)
{
    DeviceCommandData_t cmd;
    cmd.cmd = CMD_PWM_STOP;
    cmd.device_id = 0;
    Device_Control(0,&cmd);
}




int32_t main(void)
{
    
    SystemClk_Init();

    SysMainState_Init();   // 主状态机初始化

    timer0_Init(1000);  //100000 即1000ms



    Hardware_Init();       // 硬件初始化

    ring_buffer_init(&current_buffer);

    tickTimer_DelayMs(70);
  // 初始化设备管理器
    DeviceManager_Init();
    phv = MotorPWM_Create(PHV_PORT, PHV_PIN, M4_TMRA4, PHV_CHANNEL, Enable);
    phu = MotorPWM_Create(PHU_PORT, PHU_PIN, M4_TMRA4, PHU_CHANNEL, Enable);

    REGISTER_PWM_DEVICE(0, "phv", phv);
    REGISTER_PWM_DEVICE(1, "phu", phu);

    // 初始化所有设备
    for (uint8_t i = 0; i < MAX_DEVICES; i++) {
        if (s_device_registry[i].used) {
            Device_Init(i);
        }
    }
    
    while(1)
    {
        XWM_Check_Process();
        SysMainState_Process();
        // USART1_Send_Test();
    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
//这个代码是一臂为IO，另一臂PWM波控制（另一臂是驱动芯片的下臂PWM波缓启动  PS：低电平输入给驱动芯片时，驱动芯片下臂输出才为高电平 ）

