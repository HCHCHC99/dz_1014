#include "dev_pwm.h"
#include "hz_timer.h"
#include <math.h>
#include <stdlib.h>
#include "string.h"

// ????
static PWM_Channel_t s_pwmChannels[MAX_PWM_CHANNELS] ;
static uint8_t       s_numChannels = 0;

PWM_Handle_t phu = NULL;
PWM_Handle_t phv = NULL;

static en_result_t PWM_Duty_3s(PWM_Channel_t* channel, float startDuty, float targetDuty,
                              uint16_t totalSteps, uint16_t stepDelayMs)
{
    float currentDuty = startDuty;
    const float step = (targetDuty > startDuty) ? 1.0f : -1.0f;

    for(uint16_t i = 0; i < totalSteps; i++) {
        currentDuty += step;

        /* ???? */
        if((step > 0 && currentDuty > targetDuty) ||
           (step < 0 && currentDuty < targetDuty)) {
            currentDuty = targetDuty;
        }

        /* ??PWM:????????????? */
        channel->ops.set_duty(channel->Timer, channel->timerChannel, currentDuty);

        /* ?????????3? */
        tickTimer_DelayMs(stepDelayMs);
    }

    /* ???? */
    return channel->ops.set_duty(channel->Timer, channel->timerChannel, targetDuty);
}

PWM_Handle_t MotorPWM_Create(en_port_t port, en_pin_t pin,
                                 M4_TMRA_TypeDef* Timer,
                                 en_timera_channel_t timerChannel,
                                 bool polarity)
{
    if(s_numChannels >= MAX_PWM_CHANNELS) {
        return NULL;
    }

    PWM_Channel_t* channel = &s_pwmChannels[s_numChannels];

    // ???????
    channel->port = port;
    channel->pin = pin;
    channel->Timer = Timer;
    channel->timerChannel = timerChannel;

    // ??????:???????????
    channel->ops = hz_pwm_ops;

    // ?????
    channel->state = PWM_IDLE;
    channel->startDuty = 0.0f;
    channel->targetDuty = 0.0f;
    channel->currentDuty = 0.0f;
    channel->frequency = 0;
    channel->totalSteps = 0;
    channel->currentStep = 0;
    channel->lastUpdateTime = 0;
    channel->stepDelayMs = 0;
    channel->polarity = polarity;
    s_numChannels++;
    return (PWM_Handle_t)channel;
}

void PWM_Destroy(PWM_Handle_t handle)
{
    PWM_Channel_t* channel = (PWM_Channel_t*)handle;
    if(channel) {
        PWM_Stop(channel);
        // ??????
        memset(channel, 0, sizeof(PWM_Channel_t));
    }
}

en_result_t PWM_setDuty_noblock(PWM_Handle_t handle, float targetDuty, float time) {
    PWM_Channel_t* channel = (PWM_Channel_t*)handle;
    if(!channel) {
        return ErrorInvalidParameter;
    }

    /* ??????? */
    if (targetDuty < 0.0f || targetDuty > 100.0f) {
        return ErrorInvalidParameter;
    }

    if (channel->state != PWM_IDLE) {
        channel->state = PWM_IDLE;
    }

    /* ???????:????????????? */
    float currentDuty = channel->ops.get_duty(channel->Timer, channel->timerChannel);

    /* ?????????????? */
    float dutyDiff = fabsf(targetDuty - currentDuty);
    uint16_t totalSteps = (uint16_t)(dutyDiff / 1.0f); // ?1%??
    uint16_t stepDelayMs = time * 1000 / totalSteps;

    /* ?????????? */
    channel->startDuty = currentDuty;
    channel->targetDuty = targetDuty;
    channel->currentDuty = currentDuty;
    channel->totalSteps = totalSteps;
    channel->currentStep = 0;
    channel->stepDelayMs = stepDelayMs;
    channel->lastUpdateTime = tickTimer_GetCount();
    channel->state = MOTOR_PWM_RAMPING;

    /* ?????????:????????????? */
    return channel->ops.set_duty(channel->Timer, channel->timerChannel, currentDuty);
}

en_result_t PWM_setDuty_block(PWM_Handle_t handle, float targetDuty) {
    PWM_Channel_t* channel = (PWM_Channel_t*)handle;
    if(!channel) {
        return ErrorInvalidParameter;
    }

    /* ??????? */
    if (targetDuty < 0.0f || targetDuty > 100.0f) {
        return ErrorInvalidParameter;
    }

    /* ???????:????????????? */
    float currentDuty = channel->ops.get_duty(channel->Timer, channel->timerChannel);

    /* ????????? */
    float dutyDiff = fabsf(targetDuty - currentDuty);
    uint16_t totalSteps = (uint16_t)(dutyDiff / 1.0f); // ?1%??
    uint16_t stepDelayMs = MOTORPWM_BLOCK_TIME * 1000 / totalSteps;         // ???3?

    /* ????????????? */
    return PWM_Duty_3s(channel, currentDuty, targetDuty, totalSteps, stepDelayMs);
}

// ??????????
void PWM_DutyUpdate(PWM_Handle_t handle) {
    PWM_Channel_t* channel = (PWM_Channel_t*)handle;

    channel->frequency=channel->ops.get_frequency(channel->Timer);

    if(!channel || channel->state != MOTOR_PWM_RAMPING) {
        return;
    }

    uint32_t currentTime = tickTimer_GetCount();
    uint32_t elapsedTime = currentTime - channel->lastUpdateTime;

    if (elapsedTime >= channel->stepDelayMs) {
        /* ????????? */
        channel->currentStep++;

        if (channel->currentStep >= channel->totalSteps) {
            /* ????? */
            channel->currentDuty = channel->targetDuty;
            channel->state = MOTOR_PWM_COMPLETE;
        } else {
            /* ???????? */
            const float step = (channel->targetDuty > channel->startDuty) ? 1.0f : -1.0f;
            channel->currentDuty += step;

            /* ???? */
            if ((step > 0 && channel->currentDuty > channel->targetDuty) ||
                (step < 0 && channel->currentDuty < channel->targetDuty)) {
                channel->currentDuty = channel->targetDuty;
            }
        }

        /* ??PWM???:????????????? */
        channel->ops.set_duty(channel->Timer, channel->timerChannel, channel->currentDuty);

        /* ????? */
        channel->lastUpdateTime = currentTime;
    }
}

// ????PWM???(?????)
en_result_t PWM_setDutyInstant(PWM_Handle_t handle, float duty) {
    PWM_Channel_t* channel = (PWM_Channel_t*)handle;
    if(!channel) {
        return ErrorInvalidParameter;
    }

    /* ??????? */
    if (duty < 0.0f || duty > 100.0f) {
        return ErrorInvalidParameter;
    }

    /* ????????,??????? */
    if (channel->state == MOTOR_PWM_RAMPING) {
        PWM_StopRamp(handle);
    }

    /* ???????:????????????? */
    en_result_t result = channel->ops.set_duty(channel->Timer, channel->timerChannel, duty);

    /* ??????,????????? */
    if (result == Ok) {
        channel->currentDuty = duty;
    }

    return result;
}

// ??????(??????????)
PWM_State_t PWM_GetState(PWM_Handle_t handle) {
    PWM_Channel_t* channel = (PWM_Channel_t*)handle;
    return channel ? channel->state : PWM_IDLE;
}

bool PWM_IsRampingComplete(PWM_Handle_t handle) {
    PWM_Channel_t* channel = (PWM_Channel_t*)handle;
    return channel && channel->state == MOTOR_PWM_COMPLETE;
}

void PWM_StopRamp(PWM_Handle_t handle) {
    PWM_Channel_t* channel = (PWM_Channel_t*)handle;
    if(channel) {
        channel->state = PWM_IDLE;
    }
}

void PWM_SlowStart(PWM_Handle_t handle,float slowstart_start_duty,float slowstart_target_duty, float time) {
    PWM_Channel_t* channel = (PWM_Channel_t*)handle;
    if(channel) {
        channel->ops.set_duty(channel->Timer, channel->timerChannel, slowstart_start_duty);
        channel->ops.compare_enable(channel->Timer, channel->timerChannel);
        PWM_setDuty_noblock(handle, slowstart_target_duty, time);
    }
}

void PWM_Stop(PWM_Handle_t handle) {
    PWM_Channel_t* channel = (PWM_Channel_t*)handle;
    if(channel) {
        channel->ops.compare_disable(channel->Timer, channel->timerChannel);
        channel->ops.init(channel->port, channel->pin, channel->Timer, channel->timerChannel, 90.0f, channel->polarity); // ???????
    }
}

void PWM_InitAllChannels(void)
{
    memset(s_pwmChannels, 0, sizeof(s_pwmChannels));
    s_numChannels = 0;
}

en_result_t PWM_setFreq(PWM_Handle_t handle, uint16_t freqHz) {
    PWM_Channel_t* channel = (PWM_Channel_t*)handle;
    if(!channel) {
        return ErrorInvalidParameter;
    }

    // ????????,???????
    if (channel->state == MOTOR_PWM_RAMPING) {
        PWM_StopRamp(handle);
    }

    // ?????????????
    en_result_t result = channel->ops.set_frequency(channel->Timer, freqHz);

    // ????????,?????????(????????????????)
    if (result == Ok) {
        channel->currentDuty = channel->ops.get_duty(channel->Timer, channel->timerChannel);
    }

    return result;
}

// ============================================================================
// ???????????
// ============================================================================

en_result_t PWM_DeviceRead(void* handle, void* data, uint32_t size)
{
    PWM_Channel_t* channel = (PWM_Channel_t*)handle;
    PwmDeviceData_t* pwm_data = (PwmDeviceData_t*)data;

    if (!channel || !pwm_data || size < sizeof(PwmDeviceData_t)) {
        return ErrorInvalidParameter;
    }

    // ??PWM????
    pwm_data->duty_cycle = channel->currentDuty;
    pwm_data->frequency = (uint16_t)channel->frequency;
    pwm_data->state = (uint8_t)channel->state;

    return Ok;
}

en_result_t PWM_DeviceWrite(void* handle, const void* data, uint32_t size)
{
    PWM_Channel_t* channel = (PWM_Channel_t*)handle;
    const PwmDeviceData_t* pwm_data = (const PwmDeviceData_t*)data;

    if (!channel || !pwm_data || size < sizeof(PwmDeviceData_t)) {
        return ErrorInvalidParameter;
    }

    // ?????
    if (pwm_data->duty_cycle >= 0.0f && pwm_data->duty_cycle <= 100.0f) {
        return PWM_setDuty_noblock((PWM_Handle_t)channel, pwm_data->duty_cycle, 3);
    }

    return ErrorInvalidParameter;
}

en_result_t PWM_DeviceInit(void* handle)
{
    PWM_Channel_t* channel = (PWM_Channel_t*)handle;
    if (!channel) {
        return ErrorInvalidParameter;
    }

    // ???PWM??
    channel->ops.init(channel->port, channel->pin, channel->Timer, channel->timerChannel, 90.0f, channel->polarity); // ???????
    channel->ops.compare_enable(channel->Timer, channel->timerChannel);

    return Ok;
}

en_result_t PWM_DeviceControl(void* handle, DeviceCommandData_t* cmd)
{
    PWM_Channel_t* channel = (PWM_Channel_t*)handle;
    if (!channel || !cmd) {
        return ErrorInvalidParameter;
    }

    switch (cmd->cmd) {
        case CMD_PWM_SET_DUTY_INSTANT:{
            if(cmd->params && cmd->param_size == sizeof(PwmDutyParams_t)){
                PwmDutyParams_t* params = (PwmDutyParams_t*)cmd->params;
                return PWM_setDutyInstant((PWM_Handle_t)channel, params->duty_cycle);
            }
            break;
        }

        case CMD_PWM_SET_DUTY_SLOW: {
            if (cmd->params && cmd->param_size == sizeof(PwmDutyParams_t)) {
                PwmDutyParams_t* params = (PwmDutyParams_t*)cmd->params;
                return PWM_setDuty_noblock((PWM_Handle_t)channel, params->duty_cycle, params->change_duty_time);
            }
            break;
        }

        case CMD_PWM_SET_FREQUENCY: {
            if (cmd->params && cmd->param_size == sizeof(PwmFreqParams_t)) {
                volatile PwmFreqParams_t* params = (PwmFreqParams_t*)cmd->params;
                channel->ops.set_frequency(channel->Timer, params->frequency);
                channel->frequency = params->frequency;
                return Ok;
            }
            break;
        }

        case CMD_PWM_SLOWSTART:{
            if(cmd->params && cmd->param_size == sizeof(PwmDutyParams_t)){
                PwmDutyParams_t* params = (PwmDutyParams_t*)cmd->params;
                PWM_SlowStart((PWM_Handle_t)channel, params->slowstart_start_duty, params->slowstart_target_duty, params->change_duty_time);
                return Ok;
            }
            break;
        }

        case CMD_PWM_STOP:
            PWM_Stop((PWM_Handle_t)channel);
            return Ok;

        default:
            break;
    }

    return ErrorInvalidParameter;
}

en_result_t PWM_DeviceUpdate(void* handle)
{
    PWM_Channel_t* channel = (PWM_Channel_t*)handle;
    if (!channel) {
        return ErrorInvalidParameter;
    }

    // ??PWM??(?????)
    PWM_DutyUpdate((PWM_Handle_t)channel);
    return Ok;
}

void SlowStart_PWMDuty(PWM_Handle_t device_name)
{
    // ????????PWM??
    DeviceCommandData_t cmd;
    PwmDutyParams_t duty_params;
    memset(&cmd, 0, sizeof(DeviceCommandData_t));
    memset(&duty_params, 0, sizeof(PwmDutyParams_t));
    if(device_name==phv)
    {
        cmd.device_id = 0;
    }
    else if(device_name==phu)
    {
        cmd.device_id = 1;
    }

    duty_params.change_duty_time = PWM_SLOWSTART_TIME;
    duty_params.slowstart_start_duty = PWM_SLOWSTART_START_DUTY;
    duty_params.slowstart_target_duty = PWM_SLOWSTART_TARGET_DUTY;

    cmd.cmd = CMD_PWM_SLOWSTART;
    cmd.params = &duty_params;
    cmd.param_size = sizeof(PwmDutyParams_t);
    Device_Control(cmd.device_id, &cmd);
}

void SetDutyInstant_PWMDuty(PWM_Handle_t device_name, float target_duty)
{
    DeviceCommandData_t cmd;
    PwmDutyParams_t duty_params;
    memset(&cmd, 0, sizeof(DeviceCommandData_t));
    memset(&duty_params, 0, sizeof(PwmDutyParams_t));
    if(device_name==phv)
    {
        cmd.device_id = 0;
    }
    else if(device_name==phu)
    {
        cmd.device_id = 1;
    }

    duty_params.duty_cycle = target_duty;

    cmd.cmd = CMD_PWM_SET_DUTY_INSTANT;
    cmd.params = &duty_params;
    cmd.param_size = sizeof(PwmDutyParams_t);
    Device_Control(cmd.device_id, &cmd);
}

void Positive_Run(void)
{
    //HOV LOU 1
    //HOU LOV 0
    // SlowStart_PWMDuty(phu);//98->2
    SetDutyInstant_PWMDuty(phu,2);
    SetDutyInstant_PWMDuty(phv,98);

}

void Negative_Run(void)
{
    //HOV LOU 0
    //HOU LOV 1
    // SlowStart_PWMDuty(phv); //98->2
    SetDutyInstant_PWMDuty(phv,2);
    SetDutyInstant_PWMDuty(phu,98);
}

void Stop(void)
{
    //HOV LOU 0
    //HOU LOV 0
    SetDutyInstant_PWMDuty(phu,0);
    SetDutyInstant_PWMDuty(phv,0);
}
void All_DevicePwm_Update(void)
{
    // ????????
    for (uint8_t i = 0; i < MAX_DEVICES; i++) {
        if (s_device_registry[i].used) {
            Device_Update(i);
        }
    }
}
